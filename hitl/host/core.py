import ctypes as C, hashlib, json, lzma, math, os, queue, socket, struct, subprocess, sys, threading, time, uuid
from collections import Counter, deque
from pathlib import Path
from pymavlink.dialects.v20 import common as mav
from .abi import (
    CAP_WIND,
    Model,
    Worker,
    header,
    Input,
    io_stats,
    io_drain,
    io_events,
    diagnostics,
    clock_calibration,
)
from .scenario import load_scene, load_wind
from .packets import Encoder
from .errors import PreconditionError, OperationError
from .release import read_release
from .activity import MacActivity
from .param_wire import param_bits, with_bits, encode as encode_param
from . import mavftp
from .sd_files import SdFiles

RECONNECT_TIMEOUT_S = 45.0
LOG_DATA_BYTES = 90  # MAVLink LOG_DATA payload
LOG_REQUEST_BYTES = LOG_DATA_BYTES * 4096  # one LOG_REQUEST_DATA range (about 360 kB)
ULOG_MAGIC = b'ULog\x01\x12\x35'
LOG_NO_PROGRESS_S = 30.0  # fetch_log gives up after this long without a new piece (PX4 pauses ~4 s)
# fetch_log asks again only after this long without a piece: PX4 pauses LOG_DATA for ~2 s at times, and asking
# sooner only makes it resend (2026-10-16: 53 % duplicate pieces with 0.5 s).
LOG_STALL_S = 3.0
LOG_HOLE_JOIN = 64  # fetch_log's last part: missing runs this close (pieces) are asked for in one range
# param_readback: the firmware's own parameter definitions (PX4 build output) name every parameter, used or not.
PARAM_LIST = Path('deploy/00_native/firmware/parameters.json.xz')
PARAM_BATCH = 20  # PARAM_REQUEST_READ in flight at once; PX4 drops a reply silently when its TX buffer is full
PARAM_ROUNDS = 5  # re-request rounds for parameters that did not answer
PARAM_QUIET_S = 0.5  # a batch (or the list stream) is over after this long without a new PARAM_VALUE
PARAM_NO_PROGRESS_S = 15.0  # param_readback gives up after the initial list wait this long without a PARAM_VALUE
PARAM_RELEASE = Path('deploy/00_native')  # param_plan / param_deploy: the release whose params.json is applied
PARAM_SET_TRIES = 3  # PARAM_SET attempts per parameter; each waits PARAM_SET_WAIT_S for the matching PARAM_VALUE
PARAM_SET_WAIT_S = 2.0
PARAM_SAVE_WAIT_S = 3.0  # after the last write, before the reboot: PX4 autosave waits >= 300 ms, at most every 2 s


class ParamTxnStop(OperationError):
    """A parameter deploy precondition went away mid-transaction: stop, write nothing more, no rollback."""


def param_consistency(log, count):
    """Problems in a readback's PARAM_VALUE stream (name, index, count, type, bits per message): the used
    indices must be 0..count-1, one name each; a name keeps one index (-1 = unused), type and value; count does not
    change."""
    problems = []
    counts = sorted({entry[2] for entry in log})
    if len(counts) > 1:
        problems.append({'problem': 'count_changed', 'counts': counts})
    names_by_index, index_by_name, value_by_name = {}, {}, {}
    for name, index, _, kind, bits in log:
        if index >= 0:
            names_by_index.setdefault(index, set()).add(name)
        index_by_name.setdefault(name, set()).add(index)  # -1 (unused) included: used -> unused is a change
        value_by_name.setdefault(name, set()).add((kind, bits))
    for index in sorted(names_by_index):
        if len(names_by_index[index]) > 1:
            problems.append({'problem': 'index_conflict', 'index': index, 'names': sorted(names_by_index[index])})
        if count is None or index >= count:
            problems.append({'problem': 'index_out_of_range', 'index': index, 'count': count})
    for name in sorted(index_by_name):
        if len(index_by_name[name]) > 1:
            problems.append({'problem': 'name_index_conflict', 'name': name, 'indices': sorted(index_by_name[name])})
    for name in sorted(value_by_name):
        if len(value_by_name[name]) > 1:  # e.g. changed by QGC during the readback
            problems.append({'problem': 'value_changed', 'name': name, 'seen': sorted(value_by_name[name])})
    return problems


FC_ONLY_MISSING = 'model release (FC-only on this platform)'


class Core(SdFiles):
    def __init__(
        self,
        runtime,
        transport,
        log_dir,
        network=True,
        unarmed_only=False,
        fake_state_dir=None,
        route_settings=None,
    ):
        if not transport.fake and (fake_state_dir is not None or route_settings is not None):
            raise ValueError('State/route overrides are restricted to fake FC tests')
        if transport.fake and fake_state_dir is None:
            # The real state directory holds the real owner's recovery marker and parameter deploy latch.
            raise ValueError('A fake FC needs its own state directory (fake_state_dir)')
        self.fake_state_dir = fake_state_dir
        self.runtime = Path(runtime).resolve()
        self.project = self.runtime.parent
        self.transport = transport
        self.log_dir = Path(log_dir)
        self.log_dir.mkdir(parents=True, exist_ok=True)
        release, self.manifest = read_release(self.runtime)
        # An FC-only release (no model library; Linux, where MATLAB code generation is unavailable) still owns
        # the FC: status, readback, logs, SD card. Starting or resetting the model is refused (missing_conditions).
        self.fc_only = not self.manifest.get('library')
        self.model = None if self.fc_only else Model(release / self.manifest['library'])
        self.worker = Worker(release / self.manifest['worker'], self.model)
        if not self.fc_only and (
            self.model.info.build_id.decode() != self.manifest['build_id']
            or self.model.info.platform.decode() != self.manifest['platform']
            or self.model.info.fixed_step_ns != 4000000
            or self.model.info.output_phase != 0
        ):
            raise ValueError('Loaded model metadata mismatch')
        self.clock_sync = clock_calibration(self.worker)
        (self.log_dir / 'clock_sync.json').write_text(json.dumps(self.clock_sync, indent=2))
        self.profile = json.loads((release / 'profile.json').read_text())
        self.scene, self.initial = load_scene(self.runtime / 'scenarios/ground.json')
        if not self.fc_only:
            self.model.check(self.model.lib.hil_model_reset(C.byref(self.initial)))
        self.instance_id = str(uuid.uuid4())
        self.run_id = None
        self.host = 'ready'
        self.state = 'initialized'
        self.recovery = 'idle'
        self.recovery_required = False
        self.live_sent = False
        self.error = None
        self.unarmed_only = unarmed_only
        from .ipc import state_path

        self.recovery_file = state_path(self.runtime, fake_state_dir) / 'recovery.json'
        self.recovery_file.parent.mkdir(parents=True, exist_ok=True)
        if not transport.fake and self.recovery_file.exists():
            self.recovery_required = bool(
                json.loads(self.recovery_file.read_text()).get('pending', True)
            )
        # A parameter deploy that did not end PASS stays pending across owner restarts.
        self.param_latch_file = self.recovery_file.parent / 'param_deploy.json'
        self.param_deploy_pending = self._load_param_latch()
        self.operation_lock = threading.RLock()
        self.output_lock = threading.RLock()
        self.fc_lock = threading.RLock()
        self.quit = threading.Event()
        self.flush_request = threading.Event()  # flush_logs(): ask the log thread to drain and flush
        self.flush_done = threading.Event()
        self.fc = {
            'state': 'disconnected',
            'armed': None,
            'identity': None,
            'heartbeat_at': None,
            'SYS_HITL': None,
            'custom_mode': None,
        }
        self.telemetry = {}
        self.parameters = {}
        self.fc_messages = deque(maxlen=32)
        self.recovery_details = {}
        self.fc_generation = 0
        self.controls = [0.0] * 16
        self.control_valid = False
        self.control_at = 0
        self.control_sequence = 0
        self.last_control_frame = None
        self.pending_control = False
        self.parser = mav.MAVLink(None)
        self.parser.robust_parsing = True
        self.writer = mav.MAVLink(None, srcSystem=2, srcComponent=51)
        self.writer_lock = threading.Lock()
        self.encoder = Encoder(self.profile['gps_quality'])
        self.last_output = 0
        self.model_epoch_us = 0
        self.last_wire_us = 0
        self.last_agent_sticks = -math.inf
        self.counters = Counter()
        self.output = None
        self.native_io = False
        self.reboot_quiet = False
        self.reboot_queue_cleared = False
        self.commands = queue.Queue(maxsize=128)
        self.records = queue.Queue(maxsize=4096)
        self.last_ack = None
        self.ack_history = deque(maxlen=128)
        self.recover_context = None
        self.log_fetch = None  # fetch_log(): MAVLink log-download state, filled by _receive
        self.param_fetch = None  # param_readback(): PARAM_VALUE and AUTOPILOT_VERSION collected by _receive
        self.param_watch = None  # param_deploy: every PARAM_VALUE (name, bits) seen during the transaction
        self.ftp = None  # sd_* (sd_files.py): MAVLink FTP replies addressed to the owner, filled by _receive
        self.activity = MacActivity()  # App Nap guard, held from model start to model stop
        # The same guard for FC operations while the model is stopped (fetch-log, param-readback, param-deploy,
        # recover): without it the idle owner's serial reads were ~27 ms apart (2026-10-16: 32 instead of 740 kB/s;
        # consistent with App Nap, no A/B).
        self.transfer_activity = MacActivity()
        self.transfer_depth = 0
        self.transfer_lock = threading.Lock()
        self.activity_run_ok = None  # did the guard start with this run's model (None before the first run)
        self.pilot = None
        self.pilot_session = None
        self.pilot_log = None
        self.pilot_dir = self.log_dir / 'pilot'
        self.pilot_dir.mkdir(exist_ok=True)
        self.operations = deque()  # ipc.remember(): the newest 32 finished ones and every unfinished one
        self.route_lock = threading.RLock()
        self.external_controls_enabled = False
        self.control_gate_reason = 'model not running'
        self.pilot_stopping = False
        self.routes = {}
        if network:
            settings = route_settings or {'qgc': [14575, 14550], 'agent': [14576, 14560]}
            if set(settings) != set(['qgc', 'agent']):
                raise ValueError('Both qgc and agent routes are required')
            for role, (local, dest) in settings.items():
                sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                sock.bind(('127.0.0.1', local))
                sock.setblocking(False)
                self.routes[role] = (sock, ('127.0.0.1', dest))
        from .runtime_identity import capture

        active_checksum = hashlib.sha256((release / 'manifest.json').read_bytes()).hexdigest()
        self.runtime_identity = capture(
            self.project, self.log_dir, self.manifest['build_id'], active_checksum
        )
        self.logger = threading.Thread(target=self._record, name='hil-log', daemon=True)
        self.logger.start()
        self.io = threading.Thread(target=self._io, name='hil-serial', daemon=True)
        self.io.start()

    def _log(self, channel, data):
        try:
            self.records.put_nowait((self.worker.now(), channel, bytes(data)))
        except queue.Full:
            self.counters['log_drops'] += 1

    def _record(self):
        with (
            (self.log_dir / 'wire.bin').open('ab') as wire,
            (self.log_dir / 'timing.csv').open('a') as trace,
            (self.log_dir / 'io_events.jsonl').open('a') as events,
        ):
            while not self.quit.is_set() or not self.records.empty():
                for event in io_events(self.worker):
                    events.write(json.dumps(event) + '\n')
                for stamp, kind, data in io_drain(self.worker):
                    wire.write(struct.pack('<QBI', stamp, 2 if kind else 4, len(data)) + data)
                for row in self.worker.drain():
                    trace.write(','.join(map(str, row)) + '\n')
                    self.counters['trace_records'] += 1
                if self.flush_request.is_set():
                    # Barrier for flush_logs(): everything produced so far is on disk when flush_done is set.
                    while True:
                        try:
                            stamp, channel, data = self.records.get_nowait()
                        except queue.Empty:
                            break
                        wire.write(struct.pack('<QBI', stamp, channel, len(data)) + data)
                    for stream in (wire, trace, events):
                        stream.flush()
                    self.flush_request.clear()
                    self.flush_done.set()
                try:
                    stamp, channel, data = self.records.get(timeout=0.02)
                except queue.Empty:
                    # Idle (e.g. model stopped): flush, so a running owner's logs are complete on disk for
                    # flight-summary and the HITL trace conversion.
                    for stream in (wire, trace, events):
                        stream.flush()
                    continue
                wire.write(struct.pack('<QBI', stamp, channel, len(data)) + data)
            for event in io_events(self.worker):
                events.write(json.dumps(event) + '\n')
            for stamp, kind, data in io_drain(self.worker):
                wire.write(struct.pack('<QBI', stamp, 2 if kind else 4, len(data)) + data)
            for row in self.worker.drain():
                trace.write(','.join(map(str, row)) + '\n')
                self.counters['trace_records'] += 1

    def _fault(self, reason):
        self.error = reason
        self._block_external('fault: ' + reason)
        self._stop_model('fault')
        (self.pilot_dir / 'stop').touch()

    def _send_message(self, message):
        with self.writer_lock:
            raw = message.pack(self.writer)
            self.writer.seq = (self.writer.seq + 1) % 256
        with self.route_lock:
            self.commands.put_nowait(raw)

    def _receive(self, raw, now):
        self._log(1, raw)
        messages = self.parser.parse_buffer(raw) or []
        with self.fc_lock:
            for m in messages:
                name = m.get_type()
                identity = [m.get_srcSystem(), m.get_srcComponent()]
                if name == 'BAD_DATA':
                    self.counters['bad_frames'] += 1
                    continue
                if name == 'HEARTBEAT' and m.autopilot == 12 and m.type == 1:
                    if self.fc['identity'] and self.fc['identity'] != identity:
                        self.counters['foreign_heartbeat'] += 1
                        continue
                    first = self.fc['identity'] is None
                    self.fc.update(
                        state='verified',
                        identity=identity,
                        armed=bool(m.base_mode & 128),
                        heartbeat_at=now,
                        custom_mode=m.custom_mode,
                        system_status=m.system_status,
                    )
                    if first:
                        if self.state != 'fault':
                            self.error = None
                        self._send_message(
                            mav.MAVLink_param_request_read_message(*identity, b'SYS_HITL', -1)
                        )
                        self.counters['verified_connections'] += 1
                if identity != self.fc['identity']:
                    continue
                self.counters['rx_' + name] += 1
                if name == 'HIL_ACTUATOR_CONTROLS':
                    frame = bytes(m.get_msgbuf())
                    if frame == self.last_control_frame:
                        self.counters['duplicate_controls'] += 1
                        continue
                    if not all(math.isfinite(x) and abs(x) <= 1 for x in m.controls):
                        self.counters['invalid_controls'] += 1
                        continue
                    if self.pending_control:
                        self.counters['controls_replaced'] += 1
                    self.last_control_frame = frame
                    self.controls = list(m.controls)
                    self.control_valid = True
                    self.control_at = self.worker.now()
                    self.control_sequence += 1
                    self.pending_control = True
                if name == 'PARAM_VALUE':
                    key = m.param_id.rstrip('\0')
                    bits = param_bits(m)
                    value = struct.unpack('<i' if m.param_type == 6 else '<f', bits)[0]
                    self.parameters[key] = value
                    if key == 'SYS_HITL':
                        self.fc['SYS_HITL'] = value
                    if self.param_watch is not None and not key.startswith('_'):
                        self.param_watch.append((key, bits.hex()))
                    params = self.param_fetch
                    if params is not None and not key.startswith('_'):  # '_HASH_CHECK' is not a parameter
                        # PX4 sends param_index -1 (65535 on the wire) for a parameter that is not used.
                        index = m.param_index if m.param_index != 65535 else -1
                        params['values'][key] = {'value': value, 'type': m.param_type, 'index': index,
                                                 'bits': bits.hex()}
                        params['log'].append((key, index, m.param_count, m.param_type, bits.hex()))
                        if index >= 0:
                            params['by_index'][index] = key
                        params['count'] = m.param_count
                        params['activity'] = params['received_at'] = now
                        params['received'] += 1
                if name == 'FILE_TRANSFER_PROTOCOL' and self.ftp is not None and m.target_system in [0, 2] \
                        and m.target_component in [0, 51]:  # replies to another client (QGC) are not ours
                    self.ftp['replies'].append(mavftp.unpack(m.payload))
                if name == 'AUTOPILOT_VERSION' and self.param_fetch is not None:
                    custom = bytes(m.flight_custom_version)
                    self.param_fetch['autopilot_version'] = {
                        'flight_sw_version': m.flight_sw_version,
                        'board_version': m.board_version,
                        'flight_custom_version': custom.hex(),
                        # PX4 (mavlink_main.cpp): the first 5 bytes of the git hash, little-endian, in the top 5
                        # bytes; the low 3 bytes are the vendor version (0 for a plain vX.Y.Z tag).
                        'git_hash_prefix': custom[3:][::-1].hex(),
                        # The first 16 hex digits of the NuttX commit (os_git_version_binary, little-endian):
                        # tells a firmware with a NuttX patch committed apart from the native one.
                        'os_custom_version': bytes(m.os_custom_version).hex(),
                        'os_hash_prefix': bytes(m.os_custom_version)[::-1].hex(),
                        'vendor_id': m.vendor_id,
                        'product_id': m.product_id,
                        'uid': m.uid,
                    }
                fetch = self.log_fetch
                if fetch is not None and name == 'LOG_ENTRY':
                    fetch['entries'][m.id] = {'id': m.id, 'num_logs': m.num_logs, 'time_utc': m.time_utc,
                                              'size': m.size}
                    fetch['activity'] = now
                if fetch is not None and name == 'LOG_DATA' and m.id == fetch.get('id'):
                    # PX4 sends 90-byte pieces from the requested offset (requests are 90-aligned).
                    index, size = m.ofs // LOG_DATA_BYTES, fetch['size']
                    if m.ofs % LOG_DATA_BYTES == 0 and index < len(fetch['have']) and not fetch['have'][index]:
                        # A piece counts only when complete: 90 bytes, or the exact remainder at the end.
                        if m.count == min(LOG_DATA_BYTES, size - m.ofs):
                            fetch['data'][m.ofs:m.ofs + m.count] = bytes(m.data[:m.count])
                            fetch['have'][index] = 1
                            fetch['received'] += 1
                        else:
                            fetch['short_pieces'] = fetch.get('short_pieces', 0) + 1
                    fetch['activity'] = now
                if name == 'STATUSTEXT':
                    self.fc_messages.append(
                        {'at': now, 'severity': m.severity, 'text': m.text.rstrip('\0')}
                    )
                if name in ['ATTITUDE', 'LOCAL_POSITION_NED', 'VFR_HUD', 'GLOBAL_POSITION_INT']:
                    if name == 'ATTITUDE':
                        values = {
                            'roll_deg': math.degrees(m.roll),
                            'pitch_deg': math.degrees(m.pitch),
                            'yaw_deg': math.degrees(m.yaw),
                        }
                    elif name == 'LOCAL_POSITION_NED':
                        values = {
                            'height_local_m': -m.z,
                            'climb_m_s': -m.vz,
                            'north_m': m.x,
                            'east_m': m.y,
                        }
                    elif name == 'VFR_HUD':
                        values = {'airspeed_m_s': m.airspeed, 'groundspeed_m_s': m.groundspeed}
                    else:
                        values = {
                            'latitude_deg': m.lat / 1e7,
                            'longitude_deg': m.lon / 1e7,
                            'altitude_msl_m': m.alt / 1000,
                            'relative_altitude_m': m.relative_alt / 1000,
                        }
                    self.telemetry[name] = {
                        'received_at': now,
                        'values': {k: v if math.isfinite(v) else None for k, v in values.items()},
                    }
                if name == 'COMMAND_ACK':
                    self.last_ack = (now, m.command, m.result)
                    ack = {
                        'at': now,
                        'command': m.command,
                        'result': m.result,
                        'identity': identity,
                        'generation': self.fc_generation,
                    }
                    self.ack_history.append(ack)
                    context = self.recover_context
                    if (
                        context
                        and identity == context['identity']
                        and context['generation'] == self.fc_generation
                        and m.command == context['command']
                        and now >= context['sent_at']
                        and getattr(m, 'target_system', 0) in [0, 2]
                        and getattr(m, 'target_component', 0) in [0, 51]
                    ):
                        context['ack_result'] = m.result
                        if (
                            m.command == 246
                            and m.result == mav.MAV_RESULT_ACCEPTED
                            and not self.transport.fake
                        ):
                            self.reboot_quiet = True
                            self.reboot_queue_cleared = False
                            self.recovery_details['reboot_ack_at_s'] = now
                        if m.result not in [mav.MAV_RESULT_ACCEPTED, mav.MAV_RESULT_IN_PROGRESS]:
                            context['rejected'] = m.result
        if self.fc['state'] == 'verified':
            for sock, dest in self.routes.values():
                try:
                    sock.sendto(raw, dest)
                except OSError:
                    self.counters['route_errors'] += 1

    def _disconnect(self, reason):
        self.error = reason
        if self.state == 'running':
            self._fault(reason)
        self.worker.lib.hil_io_stop()
        self.native_io = False
        self.transport.close()
        self.fc_generation += 1
        with self.fc_lock:
            self.telemetry.clear()
            self.parameters.clear()
            self.fc.update(
                state='disconnected', armed=None, identity=None, heartbeat_at=None, SYS_HITL=None
            )
            self.controls = [0.0] * 16
            self.control_valid = self.pending_control = False
            self.control_sequence = 0
            self.last_control_frame = None
        self.parser = mav.MAVLink(None)
        self.parser.robust_parsing = True
        while not self.commands.empty():
            try:
                self.commands.get_nowait()
            except queue.Empty:
                break
        self.counters['disconnects'] += 1

    def _route_inputs(self, now):
        for role, (sock, dest) in self.routes.items():
            for _ in range(32):
                try:
                    raw, address = sock.recvfrom(65536)
                except BlockingIOError:
                    break
                if address[0] != '127.0.0.1' or self.fc['state'] != 'verified':
                    continue
                if self.unarmed_only:
                    self.counters['validation_blocked_external_input'] += 1
                    continue
                with self.route_lock:
                    # Everything from QGC and the agent is forwarded, whether or not the model runs
                    # (HIL drives no hardware). The only rerouting is the
                    # stick handover: QGC sticks go to the agent while the managed pilot is flying.
                    helpers = getattr(self.transport, 'helpers', None)
                    if helpers and self.external_controls_enabled:
                        other, sticks = helpers.split_manual_control(raw)
                        if role == 'agent' and sticks:
                            self.last_agent_sticks = now
                        if role == 'qgc' and sticks and now - self.last_agent_sticks < 0.2:
                            self.routes['agent'][0].sendto(sticks, self.routes['agent'][1])
                            raw = other
                            self.counters['manual_handover'] += 1
                    if raw:
                        self._write(raw)
                        self._log(3, raw)

    def _block_external(self, reason):
        with self.route_lock:
            self.external_controls_enabled = False
            self.control_gate_reason = reason
            self.last_agent_sticks = -math.inf
            while not self.commands.empty():
                try:
                    self.commands.get_nowait()
                    self.counters['purged_commands'] += 1
                except queue.Empty:
                    break
            if self.native_io:
                self.worker.lib.hil_io_clear_commands()
            for sock, dest in self.routes.values():
                for _ in range(128):
                    try:
                        sock.recvfrom(65536)
                        self.counters['purged_external_datagrams'] += 1
                    except BlockingIOError:
                        break

    def _allow_external(self):
        self._block_external('resuming control')
        with self.route_lock:
            if (
                self.state == 'running'
                and self.recovery in ['idle', 'complete']
                and not self.unarmed_only
            ):
                self.external_controls_enabled = True
                self.control_gate_reason = None

    def _io(self):
        connected = False
        retry = 0
        probe_started = 0
        next_link_heartbeat = 0
        next_hitl_read = 0
        while not self.quit.is_set():
            now = time.monotonic()
            try:
                if not connected:
                    if now < retry:
                        time.sleep(0.01)
                        continue
                    if self.transport.open() is False:
                        retry = now + 0.1
                        time.sleep(0.01)
                        continue
                    connected = True
                    probe_started = now
                    self.reboot_quiet = False
                    self.fc['state'] = 'probing'
                    descriptor = getattr(self.transport, 'native_fd', None)
                    if descriptor is not None:
                        if self.worker.lib.hil_io_start(descriptor):
                            raise OSError('Native serial writer start failed')
                        self.native_io = True
                if self.reboot_quiet and hasattr(self.transport, 'inspect_link'):
                    phase = self.transport.inspect_link()
                    if phase in ['bootloader', 'absent']:
                        raise OSError('PX4 reboot: waiting for application USB')
                raw = self.transport.read()
                if raw:
                    self._receive(raw, now)
                if self.reboot_quiet:
                    if not self.reboot_queue_cleared:
                        self._block_external('PX4 reboot quiet window')
                        self.reboot_queue_cleared = True
                    # Continue receiving/link observation, but no MAVLink bytes enter bootloader.
                    time.sleep(0.001)
                    continue
                # PX4 USB may wait for a GCS packet before transmitting after reconnect.
                # This is link keepalive only, and never publishes model sensors or control.
                if now >= next_link_heartbeat:
                    with self.writer_lock:
                        packet = mav.MAVLink_heartbeat_message(
                            mav.MAV_TYPE_GCS,
                            mav.MAV_AUTOPILOT_INVALID,
                            0,
                            0,
                            mav.MAV_STATE_ACTIVE,
                            3,
                        ).pack(self.writer)
                        self.writer.seq = (self.writer.seq + 1) % 256
                    self._write(packet)
                    self._log(5 if self.native_io else 4, packet)
                    next_link_heartbeat = now + 1
                if self.fc['SYS_HITL'] is None and now >= next_hitl_read:
                    target = self.fc['identity'] or [
                        1,
                        1,
                    ]  # Read-only probe of this recipe's expected FC.
                    with self.writer_lock:
                        packet = mav.MAVLink_param_request_read_message(
                            *target, b'SYS_HITL', -1
                        ).pack(self.writer)
                        self.writer.seq = (self.writer.seq + 1) % 256
                    self._write(packet)
                    self._log(5 if self.native_io else 4, packet)
                    next_hitl_read = now + 1
                if self.fc['identity'] is None and now - probe_started > 5:
                    raise OSError('Heartbeat probe timeout')
                hb = self.fc['heartbeat_at']
                if hb is not None and now - hb > 3:
                    self.fc['state'] = 'stale'
                    if self.state == 'running':
                        self._fault('FC heartbeat stale')
                if self.fc['state'] == 'verified':
                    self._route_inputs(now)
                    for _ in range(8):
                        with self.route_lock:
                            try:
                                packet = self.commands.get_nowait()
                            except queue.Empty:
                                break
                            self._write(packet)
                            self._log(5 if self.native_io else 4, packet)
                if self.state == 'running':
                    with self.fc_lock:
                        inp = header(Input)
                        inp.controls[:] = self.controls
                        inp.actuator_valid = int(self.control_valid)
                        inp.actuator_new = int(self.pending_control)
                        inp.input_sequence = self.control_sequence
                        inp.actuator_age_ns = self.control_at
                        inp.fc_armed = 2 if self.fc['armed'] is None else int(self.fc['armed'])
                        self.pending_control = False
                    self.worker.lib.hil_worker_input(C.byref(inp))
                    s = self.worker.snapshot()
                    if self.unarmed_only and self.fc['armed']:
                        self._fault('Unarmed validation boundary: FC armed')
                        continue
                    if self.control_valid and self.worker.now() - self.control_at >= 2000000000:
                        self._fault('Controls stale for 2 seconds')
                        continue
                    if s.fault:
                        self._fault('Native worker fault ' + str(s.fault))
                        continue
                    if self.native_io:
                        info = io_stats(self.worker)
                        if info['errors'] or info['log_drops']:
                            raise OSError('Native serial/log failure')
                    if s.produced and s.produced != self.last_output:
                        if self.last_output and not self.native_io:
                            self.counters['sensor_samples_not_forwarded'] += max(
                                0, s.produced - self.last_output - 1
                            )
                        self.last_output = s.produced
                        self.output = s.output
                        with self.output_lock:
                            if self.state != 'running':
                                continue
                            for kind, packet in (
                                []
                                if self.native_io
                                else self.encoder.encode(s.output, self.model_epoch_us)
                            ):
                                if self.fc['state'] != 'verified':
                                    raise OSError('FC not verified at write')
                                self.transport.write(packet)
                                self._log(2, packet)
                                self.counters['tx_' + kind] += 1
                                self.live_sent = True
                        if self.native_io and io_stats(self.worker)['sensor']:
                            self.live_sent = True
                        self.last_wire_us = self.model_epoch_us + s.output.model_time_ns // 1000
                time.sleep(0.0005)
            except (OSError, serial_error()) as exc:
                self._disconnect(str(exc))
                connected = False
                retry = time.monotonic() + 1
            except Exception as exc:
                self._fault(type(exc).__name__ + ': ' + str(exc))
                time.sleep(0.01)
        self.worker.lib.hil_io_stop()
        self.transport.close()

    def _write(self, raw):
        if self.native_io:
            for k in range(0, len(raw), 280):
                chunk = raw[k : k + 280]
                if self.worker.lib.hil_io_command(chunk, len(chunk)):
                    raise OSError('Native command queue overflow')
        else:
            self.transport.write(raw)

    def status(self):
        now = time.monotonic()
        s = self.worker.snapshot()
        if self.native_io:
            native = io_stats(self.worker)
            native_metrics = {
                'tx_HIL_SENSOR': native['sensor'],
                'tx_HIL_GPS': native['gps'],
                'tx_DISTANCE_SENSOR': self.worker.lib.hil_io_range_count() if hasattr(self.worker.lib, 'hil_io_range_count') else 0,
                'tx_HEARTBEAT': native['heartbeat'],
                'serial_errors': native['errors'],
                'serial_max_write_ms': native['max_write_ns'] / 1e6,
                'native_sensor_replacements': native['replaced'],
                'native_log_drops': native['log_drops'],
            }
            for name, value in native_metrics.items():
                self.counters[name] = value
        hb = self.fc['heartbeat_at']
        fresh = hb is not None and now - hb <= 3
        missing = []
        if self.fc_only:
            missing.append(FC_ONLY_MISSING)
        if not fresh:
            missing.append('fresh FC heartbeat')
        if self.fc['SYS_HITL'] != 1:
            missing.append('SYS_HITL=1 confirmed')
        if self.fc['armed'] is not False:
            missing.append('armed=false confirmed')
        if self.recovery_required:
            missing.append('recovery after prior sensor run')
        if self.state == 'running':
            missing.append('model already running')
        if self.recovery not in ['idle', 'complete', 'failed']:
            missing.append('recovery in progress')
        if self.param_deploy_pending:
            latch = self.param_deploy_pending
            missing.append('parameter deploy pending: ' + str(latch.get('journal') or latch.get('error')))
        value = {
            'schema_version': 1,
            'instance_id': self.instance_id,
            'run_id': self.run_id,
            'host': self.host,
            'model': self.state,
            'build_id': self.manifest['build_id'],
            'fake_fc': bool(self.transport.fake),  # clients check this, never their own --fake-fc flag alone
            'platform': self.manifest['platform'],
            'fc_only': self.fc_only,
            'scene': self.scene,
            'fc': dict(
                self.fc,
                heartbeat_age_s=None if hb is None else now - hb,
                connection_generation=self.fc_generation,
            ),
            'recovery': self.recovery,
            'recovery_required': self.recovery_required,
            'param_deploy_pending': self.param_deploy_pending,
            'unarmed_only': self.unarmed_only,
            'ready_to_start_model': not missing,
            'missing_conditions': missing,
            # The same FC conditions without the model: FC operations (readback, logs, SD) on an FC-only release.
            'fc_ready': not [m for m in missing if m != FC_ONLY_MISSING],
            'ready_for_takeoff': self.state == 'running'
            and fresh
            and self.fc['SYS_HITL'] == 1
            and self.scene.get('height_agl_m', 0) == 0
            and not self.scene.get('carrier')
            and not self.unarmed_only
            and self.fc['identity'] == [1, 1]
            and self.fc['armed'] is False,
            'ready_after_recovery': self.recovery == 'complete'
            and fresh
            and self.fc['armed'] is False,
            'wind_reminders': list(getattr(self, 'wind_reminders', [])),
            'model_time_s': s.output.model_time_ns / 1e9 if s.produced else 0,
            'produced_steps': s.produced,
            'misses': s.misses,
            'max_lateness_ms': s.max_lateness_ns / 1e6,
            'max_step_ms': s.max_step_ns / 1e6,
            'controls_age_ms': (
                None if not self.control_valid else (self.worker.now() - self.control_at) / 1e6
            ),
            'controls_stale': self.control_valid
            and self.worker.now() - self.control_at > 100000000,
            'metrics': {'tx_HIL_SENSOR': 0, 'tx_HIL_GPS': 0, **self.counters},  # sent counts always explicit
            'last_error': self.error,
            'operations': self._operations_view(),
            'operations_summarized': True,  # the whole operation: the 'operation' request with its request_id
            'telemetry': {
                k: {
                    'age_s': now - v['received_at'],
                    'stale': now - v['received_at'] > 1,
                    'values': v['values'],
                }
                for k, v in list(self.telemetry.items())
            },
            'scheduler': {
                'qos_result': self.worker.lib.hil_worker_qos_status(),
                'time_constraint_result': self.worker.lib.hil_worker_policy_status(),
                'activity': self.activity.diagnostics(),
                'activity_run_ok': self.activity_run_ok,
                'transfer_activity': self.transfer_activity.diagnostics(),
            },
            'pilot': {'running': bool(self.pilot and self.pilot.poll() is None)},
        }
        value['runtime_identity'] = dict(self.runtime_identity)
        if hasattr(self.transport, 'link_diagnostics'):
            value['connection'] = self.transport.link_diagnostics()
        value['log_dir'] = str(self.log_dir)
        value['parameters'] = dict(self.parameters)
        value['fc_messages'] = list(self.fc_messages)
        value['recovery_details'] = dict(self.recovery_details)
        safe = lambda x: x if math.isfinite(x) else None
        value['model_truth'] = (
            {
                'gps_lla_msl': [safe(x) for x in s.output.gps_lla_msl],
                'velocity_ned_m_s': [safe(x) for x in s.output.velocity_ned_m_s],
                'ground_speed_m_s': safe(s.output.ground_speed_m_s),
                'course_deg': safe(s.output.course_deg),
                'carrier_phase': int(s.output.reserved) if self.scene.get('carrier') else 0,
            }
            if s.produced
            else None
        )
        value['diagnostics'] = diagnostics(self.worker)
        value['clock_sync'] = self.clock_sync
        value['control_gate'] = {
            'enabled': self.external_controls_enabled,
            'reason': self.control_gate_reason,
        }
        value['ack_history'] = list(self.ack_history)
        value['pilot'] = self._pilot_status()
        return value

    def read_parameters(self, names):
        if (
            not isinstance(names, list)
            or len(names) > 32
            or any(
                not isinstance(n, str)
                or not 1 <= len(n) <= 16
                or not all(c.isalnum() or c == '_' for c in n)
                for n in names
            )
        ):
            raise ValueError('Parameter names must be 1..16 alphanumeric/underscore characters')
        if self.fc['state'] != 'verified' or not self.fc['identity']:
            raise PreconditionError('No verified FC for parameter read')
        for name in names:
            self._send_message(
                mav.MAVLink_param_request_read_message(*self.fc['identity'], name.encode(), -1)
            )
        return {'requested': names}

    STATUS_TEXT = 2000  # characters of one string in an operation summary
    STATUS_OPERATION_BYTES = 8192  # one operation summary; with at most 32 finished ones the list stays bounded

    def _operations_view(self):
        """status 'operations': every operation only as a summary — its scalar fields and scalar result fields,
        strings cut to STATUS_TEXT, at most STATUS_OPERATION_BYTES each (otherwise without its result) — so the
        reply stays small however large the results (log lists, readback summaries) are. The whole operation
        comes from the 'operation' request with its request_id (ipc.request does that when it ends)."""

        def short(value):
            return value[:self.STATUS_TEXT] if isinstance(value, str) else value

        view = []
        for op in list(self.operations):
            item = {k: short(v) for k, v in op.items() if k != 'result'
                    and (v is None or isinstance(v, (str, int, float, bool)))}
            result = op.get('result')
            if isinstance(result, dict):
                item['result'] = {k: short(v) for k, v in result.items()
                                  if v is None or isinstance(v, (str, int, float, bool))}
            elif result is None or isinstance(result, (int, float, bool)):
                item['result'] = result
            if len(json.dumps(item, allow_nan=True)) > self.STATUS_OPERATION_BYTES:
                item = {k: item[k] for k in ['request_id', 'action', 'state', 'exit_code'] if k in item}
                item['error'] = short(op.get('error'))
                # Bytes as sent (JSON escapes non-ASCII characters): cut the error until the summary fits.
                limit = self.STATUS_OPERATION_BYTES
                while isinstance(item['error'], str) and len(json.dumps(item, allow_nan=True)) > limit:
                    item['error'] = item['error'][:len(item['error']) // 2]
            item['result_summarized'] = True
            view.append(item)
        return view

    def _transfer(self, function, *args, **kwargs):
        """Run a long FC transfer under the transfer App Nap guard (nested transfers share it). A guard that
        cannot be taken does not stop the transfer: it is only slower, and the result says so."""
        with self.transfer_lock:
            self.transfer_depth += 1
            held = self.transfer_activity.set_active(True)
        try:
            result = function(*args, **kwargs)
            if isinstance(result, dict):
                result = dict(result, app_nap_guard=held)
            return result
        finally:
            with self.transfer_lock:
                self.transfer_depth -= 1
                if self.transfer_depth == 0:
                    self.transfer_activity.set_active(False)

    def fetch_log(self, out, log_id=None, list_only=False):
        return self._transfer(self._fetch_log, out, log_id, list_only)

    def param_readback(self, out, param_list=None, txn=None):
        return self._transfer(self._param_readback, out, param_list, txn)

    def release_param_list(self, release):
        """The parameter list of a deploy release's firmware (params.json firmware_params); the native one by default."""
        release_dir = Path(release) if release else self.project / PARAM_RELEASE
        config = json.loads((release_dir / 'params.json').read_text())
        return release_dir / config['firmware_params']['file']

    def _fetch_log(self, out, log_id=None, list_only=False):
        """Download one FC ULog (the newest, or log_id) over the owner's own link into directory out; with
        list_only, return the FC's log list (no download, out unused). Without log_id the list must be
        complete (as many LOG_ENTRY as num_logs), so 'newest' is really the newest.

        MAVLink log protocol, as QGC's log download: LOG_REQUEST_LIST -> LOG_ENTRY, LOG_REQUEST_DATA ranges
        -> LOG_DATA (90-byte pieces), LOG_REQUEST_END. One forward pass over the file, range after range; when
        the stream stalls (LOG_STALL_S) it is asked to go on from where it stopped, never from an earlier gap
        (that would resend what already arrived); the pieces lost on the way are asked for at the end, each
        missing run on its own (runs within LOG_HOLE_JOIN pieces of each other in one range).
        Only while the model is stopped, so it never competes with the 4 ms loop.
        """
        if self.state == 'running':
            raise PreconditionError('Model running; stop the model before fetching the FC log')
        if self.fc['state'] != 'verified' or not self.fc['identity']:
            raise PreconditionError('No verified FC for log download')
        identity, generation = tuple(self.fc['identity']), self.fc_generation
        if not list_only:
            out = Path(out)
            out.mkdir(parents=True, exist_ok=True)

        def stalled(seconds):
            with self.fc_lock:
                return time.monotonic() - self.log_fetch['activity'] > seconds

        def check_link():
            # Only a reconnect aborts. While PX4 streams LOG_DATA its heartbeat can be more than 3 s late,
            # so the FC is briefly 'stale' (commands then wait in the queue until it is verified again).
            if self.fc_generation != generation:
                raise OperationError('FC connection changed during log download')

        with self.fc_lock:
            self.log_fetch = {'entries': {}, 'activity': time.monotonic(), 'id': None}
        try:
            self._send_message(mav.MAVLink_log_request_list_message(*identity, 0, 0xFFFF))
            deadline = time.monotonic() + 10
            while time.monotonic() < deadline:
                check_link()
                with self.fc_lock:
                    entries = dict(self.log_fetch['entries'])
                if entries and len(entries) >= max(e['num_logs'] for e in entries.values()):
                    break
                if entries and stalled(1.0):
                    break
                time.sleep(0.05)
            if not entries:
                raise OperationError('FC returned no log list (LOG_ENTRY)')
            complete = len(entries) >= max(e['num_logs'] for e in entries.values())
            if list_only:
                return {'entries': [entries[k] for k in sorted(entries)], 'complete': complete,
                        'newest_id': max(entries)}
            if log_id is None and not complete:
                raise OperationError(f'FC log list incomplete ({len(entries)} of {max(e["num_logs"] for e in entries.values())}); '
                                     'retry, or give the log id')
            chosen = entries.get(log_id) if log_id is not None else entries[max(entries)]
            if chosen is None:
                raise OperationError(f'FC has no log id {log_id}; ids {sorted(entries)}')
            size = int(chosen['size'])
            if size <= 0:
                raise OperationError(f'FC log {chosen["id"]} is empty')
            pieces = -(-size // LOG_DATA_BYTES)
            with self.fc_lock:
                self.log_fetch.update(id=chosen['id'], size=size, data=bytearray(size), have=bytearray(pieces),
                                      received=0, activity=time.monotonic())
            started = time.monotonic()
            deadline = started + 60 + size / 20e3  # at least 20 kB/s
            requests = 0
            asked = [0, 0]  # the current LOG_REQUEST_DATA range [offset, end) in bytes
            forward_end = 0  # the forward pass has asked for everything before this offset
            filling = False  # after the forward pass: asking for the missing runs
            progress, progress_at = -1, time.monotonic()

            def ask(offset, end):
                nonlocal requests
                self._send_message(mav.MAVLink_log_request_data_message(*identity, chosen['id'], offset, end - offset))
                asked[:] = [offset, end]
                requests += 1
                with self.fc_lock:
                    self.log_fetch['activity'] = time.monotonic()

            while True:
                check_link()
                with self.fc_lock:
                    fetch = self.log_fetch
                    received, first = fetch['received'], fetch['have'].find(0)
                    low, high = asked[0] // LOG_DATA_BYTES, -(-asked[1] // LOG_DATA_BYTES)
                    top = fetch['have'].rfind(1, low, high) if high > low else -1  # where the stream got to
                    asked_missing = fetch['have'].find(0, low, high) if high > low else -1
                    span = None
                    if first >= 0:  # the next missing run, joined with the runs close behind it
                        have, limit = fetch['have'], min(first + LOG_REQUEST_BYTES // LOG_DATA_BYTES, pieces)
                        end = have.find(1, first)
                        end = pieces if end < 0 else end
                        while end < limit:
                            near = have.find(0, end, min(end + LOG_HOLE_JOIN, pieces))
                            if near < 0:
                                break
                            end = have.find(1, near)
                            end = pieces if end < 0 else end
                        span = (first * LOG_DATA_BYTES, min(end, limit) * LOG_DATA_BYTES)
                if first < 0:
                    break
                now = time.monotonic()
                if received != progress:
                    progress, progress_at = received, now
                if now > deadline:
                    raise OperationError(f'FC log download timed out at {received}/{pieces} pieces')
                if now - progress_at > LOG_NO_PROGRESS_S:
                    raise OperationError(f'FC log download made no progress for {LOG_NO_PROGRESS_S:.0f} s '
                                         f'at {received}/{pieces} pieces')
                # Not while the FC is stale: queued requests would only pile up behind the paused writer.
                if self.fc['state'] == 'verified':
                    if filling:  # the next missing runs once the asked ones arrived; the same again after a stall
                        if asked_missing < 0 or stalled(LOG_STALL_S):
                            ask(span[0], min(span[1], size))
                    elif requests == 0 or top == high - 1:  # the stream got to the end of the asked range
                        if forward_end < size:
                            ask(forward_end, min(forward_end + LOG_REQUEST_BYTES, size))
                            forward_end = asked[1]
                        else:
                            filling = True
                            ask(span[0], min(span[1], size))
                    elif stalled(LOG_STALL_S):  # go on from where the stream stopped
                        ask((top + 1) * LOG_DATA_BYTES if top >= 0 else asked[0], asked[1])
                time.sleep(0.02)
            seconds = time.monotonic() - started
            with self.fc_lock:
                data = bytes(self.log_fetch['data'])
            if not data.startswith(ULOG_MAGIC):
                raise OperationError('Downloaded FC log is not a ULog file')
            stamp = time.strftime('%Y%m%d_%H%M%S', time.gmtime(chosen['time_utc'])) if chosen['time_utc'] else 'unknown'
            path = out / f'fc_log_{chosen["id"]}_{stamp}Z.ulg'
            path.write_bytes(data)
            return {'path': str(path), 'id': chosen['id'], 'size': size, 'sha256': hashlib.sha256(data).hexdigest(),
                    'time_utc': chosen['time_utc'], 'logs_on_fc': len(entries), 'seconds': round(seconds, 2),
                    'kB_per_s': round(size / 1e3 / max(seconds, 1e-6), 1), 'requests': requests,
                    'short_pieces_rejected': self.log_fetch.get('short_pieces', 0)}
        finally:
            try:
                self._send_message(mav.MAVLink_log_request_end_message(*identity))
            except Exception:
                pass
            with self.fc_lock:
                self.log_fetch = None

    def _param_readback(self, out, param_list=None, txn=None):
        """Read every FC parameter into directory out (read-only). PX4 lists only the used parameters
        (PARAM_REQUEST_LIST sends param_count_used() of them); every other name in the firmware's parameter
        list answers PARAM_REQUEST_READ by name. So: the list first, missing indices re-requested, then the
        remaining names one by one, in batches, with re-request rounds (PX4 drops a reply when its TX buffer is
        full, and does not answer an unknown name at all). Only while the model is stopped.

        Writes params_<UTC>.json (value, MAVLink type, raw bits, used index per parameter) and a QGC-format
        .params file. verdict PASS needs every used index, every listed name, no name outside the list, no
        type different from the list, a consistent PARAM_VALUE stream (param_consistency) and AUTOPILOT_VERSION;
        anything else is INCOMPLETE (the file is still written). Values and 'bits' come from the wire bytes.

        Inside a param_deploy transaction (txn), every send and every wait passes the transaction gate
        (_param_txn_gate: unarmed, same link and generation, ...) and the gate is checked once more at the end:
        a failed gate is ParamTxnStop, not an INCOMPLETE readback.
        """
        if self.state == 'running':
            raise PreconditionError('Model running; stop the model before reading the FC parameters')
        if self.fc['state'] != 'verified' or not self.fc['identity']:
            raise PreconditionError('No verified FC for parameter readback')
        identity, generation = tuple(self.fc['identity']), self.fc_generation
        list_path = Path(param_list) if param_list else self.project / PARAM_LIST
        raw = list_path.read_bytes()
        listing = json.loads(lzma.decompress(raw) if list_path.suffix == '.xz' else raw)
        kinds = {'Int32': 6, 'Float': 9}  # PX4 parameters.json type -> MAVLink param_type
        unknown = {p['type'] for p in listing['parameters']} - set(kinds)
        if unknown:
            raise ValueError(f'Parameter list has unknown types {sorted(unknown)}: {list_path}')
        defined = {p['name']: kinds[p['type']] for p in listing['parameters']}
        out = Path(out)
        out.mkdir(parents=True, exist_ok=True)

        def check_link():
            if txn is not None:
                self._param_txn_gate(txn)
            elif self.fc_generation != generation:
                raise OperationError('FC connection changed during parameter readback')

        def wait_quiet(done, limit):
            # Until done(), or PARAM_QUIET_S without a request or a PARAM_VALUE (at most limit seconds).
            end = time.monotonic() + limit
            while time.monotonic() < end:
                check_link()
                with self.fc_lock:
                    if done():
                        return
                    if time.monotonic() - self.param_fetch['activity'] > PARAM_QUIET_S:
                        return
                time.sleep(0.02)

        def request_rounds(pending, message):
            # Re-request whatever is still missing; stop early when a whole round gets no reply at all.
            nonlocal requests
            for _ in range(PARAM_ROUNDS):
                todo = pending()
                if not todo:
                    return
                with self.fc_lock:
                    before = self.param_fetch['received']
                for start in range(0, len(todo), PARAM_BATCH):
                    batch = set(todo[start:start + PARAM_BATCH])
                    with self.fc_lock:
                        self.param_fetch['activity'] = time.monotonic()
                    for item in todo[start:start + PARAM_BATCH]:
                        check_link()
                        self._send_message(message(item))
                        requests += 1
                    wait_quiet(lambda: not (batch & set(pending())), 2.0)
                with self.fc_lock:
                    if self.param_fetch['received'] == before:
                        return

        started = time.monotonic()
        with self.fc_lock:
            self.param_fetch = self._param_collector(started)
        requests = 0
        names = sorted(defined)
        try:
            # AUTOPILOT_VERSION (git hash, board) identifies the firmware the values belong to.
            check_link()
            self._send_message(mav.MAVLink_command_long_message(*identity, 512, 0, 148, 0, 0, 0, 0, 0, 0))
            check_link()
            self._send_message(mav.MAVLink_param_request_list_message(*identity))
            requests += 1
            end = time.monotonic() + 120
            while time.monotonic() < end:  # the list stream, until complete or quiet
                check_link()
                with self.fc_lock:
                    fetch = self.param_fetch
                    now = time.monotonic()
                    if fetch['count'] is not None and (len(fetch['by_index']) >= fetch['count']
                                                       or now - fetch['received_at'] > PARAM_QUIET_S * 2):
                        break
                    if now - fetch['received_at'] > PARAM_NO_PROGRESS_S:
                        raise OperationError(f'FC sent no PARAM_VALUE for {PARAM_NO_PROGRESS_S:.0f} s')
                time.sleep(0.05)
            with self.fc_lock:
                if self.param_fetch['count'] is None:
                    raise OperationError('FC answered the parameter list with no PARAM_VALUE')

            def missing_indices():
                with self.fc_lock:
                    fetch = self.param_fetch
                    return [i for i in range(fetch['count']) if i not in fetch['by_index']]

            def missing_names():
                with self.fc_lock:
                    values = self.param_fetch['values']
                    return [n for n in names if n not in values]

            request_rounds(missing_indices,
                           lambda i: mav.MAVLink_param_request_read_message(*identity, b'', i))
            request_rounds(missing_names,
                           lambda n: mav.MAVLink_param_request_read_message(*identity, n.encode(), -1))
            if self.param_fetch['autopilot_version'] is None:  # one more try; it does not decide the verdict
                with self.fc_lock:
                    self.param_fetch['activity'] = time.monotonic()
                check_link()
                self._send_message(mav.MAVLink_command_long_message(*identity, 512, 0, 148, 0, 0, 0, 0, 0, 0))
                wait_quiet(lambda: self.param_fetch['autopilot_version'] is not None, 1.0)
            seconds = time.monotonic() - started
            with self.fc_lock:
                fetch = self.param_fetch
                values = {k: dict(v) for k, v in fetch['values'].items()}
                count = fetch['count']
                used_missing = [i for i in range(count) if i not in fetch['by_index']]
                version = fetch['autopilot_version']
                consistency = param_consistency(fetch['log'], count)
            if txn is not None:
                check_link()  # the gate still holds after the last PARAM_VALUE
        finally:
            with self.fc_lock:
                self.param_fetch = None
        missing = [n for n in sorted(defined) if n not in values]
        extra = sorted(n for n in values if n not in defined)
        mismatch = sorted(n for n in values if n in defined and values[n]['type'] != defined[n])
        # A readback without AUTOPILOT_VERSION cannot say which firmware the values belong to.
        identity_missing = version is None
        verdict = ('PASS' if not (used_missing or missing or extra or mismatch or consistency or identity_missing)
                   else 'INCOMPLETE')
        stamp = time.strftime('%Y%m%d_%H%M%SZ', time.gmtime())
        for entry in values.values():  # JSON has no NaN/Inf: such a Float keeps its exact 'bits', value null
            if entry['type'] != 6 and not math.isfinite(entry['value']):
                entry['value'] = None
        report = {
            'schema_version': 2,
            'created_utc': stamp,
            'fc_identity': list(identity),
            'autopilot_version': version,
            'build_id': self.manifest['build_id'],
            'param_list': {'path': str(list_path), 'sha256': hashlib.sha256(raw).hexdigest(), 'count': len(defined)},
            'verdict': verdict,
            'param_count_used': count,
            'used_received': count - len(used_missing),
            'by_name_received': sum(1 for v in values.values() if v['index'] < 0),
            'used_index_missing': used_missing,
            'missing': missing,
            'not_in_list': extra,
            'type_mismatch': mismatch,
            'consistency': consistency,
            'identity_missing': identity_missing,
            'seconds': round(seconds, 2),
            'requests': requests,
            'parameters': [dict(name=k, **values[k]) for k in sorted(values)],
        }
        base = out / f'params_{stamp}'
        text = json.dumps(report, indent=1, allow_nan=False) + '\n'
        lines = [
            '# FC parameters read back by the HIL runtime (param-readback); QGC format.',
            f'# Created {stamp}; FC {identity[0]}/{identity[1]}; verdict {verdict}; see {base.name}.json.',
            '# Vehicle-Id\tComponent-Id\tName\tValue\tType',
        ]
        for name in sorted(values):
            v = values[name]
            shown = str(v['value']) if v['type'] == 6 else repr(struct.unpack('<f', bytes.fromhex(v['bits']))[0])
            lines.append(f'{identity[0]}\t{identity[1]}\t{name}\t{shown}\t{v["type"]}')
        for path, content in [(base.with_suffix('.json'), text), (base.with_suffix('.params'), '\n'.join(lines) + '\n')]:
            temp = path.with_suffix(path.suffix + '.tmp')
            temp.write_text(content)
            os.replace(temp, path)
        summary = {k: report[k] for k in report if k != 'parameters'}
        for key in ['missing', 'not_in_list', 'type_mismatch', 'used_index_missing', 'consistency']:
            summary[key + '_count'] = len(report[key])
            summary[key] = report[key][:20]
        summary.update(path=str(base.with_suffix('.json')), params_file=str(base.with_suffix('.params')),
                       sha256=hashlib.sha256(text.encode()).hexdigest(), total_received=len(values))
        return summary

    @staticmethod
    def _param_collector(now=None):
        """What _receive fills while param_fetch is set (PARAM_VALUE stream and AUTOPILOT_VERSION)."""
        now = time.monotonic() if now is None else now
        return {'values': {}, 'by_index': {}, 'count': None, 'activity': now, 'received_at': now, 'received': 0,
                'autopilot_version': None, 'log': []}

    def _check_params_module(self):
        """deploy/check_params.py, loaded from the project (the same judgement the user runs offline)."""
        import importlib.util

        spec = importlib.util.spec_from_file_location('check_params', self.project / 'deploy/check_params.py')
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        return module

    def _param_deploy_preconditions(self, what='Parameter deploy'):
        reasons = []
        if self.state == 'running':
            reasons.append('model stopped')
        if self.recovery_required:
            reasons.append('no PX4 recovery pending')
        if self.recovery not in ['idle', 'complete', 'failed']:
            reasons.append('no recovery in progress')
        if self.fc['state'] != 'verified' or not self.fc['identity']:
            reasons.append('a verified FC')
        if self.fc['armed'] is not False:
            reasons.append('FC confirmed unarmed')
        if self.fc['SYS_HITL'] != 1:
            reasons.append('SYS_HITL=1 confirmed')
        if self.pilot and self.pilot.poll() is None:
            reasons.append('no pilot running')
        if reasons:
            raise PreconditionError(what + ' needs: ' + '; '.join(reasons))

    def _param_txn_dir(self, out, action):
        """A new directory per plan / deploy, so an earlier transaction's files are never overwritten."""
        path = Path(out) / f'{action}_{time.strftime("%Y%m%d_%H%M%SZ", time.gmtime())}_{uuid.uuid4().hex[:8]}'
        path.mkdir(parents=True)
        return path

    def param_plan(self, out, release=None):
        """Read-only preview of param_deploy: a full readback, deploy/check_params.py, and the writes it would
        make (only the release's 'set' values that differ). Writes <out>/plan_<UTC>_<id>/ (before/, plan.json)."""
        with self.operation_lock:
            plan, path = self._param_plan(self._param_txn_dir(out, 'plan'), release)
            summary = {k: plan[k] for k in plan if k != 'unmanaged_differences'}
            return dict(summary, path=str(path))

    def _param_plan(self, out, release, txn=None):
        """txn (param_deploy): the readback passes the transaction gate (unarmed, same link and generation, ...)."""
        self._param_deploy_preconditions()
        release_dir = Path(release) if release else self.project / PARAM_RELEASE
        checker = self._check_params_module()
        config = json.loads((release_dir / 'params.json').read_text())
        firmware = checker.load_firmware(release_dir / config['firmware_params']['file'])
        generation = self.fc_generation
        before = self.param_readback(out / 'before', self.release_param_list(release_dir), txn=txn)
        report = checker.check(release_dir, before['path'])
        values = {p['name']: p for p in json.loads(Path(before['path']).read_text())['parameters']}
        blocking, writes = [], []
        if self.fc_generation != generation:
            blocking.append({'check': 'FC connection changed during the plan readback'})
        for key in ['local_mismatch', 'identity', 'structure', 'not_read', 'not_in_firmware', 'unknown_name',
                    'fc_owned_set_changed']:
            if report[key]:
                blocking.append({'check': key, 'items': report[key][:10]})
        for item in report['wrong_value']:
            name = item['name']
            if item['section'] != 'set':  # airframe / boot / verify_only values are checked, never written
                blocking.append({'check': 'differs in ' + item['section'] + ' (not written by deploy)', 'items': [item]})
                continue
            if name == 'SYS_HITL' and item['expected'] != 1:
                blocking.append({'check': 'SYS_HITL must stay 1 in a HITL recipe', 'items': [item]})
                continue
            kind = values[name]['type']
            writes.append({'name': name, 'type': kind, 'old': values[name]['value'], 'old_bits': values[name]['bits'],
                           'new': item['expected'], 'new_bits': encode_param(kind, item['expected']).hex(),
                           'reboot': bool(firmware[name].get('rebootRequired', False))})
        plan = {
            'schema_version': 2,
            'transaction': out.name,
            'created_utc': time.strftime('%Y%m%d_%H%M%SZ', time.gmtime()),
            'release': str(release_dir),
            'params_sha256': hashlib.sha256((release_dir / 'params.json').read_bytes()).hexdigest(),
            'fc_identity': before['fc_identity'],
            'fc_generation': generation,
            'autopilot_version': before['autopilot_version'],
            'runtime_source_id': self.runtime_identity.get('source_id'),
            'before': before['path'],
            'before_sha256': before['sha256'],
            'check_verdict': report['verdict'],
            'writes': writes,
            'unmanaged_count': len(report['not_default']),  # reported only: deploy never writes them
            'unmanaged_differences': report['not_default'],
            'blocking': blocking,
        }
        path = out / 'plan.json'
        temp = path.with_suffix('.tmp')
        temp.write_text(json.dumps(plan, indent=1, allow_nan=False) + '\n')
        os.replace(temp, path)
        return plan, path

    def _param_plan_ready(self, plan, path):
        """The plan is final; the writes start right after this. Nothing to do: consent is given in the
        conversation (user 2026-10-17). Tests use this point to change the FC between the plan and the writes."""

    def _fc_version(self, identity):
        """A fresh AUTOPILOT_VERSION (board, git prefix, uid) of the FC, or None."""
        for _ in range(PARAM_SET_TRIES):
            with self.fc_lock:
                self.param_fetch = self._param_collector()
            try:
                self._send_message(mav.MAVLink_command_long_message(*identity, 512, 0, 148, 0, 0, 0, 0, 0, 0))
                end = time.monotonic() + PARAM_SET_WAIT_S
                while time.monotonic() < end:
                    with self.fc_lock:
                        version = self.param_fetch['autopilot_version']
                    if version:
                        return version
                    time.sleep(0.02)
            finally:
                with self.fc_lock:
                    self.param_fetch = None
        return None

    def _param_txn_gate(self, txn, final=False):
        """Before every PARAM_SET / PARAM_REQUEST_READ of a transaction: the same FC link and connection
        generation, a fresh heartbeat, unarmed, SYS_HITL=1, the model stopped, no pilot. Otherwise ParamTxnStop:
        the transaction stops and nothing more is written, not even a rollback. final=True is the check of the
        final decision, under fc_lock right after a normal gate: the same conditions, a heartbeat age too, but
        no waiting (nothing is received while fc_lock is held, so a stale heartbeat stops at once). The SD file
        transactions (sd_files.py) use the same gate; txn['what'] names the transaction in the message."""
        what = txn.get('what', 'Parameter deploy')
        end = time.monotonic() + 5  # a heartbeat can be a few seconds late while PX4 streams parameters
        while True:
            fc, reasons = self.fc, []
            if self.state == 'running':
                reasons.append('model stopped')
            if self.recovery_required or self.recovery not in ['idle', 'complete', 'failed']:
                reasons.append('no PX4 recovery pending or in progress')
            if fc['state'] != 'verified' or tuple(fc['identity'] or ()) != txn['identity']:
                reasons.append('the same verified FC link')
            if self.fc_generation != txn['generation']:
                reasons.append('the same connection generation')
            if fc['armed'] is not False:
                reasons.append('FC unarmed')
            if fc['SYS_HITL'] != 1:
                reasons.append('SYS_HITL=1')
            if self.pilot and self.pilot.poll() is None:
                reasons.append('no pilot running')
            stale = fc['heartbeat_at'] is None or time.monotonic() - fc['heartbeat_at'] > 3
            if not reasons and not stale:
                return
            if reasons or final or time.monotonic() > end:
                raise ParamTxnStop(what + ' stopped: needs ' + '; '.join(reasons or ['a fresh FC heartbeat']))
            time.sleep(0.05)

    def _param_txn_identify(self, txn):
        """The FC is the planned one (uid, board, git), asked on one unchanged connection generation. A new
        generation is accepted only after the transaction's own reboot (txn['rebooting']); any other
        reconnection, also one during the query itself, stops the transaction."""
        what = txn.get('what', 'Parameter deploy')
        generation = self.fc_generation
        if generation != txn['generation'] and not txn.get('rebooting'):
            raise ParamTxnStop(what + ' stopped: the FC reconnected without this transaction rebooting it')
        identity = tuple(self.fc['identity'] or ())
        version = self._fc_version(identity) if identity else None
        if self.fc_generation != generation:
            raise ParamTxnStop(what + ' stopped: the FC reconnected while it was being identified')
        planned = txn['version']
        if not version or identity != txn['identity'] or any(
                version.get(k) != planned.get(k) for k in ['uid', 'board_version', 'git_hash_prefix', 'os_hash_prefix']):
            raise ParamTxnStop(f'{what} stopped: not the planned FC (planned {planned}, now {version})')
        txn['generation'] = generation

    def _param_set(self, txn, name, kind, bits, answers=None):
        """PARAM_SET with the exact value bytes; returns when the FC answers PARAM_VALUE with the same name,
        type and bytes. A PARAM_VALUE with other bytes (e.g. an older answer) is not an acknowledgement; every
        answer's bytes are appended to answers (the FC may have stored another value because of this write). The
        answer only says the value is in RAM: persistence is proven by the reboot and readback afterwards."""
        identity = txn['identity']
        message = mav.MAVLink_param_set_message(*identity, name.encode(), 0.0, kind)
        seen = None
        for _ in range(PARAM_SET_TRIES):
            self._param_txn_gate(txn)
            with self.fc_lock:
                self.param_fetch = self._param_collector()
            try:
                with self.writer_lock:
                    raw = message.pack(self.writer)
                    self.writer.seq = (self.writer.seq + 1) % 256
                with self.route_lock:
                    self.commands.put_nowait(with_bits(raw, bits, message.crc_extra))
                end = time.monotonic() + PARAM_SET_WAIT_S
                while time.monotonic() < end:
                    with self.fc_lock:
                        got = self.param_fetch['values'].get(name)
                    if got:
                        if got is not seen and answers is not None:
                            answers.append(got['bits'])
                        seen = got
                        if got['type'] == kind and got['bits'] == bits.hex():
                            return
                    time.sleep(0.02)
            finally:
                with self.fc_lock:
                    self.param_fetch = None
        raise OperationError(f'{name}: no matching PARAM_VALUE after {PARAM_SET_TRIES} PARAM_SET (sent {bits.hex()}, '
                             f'last answer {seen and (seen["type"], seen["bits"])})')

    def _param_read(self, txn, name):
        """(type, bits hex) of one parameter, read by name."""
        for _ in range(PARAM_SET_TRIES):
            self._param_txn_gate(txn)
            with self.fc_lock:
                self.param_fetch = self._param_collector()
            try:
                self._send_message(mav.MAVLink_param_request_read_message(*txn['identity'], name.encode(), -1))
                end = time.monotonic() + PARAM_SET_WAIT_S
                while time.monotonic() < end:
                    with self.fc_lock:
                        got = self.param_fetch['values'].get(name)
                    if got:
                        return got['type'], got['bits']
                    time.sleep(0.02)
            finally:
                with self.fc_lock:
                    self.param_fetch = None
        raise OperationError(f'{name}: no PARAM_VALUE for PARAM_REQUEST_READ')

    def _param_save_and_reboot(self, txn):
        """PX4 saves a change at least 300 ms later and at most every 2 s, and a reboot drops what is not saved
        yet: wait PARAM_SAVE_WAIT_S after the last write, then reboot (recover) and accept the new connection
        generation only for the planned FC. The readback after it is the evidence that the values were saved."""
        end = time.monotonic() + PARAM_SAVE_WAIT_S
        while time.monotonic() < end:
            self._param_txn_gate(txn)
            time.sleep(0.1)
        txn['rebooting'] = True  # only this reboot may change the connection generation
        self.recover()
        self._param_txn_identify(txn)
        txn['rebooting'] = False

    def _param_watch_conflicts(self, plan, before):
        """Values announced during the transaction that nobody planned: another writer (e.g. QGC), even when it
        changed a value back again before the final readback (A -> B -> A)."""
        planned = {w['name']: {w['old_bits'], w['new_bits']} for w in plan['writes']}
        with self.fc_lock:
            seen = list(self.param_watch or [])
        conflicts = set()
        for name, bits in seen:
            allowed = planned.get(name, {before.get(name, {}).get('bits')})
            if bits not in allowed:
                conflicts.add(name)
        return sorted(conflicts)

    def _param_commit(self, txn, plan, before):
        """The end of a deploy that can be PASS (a no-op or a verified write): the final decision and clearing the
        pending latch as one commit. What may be slow comes first — the latch file's new content is written to a
        temporary file, and the normal gate may wait for a heartbeat. Then under fc_lock (no heartbeat or
        PARAM_VALUE is handled meanwhile): the gate without waiting (a heartbeat under 3 s old), nothing announced
        as changed, and the rename that clears the latch file. The heartbeat that decided must still be under 3 s
        old when the rename has finished; otherwise the latch is set back as it was and the deploy stops. A file
        that cannot be written stops too (ParamTxnStop); an announced change is an OperationError."""
        previous = self.param_deploy_pending
        temp = self.param_latch_file.with_suffix('.tmp')
        if previous is not None:
            try:
                temp.write_text(json.dumps({'pending': False}, indent=2) + '\n')
            except OSError as exc:
                raise ParamTxnStop(f'Parameter deploy stopped: the pending latch could not be written ({exc})')
        self._param_txn_gate(txn)
        with self.fc_lock:
            self._param_txn_gate(txn, final=True)
            watched = self._param_watch_conflicts(plan, before)
            if watched:
                raise OperationError(f'verification: {watched[:10]} changed during the deploy (another writer '
                                     'such as QGC, or PX4 itself)')
            if previous is None:
                return
            try:
                os.replace(temp, self.param_latch_file)
            except OSError as exc:
                raise ParamTxnStop(f'Parameter deploy stopped: the pending latch could not be cleared ({exc})')
            if time.monotonic() - self.fc['heartbeat_at'] > 3:
                self._param_pending(previous)  # the decision is older than the heartbeat limit: set it back
                raise ParamTxnStop('Parameter deploy stopped: clearing the pending latch outlasted the FC heartbeat '
                                   'it was decided on (over 3 s); it stays pending')
            self.param_deploy_pending = None

    def _param_deploy_verify(self, plan, release, after, which, writes):
        """(problems, others) of the readback after writing (which='new') or restoring (which='old') the given
        writes: problems = a value not as intended, an unclean check (identity, structure), another FC; others =
        parameters that changed since the plan's readback without being planned (another writer, or PX4 itself)."""
        checker = self._check_params_module()
        report = checker.check(Path(release), after['path'])
        problems = []
        for key in ['local_mismatch', 'identity', 'structure', 'not_read', 'not_in_firmware', 'unknown_name',
                    'fc_owned_set_changed']:
            if report[key]:
                problems.append(f'{key}: {report[key][:5]}')
        if (after.get('autopilot_version') or {}).get('uid') != (plan.get('autopilot_version') or {}).get('uid'):
            problems.append('readback is from another FC (uid)')
        before = {p['name']: p for p in json.loads(Path(plan['before']).read_text())['parameters']}
        now = {p['name']: p for p in json.loads(Path(after['path']).read_text())['parameters']}
        planned = {w['name']: w for w in writes}
        others = []
        for name in sorted(set(before) | set(now)):
            if name in planned:
                want = planned[name][which + '_bits']
                if now.get(name, {}).get('bits') != want:
                    problems.append(f'{name}: {now.get(name, {}).get("bits")} on the FC, {want} intended')
            elif before.get(name, {}).get('bits') != now.get(name, {}).get('bits'):
                others.append(name)
        if which == 'new' and report['wrong_value']:
            problems.append(f'wrong_value: {report["wrong_value"][:5]}')
        return problems, others

    def _param_pending(self, value):
        """The deploy latch (state dir param_deploy.json): while pending, the model does not start
        (status missing_conditions); only a param-deploy that ends PASS clears it. Fail closed: pending holds in
        memory before the file is written; a clear holds in memory only after the file was replaced (an I/O error
        raises and leaves the latch pending)."""
        if value is not None:
            self.param_deploy_pending = value
        temp = self.param_latch_file.with_suffix('.tmp')
        temp.write_text(json.dumps(value or {'pending': False}, indent=2) + '\n')
        os.replace(temp, self.param_latch_file)
        self.param_deploy_pending = value

    def _load_param_latch(self):
        """The latch at owner start. Only {"pending": false, ...} means clear; a file that cannot be read or is
        not {"pending": true/false, ...} counts as pending (fail closed), with the reason in 'error'."""
        if not self.param_latch_file.exists():
            return None
        try:
            latch = json.loads(self.param_latch_file.read_text())
        except (OSError, ValueError) as exc:
            return {'pending': True, 'state': 'unreadable',
                    'error': f'{self.param_latch_file} cannot be read ({exc}); inspect it, do not delete it'}
        if not isinstance(latch, dict) or not isinstance(latch.get('pending'), bool):
            return {'pending': True, 'state': 'invalid',
                    'error': f'{self.param_latch_file} is not {{"pending": true/false, ...}}; inspect it, do not delete it'}
        return latch if latch['pending'] else None

    def param_deploy(self, out, release=None):
        """Write the release's 'set' values that differ on the FC. In a new <out>/deploy_<UTC>_<id>/ directory: plan
        (readback + check) -> the planned FC again (uid, board, git) -> for each value: gate (same FC
        link and generation, fresh heartbeat, unarmed, SYS_HITL=1), the current value must still be the planned
        old one, PARAM_SET with the exact bytes until the identical PARAM_VALUE -> wait for PX4's save, reboot
        (recover), the planned FC again -> readback + check: PASS only when every planned value is there and
        nothing else changed or was announced changed (every PARAM_VALUE from before the plan's readback to the
        last readback is watched). A failure restores this deploy's own values (only the exact value it wrote; any
        other value is left alone and reported) and reports FAIL. A failed gate, an unexpected reconnection,
        another FC, or a rollback that cannot finish stops at once. Every outcome other than PASS leaves the
        deploy pending (also FAIL after a complete rollback): the model will not start until a param-deploy ends
        PASS. journal.json is rewritten after every step."""
        with self.operation_lock:
            with self.fc_lock:
                self.param_watch = []  # from before the plan's readback to the end
            try:
                return self._transfer(self._param_deploy, out, release)
            finally:
                with self.fc_lock:
                    self.param_watch = None

    def _param_deploy(self, out, release):
        out = self._param_txn_dir(out, 'deploy')
        release_dir = Path(release) if release else self.project / PARAM_RELEASE
        self._param_deploy_preconditions()
        # The transaction (link, generation) starts before the plan's readback, which passes its gate too.
        txn = {'identity': tuple(self.fc['identity']), 'generation': self.fc_generation, 'version': {}}
        plan, plan_path = self._param_plan(out, release_dir, txn)
        txn['version'] = plan['autopilot_version'] or {}
        plan_sha = hashlib.sha256(plan_path.read_bytes()).hexdigest()
        result = {'transaction': str(out), 'plan': str(plan_path), 'writes': plan['writes'],
                  'unmanaged_count': plan['unmanaged_count']}
        journal = {'schema_version': 2, 'transaction': out.name, 'plan': str(plan_path), 'plan_sha256': plan_sha,
                   'state': 'planned', 'entries': [dict(w, status='pending') for w in plan['writes']]}
        journal_path = out / 'journal.json'

        def save():
            temp = journal_path.with_suffix('.tmp')
            temp.write_text(json.dumps(journal, indent=1, allow_nan=False) + '\n')
            os.replace(temp, journal_path)

        save()
        if plan['blocking']:
            journal['state'] = 'refused'
            save()
            raise PreconditionError('Parameter deploy refused: ' + json.dumps(plan['blocking'])[:800]
                                    + f' (plan {plan_path})')
        before = {p['name']: p for p in json.loads(Path(plan['before']).read_text())['parameters']}
        if not plan['writes']:
            # Nothing to write, but a PASS clears the pending latch: the same checks as a write's end — the same FC
            # on the same link, then the commit (_param_commit: the gate and nothing announced as changed since
            # before the plan's readback, decided after every wait and still fresh when the latch is cleared).
            try:
                self._param_txn_gate(txn)
                self._param_txn_identify(txn)
                try:
                    journal['state'] = 'no_op'  # nothing sent: no PARAM_SET, no reboot
                    save()
                except OSError as exc:
                    raise OperationError(f'the journal could not be written ({exc})')
                self._param_commit(txn, plan, before)
            except OperationError as exc:
                journal.update(state='no_op_refused', error=str(exc))
                save()
                raise OperationError(f'Parameter deploy found nothing to write but cannot end PASS ({exc}); '
                                     f'nothing was written, a pending deploy stays pending: see {journal_path}')
            return dict(result, verdict='PASS', written=[], journal=str(journal_path),
                        note='the FC already has every set value: no PARAM_SET, no reboot')
        self._param_plan_ready(plan, plan_path)
        self._param_deploy_preconditions()  # again, right before the first write
        entries = journal['entries']
        try:
            if self.fc_generation != txn['generation']:
                raise ParamTxnStop('Parameter deploy stopped: the FC reconnected after the plan')
            self._param_txn_identify(txn)
        except ParamTxnStop:
            journal['state'] = 'stopped_before_writing'
            save()
            raise
        self._param_pending({'pending': True, 'journal': str(journal_path), 'state': 'writing',
                             'transaction': out.name})
        journal['state'] = 'writing'
        save()
        try:
            for entry in entries:
                current = self._param_read(txn, entry['name'])
                if current != (entry['type'], entry['old_bits']):
                    raise OperationError(f"{entry['name']} changed since the plan ({current[1]}, planned "
                                         f"{entry['old_bits']}): another writer?")
                entry['status'] = 'attempted'  # the FC may apply it even if the answer is lost
                entry['answers'] = []
                save()
                self._param_set(txn, entry['name'], entry['type'], bytes.fromhex(entry['new_bits']),
                                entry['answers'])
                entry['status'] = 'written'
                save()
            journal['state'] = 'saving_and_rebooting'
            save()
            self._param_save_and_reboot(txn)
            journal['state'] = 'verifying'
            save()
            after = self.param_readback(out / 'after', self.release_param_list(release_dir), txn=txn)
            problems, others = self._param_deploy_verify(plan, release_dir, after, 'new', entries)
            watched = self._param_watch_conflicts(plan, before)
            if others or watched:
                problems.append(f'{sorted(set(others) | set(watched))[:10]} changed during the deploy '
                                '(another writer such as QGC, or PX4 itself)')
            if problems:
                raise OperationError('verification: ' + '; '.join(problems[:10]))
            try:
                journal['state'] = 'complete'
                save()
            except OSError as exc:  # written and verified, but not recorded: stop, keep it pending, no rollback
                raise ParamTxnStop(f'Parameter deploy stopped: the journal could not be written ({exc}); the values '
                                   'were written and verified')
            self._param_commit(txn, plan, before)  # the final decision and the latch clear, as for a no-op
            return dict(result, verdict='PASS', written=[e['name'] for e in entries], after=after['path'],
                        journal=str(journal_path))
        except ParamTxnStop as stop:
            journal.update(state='stopped', error=str(stop))
            save()
            self._param_pending({'pending': True, 'journal': str(journal_path), 'state': 'stopped',
                                 'transaction': out.name, 'error': str(stop)})
            raise OperationError(f'{stop}; nothing more was written and no rollback was tried. The deploy is '
                                 f'pending (model start refused) until a param-deploy ends PASS: see {journal_path}')
        except Exception as exc:
            journal.update(state='rolling_back', error=str(exc))
            save()
            try:
                restored, others = self._param_rollback(txn, plan, release_dir, out, entries, save)
            except ParamTxnStop as stop:
                journal.update(state='stopped', rollback_error=str(stop))
                save()
                self._param_pending({'pending': True, 'journal': str(journal_path), 'state': 'stopped',
                                     'transaction': out.name, 'error': str(stop)})
                raise OperationError(f'Parameter deploy failed ({exc}) and then stopped ({stop}); no rollback '
                                     f'was tried after that. The deploy is pending (model start refused): '
                                     f'see {journal_path}')
            except Exception as rollback_exc:
                journal.update(state='rollback_failed', rollback_error=str(rollback_exc))
                save()
                self._param_pending({'pending': True, 'journal': str(journal_path), 'state': 'rollback_failed',
                                     'transaction': out.name, 'error': str(rollback_exc)})
                raise OperationError(f'Parameter deploy failed ({exc}) and the rollback did not finish '
                                     f'({rollback_exc}); FC parameters may be partly written. The deploy is '
                                     f'pending (model start refused): see {journal_path}')
            journal.update(state='rolled_back', others_changed=others)
            save()
            # FAIL keeps the deploy pending even after a complete rollback (user 2026-10-16): a person looks
            # at the journal; a later param-deploy that ends PASS (a no-op when the FC matches) clears it.
            self._param_pending({'pending': True, 'journal': str(journal_path), 'state': 'rolled_back',
                                 'transaction': out.name, 'error': str(exc)})
            return dict(result, verdict='FAIL', error=str(exc), restored=restored, others_changed=others,
                        journal=str(journal_path))

    def _param_rollback(self, txn, plan, release_dir, out, entries, save):
        """Restore this deploy's own values: for every attempted / written entry the current value decides —
        the planned old one (never applied, or already restored): nothing; exactly the value this deploy wrote:
        write the old one back; anything else, also a value seen in 'answers' (a PARAM_VALUE carries no
        transaction id, so it may be someone else's): left alone, and the rollback does not finish.
        The connection generation may differ only after this transaction's own reboot (recover); any other
        reconnection is ParamTxnStop."""
        if txn['generation'] != self.fc_generation:
            if not txn.get('rebooting'):
                raise ParamTxnStop('Parameter deploy stopped: the FC reconnected without this deploy rebooting it')
            self._param_txn_identify(txn)  # the failure came during the transaction's own reboot
            txn['rebooting'] = False
        touched = [e for e in entries if e['status'] in ['attempted', 'written']]
        foreign = []
        for entry in touched:
            current = self._param_read(txn, entry['name'])
            if current == (entry['type'], entry['old_bits']):
                entry['rollback'] = 'unchanged'
            elif current == (entry['type'], entry['new_bits']):
                self._param_set(txn, entry['name'], entry['type'], bytes.fromhex(entry['old_bits']))
                entry['rollback'] = 'written'
            else:
                entry['rollback'] = 'foreign_value'
                foreign.append(entry['name'])
            save()
        if foreign:
            raise OperationError(f'{foreign} hold a value neither planned nor written by this deploy: left alone')
        if any(e.get('rollback') == 'written' for e in touched):
            self._param_save_and_reboot(txn)
        restored = self.param_readback(out / 'rollback', self.release_param_list(release_dir), txn=txn)
        problems, others = self._param_deploy_verify(plan, release_dir, restored, 'old', touched)
        if problems:
            raise OperationError('rollback verification: ' + '; '.join(problems[:10]))
        return [e['name'] for e in touched if e.get('rollback') == 'written'], others

    def _pilot_status(self):
        session = dict(self.pilot_session or {})
        alive = bool(self.pilot and self.pilot.poll() is None)
        try:
            data = json.loads((self.pilot_dir / 'status.json').read_text())
            if data.get('session_id') == session.get('session_id') and session:
                session.update(data)
        except (OSError, ValueError):
            pass
        session['running'] = alive
        session['exit_code'] = None if self.pilot is None else self.pilot.poll()
        if (
            not alive
            and self.pilot_session
            and self.pilot_session['phase'] in ['starting', 'running']
        ):
            self.pilot_session['phase'] = 'ended' if session['exit_code'] == 0 else 'failed'
            self.pilot_session['exit_code'] = session['exit_code']
            session['phase'] = self.pilot_session['phase']
            if session['exit_code'] and self.error is None:
                self.error = (
                    'Pilot exited '
                    + str(session['exit_code'])
                    + ': '
                    + str(session.get('reason', 'inspect pilot console.log'))
                )
        return session

    def _remember_recovery(self, pending):
        if self.transport.fake:
            return
        value = {
            'pending': bool(pending),
            'instance_id': self.instance_id,
            'run_id': self.run_id,
            'fc_identity': self.fc['identity'],
            'build_id': self.manifest['build_id'],
            'reason': (
                'sensor run or incomplete recovery requires explicit recovery'
                if pending
                else 'recovery confirmed by fresh heartbeat'
            ),
        }
        temp = self.recovery_file.with_suffix('.tmp')
        temp.write_text(json.dumps(value, indent=2))
        os.replace(temp, self.recovery_file)

    def start(self, scene=None):
        with self.operation_lock:
            if scene is not None:
                new, initial = load_scene(scene)
            else:
                new, initial = self.scene, self.initial
            status = self.status()
            generation = self.fc_generation
            if not status['ready_to_start_model']:
                raise PreconditionError('; '.join(status['missing_conditions']))
            if not self.transport.fake and (
                new.get('height_agl_m', 0) != 0 or any(initial.body_velocity_m_s)
            ):
                raise ValueError('Air-start currently supports offline validation only')
            flat_rangefinder = new.get('flat_rangefinder', False)
            if flat_rangefinder and (new['aircraft_id'] != 'skysurfer' or not self.model.info.capabilities & 8 or self.model.info.raw_count != 58):
                raise PreconditionError('Flat rangefinder requires the SkySurfer truth layout')
            if flat_rangefinder and not hasattr(self.worker.lib, 'hil_io_flat_rangefinder'):
                raise PreconditionError('Update model release for flat rangefinder support')
            wind, reminders = load_wind(new)
            self.scene, self.initial = new, initial
            self.model.check(self.model.lib.hil_model_reset(C.byref(initial)))
            if 'carrier' in new:
                if any(wind['steady_ned_m_s']) or any(wind['gust_magnitude_m_s']) or wind['shear_speed_6m_m_s'] or wind['turbulence_speed_6m_m_s']:
                    raise ValueError('Carrier phase one requires no wind')
            elif wind is not None and (self.model.info.capabilities & CAP_WIND):
                self.model.configure_wind(wind)
            elif wind is not None and (any(wind['steady_ned_m_s']) or any(wind['gust_magnitude_m_s'])
                                       or wind['shear_speed_6m_m_s'] or wind['turbulence_speed_6m_m_s']):
                raise ValueError('This model release has no wind support')
            if 'carrier' in new:
                self.model.configure_carrier(new['carrier'])
            self.wind_reminders = reminders
            self.controls = [0.0] * 16
            self.control_valid = self.pending_control = False
            self.control_sequence = 0
            self.last_control_frame = None
            self.last_output = 0
            self.encoder = Encoder(self.profile['gps_quality'], flat_rangefinder=flat_rangefinder)
            self.live_sent = False
            self.error = None
            self.run_id = str(uuid.uuid4())
            self.model_epoch_us = max(time.time_ns() // 1000, self.last_wire_us + 1)
            (self.log_dir / (self.run_id + '.json')).write_text(
                json.dumps(
                    {
                        'scene': new,
                        'wind_reminders': reminders,
                        'build_id': self.manifest['build_id'],
                        'run_id': self.run_id,
                        'model_epoch_us': self.model_epoch_us,
                    },
                    indent=2,
                )
            )
            if self.native_io:
                self.worker.lib.hil_io_config(self.model_epoch_us, 3 | (10 << 8))
                if hasattr(self.worker.lib, 'hil_io_flat_rangefinder'):
                    self.worker.lib.hil_io_flat_rangefinder(int(flat_rangefinder))
            if self.fc_generation != generation or self.fc['state'] != 'verified':
                raise PreconditionError('FC connection changed during model initialization')
            self._block_external('model start')
            self._remember_recovery(True)
            # Before the first 4 ms wait, not when the GUI's poll later sees the model running: a backgrounded
            # owner may be timer-coalesced (App Nap) and the worker then wakes late (fault 5). If the guard cannot
            # be taken the model still starts (no new blocking condition), but the run is marked: flight-summary
            # will not pass it (status.scheduler.activity_run_ok).
            self.activity_run_ok = self.activity.set_active(True)
            if not self.activity_run_ok:
                self.counters['activity_failures'] += 1
            try:
                self.worker.start()
            except BaseException:
                self.activity.set_active(False)
                raise
            self.state = 'running'
            self._allow_external()
            if self.fc_generation != generation:
                self._fault('FC connection changed during model start')
                raise OperationError(self.error)

    def stop_pilot(self, restore_controls=True):
        with self.operation_lock:
            self._block_external('stopping pilot')
            self.pilot_stopping = True
            (self.pilot_dir / 'stop').touch()
            forced = False
            try:
                if self.pilot and self.pilot.poll() is None:
                    try:
                        self.pilot.wait(timeout=6)
                    except subprocess.TimeoutExpired:
                        forced = True
                        self.pilot.terminate()
                        try:
                            self.pilot.wait(timeout=1)
                        except subprocess.TimeoutExpired:
                            self.pilot.kill()
                            self.pilot.wait(timeout=1)
                if forced:
                    raise OperationError(
                        'Pilot stop timed out; process terminated, model stopped, recovery required'
                    )
            except Exception as exc:
                self.error = str(exc)
                self.recovery_required = True
                self._stop_model('fault')
                if self.pilot_session:
                    self.pilot_session.update(phase='failed', error=str(exc))
                raise
            finally:
                self.pilot_stopping = False
                self._block_external('pilot stopped')
                if self.pilot_log and (not self.pilot or self.pilot.poll() is not None):
                    self.pilot_log.close()
                    self.pilot_log = None
            if self.pilot_session:
                self.pilot_session.update(
                    phase='ended', exit_code=self.pilot.poll() if self.pilot else None
                )
            if restore_controls:
                self._allow_external()

    def _stop_model(self, state='stopped'):
        final_snapshot = self.worker.snapshot()
        self.last_wire_us = max(
            self.last_wire_us, self.model_epoch_us + final_snapshot.output.model_time_ns // 1000
        )
        with self.output_lock:
            self.state = state
            self.worker.stop()
            self.activity.set_active(False)
            self.recovery_required = (
                self.recovery_required
                or self.live_sent
                or (self.native_io and io_stats(self.worker)['sensor'] > 0)
            )

    def flush_logs(self, timeout=2.0):
        """Wait until the log thread has written and flushed everything produced so far (model-stop's barrier,
        so wire.bin and timing.csv are complete on disk for flight-summary). False after a timeout (counted)."""
        self.flush_done.clear()
        self.flush_request.set()
        if self.flush_done.wait(timeout):
            return True
        self.counters['log_flush_timeouts'] += 1
        return False

    def stop(self):
        with self.operation_lock:
            self._block_external('model stop')
            (self.pilot_dir / 'stop').touch()
            self._stop_model()
            try:
                self.stop_pilot(restore_controls=False)
            finally:
                self._stop_model('fault' if self.state == 'fault' else 'stopped')
                self.flush_logs()

    def reset(self, scene=None):
        with self.operation_lock:
            if self.state == 'running':
                raise PreconditionError('Stop the model before reset')
            if self.recovery_required:
                raise PreconditionError('PX4 recovery required before next live reset/start')
            if scene is not None:
                self.scene, self.initial = load_scene(scene)
            if not self.fc_only:  # FC-only: recover still ends here; only the host state is reset
                self.model.check(self.model.lib.hil_model_reset(C.byref(self.initial)))
            self.state = 'initialized'
            self.run_id = str(uuid.uuid4())
            self.controls = [0.0] * 16
            self.control_valid = self.pending_control = False
            self.control_sequence = 0
            self.last_control_frame = None
            self.telemetry.clear()

    def _recover_command(self, identity, command, *params):
        with self.fc_lock:
            self.recover_context = {
                'identity': identity,
                'generation': self.fc_generation,
                'command': command,
                'sent_at': time.monotonic(),
                'ack_result': None,
                'rejected': None,
            }
            self.recover_command_at = self.recover_context['sent_at']
            self.expected_recover_command = command
        self._send_message(mav.MAVLink_command_long_message(*identity, command, 0, *params))

    def _wait(self, predicate, seconds, reason):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            with self.fc_lock:
                context = self.recover_context
                if context and context['rejected'] is not None:
                    raise OperationError(
                        'FC command '
                        + str(context['command'])
                        + ' rejected: '
                        + str(context['rejected'])
                    )
                if (
                    context
                    and context['command'] == 400
                    and context['generation'] != self.fc_generation
                ):
                    raise OperationError('FC connection changed while disarming')
                if (
                    not context or context['ack_result'] == mav.MAV_RESULT_ACCEPTED
                ) and predicate():
                    return
            time.sleep(0.02)
        raise OperationError(reason)

    def recover(self, allow_hil_force_disarm=False):
        # Its ACK, heartbeat and reconnect waits run with the model stopped: the transfer App Nap guard too.
        return self._transfer(self._recover, allow_hil_force_disarm)

    def _recover(self, allow_hil_force_disarm=False):
        if self.unarmed_only and not self.transport.fake:
            raise PreconditionError(
                'This host is restricted to unarmed validation; reboot/recover requires the next authorized run'
            )
        with self.operation_lock:
            try:
                self.recovery_details = {
                    'force_disarm_allowed': bool(allow_hil_force_disarm),
                    'force_disarm_used': False,
                }
                self.recovery_required = True
                self._remember_recovery(True)
                self.recovery = 'stopping_pilot'
                self.stop()
                self.recovery = 'stopping_model'
                fc = self.fc.copy()
                identity = fc['identity']
                if (
                    not identity
                    or fc['SYS_HITL'] != 1
                    or fc['heartbeat_at'] is None
                    or time.monotonic() - fc['heartbeat_at'] > 3
                ):
                    raise PreconditionError('Recovery requires fresh verified SYS_HITL=1 FC')
                self.recovery = 'disarming'
                self._recover_command(identity, 400, 0, 0, 0, 0, 0, 0, 0)
                unarmed = (
                    lambda: self.fc['identity'] == identity
                    and self.fc['state'] == 'verified'
                    and self.fc['heartbeat_at'] is not None
                    and self.fc['heartbeat_at'] > self.recover_command_at
                    and self.fc['armed'] is False
                )
                try:
                    self._wait(unarmed, 5, 'Disarm ACK/fresh heartbeat timeout')
                except OperationError:
                    context = dict(self.recover_context or {})
                    self.recovery_details['normal_disarm_rejected'] = context.get('rejected')
                    permitted = (
                        allow_hil_force_disarm
                        and context.get('command') == 400
                        and context.get('rejected')
                        in [mav.MAV_RESULT_TEMPORARILY_REJECTED, mav.MAV_RESULT_DENIED]
                        and context.get('identity') == identity
                        and context.get('generation') == self.fc_generation
                        and self.fc['identity'] == identity
                        and self.fc['state'] == 'verified'
                        and self.fc['SYS_HITL'] == 1
                        and self.fc['armed'] is True
                        and self.fc['heartbeat_at'] is not None
                        and time.monotonic() - self.fc['heartbeat_at'] <= 3
                        and self.state in ['stopped', 'fault']
                        and not self.worker.snapshot().running
                        and not self.external_controls_enabled
                        and not (self.pilot and self.pilot.poll() is None)
                    )
                    if not permitted:
                        raise
                    self.recovery = 'force_disarming'
                    self.recovery_details['force_disarm_used'] = True
                    self._recover_command(identity, 400, 0, 21196, 0, 0, 0, 0, 0)
                    self._wait(unarmed, 5, 'HIL force-disarm ACK/fresh heartbeat timeout')
                self.recovery = 'rebooting'
                self._recover_command(identity, 246, 1, 0, 0, 0, 0, 0, 0)
                self._wait(
                    lambda: self.fc['heartbeat_at'] is not None
                    and time.monotonic() - self.fc['heartbeat_at'] > 2
                    or self.fc['state'] == 'disconnected',
                    8,
                    'No reboot link-down evidence',
                )
                down = time.monotonic()
                self.recovery = 'waiting_link'
                self.recover_context = None
                self._wait(
                    lambda: self.fc['identity'] == identity
                    and self.fc['heartbeat_at'] is not None
                    and self.fc['heartbeat_at'] > down
                    and self.fc['armed'] is False,
                    RECONNECT_TIMEOUT_S,
                    'Reconnection heartbeat timeout (application USB/boot wait)',
                )
                self.fc['SYS_HITL'] = None
                self._send_message(
                    mav.MAVLink_param_request_read_message(*identity, b'SYS_HITL', -1)
                )
                self._wait(lambda: self.fc['SYS_HITL'] == 1, 5, 'SYS_HITL re-confirm timeout')
                self.recovery_required = False
                self.live_sent = False
                self.fc_generation += 1
                self.reset()
                self.recovery = 'complete'
                self.error = None
                self._remember_recovery(False)
            except Exception as exc:
                self.recovery = 'failed'
                self.error = str(exc)
                self.recovery_required = True
                self.reboot_quiet = False
                self._remember_recovery(True)
                self._block_external('recovery failed')
                raise

    def takeoff(self, arguments):
        if not self.status()['ready_for_takeoff']:
            raise PreconditionError('Model/FC/ground scenario not ready for takeoff')
        if self.pilot and self.pilot.poll() is None:
            raise PreconditionError('Pilot already running')
        reserved = [
            '--managed',
            '--validate-only',
            '--managed-session',
            '--start-gate',
            '--log-dir',
            '--bridge-port',
            '--listen-port',
        ]
        if any(a.split('=')[0] in reserved for a in arguments):
            raise PreconditionError(
                'Managed pilot routing/session/log options are owned by the host'
            )
        if 'agent' not in self.routes:
            raise PreconditionError('Agent route unavailable')
        args = []
        for name, value in self.profile['automation'].items():
            args += ['--' + name.replace('_', '-'), str(value)]
        entry = str(self.project / 'hitl/autotakeoff/autotakeoff.py')
        base = [
            sys.executable,
            '-B',
            entry,
            'takeoff',
            *args,
            *arguments,
            '--log-dir',
            str(self.pilot_dir),
            '--bridge-port',
            str(self.routes['agent'][0].getsockname()[1]),
            '--listen-port',
            str(self.routes['agent'][1][1]),
        ]
        validation = subprocess.run(
            [*base, '--validate-only'], capture_output=True, text=True, timeout=8
        )
        if validation.returncode:
            raise PreconditionError(
                validation.stderr.strip().splitlines()[-1]
                if validation.stderr
                else 'Invalid pilot arguments'
            )
        self._block_external('pilot startup')
        session = str(uuid.uuid4())
        generation = self.fc_generation
        run = self.run_id
        self.pilot_session = {
            'session_id': session,
            'run_id': run,
            'phase': 'starting',
            'milestone': None,
            'flight_complete': False,
        }
        gate = self.pilot_dir / 'start.json'
        gate.unlink(missing_ok=True)
        (self.pilot_dir / 'stop').unlink(missing_ok=True)
        (self.pilot_dir / 'status.json').unlink(missing_ok=True)
        self.pilot_log = (self.pilot_dir / 'console.log').open('ab')
        try:
            self.pilot = subprocess.Popen(
                [*base, '--managed', '--managed-session', session, '--start-gate', str(gate)],
                stdout=self.pilot_log,
                stderr=subprocess.STDOUT,
            )
            deadline = time.monotonic() + 5
            while time.monotonic() < deadline:
                rc = self.pilot.poll()
                if rc is not None:
                    raise OperationError(
                        'Pilot startup failed, exit '
                        + str(rc)
                        + ': '
                        + self._pilot_console_reason()
                    )
                status = self._pilot_status()
                if status.get('state') == 'startup_ready' and status.get('pid') == self.pilot.pid:
                    break
                time.sleep(0.02)
            else:
                raise OperationError('Pilot startup handshake timeout')
            if (
                self.fc_generation != generation
                or self.run_id != run
                or not self.status()['ready_for_takeoff']
            ):
                raise OperationError('Model/FC changed during pilot startup')
            temp = gate.with_suffix('.tmp')
            temp.write_text(json.dumps({'session_id': session}))
            os.replace(temp, gate)
            self._allow_external()
            self.pilot_session.update(phase='running', milestone='pilot_started')
            return {
                'session_id': session,
                'run_id': run,
                'milestone': 'pilot_started',
                'flight_complete': False,
            }
        except Exception as exc:
            try:
                self.stop_pilot()
            except Exception:
                pass
            self.pilot_session.update(phase='failed', error=str(exc))
            self.error = str(exc)
            raise

    def _pilot_console_reason(self):
        try:
            return (self.pilot_dir / 'console.log').read_text(errors='replace').splitlines()[-1]
        except (OSError, IndexError):
            return 'inspect pilot console.log'

    def close(self, force=False):
        if self.state == 'running' or self.recovery_required:
            if not force:
                raise PreconditionError(
                    'Shutdown requires recover-and-exit or explicit force-exit; PX4 recovery incomplete'
                )
            try:
                self.stop()
            except Exception:
                if not force:
                    raise
                self.recovery_required = True
                self.recovery = 'failed'
                self.counters['shutdown_stop_failures'] += 1
        self._block_external('host shutdown')
        self.host = 'shutting_down'
        self.worker.stop()
        self.activity.set_active(False)
        self.quit.set()
        self.io.join(timeout=2)
        self.logger.join(timeout=2)
        if not self.fc_only:
            self.model.lib.hil_model_terminate()
        for sock, dest in self.routes.values():
            sock.close()


def serial_error():
    import serial

    return serial.SerialException

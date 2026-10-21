#!/usr/bin/env python3
"""Run the SITL: minimal PX4 with the plant_sim plant module (sitl/README.md).

  <venv python> -B sitl/run_sitl.py \
      --out <new folder> [--scenario sitl/scenarios/stabilized_steps.json] \
      [--duration S] [--instances N]

The scenario JSON (sitl/scenarios/) gives the mode, the air-start state, a settling time and the stick steps
(step times count from the end of settle_s; trace.csv times count from the first plant step). With "trim": true
the start is the plant's level-flight trim at the given airspeed (sitl/trim.py): pitch = angle of attack,
throttle stick from the trim throttle, and in Stabilized a base pitch stick that holds the trim pitch.
"base_sticks" (absolute) replaces those base sticks; "base" adds offsets to them.
--ulog (default; --no-ulog skips it) starts PX4's logger with the HITL logging profile (SDLOG_PROFILE from deploy/00_native/
hitl_overrides.params): <instance>/log/<date>/*.ulg, the input of sitl/replay.py's self-test.
--logger-topics FILE (with --ulog) replaces that profile by PX4's custom topic list (lines "topic interval_ms
instance", copied to <instance>/etc/logging/logger_topics.txt), e.g. every input at full rate.
--start CMD (repeatable) starts one more PX4 command after the mode's modules and before plant_sim, e.g.
--start "guidance_template start" for a module of modules/src (a scenario's "start" list adds to these; see
sitl/scenarios/template_offboard.json). An A/B baseline needs a native PX4 mode with the same task targets: in an
OFFBOARD scenario the same run without the module only removes the guidance (sitl/README.md).
--replay ULOG instead replays a ULog through PX4's own controllers (sitl/replay.py, needs
`python3 sitl/build_sitl.py --replay`) and compares the recomputed outputs with the logged ones
(analysis/replay_compare.py): <out>/replay_report.json.

Each instance gets its own working directory under --out with the generated startup script, the PX4
log and the module output (trace.csv, summary.json). Parameters come from deploy/00_native/ (FC export
plus HITL overrides), translated for SITL: outputs on PWM_MAIN_FUNC instead of HIL_ACT_FUNC, simulated
IMU device ids, SYS_HITL=0, FC sensor calibration dropped. Prints one JSON line with every instance's
real-time factor and the aggregate. Exit 0 only when every instance ran its whole scenario: killed at --timeout,
"stalled" (below), a PX4 exit other than its own shutdown after the summary, no summary.json, or a plant_sim
"end" other than "complete" (diverged, stopped, or a SITL build from before that field) is listed in the
instance's "problems", run.json says "complete": false and the exit code is 2.

Each instance runs in its own process group. One that has not started plant_sim (no trace.csv) within
--startup-timeout s, whose trace.csv has not grown for --stall-timeout s, or that exited without a summary is
killed together with its group (its start-up script included: killing PX4 alone leaves the script sending
commands to the next PX4 of that instance number) and restarted, at most --retries times; after that it is
reported as "stalled" instead of being waited for until --timeout. A restart keeps the failed attempt's whole
folder as instance_<i>_attempt<n>. The groups do not get a terminal's Ctrl-C: a Ctrl-C, SIGTERM or SIGHUP to the
runner stops every instance's group before the runner exits.

The PX4 build is <PX4 tree>/build/px4_sitl_default_sim2hitl (python3 sitl/build_sitl.py; SIM2HITL_SITL_SUFFIX
replaces _sim2hitl); the tree location comes from px4/check_px4_tree.py (SIM2HITL_PX4_ROOT or the local config
file). Before launching it reads which plant library that build loads (build_sitl.loaded_library: macOS otool,
Linux ldd) and warns when it is not the active model release's (rebuild with build_sitl.py); run.json records
both. --instance-base N numbers the PX4 instances from N (their /tmp/px4-sock-<n>), so two runs at the same
time do not collide.
"""
import argparse
import json
import math
import os
import signal
import sys
import struct
import subprocess
import time
from pathlib import Path

sys.dont_write_bytecode = True  # imports trim, replay, analysis modules: no __pycache__ in the project
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'sitl'))
sys.path.insert(0, str(ROOT / 'plant'))
import wind as plant_wind  # noqa: E402  same wind description as the HITL scene
NATIVE = ROOT / 'deploy' / '00_native'
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'px4'))
from check_px4_tree import px4_root  # noqa: E402
from build_sitl import active_plant_lib, build_dir, loaded_library  # noqa: E402
# Parameters that belong to the physical FC or to modules this SITL does not run.
EXCLUDED_PREFIXES = ('CAL_', 'UAVCAN_', 'RC', 'MAV_', 'SER_', 'BAT', 'EKF2_', 'SDLOG_', 'DSHOT_', 'UXRCE_',
                     'HIL_ACT_', 'PWM_', 'SYS_HITL', 'SYS_AUTOSTART', 'SYS_AUTOCONFIG', 'SYS_USB_AUTO')
SIM_IMU_ID = 1310988  # DRV_IMU_DEVTYPE_SIM, bus 1, address 1 (plant_sim's PX4Accelerometer / PX4Gyroscope)
# PX4 modules per mode, started before plant_sim: only what the mode needs, no EKF2 (sitl/README.md).
COMMON = ['sensors start', 'pwm_out_sim start -m sim', 'control_allocator start', 'fw_rate_control start']
MODULES = {'acro': COMMON, 'stabilized': COMMON + ['fw_att_control start'],
           # OFFBOARD (velocity): PX4's own altitude/speed control under a guidance module started by "start"
           'offboard': COMMON + ['fw_att_control start', 'fw_lat_lon_control start'],
           'mission': ['dataman start'] + COMMON + ['fw_att_control start', 'fw_lat_lon_control start',
                                                      'fw_mode_manager start', 'navigator start']}


def read_params(path):
    """QGC .params file -> {name: (value text, QGC type)}; type 6 = INT32, 9 = FLOAT."""
    params = {}
    for line in path.read_text().splitlines():
        fields = line.strip().split('\t')
        if len(fields) == 5 and not line.startswith('#'):
            params[fields[2]] = (fields[3], int(fields[4]))
    return params


def sitl_params():
    fc = read_params(NATIVE / 'fc_params_20261013.params')
    hitl = read_params(NATIVE / 'hitl_overrides.params')
    params = {k: v for k, v in fc.items() if not k.startswith(EXCLUDED_PREFIXES)}
    for channel in range(1, 17):
        params[f'PWM_MAIN_FUNC{channel}'] = (hitl.get(f'HIL_ACT_FUNC{channel}', ('0', 6))[0], 6)
    params['PWM_MAIN_REV'] = (hitl.get('HIL_ACT_REV', ('0', 6))[0], 6)
    params.update({'SYS_HITL': ('0', 6), 'CAL_ACC0_ID': (str(SIM_IMU_ID), 6), 'CAL_GYRO0_ID': (str(SIM_IMU_ID), 6)})
    return params


def params_bson(params):
    """The parameters as the BSON document PX4's `param import` reads (int32 0x10, double 0x01)."""
    body = b''
    for name, (value, kind) in sorted(params.items()):
        key = name.encode() + b'\0'
        if kind == 6:
            body += b'\x10' + key + struct.pack('<i', int(float(value)))
        else:
            body += b'\x01' + key + struct.pack('<d', float(value))
    return struct.pack('<i', len(body) + 5) + body + b'\0'


def startup_script(params_file, mode, duration, out, ulog=False, start=()):
    # One `param import` instead of one `param set` client call per parameter: with several instances
    # starting at once, single px4-param client calls occasionally never returned.
    lines = ['#!/bin/sh', 'set +e', '. px4-alias.sh',
             # The PX4 server thread can still be starting when this script runs (several instances at once).
             'n=0; until px4-ver --instance $px4_instance all >/dev/null 2>&1 || [ $n -ge 200 ]; do sleep 0.05; n=$((n+1)); done',
             'param select parameters.bson', f'param import {params_file}']
    lines += MODULES[mode]
    lines += list(start)  # --start: custom modules, after the mode's PX4 modules
    # Last, so every PX4 work queue exists before the simulated clock starts (reproducible lockstep).
    lines.append(f'plant_sim start -d {duration} -o {out} -s {out}/scenario.txt')
    if ulog:
        # After plant_sim so its topics (airspeed_validated...) exist: the logger only takes optional topics that
        # are advertised when it starts. -f: from start until shutdown; -p: one write per angular-velocity sample.
        lines.append('logger start -f -t -b 1000 -p vehicle_angular_velocity')
    return '\n'.join(lines) + '\n'


def scenario_text(scenario, params):
    """The scenario JSON as the line records plant_sim reads (PlantSim::load_scenario)."""
    initial = scenario['initial']
    airspeed = initial['airspeed_m_s']
    alpha_deg, throttle = 0.0, 0.0
    trim_result = None
    if scenario.get('trim'):
        import trim
        trim_result = trim.trim(trim.Core(trim.default_plant_lib()), airspeed, initial['height_agl_m'])
        trim_result = {k: v for k, v in trim_result.items() if k not in ('state', 'controls', 'surface_deg')}
        alpha_deg, throttle = trim_result['alpha_deg'], trim_result['throttle']
    alpha = math.radians(alpha_deg)
    base = {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0, 'throttle': 2.0 * throttle - 1.0}
    if scenario['mode'] == 'stabilized':
        # fw_att_control: pitch setpoint = -pitch stick * FW_MAN_P_MAX + FW_PSP_OFF (deg)
        p_max = float(params.get('FW_MAN_P_MAX', ('30', 9))[0])
        psp_off = float(params.get('FW_PSP_OFF', ('0', 9))[0])
        base['pitch'] = -(alpha_deg - psp_off) / p_max
    # Absolute base sticks (manual_control_setpoint units) replace the trim-derived ones; "trim" then only sets
    # the air start. Used when the sticks must equal a fixed HITL command sequence.
    base.update(scenario.get('base_sticks', {}))
    for k, v in scenario.get('base', {}).items():
        base[k] += v
    lines = [f"mode {scenario['mode']}",
             'ic {:.9g} {:.9g} 0 {:.9g} 0 {:.9g} {:.9g} {:.12g} {:.12g}'.format(
                 initial['height_agl_m'], airspeed * math.cos(alpha), airspeed * math.sin(alpha), alpha_deg,
                 initial.get('heading_deg', 0.0), initial['latitude_deg'], initial['longitude_deg']),
             'base {roll:.9g} {pitch:.9g} {yaw:.9g} {throttle:.9g}'.format(**base)]
    if 'carrier' in scenario:
        carrier = scenario['carrier']
        keys = ('wait_s', 'climb_s', 'cruise_s', 'height_m', 'speed_m_s')
        if set(carrier) != set(keys):
            raise ValueError('carrier requires exactly ' + ', '.join(keys))
        values = [carrier[k] for k in keys]
        if any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) for v in values):
            raise ValueError('carrier values must be finite numbers')
        if (values[0] < 0 or values[1] < 1 or values[2] < 0 or not 0 < values[3] <= 10000
                or not 0 < values[4] <= 200):
            raise ValueError('carrier values outside supported limits')
        if scenario.get('wind') is not None or scenario.get('trim') or initial['height_agl_m'] != 0 or airspeed != 0 or initial.get('heading_deg', 0) != 0:
            raise ValueError('carrier phase one requires north-facing ground start, no trim and no wind')
        lines.append('carrier ' + ' '.join(format(v, '.12g') for v in values))
    if scenario.get('native_carrier_circuit'):
        if scenario['mode'] != 'mission' or 'carrier' not in scenario:
            raise ValueError('native circuit requires mission mode and carrier')
        sys.path.insert(0, str(ROOT / 'missions'))
        from carrier_circuit import sitl_lines
        lines.extend(sitl_lines())
    settle = scenario.get('settle_s', 0.0)  # step times in the JSON are counted after the settling time
    for step in scenario.get('steps', []):
        lines.append('step {:.9g} {:.9g} {:.9g} {:.9g} {:.9g} {:.9g}'.format(
            settle + step['start_s'], settle + step['end_s'], step.get('roll', 0.0), step.get('pitch', 0.0), step.get('yaw', 0.0),
            step.get('throttle', 0.0)))
    # Wind: the scenario's "wind" object (plant/wind.py); SITL starts in the air.
    wind_config, wind_reminders = plant_wind.parse(scenario.get('wind'), ground_start=initial['height_agl_m'] == 0)
    if scenario.get('wind') is not None:
        lines.append(plant_wind.sitl_line(wind_config))
    return '\n'.join(lines) + '\n', trim_result, wind_reminders


def plant_check(px4):
    """The plant library this PX4 build loads (build_sitl.loaded_library: macOS otool, Linux ldd) next to the active
    model release's. The build finds it through the rpath written when it was built, so after a new release, or a
    rebuild from another checkout sharing the PX4 tree, it silently runs the plant it was built with until
    sitl/build_sitl.py runs again."""
    try:
        active = active_plant_lib().resolve()
    except (OSError, ValueError, KeyError, TypeError):
        active = None
    loaded = None
    if active is not None and px4.is_file():
        try:
            found = loaded_library(px4, active.name)
        except (OSError, subprocess.CalledProcessError):
            found = None
        loaded = found.resolve() if found else None
    return {'loaded': str(loaded) if loaded else None, 'active_release': str(active) if active else None,
            'same': None if loaded is None else loaded == active}


def instance_problems(result, terminated):
    """Why an instance did not run its whole scenario ([] when it did). PX4 shuts itself down after plant_sim's
    summary or is terminated here a few seconds later: both are the expected end."""
    problems = []
    if result['exit'] == 'timeout':
        problems.append('killed at --timeout')
    elif result['exit'] == 'stalled':
        problems.append('stalled: no progress within --startup-timeout / --stall-timeout after the retries')
    elif result['exit'] != 0 and not terminated:
        problems.append(f'PX4 exited with {result["exit"]}')
    if 'error' in result:
        problems.append(result['error'])
    elif result.get('end') != 'complete':
        problems.append(f'plant_sim end {result.get("end")!r} after {result.get("steps")} of {result.get("requested_steps")} steps'
                        + ('' if 'end' in result else ' (summary from a SITL build before "end": run sitl/build_sitl.py)'))
    return problems


def replay_main(args):
    import replay
    sys.path.insert(0, str(ROOT / 'analysis'))
    import replay_compare
    import ulog
    result = replay.run(args.replay, args.out, args.speed)
    report = {'replay': result}
    if result['replayed_ulog']:
        report.update(replay_compare.compare(ulog.read(args.replay), ulog.read(result['replayed_ulog']), args.tolerance,
                                             replay.INPUTS))
    report = replay_compare.jsonable(report)
    (Path(args.out) / 'replay_report.json').write_text(json.dumps(report, indent=2, allow_nan=False))
    print(json.dumps(report, allow_nan=False))
    return 0 if result['replayed_ulog'] and result['exit'] == 0 and not result['timed_out'] else 2


def stop(proc, sig):
    """Signal PX4 and everything it started (start-up script, PX4 client commands): the instance's process group."""
    try:
        os.killpg(proc.pid, sig)
    except ProcessLookupError:
        pass


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--scenario', type=Path, default=ROOT / 'sitl' / 'scenarios' / 'stabilized_steps.json')
    parser.add_argument('--duration', type=float, help='simulated seconds after the first actuator output (default: the scenario)')
    parser.add_argument('--instances', type=int, default=1)
    parser.add_argument('--instance-base', type=int, default=0,
                        help='PX4 instance number of the first instance (its /tmp/px4-sock-<n>): another base keeps two\n'
                             'runs at the same time apart')
    parser.add_argument('--timeout', type=float, default=600.0)
    parser.add_argument('--stagger', type=float, default=1.0, help='seconds between instance launches')
    parser.add_argument('--retries', type=int, default=2)
    parser.add_argument('--startup-timeout', type=float, default=60.0)
    parser.add_argument('--stall-timeout', type=float, default=60.0, help='seconds without trace.csv growing: stuck')
    parser.add_argument('--ulog', action=argparse.BooleanOptionalAction, default=True,
                        help="record a ULog with the HITL logging profile (default; --no-ulog to skip)")
    parser.add_argument('--logger-topics', type=Path, help='custom logger topic list instead of the profile')
    parser.add_argument('--start', action='append', default=[], metavar='CMD',
                        help='one more PX4 command before plant_sim, e.g. "guidance_template start" (repeatable)')
    parser.add_argument('--replay', type=Path, metavar='ULOG', help='replay this ULog through the PX4 controllers')
    parser.add_argument('--speed', type=float, default=5.0, help='--replay speed factor (self-test: 5 = 1, 10 slightly worse)')
    parser.add_argument('--tolerance', type=float, default=1e-3, help='--replay: report differences above this')
    args = parser.parse_args()
    if args.replay:
        return replay_main(args)
    px4_tree = px4_root()
    px4 = build_dir(px4_tree) / 'bin' / 'px4'
    plant = plant_check(px4)
    if plant['same'] is False:
        print(f"plant: this SITL build loads {plant['loaded']}, the active model release is {plant['active_release']}; "
              'rebuild with sitl/build_sitl.py (unless it was built with --plant-lib on purpose)', file=sys.stderr)
    params = sitl_params()
    if args.ulog:
        params['SDLOG_PROFILE'] = read_params(NATIVE / 'hitl_overrides.params')['SDLOG_PROFILE']
    scenario = json.loads(args.scenario.read_text())
    if scenario.get('native_carrier_circuit'):
        sys.path.insert(0, str(ROOT / 'missions'))
        from carrier_circuit import PARAMETERS
        params.update({k: (str(v), 9) for k, v in PARAMETERS.items()})
    duration = args.duration or scenario.get('settle_s', 0.0) + scenario['duration_s']
    text, trim_result, wind_reminders = scenario_text(scenario, params)
    for reminder in wind_reminders:
        print('wind: ' + reminder, file=sys.stderr)
    env = dict(os.environ, PATH=f'{px4.parent}:{os.environ.get("PATH", "")}')
    # The start-up script waits for the PX4 server, launches are staggered, and a stuck instance (see the module
    # docstring) is restarted with its whole process group (at most --retries times; counted in the report).
    # px4.log is block-buffered, so it is not polled.

    def launch(i):
        work = (args.out / f'instance_{i}').resolve()
        work.mkdir(parents=True, exist_ok=True)
        for name in ('summary.json', 'trace.csv'):
            (work / name).unlink(missing_ok=True)
        script = work / 'sitl_rcS'
        (work / 'sitl_params.bson').write_bytes(params_bson(params))
        (work / 'scenario.txt').write_text(text)
        if args.logger_topics:
            (work / 'etc' / 'logging').mkdir(parents=True, exist_ok=True)
            (work / 'etc' / 'logging' / 'logger_topics.txt').write_text(args.logger_topics.read_text())
        script.write_text(startup_script(work / 'sitl_params.bson', scenario['mode'], duration, work, args.ulog,
                                              list(args.start) + scenario.get('start', [])))
        log = open(work / 'px4.log', 'w')
        proc = subprocess.Popen([str(px4), '-d', '-i', str(args.instance_base + i), '-s', str(script), '-w', str(work)],
                                stdout=log, stderr=subprocess.STDOUT, env=env, start_new_session=True)
        return {'work': work, 'log': log, 'proc': proc, 'summary_seen': None, 'trace_size': -1,
                'progress': time.monotonic(), 'terminated': False}

    # The instances' process groups do not get a terminal's Ctrl-C: whatever stops this runner (Ctrl-C, SIGTERM,
    # SIGHUP, an error) first stops every instance still running.
    for sig in (signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, lambda signum, frame: sys.exit(128 + signum))
    start = time.monotonic()
    runs = {}
    codes = {}
    try:
        for i in range(args.instances):
            runs[i] = launch(i)
            time.sleep(args.stagger)
        retries = {i: 0 for i in runs}
        attempts = {i: [] for i in runs}  # folders of the failed attempts, kept as evidence
        # Poll every instance together. The module writes summary.json when the scenario ends and then asks
        # PX4 to shut down; terminate PX4 ourselves if it has not exited a few seconds after the summary.
        while len(codes) < len(runs) and time.monotonic() - start < args.timeout:
            for i, run in runs.items():
                if i in codes:
                    continue
                proc, work = run['proc'], run['work']
                code = proc.poll()
                done = (work / 'summary.json').is_file()
                if not done:
                    trace = work / 'trace.csv'
                    size = trace.stat().st_size if trace.is_file() else -1
                    if size != run['trace_size']:
                        run['trace_size'], run['progress'] = size, time.monotonic()
                    stuck = time.monotonic() - run['progress'] > (args.startup_timeout if size < 0 else args.stall_timeout)
                    if code is not None or stuck:
                        if code is None:
                            stop(proc, signal.SIGKILL)
                            proc.wait()
                        if retries[i] < args.retries:
                            run['log'].close()
                            retries[i] += 1
                            kept = work.with_name(f'{work.name}_attempt{retries[i]}')
                            work.rename(kept)  # the evidence stays; the next attempt starts in a new folder of the old name
                            attempts[i].append(kept.name)
                            runs[i] = launch(i)
                            continue
                        code = 'stalled' if code is None else code
                if code is None and done:
                    run['summary_seen'] = run['summary_seen'] or time.monotonic()
                    if time.monotonic() - run['summary_seen'] > 5.0:
                        stop(proc, signal.SIGTERM)
                        code = proc.wait()
                        run['terminated'] = True
                if code is not None:
                    codes[i] = code
            time.sleep(0.05)
    except BaseException:
        for i, run in runs.items():
            if i not in codes:
                stop(run['proc'], signal.SIGKILL)
        raise
    results = []
    for i, run in runs.items():
        if i not in codes:
            stop(run['proc'], signal.SIGKILL)
            run['proc'].wait()
            codes[i] = 'timeout'
        run['log'].close()
        summary = run['work'] / 'summary.json'
        result = json.loads(summary.read_text()) if summary.is_file() else {'error': 'no summary'}
        result.update({'instance': i, 'exit': codes[i], 'retries': retries[i], 'failed_attempts': attempts[i]})
        result['problems'] = instance_problems(result, run['terminated'])
        results.append(result)
    incomplete = [r['instance'] for r in results if r['problems']]
    wall = time.monotonic() - start
    sim = sum(r.get('sim_s', 0.0) for r in results)
    stepping = [r for r in results if 'wall_start_unix_us' in r]
    window = (max(r['wall_end_unix_us'] for r in stepping) - min(r['wall_start_unix_us'] for r in stepping)) * 1e-6 if stepping else 0.0
    report = {'scenario': scenario['name'], 'complete': not incomplete, 'start': args.start, 'trim': trim_result, 'wind_reminders': wind_reminders, 'plant_library': plant, 'instances': results, 'parameters': len(params), 'wall_s_total': wall,
              # Throughput: simulated seconds of all instances per wall second, from the first instance's
              # first plant step to the last instance's last (only meaningful when the runs overlap).
              'throughput_realtime_factor': sum(r['sim_s'] for r in stepping) / window if window else 0.0,
              'stepping_window_s': window,
              'realtime_factor_including_startup': sim / wall if wall else 0.0}
    (args.out / 'run.json').write_text(json.dumps(report, indent=2))
    print(json.dumps(report))
    if incomplete:
        print(f'not run through: instance {incomplete}, see "problems" in {args.out / "run.json"}', file=sys.stderr)
    return 2 if incomplete else 0


if __name__ == '__main__':
    sys.exit(main())

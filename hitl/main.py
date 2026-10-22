import argparse, json, signal, sys, time, subprocess
from pathlib import Path
from host.core import Core
from host.transport import SerialTransport, FakeFC
from host.ipc import Endpoint, request, reserve

RUNTIME = Path(__file__).resolve().parent


def main(argv=None):
    arguments = list(sys.argv[1:] if argv is None else argv)
    if arguments[:1] == ['takeoff'] and any(x in arguments for x in ['--help', '-h']):
        return subprocess.call(
            [
                sys.executable,
                '-B',
                str(RUNTIME / 'autotakeoff/autotakeoff.py'),
                'takeoff',
                '--help',
            ]
        )
    p = argparse.ArgumentParser(
        description='Shared-library HIL: one model/serial owner (bridge) and the commands that talk to it'
    )
    p.add_argument(
        'action',
        nargs='?',
        default='bridge',
        choices=[
            'bridge',
            'status',
            'model-start',
            'model-stop',
            'model-reset',
            'takeoff',
            'stop-takeoff',
            'recover',
            'reboot',
            'shutdown',
            'recover-and-exit',
            'test-takeoff-hold',
            'preflight',
            'flight-summary',
            'fetch-log',
            'param-readback',
            'param-plan',
            'param-deploy',
            'sd-list',
            'sd-get',
            'sd-put',
            'sd-rm',
            'hitl-run',
            'carrier-run', 'carrier-circuit-run',
            'build-firmware',
            'flash',
            'deploy-test',
        ],
    )
    p.add_argument('--fake-fc', action='store_true')
    p.add_argument('--fake-state-dir', type=Path)
    p.add_argument('--fake-config', type=Path)
    p.add_argument('--unarmed-only', action='store_true')
    p.add_argument('--port', default='auto')
    p.add_argument('--aircraft', default='skysurfer')
    p.add_argument('--scenario', type=Path)
    p.add_argument('--mission-profile', choices=['carrier_circuit','carrier_orbit_landing'], default='carrier_circuit')
    p.add_argument('--log-dir', type=Path)
    p.add_argument('--out', type=Path,
                   help='fetch-log: directory for the downloaded FC ULog; param-readback/-plan/-deploy: directory for the files')
    p.add_argument('--release', type=Path,
                   help='param-readback/-plan/-deploy, hitl-run, build-firmware, flash, deploy-test: deploy recipe '
                        'folder (default deploy/00_native)')
    p.add_argument('--flash', action='store_true', help='deploy-test: build if needed and flash this recipe')
    p.add_argument('--reuse', action='store_true', help='deploy-test: the FC already runs this recipe; no flashing')
    p.add_argument('--no-hitl', action='store_true', help='deploy-test: stop after parameters and boot file')
    p.add_argument('--log-id', type=int, help='fetch-log: FC log id (default: the newest)')
    p.add_argument('--list', action='store_true', help='fetch-log: only list the FC logs')
    p.add_argument('--sd-path', help='sd-*: path on the FC SD card (sd-list default /fs/microsd/etc)')
    p.add_argument('--file', type=Path, help='sd-put: the local file to put on the SD card')
    p.add_argument('--json', action='store_true')
    p.add_argument('--force-exit', action='store_true')
    p.add_argument('--duration', type=float)
    p.add_argument(
        '--hil-force-disarm',
        action='store_true',
        help='recover: allow force-disarm only after stopped HIL model and explicit normal-disarm rejection',
    )
    if arguments[:1] == ['test-takeoff-hold']:
        p.add_argument('--altitude', type=float, default=100)
        p.add_argument('--hold-seconds', type=float, default=60)
        p.add_argument('--hold-timeout', type=float, default=180)
        p.add_argument('--flight-timeout', type=float, default=300)
    if arguments[:1] == ['hitl-run']:
        # Default flight of hitl/flight_test.json; other values run and are recorded in the report.
        for name in ['--altitude', '--hold-seconds', '--hold-timeout', '--flight-timeout']:
            p.add_argument(name, type=float)
    if arguments[:1] in (['hitl-run'], ['test-takeoff-hold']):
        p.add_argument('--after', help='after-takeoff script of hitl/commands (default after_takeoff_loiter)')
        p.add_argument('--check', help='hitl-run: script judging the module on the FC log (e.g. analysis/template_check.py)')
    args, extra = p.parse_known_args(argv)
    if (args.fake_state_dir or args.fake_config) and not args.fake_fc:
        raise ValueError(
            'Fake state/config requires --fake-fc; real recovery state cannot be overridden'
        )
    if args.fake_fc and not args.fake_state_dir:
        # Without its own state directory a fake client would talk to the real owner, and a fake owner
        # (bridge) would use the real owner's state: its recovery marker and parameter deploy latch.
        raise ValueError('--fake-fc needs --fake-state-dir (a separate fake owner)')
    if args.fake_config and args.action not in ['bridge', 'test-takeoff-hold']:
        raise ValueError('--fake-config is only supported when starting a fake host')
    if args.aircraft != 'skysurfer':
        raise ValueError('Aircraft recipe not available')
    if args.hil_force_disarm and args.action not in ['recover', 'reboot', 'recover-and-exit']:
        raise ValueError('--hil-force-disarm is only for recovery')
    if args.action == 'fetch-log' and not args.out and not args.list:
        raise ValueError('fetch-log needs --out <directory> (or --list)')
    if args.action in ['param-readback', 'param-plan', 'param-deploy', 'sd-get', 'sd-put', 'sd-rm'] and not args.out:
        raise ValueError(args.action + ' needs --out <directory>')
    if args.action in ['sd-get', 'sd-put', 'sd-rm'] and not args.sd_path:
        raise ValueError(args.action + ' needs --sd-path <path on the SD card>')
    if args.action == 'sd-put' and not args.file:
        raise ValueError('sd-put needs --file <local file>')
    if args.action in ['preflight', 'flight-summary']:
        # Read-only agent tools: no owner, serial port or FC traffic.
        from host.agent_tools import flight_summary, preflight

        if args.action == 'preflight':
            value = preflight(RUNTIME, args.port, args.fake_state_dir)
            print(json.dumps(value, indent=2, allow_nan=False))
            return 0 if value['ready'] else 2
        if not args.log_dir:
            raise ValueError('flight-summary needs --log-dir <test-takeoff-hold output directory>')
        value = flight_summary(args.log_dir)
        print(json.dumps(value, indent=2, allow_nan=False))
        return 0 if value['verdict'] == 'PASS' else 3
    if args.action == 'test-takeoff-hold':
        if extra:
            raise ValueError('Unknown test arguments: ' + ' '.join(extra))
        from host.flight_test import run_test

        return run_test(RUNTIME, args)
    if args.action in ('carrier-run', 'carrier-circuit-run'):
        if args.fake_fc or args.fake_state_dir or args.unarmed_only:
            raise ValueError('carrier-run requires an explicitly authorized real FC run, not fake/unarmed-only flags')
        if extra or not args.scenario or not args.release or not args.out:
            raise ValueError('carrier-run needs --scenario, --release and a new --out')
        return subprocess.call([sys.executable, '-B', str(RUNTIME / 'carrier_test.py'),
                                '--scenario', str(args.scenario), '--release', str(args.release),
                                '--out', str(args.out)] + (['--circuit', '--mission-profile', args.mission_profile] if args.action == 'carrier-circuit-run' else []))
    if args.action == 'hitl-run':
        if extra:
            raise ValueError('Unknown hitl-run arguments: ' + ' '.join(extra))
        from host.hitl_run import run_hitl

        return run_hitl(RUNTIME, args)
    if args.action == 'build-firmware':
        if not args.release:
            raise ValueError('build-firmware needs --release deploy/<name>')
        return subprocess.call([sys.executable, '-B', str(RUNTIME.parent / 'px4/build_firmware.py'), '--release',
                                str(args.release)])
    if args.action in ['flash', 'deploy-test']:
        # Stage 3 (host/deploy.py): a client of the owner like hitl-run; plan only without --flash / --reuse.
        if args.flash and args.reuse:
            raise ValueError('--flash or --reuse, not both')
        if args.fake_fc:
            raise ValueError(args.action + ' drives a physical FC only')
        from host.deploy import run_deploy

        report, path = run_deploy(RUNTIME, args)
        print(json.dumps({'verdict': report['verdict'], 'scope': report['scope'], 'report': str(path),
                          'steps': [(s['step'], s['verdict']) for s in report['steps']]}))
        return {'PASS': 0, 'PLAN': 0}.get(report['verdict'], 3)
    if args.action != 'bridge':
        result = request(
            RUNTIME,
            args.action,
            {
                'scene': json.loads(args.scenario.read_text()) if args.scenario else None,
                'argv': extra,
                'force': args.force_exit,
                'hil_force_disarm': args.hil_force_disarm,
                'out': str(args.out.resolve()) if args.out else None,
                'log_id': args.log_id,
                'list': args.list,
                'release': str(args.release.resolve()) if args.release else None,
                'sd_path': args.sd_path,
                'file': str(args.file.resolve()) if args.file else None,
            },
            fake_state_dir=args.fake_state_dir,
            # param-deploy: readbacks, reboots, and possibly a rollback. sd-put / sd-rm: up to two reboots.
            timeout_s=(1800 if args.action in ['fetch-log', 'param-deploy', 'sd-put', 'sd-rm']
                       else 600 if args.action in ['param-readback', 'param-plan', 'sd-get']
                       else 90 if args.action in ['recover', 'reboot', 'recover-and-exit'] else 45),
        )
        print(json.dumps(result.get('result', result), allow_nan=False))
        if args.action == 'param-readback' and result['status'] == 'complete':
            # The files are written either way; exit 5 says the readback is not complete (verdict INCOMPLETE).
            return 0 if result['operation']['result']['verdict'] == 'PASS' else 5
        if args.action in ['param-deploy', 'sd-put', 'sd-rm'] and result['status'] == 'complete':
            # FAIL with the written values restored is a completed operation, not a success.
            return 0 if result['operation']['result']['verdict'] == 'PASS' else 3
        return (
            0 if result['status'] == 'complete' else result.get('operation', {}).get('exit_code', 3)
        )
    if extra:
        raise ValueError('Unknown arguments: ' + ' '.join(extra))
    from host.ipc import state_path

    log = args.log_dir or state_path(RUNTIME, args.fake_state_dir) / (
        'logs-' + time.strftime('%Y%m%d_%H%M%S')
    )
    # Acquire ownership before any model or serial initialization.
    owner_path = state_path(RUNTIME, args.fake_state_dir)
    owner_path.mkdir(parents=True, exist_ok=True)
    transport = (
        FakeFC(RUNTIME.parent, config_path=args.fake_config)
        if args.fake_fc
        else SerialTransport(RUNTIME.parent, args.port)
    )
    core = None
    endpoint = None
    lease = None
    try:
        # Never a second owner.
        if (owner_path / 'host.json').exists():
            try:
                existing = request(RUNTIME, 'status', fake_state_dir=args.fake_state_dir)
                if existing['status'] == 'complete':
                    if bool(existing['result'].get('fake_fc')) != bool(args.fake_fc):
                        # The owner says what it drives; a client never takes its own flag for it.
                        raise RuntimeError(
                            f"Host already running with fake_fc={existing['result'].get('fake_fc')}, "
                            f'not fake_fc={args.fake_fc}'
                        )
                    raise RuntimeError('Host already running; use status/model commands')
            except (OSError, ValueError):
                pass
        lease = reserve(RUNTIME, args.fake_state_dir)
        core = Core(
            RUNTIME,
            transport,
            log,
            unarmed_only=args.unarmed_only,
            fake_state_dir=args.fake_state_dir,
            route_settings=transport.config.get('routes') if args.fake_fc else None,
        )
        endpoint = Endpoint(core, lease)

        def stop(signum, frame):
            print(
                'Stopping host; any incomplete PX4 recovery is retained in the report.',
                file=sys.stderr,
            )
            core.close(force=True)

        signal.signal(signal.SIGINT, stop)
        signal.signal(signal.SIGTERM, stop)
        start = time.monotonic()
        while not core.quit.is_set():
            if args.duration and time.monotonic() - start >= args.duration:
                core.close(force=True)
                break
            time.sleep(0.1)
        return 0
    finally:
        if core and not core.quit.is_set():
            core.close(force=True)
        if core:
            (log / 'final_status.json').write_text(
                json.dumps(core.status(), indent=2, allow_nan=False)
            )
        if endpoint:
            endpoint.close()
        elif lease:
            lease.close()


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except (OSError, RuntimeError, ValueError) as exc:
        print(str(exc), file=sys.stderr)
        raise SystemExit(getattr(exc, 'exit_code', 4))

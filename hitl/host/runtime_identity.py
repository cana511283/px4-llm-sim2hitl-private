"""Deterministic startup source inventory; never reads IPC state or secrets."""

import hashlib
import json
from datetime import datetime, timezone
from pathlib import Path


def source_files(project):
    project = Path(project)
    fixed = [
        'runbridge',
        'hitl/main.py',
        'hitl/carrier_test.py',
        'hitl/flight_test.json',
        'pyproject.toml',
        'uv.lock',
        '.python-version',
        'hitl/scenarios/ground.json',
        'hitl/aircraft/skysurfer/profile.json',
        'hitl/aircraft/skysurfer/scenario.py',
        'plant/wind.py',
    ]
    paths = {project / name for name in fixed if (project / name).is_file()}
    for folder, suffixes in [
        ('hitl/host', {'.py'}),
        ('hitl/autotakeoff', {'.py'}),
        ('hitl/commands', {'.py'}),
    ]:
        paths.update(
            p
            for p in (project / folder).rglob('*')
            if p.is_file() and p.suffix in suffixes and '__pycache__' not in p.parts
        )
    return sorted(paths, key=lambda p: p.relative_to(project).as_posix())


def inventory(project):
    project = Path(project)
    return [
        {
            'path': p.relative_to(project).as_posix(),
            'sha256': hashlib.sha256(p.read_bytes()).hexdigest(),
        }
        for p in source_files(project)
    ]


def source_id(records):
    canonical = json.dumps(
        sorted(records, key=lambda x: x['path']), sort_keys=True, separators=(',', ':')
    )
    return 'runtime-' + hashlib.sha256(canonical.encode()).hexdigest()


def capture(project, log_dir, model_build_id, manifest_checksum):
    records = inventory(project)
    value = {
        'schema_version': 1,
        'source_id': source_id(records),
        'captured_at': datetime.now(timezone.utc).isoformat(),
        'model_build_id': model_build_id,
        'model_manifest_sha256': manifest_checksum,
        'files': records,
        'scope': 'startup source snapshot; later lazy imports require unchanged source',
    }
    path = Path(log_dir) / 'runtime_source_manifest.json'
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2) + '\n')
    return {
        'schema_version': 1,
        'source_id': value['source_id'],
        'captured_at': value['captured_at'],
        'manifest': path.name,
        'model_build_id': model_build_id,
        'model_manifest_sha256': manifest_checksum,
    }


def drift(project, saved_records):
    """Explicit offline comparison; never changes the captured startup identity."""
    before = {x['path']: x['sha256'] for x in saved_records}
    after = {x['path']: x['sha256'] for x in inventory(project)}
    return sorted(
        path for path in before.keys() | after.keys() if before.get(path) != after.get(path)
    )

"""Minimal native orbit-to-alt and landing, without terrain measurements.
The orbit count is observed from the trace, not guaranteed by LOITER_TO_ALT.
"""
import carrier_circuit as baseline
SPEED = baseline.SPEED
AUTO_AFTER_RELEASE_S = 0
PARAMETERS = {**baseline.PARAMETERS, 'FW_LND_USETER': 0}
ROWS = [
    ('airspeed', 178, 0, 0, 0, [0, SPEED, -1, 0]),
    ('landing_start', 189, 0, 0, 0, [0, 0, 0, 0]),
    ('descent_orbit', 31, 800, -100, 40, [1, -100, 0, 1]),
    ('touchdown', 21, 1400, 0, 0, [0, 0, 0, None]),
]
def items():
    return baseline.items(ROWS)
def sitl_lines():
    lines = ['auto_after 60']
    for item in items():
        p = [float('nan') if x is None else x for x in item['params']]
        lines.append('mission ' + ' '.join(str(x) for x in [item['command'], *p]))
    return lines
def export(out):
    return baseline.export(out, items(), PARAMETERS, AUTO_AFTER_RELEASE_S, 'carrier_orbit_landing.plan')
if __name__ == '__main__':
    import argparse
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--out', required=True)
    export(p.parse_args().out)

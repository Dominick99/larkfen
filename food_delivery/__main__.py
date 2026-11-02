import argparse
from dataclasses import replace
import json
from pathlib import Path

from .navigation import Scenario
from .simulation import run


def main():
    parser = argparse.ArgumentParser(description='Standalone ABQ–Santa Fe flight experiment')
    parser.add_argument('--output', type=Path, default=Path('food_delivery/artifacts'))
    parser.add_argument('--seed', type=int, default=7)
    parser.add_argument('--origin', type=float, nargs=2, metavar=('LAT', 'LON'))
    parser.add_argument('--destination', type=float, nargs=2, metavar=('LAT', 'LON'))
    parser.add_argument('--duration', type=float, default=2400)
    parser.add_argument('--outage', type=float, nargs=2, default=(450, 750), metavar=('START', 'END'))
    parser.add_argument('--no-video', action='store_true')
    parser.add_argument('--evaluate', action='store_true', help='Run baseline, outage, and repeatability checks')
    args = parser.parse_args()
    scenario = replace(Scenario(), seed=args.seed, duration_s=args.duration,
                       outage_start_s=args.outage[0], outage_end_s=args.outage[1])
    if args.origin:
        scenario = replace(scenario, origin=tuple(args.origin))
    if args.destination:
        scenario = replace(scenario, destination=tuple(args.destination))
    try:
        scenario.validate()
    except ValueError as exc:
        parser.error(str(exc))
    args.output.mkdir(parents=True, exist_ok=True)
    result = run(scenario)
    (args.output / 'flight.json').write_text(json.dumps(result, indent=2, allow_nan=False), encoding='utf-8')
    from .rendering import render
    render(result, args.output, video=not args.no_video)
    print(json.dumps(result['metrics'], indent=2, allow_nan=False))
    if args.evaluate:
        baseline = run(replace(scenario, outage_start_s=0, outage_end_s=0))
        repeat = run(scenario)
        other = run(replace(scenario, seed=scenario.seed + 1))
        m = result['metrics']
        checks = {
            'arrival': m['success'],
            'alternate_seed_arrival': other['metrics']['success'],
            'gps_baseline_arrival': baseline['metrics']['success'],
            'altitude_within_100m': m['max_altitude_error_m'] < 100,
            'gps_baseline_rmse_under_30m': baseline['metrics']['position_rmse_m'] < 30,
            'outage_drift_visible': m['max_outage_error_m'] > 50,
            'gps_recovery_under_30m': m['recovery_mean_error_m'] is not None and m['recovery_mean_error_m'] < 30,
            'seed_repeatable': result['samples'] == repeat['samples'],
        }
        evaluation = dict(checks=checks, passed=all(checks.values()), outage=m,
                          gps_baseline=baseline['metrics'], alternate_seed=other['metrics'])
        (args.output / 'evaluation.json').write_text(json.dumps(evaluation, indent=2, allow_nan=False), encoding='utf-8')
        print(json.dumps(evaluation, indent=2))
        return 0 if evaluation['passed'] else 1
    return 0 if result['metrics']['success'] else 1


if __name__ == '__main__':
    raise SystemExit(main())

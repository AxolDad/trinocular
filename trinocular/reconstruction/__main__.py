"""python -m trinocular.reconstruction reconstruct MANIFEST OUTPUT"""
import argparse
import json
from pathlib import Path

from .benchmark import repeatability
from .pipeline import read_json, reconstruct, write_json


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='command', required=True)
    run = sub.add_parser('reconstruct', help='Fuse calibrated, foreground-masked observations')
    run.add_argument('manifest')
    run.add_argument('output')
    repeat = sub.add_parser('repeatability', help='Compare three or more independent benchmarked runs')
    repeat.add_argument('output')
    repeat.add_argument('reports', nargs='+')
    args = parser.parse_args()
    try:
        if args.command == 'reconstruct':
            report = reconstruct(args.manifest, args.output)
            print(json.dumps({'output': args.output, 'data_kind': report['data_kind'],
                              'points': report['fused_points'], 'triangles': report['mesh_triangles']}))
        else:
            output = Path(args.output)
            if output.exists():
                raise ValueError('Output already exists')
            report = repeatability([read_json(p) for p in args.reports])
            write_json(output, report)
            print(json.dumps(report))
    except (ValueError, KeyError, OSError) as error:
        parser.exit(2, f'Reconstruction error: {error}\n')


if __name__ == '__main__':
    main()

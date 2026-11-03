"""Object setup/coverage previews and reviewed full reconstruction."""
import argparse
import json
from pathlib import Path

from .benchmark import repeatability
from .pipeline import read_json, reconstruct, write_json
from .preview import approve_preview, coverage_preview, preflight


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='command', required=True)
    run = sub.add_parser('reconstruct', help='Fuse calibrated, foreground-masked observations')
    run.add_argument('manifest')
    run.add_argument('output')
    run.add_argument('--review', required=True, help='Accepted coverage review.json')
    for name, help_text in [('setup-preview', 'Inspect one view for focus, framing, and distance'),
                            ('coverage-preview', 'Build a coarse model from one exposure per angle')]:
        command = sub.add_parser(name, help=help_text)
        command.add_argument('manifest')
        command.add_argument('output')
    approve = sub.add_parser('approve-preview', help='Record coverage acceptance after viewing the model')
    approve.add_argument('preview_directory')
    approve.add_argument('--note', required=True)
    repeat = sub.add_parser('repeatability', help='Compare three or more independent benchmarked runs')
    repeat.add_argument('output')
    repeat.add_argument('reports', nargs='+')
    args = parser.parse_args()
    try:
        if args.command == 'reconstruct':
            report = reconstruct(args.manifest, args.output, args.review)
            print(json.dumps({'output': args.output, 'data_kind': report['data_kind'],
                              'points': report['fused_points'], 'triangles': report['mesh_triangles']}))
        elif args.command in ('setup-preview', 'coverage-preview'):
            builder = preflight if args.command == 'setup-preview' else coverage_preview
            report = builder(args.manifest, args.output)
            print(json.dumps({'output': args.output, 'mode': report['mode'], 'data_kind': report['data_kind']}))
        elif args.command == 'approve-preview':
            print(approve_preview(args.preview_directory, args.note))
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

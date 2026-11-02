"""Single-view setup check and bounded one-exposure-per-angle 3D preview.

These consume stored observations. A physical first-frame-only shutter still
depends on identifying Pro2 firmware controls and native calibration.
"""
from datetime import datetime, timezone
from pathlib import Path
import json
import os
import tempfile

import numpy as np
import open3d as o3d
from PIL import Image
from scipy.ndimage import gaussian_filter, laplace
from scipy.spatial import cKDTree

from . import color, geometry
from .pipeline import digest, observation, read_json, validate_metric_inputs, write_json


def view_key(item):
    key = (item.get('angle_id', item['id']), item.get('sensor_id', 'primary'))
    if not all(isinstance(k, str) and k for k in key):
        raise ValueError('Angle and sensor IDs must be nonempty strings')
    return key


def representatives(items):
    """First exposure per physical angle and active sensor, in capture order."""
    chosen = {}
    for order, item in enumerate(items):
        index = item.get('frame_index', 0)
        if not isinstance(index, int) or isinstance(index, bool) or index < 0:
            raise ValueError('frame_index must be a nonnegative integer')
        key = view_key(item)
        if key not in chosen or index < chosen[key][0]:
            chosen[key] = (index, order, item)
    return [entry[2] for entry in chosen.values()]


def dataset_fingerprint(manifest_path):
    """Bind review to every reconstruction input, including unused brackets."""
    import hashlib
    path = Path(manifest_path).resolve()
    config = read_json(path)
    inputs = {item['path'] for item in config['observations']}
    inputs |= {item['color_profile'] for item in config['observations'] if item.get('color_profile')}
    if config.get('reference_surface'):
        inputs.add(config['reference_surface']['path'])
    if config.get('preflight', {}).get('image'):
        inputs.add(config['preflight']['image'])
    records = [(name, digest(path.parent / name)) for name in sorted(inputs)]
    encoded = json.dumps([digest(path), records], separators=(',', ':')).encode()
    return hashlib.sha256(encoded).hexdigest()


def focus_metrics(rgb, roi, minimum=None):
    """Relative image-detail score on an explicit subject ROI, not autofocus."""
    x = np.asarray(rgb)
    if x.ndim != 3 or x.shape[2] != 3 or x.dtype != np.uint8:
        raise ValueError('Focus check expects an 8-bit RGB display image')
    if len(roi) != 4 or not all(isinstance(v, int) and not isinstance(v, bool) for v in roi):
        raise ValueError('Focus ROI must contain four integer pixel coordinates')
    left, top, right, bottom = roi
    if not (0 <= left < right <= x.shape[1] and 0 <= top < bottom <= x.shape[0]):
        raise ValueError('Focus ROI falls outside the image')
    if min(right - left, bottom - top) < 16:
        raise ValueError('Focus ROI must be at least 16 by 16 pixels')
    crop = x[top:bottom, left:right]
    gray = crop.astype(float) @ np.array([.2126, .7152, .0722])
    # Mild fixed smoothing reduces noise sensitivity; comparisons require the
    # same scene, resolution, ROI, lighting, and processing.
    score = float(np.var(laplace(gaussian_filter(gray, .5))[2:-2, 2:-2]))
    if minimum is not None and (not np.isfinite(minimum) or minimum <= 0):
        raise ValueError('Calibrated focus threshold must be finite and positive')
    return {'score': score, 'method': 'smoothed display-luminance Laplacian variance',
            'roi_pixels': list(roi), 'minimum_calibrated_score': minimum,
            'status': 'manual_review' if minimum is None else ('below_threshold' if score < minimum else 'above_threshold'),
            'dark_pixel_fraction': float(np.mean(np.max(crop, axis=2) <= 2)),
            'clipped_channel_pixel_fraction': float(np.mean(np.any(crop >= 253, axis=2)))}


def preflight(manifest_path, output):
    """Check only the first observation/image. No sweep or dense render occurs."""
    path, output = Path(manifest_path).resolve(), Path(output).resolve()
    config = read_json(path)
    if config.get('schema_version') != 1 or config.get('data_kind') not in ('hardware', 'synthetic'):
        raise ValueError('Declare schema_version=1 and data_kind')
    if output.exists():
        raise ValueError('Output already exists')
    selected = representatives(config['observations'])
    if not selected:
        raise ValueError('Setup preview requires a first observation')
    first = selected[0]
    settings = config.get('preflight', {})
    if settings.get('observation_id') != first['id']:
        raise ValueError('Setup image must identify the first selected observation')
    image_path = path.parent / settings['image']
    with Image.open(image_path) as image:
        rgb = np.asarray(image.convert('RGB'))
    focus = focus_metrics(rgb, settings['subject_roi_pixels'], settings.get('minimum_focus_score'))
    calibration = config['calibration']
    metric = config['data_kind'] == 'synthetic' or (
        calibration.get('metric_scale_verified') is True and calibration.get('projection_verified') is True
        and bool(calibration.get('evidence')))
    distance = {'status': 'calibration_required'}
    if metric:
        # Color profiling is unnecessary for a distance check.
        with np.load(path.parent / first['path'], allow_pickle=False) as sample:
            points, mask = np.asarray(sample['points'], dtype=float), sample['foreground']
        if points.ndim != 2 or points.shape[1] != 3 or mask.dtype != bool or mask.shape != (len(points),):
            raise ValueError('Invalid foreground points for preflight')
        if first['units'] not in ('m', 'mm') or not np.isfinite(points).all() or not mask.any():
            raise ValueError('Distance check requires finite foreground points and explicit units')
        points = points[mask] * (1 if first['units'] == 'm' else .001)
        ranges = np.linalg.norm(points, axis=1)
        limits = np.asarray(settings['working_distance_m'], dtype=float)
        if limits.shape != (2,) or not np.isfinite(limits).all() or not 0 < limits[0] < limits[1]:
            raise ValueError('Provide calibrated minimum/maximum working distance')
        within = (ranges >= limits[0]) & (ranges <= limits[1])
        distance = {'status': 'within_range' if within.all() else 'outside_range',
                    'minimum_m': float(ranges.min()), 'median_m': float(np.median(ranges)),
                    'maximum_m': float(ranges.max()), 'in_range_fraction': float(within.mean()),
                    'working_distance_m': limits.tolist(), 'definition': 'range from sensor origin to foreground samples'}
    report = {'schema_version': 1, 'mode': 'setup', 'data_kind': config['data_kind'],
              'observation_id': first['id'], 'image_sha256': digest(image_path),
              'focus': focus, 'distance': distance, 'review_required': True,
              'single_frame_camera_command_verified': False}
    output.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix='.setup-', dir=output.parent) as temp:
        dest = Path(temp) / 'result'
        dest.mkdir()
        Image.fromarray(rgb).save(dest / 'first-frame.png')
        l, t, r, b = focus['roi_pixels']
        Image.fromarray(rgb[t:b, l:r]).save(dest / 'focus-crop.png')
        write_json(dest / 'setup.json', report)
        import html
        distance_text = ('Distance needs calibration' if not metric else
                         f"Subject range: {distance['minimum_m']:.3f}–{distance['maximum_m']:.3f} m; {distance['status'].replace('_', ' ')}")
        page = ('<!doctype html><meta charset="utf-8"><title>Setup preview</title>'
                '<style>body{background:#11131a;color:#eee;font:17px system-ui;max-width:1100px;margin:32px auto;padding:20px}'
                'img{max-width:100%;border:1px solid #4e4261}a{color:#c5a0ff}.crop{max-width:none}</style>'
                '<h1>Setup preview</h1>'
                + ('<p>Generated test image</p>' if config['data_kind'] == 'synthetic' else '') +
                '<p>Check framing, detail, and working distance before continuing.</p>'
                f'<p>{html.escape(distance_text)}</p><p>Focus: {html.escape(focus["status"].replace("_", " "))}</p>'
                '<a href="first-frame.png" target="_blank" rel="noopener">Open full-size image</a>'
                '<p><img src="first-frame.png" alt="First view for framing review"></p>'
                '<h2>Subject detail at original pixel scale</h2><div style="overflow:auto">'
                '<img class="crop" src="focus-crop.png" alt="Full-resolution subject detail"></div>')
        (dest / 'index.html').write_text(page)
        os.rename(dest, output)
    return report


def _boundaries(mesh):
    faces = np.asarray(mesh.triangles)
    edges = np.sort(np.concatenate([faces[:, [0, 1]], faces[:, [1, 2]], faces[:, [2, 0]]]), axis=1)
    unique, counts = np.unique(edges, axis=0, return_counts=True)
    return unique[counts == 1]


def coverage_preview(manifest_path, output):
    path, output = Path(manifest_path).resolve(), Path(output).resolve()
    config = read_json(path)
    validate_metric_inputs(config)
    if output.exists():
        raise ValueError('Output already exists')
    original = config['observations']
    if len({i['id'] for i in original}) != len(original):
        raise ValueError('Observation IDs must be unique')
    items = representatives(original)
    angle_ids = list(dict.fromkeys(view_key(i)[0] for i in items))
    if len(angle_ids) < 2:
        raise ValueError('Coverage preview needs at least two distinct object angles')
    settings = config.get('preview', {})
    voxel = float(settings.get('voxel_m', config['reconstruction']['voxel_m'] * 2))
    full_voxel = float(config['reconstruction'].get('fusion_voxel_m', config['reconstruction']['voxel_m'] / 2))
    if not np.isfinite([voxel, full_voxel]).all() or full_voxel <= 0 or voxel <= full_voxel:
        raise ValueError('Preview voxel must be coarser than full-resolution fusion')
    maximum = int(settings.get('max_points_per_angle', 5000))
    total_budget = int(settings.get('max_points_total', 60000))
    if not 100 <= maximum <= 20000 or not 200 <= total_budget <= 200000:
        raise ValueError('Preview point budget is out of bounds')
    if len(items) > int(config['reconstruction'].get('max_passes', 32)):
        raise ValueError('Preview exceeds the configured all-pairs angle budget')
    maximum = min(maximum, total_budget // len(items))
    if maximum < 100:
        raise ValueError('Too many angles for the total preview budget')
    start_fingerprint = dataset_fingerprint(path)
    loaded = []
    for item in items:
        p, rgb, normals, seed, _, _ = observation(path.parent, item)
        index = np.linspace(0, len(p) - 1, min(maximum, len(p)), dtype=int)
        loaded.append((p[index], rgb[index], None if normals is None else normals[index], seed))
    clouds = [geometry.prepare(p, n, voxel) for p, _, n, _ in loaded]
    o3d.utility.random.seed(int(config['reconstruction'].get('random_seed', 0)))
    poses, registration = geometry.align(clouds, voxel, [x[3] for x in loaded],
        min_overlap=float(settings.get('min_overlap', config['reconstruction'].get('min_overlap', .35))),
        max_rmse_m=float(settings.get('max_registration_rmse_m', voxel)))
    transformed = [geometry.transform_points(x[0], pose) for x, pose in zip(loaded, poses)]
    angle_numbers = [angle_ids.index(view_key(i)[0]) for i in items]
    points, rgb, views = geometry.fuse(np.concatenate(transformed), np.concatenate([x[1] for x in loaded]),
        np.concatenate([np.full(len(x[0]), a) for x, a in zip(loaded, angle_numbers)]), voxel)
    mesh = geometry.mesh_observations(points, rgb, voxel)
    if not len(mesh.triangles):
        raise ValueError('Preview could not produce a surface; acquire more views or adjust coarse sampling')
    if len(mesh.triangles) > 12000:
        mesh = mesh.simplify_quadric_decimation(12000)
        mesh.remove_unreferenced_vertices()
    vertices = np.asarray(mesh.vertices)
    support = views[cKDTree(points).query(vertices)[1]]
    boundaries = _boundaries(mesh)
    report = {'schema_version': 1, 'mode': 'coverage', 'data_kind': config['data_kind'],
        'created_utc': datetime.now(timezone.utc).isoformat(), 'dataset_fingerprint': start_fingerprint,
        'selected_observations': [i['id'] for i in items], 'total_observations': len(original),
        'angle_count': len(angle_ids), 'selection': 'first exposure per angle and active sensor',
        'voxel_m': voxel, 'processed_samples': sum(len(x[0]) for x in loaded),
        'fused_points': len(points), 'mesh_triangles': len(mesh.triangles),
        'open_boundary_edges': len(boundaries), 'single_angle_voxel_fraction': float(np.mean(views == 1)),
        'registration': registration, 'coverage_status': 'operator_review_required',
        'coverage_complete_verified': False, 'measurement_accuracy_verified': False,
        'pose_groups': [{'angle_id': view_key(i)[0], 'sensor_id': view_key(i)[1], 'pose': p.tolist()}
                        for i, p in zip(items, poses)]}
    # If an input changed during preview construction, repeat the preview rather
    # than binding an approval to a mixture of dataset revisions.
    if dataset_fingerprint(path) != start_fingerprint:
        raise ValueError('Inputs changed while building preview; run it again')
    output.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix='.coverage-', dir=output.parent) as temp:
        dest = Path(temp) / 'result'
        dest.mkdir()
        write_json(dest / 'preview.json', report)
        np.savez_compressed(dest / 'preview.npz', points=points, colors=rgb, independent_angle_count=views)
        if not o3d.io.write_triangle_mesh(str(dest / 'preview.ply'), mesh):
            raise ValueError('Failed writing preview mesh')
        model = {'vertices': np.round(vertices, 7).tolist(), 'triangles': np.asarray(mesh.triangles).tolist(),
                 'colors': np.rint(color.linear_to_srgb(rgb[cKDTree(points).query(vertices)[1]]) * 255).astype(int).tolist(),
                 'support': support.tolist(), 'boundaries': boundaries[:5000].tolist()}
        data = {'model': model, 'report': report, 'preview_report_sha256': digest(dest / 'preview.json')}
        template = Path(__file__).with_name('preview.html').read_text()
        embedded = json.dumps(data, separators=(',', ':'), allow_nan=False).replace('<', '\\u003c')
        (dest / 'index.html').write_text(template.replace('__PREVIEW_DATA__', embedded))
        os.rename(dest, output)
    return report


def approve_preview(preview_dir, note):
    """Record the operator's explicit coverage decision after inspecting the view."""
    directory = Path(preview_dir).resolve()
    report_path = directory / 'preview.json'
    report = read_json(report_path)
    if report.get('mode') != 'coverage' or not note.strip():
        raise ValueError('A coverage preview and operator review note are required')
    target = directory / 'review.json'
    if target.exists():
        raise ValueError('Review already exists; keep each decision immutable')
    review = {'schema_version': 1, 'decision': 'approved', 'coverage_accepted_by_operator': True,
              'created_utc': datetime.now(timezone.utc).isoformat(), 'note': note.strip(),
              'dataset_fingerprint': report['dataset_fingerprint'],
              'preview_report_path': 'preview.json', 'preview_report_sha256': digest(report_path)}
    write_json(target, review)
    return target


def validate_review(manifest_path, review_path):
    if review_path is None:
        raise ValueError('Review the coverage preview before full reconstruction; provide --review review.json')
    review_path = Path(review_path).resolve()
    review = read_json(review_path)
    report_path = review_path.parent / review['preview_report_path']
    report = read_json(report_path)
    if (review.get('schema_version') != 1 or review.get('decision') != 'approved'
            or review.get('coverage_accepted_by_operator') is not True or not review.get('note', '').strip()
            or report.get('mode') != 'coverage' or report.get('angle_count', 0) < 2):
        raise ValueError('An explicit accepted coverage review is required')
    if digest(report_path) != review['preview_report_sha256']:
        raise ValueError('Preview report changed after review')
    current = dataset_fingerprint(manifest_path)
    if current != review['dataset_fingerprint'] or current != report['dataset_fingerprint']:
        raise ValueError('Preview is stale: inputs or settings changed; rebuild and review coverage')
    return {**review, 'review_sha256': digest(review_path)}, report

"""Calibrated NPZ observations -> poses, fused linear master, mesh, QA report."""
from datetime import datetime, timezone
import hashlib
import importlib.metadata
import json
import os
from pathlib import Path
import tempfile

import numpy as np
import open3d as o3d

from . import benchmark, color, geometry


def read_json(path):
    def invalid(value):
        raise ValueError(f"Non-finite JSON constant: {value}")
    return json.loads(Path(path).read_text(), parse_constant=invalid)


def write_json(path, value):
    Path(path).write_text(json.dumps(value, indent=2, allow_nan=False) + '\n')


def digest(path):
    h = hashlib.sha256()
    with open(path, 'rb') as f:
        for chunk in iter(lambda: f.read(1024 * 1024), b''):
            h.update(chunk)
    return h.hexdigest()


def observation(root, item):
    path = root / item['path']
    unit = item['units']
    if unit not in ('m', 'mm'):
        raise ValueError("Declare each observation's units explicitly: m or mm")
    with np.load(path, allow_pickle=False) as data:
        points = np.asarray(data['points'], dtype=float) * (1 if unit == 'm' else .001)
        rgb = np.asarray(data['colors'], dtype=float)
        mask = data['foreground']
        normals = np.asarray(data['normals'], dtype=float) if 'normals' in data else None
    if points.ndim != 2 or points.shape[1] != 3 or rgb.shape != points.shape:
        raise ValueError("Point and color arrays must be matching Nx3")
    if mask.shape != (len(points),) or mask.dtype != bool:
        raise ValueError("Explicit boolean foreground mask required for every observation")
    if not np.isfinite(points).all() or not np.isfinite(rgb).all() or np.any(rgb < 0):
        raise ValueError("Samples must be finite; colors must be nonnegative")
    if normals is not None:
        if normals.shape != points.shape or not np.isfinite(normals).all():
            raise ValueError("Invalid normal array")
        if np.any(np.linalg.norm(normals[mask], axis=1) < 1e-8):
            raise ValueError("Foreground normals must have nonzero length")
        normals = normals[mask]
    if np.count_nonzero(mask) < 30:
        raise ValueError("Too few foreground samples")
    rgb, points = rgb[mask], points[mask]
    encoding = item['color_encoding']
    if encoding == 'srgb':
        rgb = color.srgb_to_linear(rgb)
    elif encoding not in ('linear_srgb', 'camera_linear'):
        raise ValueError("Declare srgb, linear_srgb, or camera_linear encoding")
    quality = None
    profile = item.get('color_profile')
    if profile:
        if encoding == 'srgb':
            raise ValueError("Chart profile expects linear input")
        profile = read_json(root / profile)
        matrix = color.fit_chart(profile['training_observed_linear'], profile['training_reference_linear_srgb'])
        rgb = rgb @ matrix
        quality = color.color_error(np.asarray(profile['validation_observed_linear']) @ matrix,
                                    profile['validation_reference_linear_srgb'])
        quality['profile_sha256'] = digest(root / item['color_profile'])
        quality['matrix_row_vector_convention'] = matrix.tolist()
    elif encoding == 'camera_linear':
        raise ValueError("Camera-linear samples require a measured color profile")
    seed = item.get('object_from_sensor')
    if 'markers' in item:
        if seed is not None:
            raise ValueError("Supply marker correspondences or a pose seed, not both")
        markers = item['markers']
        seed = geometry.from_correspondences(markers['sensor_points_m'], markers['object_points_m'])
    elif seed is not None:
        seed = geometry.rigid_transform(seed)
    return points, rgb, normals, seed, quality, digest(path)


def validate_metric_inputs(config):
    if config.get('schema_version') != 1 or config.get('data_kind') not in ('synthetic', 'hardware'):
        raise ValueError("Declare schema_version=1 and data_kind=synthetic or hardware")
    calibration = config['calibration']
    if config['data_kind'] == 'hardware' and not (
        calibration.get('metric_scale_verified') is True and
        calibration.get('projection_verified') is True and calibration.get('evidence')):
        raise ValueError("Hardware inputs require verified metric scale, projection, and calibration evidence")


def reconstruct(manifest_path, output, review_path=None):
    manifest_path, output = Path(manifest_path).resolve(), Path(output).resolve()
    config = read_json(manifest_path)
    root = manifest_path.parent
    validate_metric_inputs(config)
    calibration = config['calibration']
    if output.exists():
        raise ValueError("Output already exists; select a new run directory")
    # Coverage review is part of the requested capture workflow. Verification
    # happens before loading dense observations or starting full reconstruction.
    from .preview import validate_review, view_key
    coverage_review, preview = validate_review(manifest_path, review_path)
    settings = config['reconstruction']
    voxel = float(settings['voxel_m'])
    fusion_voxel = float(settings.get('fusion_voxel_m', voxel / 2))
    if not np.isfinite([voxel, fusion_voxel]).all() or min(voxel, fusion_voxel) <= 0:
        raise ValueError("Voxel sizes must be finite and positive")
    items = config['observations']
    names = [i['id'] for i in items]
    if len(names) < 2 or len(set(names)) != len(names):
        raise ValueError("At least two observations with unique IDs required")
    # A limit makes accidental all-pairs costs visible before a long run.
    if len(items) > int(settings.get('max_passes', 32)):
        raise ValueError("Pass count exceeds configured all-pairs registration budget")
    loaded = [observation(root, item) for item in items]
    clouds = [geometry.prepare(p, n, voxel) for p, _, n, _, _, _ in loaded]
    o3d.utility.random.seed(int(settings.get('random_seed', 0)))
    reviewed_poses = {(p['angle_id'], p['sensor_id']): geometry.rigid_transform(p['pose'])
                      for p in preview['pose_groups']}
    seeds = [reviewed_poses.get(view_key(item), x[3]) for item, x in zip(items, loaded)]
    poses, registration = geometry.align(clouds, voxel, seeds,
        min_overlap=float(settings.get('min_overlap', .35)),
        max_rmse_m=float(settings.get('max_registration_rmse_m', voxel)))
    transformed = [geometry.transform_points(x[0], pose) for x, pose in zip(loaded, poses)]
    points, rgb, views = geometry.fuse(np.concatenate(transformed),
        np.concatenate([x[1] for x in loaded]),
        np.concatenate([np.full(len(x[0]), i) for i, x in enumerate(loaded)]), fusion_voxel)
    dimensional = None
    if 'benchmark' in config:
        dimensional = benchmark.dimensions(points, config['benchmark'], calibration.get('artifact_ids', []))
    mesh = geometry.mesh_observations(points, rgb, fusion_voxel)
    if not len(mesh.triangles):
        raise ValueError("No mesh triangles reconstructed; check sampling and voxel sizes")
    surface = None
    if 'reference_surface' in config:
        ref = config['reference_surface']
        if not ref['artifact_id'] or ref['artifact_id'] in calibration.get('artifact_ids', []):
            raise ValueError("Reference surface must be independent of calibration")
        with np.load(root / ref['path'], allow_pickle=False) as data:
            ref_points = np.asarray(data['points'], dtype=float)
        # Reference coordinates are already metres in the first observation frame.
        surface = benchmark.surface_error(points, ref_points)
        surface['reference_sha256'] = digest(root / ref['path'])
    report = {
        'schema_version': 1, 'created_utc': datetime.now(timezone.utc).isoformat(),
        'data_kind': config['data_kind'], 'hardware_accuracy_verified': False,
        'manifest_sha256': digest(manifest_path), 'input_sha256': [x[5] for x in loaded],
        'repeatability_group': config.get('repeatability_group'),
        'versions': {p: importlib.metadata.version(p) for p in ('open3d', 'numpy', 'scipy', 'colour-science')},
        'calibration': calibration, 'coordinate_frame': 'first observation sensor frame; metres',
        'coverage_review': coverage_review,
        'settings': settings, 'observations': names, 'registration': registration,
        'poses_first_sensor_from_sensor': [p.tolist() for p in poses],
        'color_validation': {item['id']: obs[4] for item, obs in zip(items, loaded)},
        'dimension_benchmark': dimensional, 'surface_benchmark': surface,
        'fused_points': len(points), 'mesh_vertices': len(mesh.vertices),
        'mesh_triangles': len(mesh.triangles), 'mesh_watertight': mesh.is_watertight(),
        'single_view_voxel_fraction': float(np.mean(views == 1)),
        'point_bounds_extent_mm': ((points.max(0) - points.min(0)) * 1000).tolist(),
        'review_required': ['Object alignment, especially symmetry and flipped surfaces',
                            'Surface coverage and open boundaries',
                            'Independent dimensional and color validation'],
    }
    output.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix='.reconstruction-', dir=output.parent) as tmp:
        dest = Path(tmp) / 'result'
        dest.mkdir()
        np.savez_compressed(dest / 'linear-master.npz', points=points, colors=rgb, independent_pass_count=views)
        sample_cloud = geometry.cloud(points)
        sample_cloud.colors = o3d.utility.Vector3dVector(color.linear_to_srgb(rgb))
        for filename, obj, writer in [('observations.ply', sample_cloud, o3d.io.write_point_cloud),
                                      ('mesh.ply', mesh, o3d.io.write_triangle_mesh)]:
            if not writer(str(dest / filename), obj):
                raise ValueError(f"Failed writing {filename}")
        # STL numeric coordinates use millimetres; PLY/NPZ keep SI metres.
        mesh_mm = o3d.geometry.TriangleMesh(mesh)
        mesh_mm.scale(1000, center=(0, 0, 0))
        if not o3d.io.write_triangle_mesh(str(dest / 'mesh.stl'), mesh_mm):
            raise ValueError("Failed writing millimetre STL")
        report['export_units'] = {'mesh.stl': 'mm', 'mesh.ply': 'm', 'observations.ply': 'm', 'linear-master.npz': 'm'}
        write_json(dest / 'report.json', report)
        write_json(dest / 'manifest.snapshot.json', config)
        os.rename(dest, output)
    return report

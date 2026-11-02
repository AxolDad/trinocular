"""Generate synthetic inputs with known dimensions; never a Pro2 accuracy test.

Run from the repository root:
  python examples/reconstruction_demo.py /tmp/trinocular-demo
  python -m trinocular.reconstruction reconstruct /tmp/trinocular-demo/manifest.json /tmp/trinocular-result
"""
import json
from pathlib import Path
import sys

import numpy as np
from scipy.spatial.transform import Rotation


def generate(destination, seed=17):
    root = Path(destination)
    root.mkdir(parents=True, exist_ok=True)
    rng = np.random.default_rng(seed)
    centers = np.array([[0, 0, .65], [.09, 0, .65], [.035, .07, .67]])
    radii = [.018, .023, .015]
    samples, normals = [], []
    for center, radius in zip(centers, radii):
        n = rng.normal(size=(1600, 3))
        n /= np.linalg.norm(n, axis=1, keepdims=True)
        samples.append(center + radius * n)
        normals.append(n)
    points, normals = np.concatenate(samples), np.concatenate(normals)
    truth_color = np.clip(.25 + (points - points.min(0)) / np.ptp(points, axis=0) * .45, 0, 1)
    response = np.diag([.8, 1.1, .9])
    train, validation = rng.uniform(.05, .8, (24, 3)), rng.uniform(.05, .8, (12, 3))
    profile = {'training_observed_linear': (train @ response).tolist(),
               'training_reference_linear_srgb': train.tolist(),
               'validation_observed_linear': (validation @ response).tolist(),
               'validation_reference_linear_srgb': validation.tolist()}
    (root / 'profile.json').write_text(json.dumps(profile, indent=2) + '\n')
    observations = []
    for i, angle in enumerate([0, 75, -75]):
        pose = np.eye(4)
        pose[:3, :3] = Rotation.from_euler('y', angle, degrees=True).as_matrix()
        pose[:3, 3] = [.01 * i, -.02 * i, .03 * i]
        visible = (normals @ Rotation.from_euler('y', angle, degrees=True).apply([0, 0, -1])) > -.35
        local = (points[visible] - pose[:3, 3]) @ pose[:3, :3]
        local += rng.normal(scale=.00004, size=local.shape)
        local_normals = normals[visible] @ pose[:3, :3]
        # Add an explicit stationary-background sample: the foreground mask must
        # exclude it even when its position repeats in every acquisition.
        points_with_background = np.vstack([local, [1, 1, 2]])
        np.savez_compressed(root / f'pass-{i}.npz', points=points_with_background,
            colors=np.vstack([truth_color[visible] @ response, [.1, .1, .1]]),
            normals=np.vstack([local_normals, [0, 0, -1]]),
            foreground=np.r_[np.ones(len(local), dtype=bool), False])
        guess = pose.copy()
        if i:
            guess[:3, 3] += [.0004, -.0003, .0002]
        observations.append({'id': f'orientation-{i}', 'path': f'pass-{i}.npz',
                             'units': 'm', 'color_encoding': 'camera_linear',
                             'color_profile': 'profile.json', 'object_from_sensor': guess.tolist()})
    manifest = {'schema_version': 1, 'data_kind': 'synthetic',
        'repeatability_group': 'synthetic-fixture-same-conditions',
        'calibration': {'metric_scale_verified': False, 'projection_verified': False,
                        'artifact_ids': ['synthetic-training-color-chart'],
                        'evidence': 'Generated coordinates and channel response; no camera involved'},
        'reconstruction': {'voxel_m': .002, 'fusion_voxel_m': .0008,
                           'min_overlap': .3, 'max_registration_rmse_m': .0015, 'random_seed': seed},
        'observations': observations,
        'benchmark': {'artifact_id': 'synthetic-independent-three-ball-fixture',
            'spheres': [{'id': f'ball-{i}', 'roi_center_m': c.tolist(), 'roi_radius_m': r + .004}
                        for i, (c, r) in enumerate(zip(centers, radii))],
            'distances': [{'id': f'ball-0-to-{i}', 'from': 'ball-0', 'to': f'ball-{i}',
                           'reference_mm': float(np.linalg.norm(centers[0] - centers[i]) * 1000),
                           'tolerance_mm': .15, 'reference_expanded_uncertainty_mm': 0}
                          for i in (1, 2)]}}
    (root / 'manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
    return root / 'manifest.json'


if __name__ == '__main__':
    print(generate(sys.argv[1]))

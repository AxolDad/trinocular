import copy
import importlib.util
import json
from pathlib import Path

import numpy as np
import open3d as o3d
import pytest
from scipy.spatial.transform import Rotation

from trinocular.reconstruction import benchmark, color, geometry
from trinocular.reconstruction.pipeline import observation, reconstruct, write_json
from trinocular.reconstruction.preview import approve_preview, coverage_preview

DEMO = Path(__file__).resolve().parents[1] / 'examples/reconstruction_demo.py'
spec = importlib.util.spec_from_file_location('demo', DEMO)
demo = importlib.util.module_from_spec(spec)
spec.loader.exec_module(demo)


def test_srgb_roundtrip_and_chart_validation():
    rng = np.random.default_rng(1)
    reference = rng.uniform(.05, .8, (30, 3))
    response = np.array([[.8, .01, .02], [.02, 1.1, .01], [.01, .01, .9]])
    observed = reference @ response
    profile = color.fit_chart(observed[:20], reference[:20])
    error = color.color_error(observed[20:] @ profile, reference[20:])
    assert error['maximum'] < 1e-8
    x = np.linspace(0, 1, 20)
    np.testing.assert_allclose(color.linear_to_srgb(color.srgb_to_linear(x)), x, atol=1e-12)


def test_hdr_recovers_radiance_and_flags_clipping():
    true = np.array([[.3, .8, 2.0], [0, 100, .2]])
    times = np.array([.1, .5, 1])
    frames = np.clip(times[:, None, None] * true, 0, 1)
    radiance, valid = color.merge_linear_hdr(frames, times)
    np.testing.assert_allclose(radiance[valid], true[valid])
    assert not valid[1, 0] and not valid[1, 1]
    assert np.isnan(radiance[~valid]).all()
    with pytest.raises(ValueError):
        color.merge_linear_hdr(frames, [0, .5, 1])


def test_rigid_markers_and_scale_rejection():
    source = np.array([[0, 0, 0], [1, 0, 0], [0, 1, 0], [0, 0, 1.]])
    pose = np.eye(4)
    pose[:3, :3] = Rotation.from_euler('xyz', [20, 35, -70], degrees=True).as_matrix()
    pose[:3, 3] = [.02, -.03, .7]
    target = geometry.transform_points(source, pose)
    np.testing.assert_allclose(geometry.from_correspondences(source, target), pose, atol=1e-12)
    bad = pose.copy()
    bad[:3, :3] *= 1.01
    with pytest.raises(ValueError, match='scaling'):
        geometry.rigid_transform(bad)
    with pytest.raises(ValueError, match='collinear'):
        geometry.from_correspondences(source[:3] * [1, 0, 0], target[:3] * [1, 0, 0])


def test_fusion_blends_in_linear_light_and_counts_views():
    points = np.array([[.001, 0, 0], [.002, 0, 0], [.003, 0, 0]])
    colors = color.srgb_to_linear(np.array([[0, 0, 0], [1, 1, 1], [1, 1, 1.]]))
    p, c, support = geometry.fuse(points, colors, np.array([0, 0, 1]), .01)
    np.testing.assert_allclose(c, [[2 / 3] * 3])
    assert support.tolist() == [2]
    assert color.linear_to_srgb(c)[0, 0] > .8


def test_scale_and_mask_are_explicit(tmp_path):
    rng = np.random.default_rng(9)
    points = rng.uniform(0, 10, (35, 3))
    np.savez(tmp_path / 'points.npz', points=points, colors=np.ones((35, 3)) * .5,
             foreground=np.ones(35, dtype=bool))
    item = {'path': 'points.npz', 'units': 'mm', 'color_encoding': 'linear_srgb'}
    loaded = observation(tmp_path, item)
    np.testing.assert_allclose(loaded[0], points / 1000)
    with pytest.raises(ValueError, match='units'):
        observation(tmp_path, {**item, 'units': 'guessed'})
    with pytest.raises(ValueError, match='measured color profile'):
        observation(tmp_path, {**item, 'color_encoding': 'camera_linear'})


def test_sphere_fit_and_reference_error():
    rng = np.random.default_rng(22)
    normals = rng.normal(size=(900, 3))
    normals /= np.linalg.norm(normals, axis=1, keepdims=True)
    center = np.array([.1, .2, .6])
    points = center + .025 * normals
    fit = benchmark.fit_sphere(points)
    np.testing.assert_allclose(fit['center_m'], center, atol=1e-10)
    assert abs(fit['radius_mm'] - 25) < 1e-7
    comparison = benchmark.surface_error(points, points + [.001, 0, 0])
    assert comparison['scan_to_reference']['p95_mm'] > .5


def test_calibration_artifact_cannot_validate():
    with pytest.raises(ValueError, match='independent artifact'):
        benchmark.dimensions(np.zeros((30, 3)), {'artifact_id': 'same'}, ['same'])


def test_end_to_end_seeded_object_reconstruction(tmp_path):
    manifest = demo.generate(tmp_path / 'inputs')
    out = tmp_path / 'result'
    preview = tmp_path / 'preview'
    coverage_preview(manifest, preview)
    review = approve_preview(preview, 'Reviewed the generated test surface')
    report = reconstruct(manifest, out, review)
    assert report['data_kind'] == 'synthetic'
    assert report['hardware_accuracy_verified'] is False
    assert report['mesh_triangles'] > 100
    assert len(report['registration']) == 3
    assert all(d['within_requested_tolerance'] for d in report['dimension_benchmark']['dimensions'])
    assert max(d['absolute_error_mm'] for d in report['dimension_benchmark']['dimensions']) < .15
    assert max(c['maximum'] for c in report['color_validation'].values()) < 1e-8
    # Explicit masks kept the stationary background sample out of the model.
    with np.load(out / 'linear-master.npz', allow_pickle=False) as data:
        assert data['points'][:, 2].max() < .75
        assert np.max(data['independent_pass_count']) == 3
    ply = o3d.io.read_triangle_mesh(str(out / 'mesh.ply'))
    stl = o3d.io.read_triangle_mesh(str(out / 'mesh.stl'))
    np.testing.assert_allclose(stl.get_max_bound(), ply.get_max_bound() * 1000, atol=1e-3)
    with pytest.raises(ValueError, match='already exists'):
        reconstruct(manifest, out)
    report_paths = [out / 'report.json'] * 3
    with pytest.raises(ValueError, match='same acquisition'):
        benchmark.repeatability([json.loads(p.read_text()) for p in report_paths])


def test_hardware_requires_calibration_evidence(tmp_path):
    manifest = demo.generate(tmp_path / 'inputs')
    config = json.loads(manifest.read_text())
    config['data_kind'] = 'hardware'
    write_json(manifest, config)
    with pytest.raises(ValueError, match='verified metric scale'):
        reconstruct(manifest, tmp_path / 'result')


def test_registration_disconnected_inputs_are_rejected():
    rng = np.random.default_rng(3)
    a = geometry.prepare(rng.normal(scale=.01, size=(500, 3)), None, .002)
    b = geometry.prepare(rng.normal(scale=.1, size=(500, 3)) + [2, 0, 0], None, .002)
    with pytest.raises(ValueError, match='Disconnected'):
        geometry.align([a, b], .002, seeds=[np.eye(4), np.eye(4)])


def test_repeatability_reports_bias_separately_from_spread():
    reports = []
    for i, measured in enumerate([100.1, 100.2, 100.3]):
        reports.append({'manifest_sha256': str(i), 'input_sha256': [str(i)], 'data_kind': 'synthetic',
            'repeatability_group': 'fixture-and-conditions-A',
            'dimension_benchmark': {'artifact_id': 'fixture-A', 'dimensions': [
                {'id': 'length', 'reference_mm': 100, 'tolerance_mm': .5, 'measured_mm': measured}]}})
    summary = benchmark.repeatability(reports)['dimensions'][0]
    assert summary['bias_mm'] == pytest.approx(.2)
    assert summary['sample_stddev_mm'] == pytest.approx(.1)
    reused = copy.deepcopy(reports)
    reused[1]['input_sha256'] = reused[0]['input_sha256']
    with pytest.raises(ValueError, match='independently acquired'):
        benchmark.repeatability(reused)


def test_global_registration_without_pose_seeds(tmp_path):
    path = demo.generate(tmp_path / 'inputs')
    with np.load(path.parent / 'pass-0.npz', allow_pickle=False) as data:
        p, n = data['points'][:-1], data['normals'][:-1]
    pose = np.eye(4)
    pose[:3, :3] = Rotation.from_euler('xyz', [15, -40, 25], degrees=True).as_matrix()
    pose[:3, 3] = [.03, -.02, .015]
    a = geometry.prepare(p, n, .002)
    b = geometry.prepare(geometry.transform_points(p, pose), n @ pose[:3, :3].T, .002)
    o3d.utility.random.seed(4)
    result, _, quality = geometry.pairwise(a, b, .002, None, .6, .001)
    assert quality['accepted']
    np.testing.assert_allclose(result[:3, :3], pose[:3, :3], atol=.002)
    np.testing.assert_allclose(result[:3, 3], pose[:3, 3], atol=.001)


def test_repeatability_allows_identical_configuration_with_new_acquisitions():
    reports = []
    for i in range(3):
        reports.append({'manifest_sha256': 'same-manifest-configuration', 'input_sha256': [str(i)],
            'data_kind': 'synthetic', 'repeatability_group': 'same-conditions',
            'settings': {'voxel_m': .001}, 'calibration': {'evidence': 'fixture-A'},
            'dimension_benchmark': {'artifact_id': 'independent-fixture-B', 'dimensions': [
                {'id': 'length', 'reference_mm': 100, 'tolerance_mm': .5, 'measured_mm': 100 + i * .1}]}})
    assert benchmark.repeatability(reports)['acquisition_count'] == 3
    reports[1]['settings']['voxel_m'] = .002
    with pytest.raises(ValueError, match='differ'):
        benchmark.repeatability(reports)

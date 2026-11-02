import copy
import importlib.util
import json
from pathlib import Path

import numpy as np
import pytest
from scipy.ndimage import gaussian_filter

from trinocular.reconstruction import geometry
from trinocular.reconstruction.pipeline import read_json, reconstruct, write_json
from trinocular.reconstruction.preview import (approve_preview, coverage_preview,
    dataset_fingerprint, focus_metrics, preflight, representatives, validate_review)

spec = importlib.util.spec_from_file_location('preview_demo',
    Path(__file__).resolve().parents[1] / 'examples/reconstruction_demo.py')
demo = importlib.util.module_from_spec(spec)
spec.loader.exec_module(demo)


def test_representatives_select_first_exposure_per_angle_and_sensor():
    items = [
        {'id': 'later-bracket', 'angle_id': 'front', 'sensor_id': 'center', 'frame_index': 2},
        {'id': 'first-bracket', 'angle_id': 'front', 'sensor_id': 'center', 'frame_index': 0},
        {'id': 'upper-sensor', 'angle_id': 'front', 'sensor_id': 'upper', 'frame_index': 0},
        {'id': 'rear-view', 'angle_id': 'rear', 'sensor_id': 'center', 'frame_index': 0}]
    assert [x['id'] for x in representatives(items)] == ['first-bracket', 'upper-sensor', 'rear-view']
    with pytest.raises(ValueError, match='nonnegative'):
        representatives([{'id': 'invalid', 'frame_index': -1}])


def test_focus_score_detects_synthetic_blur_without_claiming_autofocus():
    y, x = np.indices((128, 128))
    sharp = np.repeat((((x // 4 + y // 4) % 2) * 220 + 15)[..., None], 3, axis=2).astype(np.uint8)
    blurred = gaussian_filter(sharp.astype(float), sigma=(4, 4, 0)).astype(np.uint8)
    a = focus_metrics(sharp, [0, 0, 128, 128])
    b = focus_metrics(blurred, [0, 0, 128, 128], a['score'] / 2)
    assert a['score'] > b['score'] * 10
    assert a['status'] == 'manual_review'
    assert b['status'] == 'below_threshold'
    with pytest.raises(ValueError, match='outside'):
        focus_metrics(sharp, [-1, 0, 128, 128])


def test_setup_checks_only_first_view_and_keeps_full_resolution(tmp_path):
    path = demo.generate(tmp_path / 'inputs')
    # Later orientations are not needed for the single-view setup check.
    (path.parent / 'pass-1.npz').unlink()
    report = preflight(path, tmp_path / 'setup')
    assert report['observation_id'] == 'orientation-0'
    assert report['distance']['status'] == 'within_range'
    assert report['distance']['maximum_m'] < .9  # excluded stationary background
    assert report['focus']['status'] == 'manual_review'
    assert report['single_frame_camera_command_verified'] is False
    from PIL import Image
    assert Image.open(tmp_path / 'setup/first-frame.png').size == (384, 320)
    assert Image.open(tmp_path / 'setup/focus-crop.png').size == (256, 192)


def test_setup_marks_uncalibrated_hardware_distance_unavailable(tmp_path):
    path = demo.generate(tmp_path / 'inputs')
    config = read_json(path)
    config['data_kind'] = 'hardware'
    write_json(path, config)
    (path.parent / 'pass-0.npz').unlink()
    report = preflight(path, tmp_path / 'setup')
    assert report['distance']['status'] == 'calibration_required'


@pytest.fixture
def preview_run(tmp_path):
    path = demo.generate(tmp_path / 'inputs')
    config = read_json(path)
    extra = copy.deepcopy(config['observations'][0])
    extra.update(id='extra-hdr-exposure', path='extra.npz', frame_index=1)
    config['observations'].append(extra)
    write_json(path, config)
    # Intentional invalid test input proves unused exposures are not decoded
    # or sent through reconstruction by the draft mode; they are still hashed.
    (path.parent / 'extra.npz').write_bytes(b'intentionally invalid unselected test archive')
    directory = tmp_path / 'preview'
    report = coverage_preview(path, directory)
    return path, directory, report


def test_coverage_is_bounded_and_requires_visual_review(preview_run):
    path, directory, report = preview_run
    assert report['selected_observations'] == ['orientation-0', 'orientation-1', 'orientation-2']
    assert report['total_observations'] == 4
    assert report['processed_samples'] <= 10000
    assert report['mesh_triangles'] <= 12000
    assert report['voxel_m'] > read_json(path)['reconstruction']['fusion_voxel_m']
    assert report['coverage_complete_verified'] is False
    assert report['coverage_status'] == 'operator_review_required'
    html = (directory / 'index.html').read_text()
    assert '__PREVIEW_DATA__' not in html and 'Accept coverage' in html
    assert 'https://' not in html  # offline viewer has no CDN dependency


def test_explicit_coverage_acceptance_binds_all_inputs(preview_run):
    path, directory, report = preview_run
    review = approve_preview(directory, 'Inspected the test underside and transitions')
    accepted, preview = validate_review(path, review)
    assert accepted['coverage_accepted_by_operator']
    assert len(preview['pose_groups']) == 3
    (path.parent / 'extra.npz').write_bytes(b'changed unselected test exposure')
    with pytest.raises(ValueError, match='stale'):
        validate_review(path, review)


def test_preview_review_rejects_changed_report_and_empty_note(preview_run):
    path, directory, _ = preview_run
    with pytest.raises(ValueError, match='note'):
        approve_preview(directory, '  ')
    review = approve_preview(directory, 'Inspected the generated model')
    report = read_json(directory / 'preview.json')
    report['voxel_m'] *= 2
    write_json(directory / 'preview.json', report)
    with pytest.raises(ValueError, match='changed after review'):
        validate_review(path, review)


def test_full_render_stops_before_dense_processing_without_review(tmp_path, monkeypatch):
    path = demo.generate(tmp_path / 'inputs')
    def forbidden(*args, **kwargs):
        raise AssertionError('Dense processing must not start before review')
    monkeypatch.setattr(geometry, 'prepare', forbidden)
    with pytest.raises(ValueError, match='Review the coverage preview'):
        reconstruct(path, tmp_path / 'full')
    assert not (tmp_path / 'full').exists()


def test_coarse_preview_rejects_final_resolution_setting(tmp_path):
    path = demo.generate(tmp_path / 'inputs')
    config = read_json(path)
    config['preview']['voxel_m'] = config['reconstruction']['fusion_voxel_m']
    write_json(path, config)
    with pytest.raises(ValueError, match='coarser'):
        coverage_preview(path, tmp_path / 'preview')

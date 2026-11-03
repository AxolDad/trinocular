"""Dimensional validation against independent physical artifacts.

Outputs are observed errors and repeatability statistics. A reference-only
guard band is not a complete measurement uncertainty budget or certification.
"""
import json
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial import cKDTree


def fit_sphere(points):
    x = np.asarray(points, dtype=float)
    if x.ndim != 2 or x.shape[1] != 3 or len(x) < 30 or not np.isfinite(x).all():
        raise ValueError("Sphere needs at least 30 finite 3D samples")
    # Centre the algebraic initialization to reduce numerical cancellation.
    origin = x.mean(0)
    y = x - origin
    a = np.column_stack([2 * y, np.ones(len(y))])
    if np.linalg.matrix_rank(a) < 4:
        raise ValueError("Sphere samples are degenerate")
    q, *_ = np.linalg.lstsq(a, np.sum(y * y, axis=1), rcond=None)
    radius2 = q[3] + np.dot(q[:3], q[:3])
    if radius2 <= 0:
        raise ValueError("Invalid sphere initialization")
    initial = np.r_[origin + q[:3], np.sqrt(radius2)]
    def residual(v):
        return np.linalg.norm(x - v[:3], axis=1) - v[3]
    scale = max(float(np.median(np.abs(residual(initial)))), 1e-6)
    result = least_squares(residual, initial, loss='soft_l1', f_scale=scale,
                           bounds=([-np.inf] * 3 + [1e-9], [np.inf] * 4))
    if not result.success or np.linalg.cond(result.jac.T @ result.jac) > 1e8:
        raise ValueError("Sphere fit is poorly constrained; capture more of the surface")
    errors = residual(result.x)
    return {"center_m": result.x[:3].tolist(), "radius_mm": float(result.x[3] * 1000),
            "sample_count": len(x), "radial_rmse_mm": float(np.sqrt(np.mean(errors ** 2)) * 1000),
            "radial_p95_mm": float(np.percentile(np.abs(errors), 95) * 1000)}


def surface_error(points, reference_points):
    x, y = (np.asarray(a, dtype=float) for a in (points, reference_points))
    if any(a.ndim != 2 or a.shape[1] != 3 or not len(a) or not np.isfinite(a).all() for a in (x, y)):
        raise ValueError("Surface comparison requires finite nonempty Nx3 arrays")
    def stats(distances):
        return {"rmse_mm": float(np.sqrt(np.mean(distances ** 2)) * 1000),
                "p95_mm": float(np.percentile(distances, 95) * 1000),
                "maximum_mm": float(distances.max() * 1000)}
    return {"scan_to_reference": stats(cKDTree(y).query(x)[0]),
            "reference_to_scan": stats(cKDTree(x).query(y)[0]),
            "method": "bidirectional nearest samples; fixed reference frame; no scale fitting"}


def dimensions(points, spec, calibration_artifact_ids=()):
    artifact = spec['artifact_id']
    if not artifact or artifact in calibration_artifact_ids:
        raise ValueError("Validation needs an independent artifact ID")
    points = np.asarray(points, dtype=float)
    spheres = {}
    for item in spec['spheres']:
        name = item['id']
        if name in spheres:
            raise ValueError("Duplicate sphere ID")
        seed = np.asarray(item['roi_center_m'], dtype=float)
        radius = float(item['roi_radius_m'])
        if seed.shape != (3,) or not np.isfinite(seed).all() or not np.isfinite(radius) or radius <= 0:
            raise ValueError("Invalid sphere ROI")
        roi = points[np.linalg.norm(points - seed, axis=1) < radius]
        spheres[name] = fit_sphere(roi)
    rows, names = [], set()
    for item in spec['distances']:
        name = item['id']
        if name in names:
            raise ValueError("Duplicate dimension ID")
        names.add(name)
        reference = float(item['reference_mm'])
        tolerance = float(item['tolerance_mm'])
        expanded_reference_uncertainty = item.get('reference_expanded_uncertainty_mm')
        if not np.isfinite([reference, tolerance]).all() or min(reference, tolerance) <= 0:
            raise ValueError("Reference length and tolerance must be finite and positive")
        a, b = (np.array(spheres[item[k]]['center_m']) for k in ('from', 'to'))
        measured = float(np.linalg.norm(a - b) * 1000)
        error = measured - reference
        guarded = None
        if expanded_reference_uncertainty is not None:
            expanded_reference_uncertainty = float(expanded_reference_uncertainty)
            if not np.isfinite(expanded_reference_uncertainty) or expanded_reference_uncertainty < 0:
                raise ValueError("Invalid expanded reference uncertainty")
            guarded = abs(error) + expanded_reference_uncertainty <= tolerance
        rows.append({"id": name, "measured_mm": measured, "reference_mm": reference,
                     "signed_error_mm": error, "absolute_error_mm": abs(error),
                     "relative_error_percent": error / reference * 100, "tolerance_mm": tolerance,
                     "within_requested_tolerance": abs(error) <= tolerance,
                     "reference_expanded_uncertainty_mm": expanded_reference_uncertainty,
                     "within_reference_only_guard_band": guarded})
    if not rows:
        raise ValueError("At least one independent validation dimension required")
    return {"artifact_id": artifact, "source": "fused observed samples before meshing",
            "spheres": spheres, "dimensions": rows,
            "full_measurement_uncertainty_evaluated": False}


def repeatability(reports):
    """Repeated complete acquisitions, same fixture/configuration/conditions."""
    if len(reports) < 3:
        raise ValueError("At least three independent acquisitions required")
    rows, signature = {}, None
    acquisitions = [tuple(sorted(r['input_sha256'])) for r in reports]
    if len(set(acquisitions)) != len(acquisitions):
        raise ValueError("Repeated processing of the same acquisition is not repeatability; independently acquired inputs required")
    for report in reports:
        b = report['dimension_benchmark']
        if b is None:
            raise ValueError("Each acquisition needs a dimensional benchmark")
        dims = b['dimensions']
        this = (b['artifact_id'], report['data_kind'], report['repeatability_group'],
                tuple(sorted((d['id'], d['reference_mm'], d['tolerance_mm']) for d in dims)),
                json.dumps(report.get('settings', {}), sort_keys=True),
                json.dumps(report.get('calibration', {}), sort_keys=True))
        if not this[2] or (signature is not None and signature != this):
            raise ValueError("Repeatability fixture, reference dimensions, or condition group differ")
        signature = this
        for d in dims:
            rows.setdefault(d['id'], []).append(d['measured_mm'])
    refs = {d['id']: d['reference_mm'] for d in reports[0]['dimension_benchmark']['dimensions']}
    return {"acquisition_count": len(reports), "artifact_id": signature[0],
            "data_kind": signature[1], "repeatability_group": signature[2],
            "dimensions": [{"id": key, "mean_mm": float(np.mean(values)),
                            "bias_mm": float(np.mean(values) - refs[key]),
                            "sample_stddev_mm": float(np.std(values, ddof=1)),
                            "range_mm": float(np.ptp(values))} for key, values in sorted(rows.items())]}

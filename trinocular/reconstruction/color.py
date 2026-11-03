"""Explicit color encodings, chart calibration, and linear HDR integration.

An integer bit depth alone never establishes a camera's transfer function.
Chart coordinates must share a declared illuminant/white point (here D65).
"""
import numpy as np


def srgb_to_linear(rgb):
    x = np.asarray(rgb, dtype=float)
    if not np.isfinite(x).all() or np.any((x < 0) | (x > 1)):
        raise ValueError("sRGB input must be finite and normalized to [0, 1]")
    return np.where(x <= .04045, x / 12.92, ((x + .055) / 1.055) ** 2.4)


def linear_to_srgb(rgb):
    x = np.asarray(rgb, dtype=float)
    if not np.isfinite(x).all():
        raise ValueError("Linear RGB must be finite")
    # Clipping is an output/display decision; the linear master stays separate.
    x = np.clip(x, 0, 1)
    return np.where(x <= .0031308, 12.92 * x, 1.055 * x ** (1 / 2.4) - .055)


def fit_chart(observed_linear, reference_linear_srgb):
    """Fit a black-corrected camera-linear -> linear-sRGB 3x3 matrix.

    No intercept: black-level subtraction belongs upstream. Validation patches
    are supplied separately, never reused here to claim color accuracy.
    """
    x, y = (np.asarray(a, dtype=float) for a in (observed_linear, reference_linear_srgb))
    if x.ndim != 2 or x.shape != y.shape or x.shape[1] != 3 or len(x) < 6:
        raise ValueError("A chart needs at least six matching RGB patch samples")
    if not np.isfinite(x).all() or not np.isfinite(y).all() or np.any(x < 0) or np.any(y < 0):
        raise ValueError("Chart samples must be finite, nonnegative linear values")
    if np.linalg.matrix_rank(x) < 3 or np.linalg.cond(x) > 1e5:
        raise ValueError("Color chart is ill-conditioned")
    matrix, *_ = np.linalg.lstsq(x, y, rcond=None)
    return matrix


def color_error(observed_linear_srgb, reference_linear_srgb):
    """CIEDE2000 on D65-referenced linear-sRGB patch values, without clipping."""
    import colour
    x, y = (np.asarray(a, dtype=float) for a in (observed_linear_srgb, reference_linear_srgb))
    if x.ndim != 2 or x.shape != y.shape or x.shape[1] != 3 or not len(x):
        raise ValueError("Validation patches need matching nonempty Nx3 arrays")
    if not np.isfinite(x).all() or not np.isfinite(y).all():
        raise ValueError("Validation patches must be finite")
    cs = colour.RGB_COLOURSPACES['sRGB']
    def lab(a):
        xyz = colour.RGB_to_XYZ(a, cs, apply_cctf_decoding=False)
        return colour.XYZ_to_Lab(xyz, illuminant=cs.whitepoint)
    errors = np.asarray(colour.delta_E(lab(x), lab(y), method='CIE 2000'))
    return {"metric": "CIEDE2000", "white_point": "D65", "patch_count": len(x),
            "mean": float(errors.mean()), "p95": float(np.percentile(errors, 95)),
            "maximum": float(errors.max()), "per_patch": errors.tolist()}


def merge_linear_hdr(frames, exposure_s, gain=None, low=.01, high=.98):
    """Merge aligned, black-corrected linear sensor frames into relative radiance.

    Assumes a verified linear response and verified exposure/gain associations.
    No geometric alignment or deghosting is performed. Fully clipped channels
    stay invalid (NaN), rather than inventing recoverable highlight detail.
    """
    x = np.asarray(frames, dtype=float)
    t = np.asarray(exposure_s, dtype=float)
    g = np.ones_like(t) if gain is None else np.asarray(gain, dtype=float)
    if x.ndim < 2 or t.shape != (len(x),) or g.shape != t.shape:
        raise ValueError("One exposure and gain required per aligned frame")
    if not np.isfinite(x).all() or not np.isfinite(t).all() or not np.isfinite(g).all():
        raise ValueError("HDR inputs must be finite")
    if np.any((x < 0) | (x > 1)) or np.any(t <= 0) or np.any(g <= 0) or not 0 <= low < high <= 1:
        raise ValueError("Invalid HDR signal bounds, exposure, or gain")
    shape = (len(x),) + (1,) * (x.ndim - 1)
    valid = (x > low) & (x < high)
    weight = np.where(valid, np.minimum(x, 1 - x), 0)
    denom = weight.sum(axis=0)
    radiance = (weight * x / (t * g).reshape(shape)).sum(axis=0)
    out = np.full_like(radiance, np.nan)
    np.divide(radiance, denom, out=out, where=denom > 0)
    return out, denom > 0

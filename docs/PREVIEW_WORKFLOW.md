# Setup and coverage previews

Date: 2 November 2026.

## Operator workflow

1. **Setup:** Inspect the first view at the initial position: full-size image,
   original-pixel subject crop, image detail/clipping, and calibrated distance.
2. **Coverage:** Select one exposure per angle and active sensor, align a bounded
   sample, and inspect a coarse 3D surface from every side. Add angles as needed.
3. **Full processing:** Accept coverage, then reconstruct the full dataset using
   the reviewed preview poses as initialization. Changed inputs/settings require
   a new preview and review.

The implementation consumes stored observations. A Pro2 command to acquire only
the first image/set without a full sweep is still unverified. Extracting the
first stored image supports software testing but does not shorten physical
camera acquisition. Native first-view capture and capture-app integration remain
delivery stages.

## Setup preview

```bash
python -m trinocular.reconstruction setup-preview /tmp/trinocular-demo/manifest.json /tmp/trinocular-setup
```

Open `/tmp/trinocular-setup/index.html`. Outputs: full-size `first-frame.png`,
`focus-crop.png`, and `setup.json`. Only the first selected view is read.
Use suitable local paths on Windows.

Manifest `preflight` fields:

| Field | Meaning |
| --- | --- |
| `image` | First display-image path |
| `observation_id` | Its associated first selected observation |
| `subject_roi_pixels` | `[left, top, right, bottom]` in original image pixels |
| `working_distance_m` | Calibrated minimum/maximum range |
| `minimum_focus_score` | Optional scene/optics-specific threshold; otherwise manual review |

The detail score is variance of a smoothed luminance Laplacian. Texture, noise,
lighting, sharpening, size, and ROI affect it; it is not an autofocus control or
a universal focus test. Inspect the original-pixel crop. Clipping statistics
refer to the display image, not raw sensor dynamic range.

Distance uses foreground points in explicit units, reporting minimum, median,
maximum, and in-range fraction relative to the sensor origin. Hardware distance
requires verified scale/projection; image review works while calibration is
unresolved. Synthetic inputs are labelled.

## Coarse 3D coverage preview

```bash
python -m trinocular.reconstruction coverage-preview /tmp/trinocular-demo/manifest.json /tmp/trinocular-preview
```

Open `/tmp/trinocular-preview/index.html`. The offline viewer has rotation,
front/back/left/right/top/bottom views, color/coverage displays, open-edge
highlighting, and review export. It uses no CDN. Outputs also include
`preview.ply`, `preview.npz`, and `preview.json`.

Observations use `angle_id`, `sensor_id`, and `frame_index`. Lowest frame index
wins per angle/sensor, with capture order breaking ties. Without those fields,
observation ID is its angle and sensor is `primary`. Native adapters must map
each observation to one calibrated view/exposure and associate depth correctly.
Retaining active sensors preserves vertical coverage from the same angle.

Manifest `preview` fields:

| Field | Default | Purpose |
| --- | --- | --- |
| `voxel_m` | Twice full registration voxel | Coarse alignment/fusion; must exceed full fusion voxel |
| `max_points_per_angle` | 5,000 | Cap per selected angle/sensor |
| `max_points_total` | 60,000 | Total pre-registration cap |
| `min_overlap` | Full reconstruction threshold | Reject poor overlap |
| `max_registration_rmse_m` | Preview voxel | Coarse error gate |

`reconstruction.max_passes` bounds all-pairs work; output has at most 12,000
triangles. Final measurement and high-resolution texture work are skipped.
Limits do not promise a completion time. All inputs are still hashed, including
unselected HDR exposures, to bind the review to the full dataset.

Blue means support from multiple distinct angles, amber from one angle. Sensors
at the same angle do not count as independent angles. Red highlights open
edges, including boundaries caused by coarse sampling, as a diagnostic overlay.
These guide visual review; `coverage_complete_verified` stays false. An
independent reference is needed for a measured completeness assessment.

## Review gate

Use **Accept coverage** and save `review.json` beside `preview.json`, or:

```bash
python -m trinocular.reconstruction approve-preview /tmp/trinocular-preview --note "Inspected the underside and transitions between views"
python -m trinocular.reconstruction reconstruct /tmp/trinocular-demo/manifest.json /tmp/trinocular-result --review /tmp/trinocular-preview/review.json
```

This records the operator's decision. Before dense processing, the renderer
rejects missing review, changed preview reports, and changed inputs/settings,
including unselected exposures, profiles, reference surfaces, and setup image.
The final report retains review/preview hashes. After changes, use a new folder,
rebuild, inspect, and review again. This is an offline prototype; integration
with the capture app remains a delivery stage.

## Checks and references

Run `python -m pytest -q tests`. Viewer drawing, rotation, review validation,
and export can be checked with the minimal DOM/canvas runtime harness:

```bash
node tests/preview_viewer_smoke.cjs /tmp/trinocular-preview/index.html
```

The harness does not perform browser visual QA.

Huamán, Ana. "Laplace Operator." *OpenCV Documentation*,
https://docs.opencv.org/4.x/d5/db5/tutorial_laplace_operator.html.
Accessed 2 Nov. 2026. Cited for the operator; the relative-detail score and
threshold policy are project implementation choices.

Open3D. "Mesh." *Open3D 0.19.0 Documentation*,
https://www.open3d.org/docs/release/tutorial/geometry/mesh.html.
Accessed 2 Nov. 2026.

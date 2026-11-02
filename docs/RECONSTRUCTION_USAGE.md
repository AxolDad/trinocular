# Offline reconstruction foundation

This branch adds an executable processing foundation and a
[full-system specification](OBJECT_SCANNER_SYSTEM.md). It accepts calibrated,
foreground-masked point observations. The Pro2 raw-depth projection and metric
scale adapter awaits physical calibration; camera stationary capture and
manual settings remain separate firmware research.

The [two-stage preview workflow](PREVIEW_WORKFLOW.md) adds a first-view setup
check and coarse 3D coverage view. Full reconstruction requires a current
accepted coverage review before dense processing begins.

## Install and run the synthetic demonstration

Use Python 3.11 or 3.12 on an Open3D-supported system. CPU processing is enough
for the small demonstration. The capture app's requirements are independent.

```bash
python -m venv .venv
# Activate the environment using your platform's normal activation command.
python -m pip install -r requirements-reconstruction.txt
python examples/reconstruction_demo.py /tmp/trinocular-demo
python -m trinocular.reconstruction setup-preview /tmp/trinocular-demo/manifest.json /tmp/trinocular-setup
python -m trinocular.reconstruction coverage-preview /tmp/trinocular-demo/manifest.json /tmp/trinocular-preview
# Inspect both HTML views, then record your coverage review:
python -m trinocular.reconstruction approve-preview /tmp/trinocular-preview --note "Inspected the generated test surface"
python -m trinocular.reconstruction reconstruct /tmp/trinocular-demo/manifest.json /tmp/trinocular-result --review /tmp/trinocular-preview/review.json
```

On Windows, replace the example `/tmp/...` paths with suitable local paths.
Choose an unused output directory for each run. Inputs are read, never modified.
The demonstration uses generated three-ball geometry, simulated noise, known
pose seeds, and a synthetic channel response. Its reference dimensions and
color samples are generated values. Results verify software behavior and say
nothing about the physical Pro2's accuracy or color performance.

## Observation contract

Each NPZ observation contains:

| Array | Shape | Meaning |
| --- | --- | --- |
| `points` | N x 3 float | Sensor-frame XYZ, in explicitly declared metres or millimetres |
| `colors` | N x 3 float | RGB in the explicitly declared encoding; normalized, not integer code values |
| `foreground` | N bool | Explicit object mask, excluding background/supports/markers not part of the final object |
| `normals` | N x 3 float, optional | Sensor-frame normals; must be finite and nonzero for foreground points |

Encodings are `srgb`, `linear_srgb`, or `camera_linear`. Camera-linear data
requires a measured chart profile mapping black-corrected linear samples to
linear sRGB under the same lighting/white-point convention. The implementation
uses D65 for its Lab validation. HDR values may exceed 1 in the linear master;
PLY colors are clipped and encoded as sRGB for display. High-resolution image
texture projection, exposure balancing, deghosting, and UV atlases are future
stages, not implemented by the point-color export.

The example generator writes a complete manifest and profile with actual
generated data. Use it as a schema example, not as a hardware calibration.
For hardware inputs, set `data_kind` to `hardware`, identify calibration artifact
IDs/evidence, and verify metric scale and projection before enabling those
flags. The software records this operator declaration; flags alone do not
establish physical correctness.

Every observation has an ID, path, units, encoding, and optional color profile.
An optional `object_from_sensor` pose seeds alignment; it must be a rigid 4x4
transform without scale. Alternatively supply `markers.sensor_points_m` and
`markers.object_points_m` with at least three matching noncollinear points.
The marker frame must follow the object. Without seeds, FPFH/RANSAC initializes
each pair before robust ICP and global optimization.

Final coordinates are metres in the first observation's sensor frame. A common
object-frame seed does not change that output convention. Benchmark sphere ROIs
and reference surfaces must use the final coordinate frame. Scale remains fixed
throughout registration. Review symmetric objects manually.

## Outputs

| File | Content |
| --- | --- |
| `linear-master.npz` | Fused observed points in metres, unclipped linear RGB, independent-pass support per voxel |
| `observations.ply` | Fused observed points and display colors, metres |
| `mesh.ply` | Ball-pivoted colored mesh, metres |
| `mesh.stl` | Same mesh in millimetres; STL itself has no unit declaration |
| `report.json` | Poses, overlap/error gates, color and dimension metrics, hashes, versions, units, mesh status |
| `manifest.snapshot.json` | Exact parsed input configuration for reproduction |

Ball pivoting may leave open boundaries. `mesh_watertight` reports topology;
printing may require a separately tracked repair stage. The pipeline does not
silently close holes or adjust scale to make a model look complete.

## Dimensional and surface benchmarks

`benchmark` defines an independent artifact ID, sphere ROIs in metres, and known
center-to-center distances in millimetres. Each distance requires a positive
reference and a tolerance chosen before evaluation. A reference's expanded
uncertainty is optional; the report identifies a reference-only guard band and
does not claim a full measurement-uncertainty assessment. The sphere fitter
rejects degenerate and poorly constrained data, but fitting bias and real
instrument uncertainty need independent validation.

Optional `reference_surface` contains `path` and independent `artifact_id`.
The NPZ must contain `points` in metres, already aligned to the first sensor
frame. Nearest-sample bidirectional distances do not automatically align or
rescale the reference; sampling density affects the result.

For repeated complete acquisitions, give each manifest the same explicit
`repeatability_group` describing the fixture, settings, and conditions. Acquire
new source data for every run, then:

```bash
python -m trinocular.reconstruction repeatability repeatability.json run1/report.json run2/report.json run3/report.json
```

At least three independently acquired input sets are required. Identical source
hashes are rejected even if rerun under different manifest filenames. The
report separates mean bias from sample standard deviation. More acquisitions
and an uncertainty study are needed for stronger performance conclusions.

## Tests

```bash
python -m pip install 'pytest>=8,<9'
python -m pytest -q tests
```

Tests cover color/HDR math, metric units, marker transforms, scale rejection,
global registration, disconnected inputs, complete seeded reconstruction,
foreground exclusion, STL units, dimension validation, provenance, and
repeatability handling. Real-camera testing is still required at the delivery
gates in the system specification.

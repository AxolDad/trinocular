# Software validation record

Date: 2 November 2026.

## Tested environment and checks

- Python 3.12, Open3D 0.19.0, Colour 0.4.6.
- `python -m pytest -q tests`: **22 passed**.
- `git diff --check`: passed.
- CLI demonstration: completed reconstruction and exports from three generated
  object orientations; produced 5,011 fused observations and 9,309 triangles.
- The generated mesh has open boundaries and is reported as non-watertight.
- Dimensional checks operate on observed samples before meshing; STL unit
  conversion is independently checked against the metre-based PLY export.
- Setup and coverage preview CLI commands completed on the generated test data.
  The coarse coverage run selected three frames, processed 7,500 samples, and
  produced 1,018 fused samples and 2,016 triangles.
- Viewer runtime smoke check passed for drawing, rotation, required review note,
  and review export, using a minimal DOM/canvas harness. Browser visual QA has
  not been performed.
- Tests verify first-view-only setup processing, subject-range masking,
  uncalibrated-distance handling, one exposure per angle/sensor, sampling caps,
  and rejection of missing/stale/altered reviews before full processing.

All demonstration inputs, dimensions, exposure responses, and reference colors
are synthetic. Software tests establish implementation behavior; no physical
Matterport performance has been measured. The QA report explicitly records
`data_kind: synthetic` and `hardware_accuracy_verified: false`.

## Outstanding integration and physical validation

Confirm the owner's model/firmware and stationary capture behavior. Establish
native depth scale/projection, RGB/depth extrinsics, exposure interpretation,
and the Pro2 observation adapter. Then acquire independent geometric fixtures
and color charts, evaluate repeated full captures, and report physical errors
under stated conditions. Full-resolution image texturing, UV baking, automatic
foreground segmentation, and the integrated operator workflow remain stages in
the [system specification](OBJECT_SCANNER_SYSTEM.md).

See the [preview workflow](PREVIEW_WORKFLOW.md) for the implemented offline
preview stages and the first-frame-only hardware control still to verify.

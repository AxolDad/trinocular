# Software validation record

Date: 2 November 2026.

## Tested environment and checks

- Python 3.12, Open3D 0.19.0, Colour 0.4.6.
- `python -m pytest -q tests/test_reconstruction.py`: **13 passed**.
- `git diff --check`: passed.
- CLI demonstration: completed reconstruction and exports from three generated
  object orientations; produced 5,011 fused observations and 9,309 triangles.
- The generated mesh has open boundaries and is reported as non-watertight.
- Dimensional checks operate on observed samples before meshing; STL unit
  conversion is independently checked against the metre-based PLY export.

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

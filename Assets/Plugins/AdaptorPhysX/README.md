# AdaptorPhysX Unity native plugin

This folder contains the thin Unity binding for the engine-independent
AdaptorPhysX C ABI v0. The managed API remains in the existing `APEX`
namespace, under `APEX.Native`.

## Packaged Windows x64 binaries

- `x86_64/adaptorphysx.dll`
  - native source commit: `6ca73e59ca5f7d44578d0c1057350cd102d14b76`
  - build: CUDA Release, Ninja + MSVC 14.44, `sm_89`
  - SHA-256:
    `4BAEAD2D32859FA315E569F36C93606B8F668D25711A5DF379AB1ECEDDED088E`
- `x86_64/cudart64_12.dll`
  - source: CUDA Toolkit 12.6 shared runtime
  - file version: `6.14.11.12060`
  - SHA-256:
    `3DF50CF48718712F17B8C77AD72C45A1F43FF94781E302A4BE7F1D7A9B66C746`

`adaptorphysx.dll` imports only `cudart64_12.dll` from NVIDIA. It also uses the
Microsoft Visual C++ 14.x runtime; deployed Windows machines must have a
compatible VC++ Redistributable installed.

The importer enables only Windows x64 in the Editor and Windows x64 Standalone.
All other Unity targets inherit the disabled `Any` setting.

## Readback fallback

Phase 1 maps a blocking host-readable position snapshot. CUDA worlds therefore
perform a GPU-to-CPU readback before Unity consumes positions. The managed
wrapper exposes allocation-free mapped access and a reusable managed snapshot
buffer, but the transfer itself remains a Phase 4 zero-copy technical debt.

## NOTE(needs-decision): candidate ROADMAP DL-9

Human confirmation is required for both packaging decisions before P1-0 can be
marked done:

1. Keep `adaptorphysx.dll` and its runtime dependency committed directly under
   Unity `Assets/` (the default implemented here), so a fresh checkout imports
   a usable plugin without a separate staging step.
2. Keep the CUDA 12.6 shared runtime colocated and committed as
   `cudart64_12.dll` (the default implemented here), versus introducing a
   separate redistributable/install pipeline.

CUDA 12.6 `EULA.txt` section 2.6, Attachment A lists the Windows CUDA runtime as
redistributable subject to NVIDIA's distribution terms. The project owner must
confirm that the application's distribution terms and update process satisfy
those conditions. This note is not legal advice.

# AdaptorPhysX Unity native plugin

This folder contains the thin Unity binding for the engine-independent
AdaptorPhysX C ABI v0. The managed API remains in the existing `APEX`
namespace, under `APEX.Native`.

## Packaged Windows x64 binaries

The DLLs in `x86_64/` are committed generated artifacts under locked decision
DL-9. Do not edit them by hand. From the superproject root, rebuild and sync
them with:

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\AdaptorPhysX_Native\tools\sync_unity_plugin.ps1
```

The script loads VS2022 MSVC 14.44 through `vcvarsall.bat`, uses Ninja and CUDA
12.6 for a Release `sm_89` build, builds only `apx_c_api`, and then copies the
plugin and CUDA runtime into this directory. It fails unless the copied plugin
exports exactly the nine DL-7 symbols plus the three additive DL-10 v0.1
symbols, two additive DL-12 v0.2 symbols, and the additive DL-11 v0.3
`apxCut` symbol. `cudart64_12.dll` remains its only direct NVIDIA dependency.
The script also verifies that both existing Unity `.meta` files are unchanged.

- `x86_64/adaptorphysx.dll`
  - native source baseline: `71e9829d143c11b783761c2999800519b6d10aaf`
  - build: CUDA Release, Ninja + MSVC 14.44, `sm_89`
  - current packaged SHA-256:
    `E3C4060C5D5664028DECBF928B22D16641F6B45A481E1D90CF3F5E32C48F248E`
- `x86_64/cudart64_12.dll`
  - source: CUDA Toolkit 12.6 shared runtime
  - file version: `6.14.11.12060`
  - current packaged SHA-256:
    `3DF50CF48718712F17B8C77AD72C45A1F43FF94781E302A4BE7F1D7A9B66C746`

MSVC PE output is not promised to be bit-reproducible across independent
rebuilds, so a newly built DLL must not be compared with a historical packaged
DLL by hash. Functional equivalence is the gate: the script verifies the
current build source and copied target have matching hashes, the exact
DL-7/DL-10/DL-11/DL-12 export/dependency contracts hold, and the
script-produced DLL passes all Unity EditMode tests.

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

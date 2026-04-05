Vendored Python dependencies. Due to runtime restrictions, all dependencies must live here.
- Do not rely on `pip install` at runtime.
- Keep third-party code checked into `vendor/`.
- Add the package directory itself, not a wheel or an environment export.

Current Dependencies:
- `onnxruntime-gpu 1.19.2` for Python 3.9 in `vendor/onnxruntime_gpu_py39/`
- `onnxruntime 1.19.2` for Python 3.9 in `vendor/onnxruntime_cpu_py39/`

Setup:
- Run `scripts/vendor_onnxruntime.sh` to fetch and unpack both pinned wheels.

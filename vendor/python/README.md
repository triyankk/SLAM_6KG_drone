# Offline Python Wheels

This wheel bundle installs `requirements.lock` without reading another Python
environment or contacting a package index. It targets CPython 3.10 on Jetson
aarch64. It also contains the `virtualenv` fallback pinned by
`bootstrap-requirements.lock`, so Ubuntu's optional `python3.10-venv` package is
not required.

Refresh the complete bundle intentionally when dependencies change, then run
the project tests using a newly created `.venv`.

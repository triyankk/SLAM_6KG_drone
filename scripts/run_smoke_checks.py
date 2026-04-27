#!/usr/bin/env python3
"""
Run lightweight static and import-time checks for the SLAM package.

Checks performed:
- compileall on the `src` tree to catch syntax errors
- AST parse pass for each .py file
- attempt to import each top-level module under `slam_core`
- run small unit tests present in `tests/` by importing them

This avoids installing external linters and provides a reproducible smoke check.
"""

import compileall
import importlib
import os
import sys
import traceback
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = REPO_ROOT / "src"
PKG = "slam_core"


def ast_check(py_path: Path) -> bool:
    try:
        with py_path.open("r", encoding="utf-8") as fh:
            src = fh.read()
        import ast
        ast.parse(src)
        return True
    except Exception:
        print(f"AST parse failed: {py_path}")
        traceback.print_exc()
        return False


def import_modules() -> bool:
    sys.path.insert(0, str(SRC_ROOT))
    ok = True
    pkg_path = SRC_ROOT / PKG
    if not pkg_path.exists():
        print("Package path missing:", pkg_path)
        return False

    for root, _, files in os.walk(pkg_path):
        rel = Path(root).relative_to(SRC_ROOT)
        for f in files:
            if not f.endswith(".py"):
                continue
            mod_path = rel / f
            mod_name = str(mod_path.with_suffix("")).replace(os.sep, ".")
            try:
                importlib.import_module(mod_name)
                print("import ok:", mod_name)
            except Exception:
                ok = False
                print("import FAILED:", mod_name)
                traceback.print_exc()
    return ok


def run_tests() -> bool:
    sys.path.insert(0, str(SRC_ROOT))
    tests_dir = REPO_ROOT / "tests"
    if not tests_dir.exists():
        print("No tests directory found; skipping tests")
        return True
    ok = True
    for f in tests_dir.iterdir():
        if not f.name.startswith("test_") or not f.name.endswith(".py"):
            continue
        mod_name = f.stem
        try:
            spec = importlib.util.spec_from_file_location(mod_name, str(f))
            mod = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(mod)
            # run any functions starting with test_
            for name in dir(mod):
                if name.startswith("test_"):
                    fn = getattr(mod, name)
                    try:
                        fn()
                        print(f"test ok: {mod_name}.{name}")
                    except AssertionError:
                        ok = False
                        print(f"test FAILED: {mod_name}.{name}")
                        traceback.print_exc()
        except Exception:
            ok = False
            print("Failed to import test module:", f)
            traceback.print_exc()
    return ok


def main():
    print("Running compileall on src...")
    compiled = compileall.compile_dir(str(SRC_ROOT), force=True, quiet=1)
    print("compileall result:", compiled)

    print("Running AST checks...")
    all_ast_ok = True
    for py in SRC_ROOT.rglob("*.py"):
        if not ast_check(py):
            all_ast_ok = False

    print("Importing modules...")
    imports_ok = import_modules()

    print("Running unit tests...")
    tests_ok = run_tests()

    success = compiled and all_ast_ok and imports_ok and tests_ok
    print("SMOKE CHECK SUMMARY -> compiled=%s ast_ok=%s imports_ok=%s tests_ok=%s" % (compiled, all_ast_ok, imports_ok, tests_ok))
    if not success:
        sys.exit(2)


if __name__ == "__main__":
    main()

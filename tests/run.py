#!/usr/bin/env python3
import os
from pathlib import Path
import subprocess
import tempfile

root = Path(__file__).resolve().parents[1]
with tempfile.TemporaryDirectory(prefix="auton-tests-") as directory:
    for suite, flags in (("auton_tests", []), ("runtime_tests", []), ("runtime_tests", ["-DENABLE_PID_TUNER=0"])):
        binary = Path(directory) / (suite + ("_no_tuner" if flags else ""))
        includes = ["-I", str(root / "include"), "-I", str(root)]
        if suite == "runtime_tests":
            includes = ["-I", str(root / "tests/fakes")] + includes
        subprocess.run([
            os.environ.get("HOST_CXX", "clang++"), "-std=c++17", "-Wall", "-Wextra", "-Werror",
            "-fsanitize=address,undefined", "-fno-omit-frame-pointer", "-pthread", "-g",
            *flags, *includes, "-x", "c++", "-", "-o", str(binary),
        ], input=f'#include "tests/{suite}.hpp"\n', text=True, check=True)
        subprocess.run([str(binary)], check=True, timeout=30)

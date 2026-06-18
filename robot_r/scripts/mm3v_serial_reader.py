#!/usr/bin/env python3

import runpy
from pathlib import Path


if __name__ == "__main__":
    script = Path(__file__).resolve().parents[2] / "script" / "mm3v_serial_reader.py"
    runpy.run_path(str(script), run_name="__main__")

#!/usr/bin/env python3
"""bimo_console: the operator console v2 (Qt), or --headless.

    make -C firmware/host console               # the C++ core, once
    .venv/bin/python link/bimo_console.py --host <robot-ip>          # observe
    .venv/bin/python link/bimo_console.py --host <robot-ip> --drive  # drive
    .venv/bin/python link/bimo_console.py --host 127.0.0.1 --drive   # the sim

docs/console.md describes it; --help lists every flag.
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from console import options  # noqa: E402


def main(argv=None) -> int:
    opts = options.parse(argv)
    if opts.headless:
        from console import headless
        return headless.run(opts)
    from console import app
    return app.run(opts)


if __name__ == "__main__":
    sys.exit(main())

#!/usr/bin/env bash
# Every check CI runs, in CI's order. Needs: pip install -e ".[dev,sim,cad]"
# Usage: tools/check_all.sh   (from the repo root)
set -euxo pipefail
ruff check .
ruff format --check .
pytest -q tests
python -m cad.build --check
python -m cad.export --check
python tools/check_description.py
python tools/bom/summary.py
python tools/bom/book.py --check
python tools/build_urdf.py --check
python sim/mujoco/check_torques.py
python sim/mujoco/check_symmetry.py

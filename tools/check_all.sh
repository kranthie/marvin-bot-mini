#!/usr/bin/env bash
# Every check CI runs, in CI's order. Needs: pip install -e ".[dev]"
# Usage: tools/check_all.sh   (from the repo root)
set -euxo pipefail
ruff check .
ruff format --check .
pytest -q tests
python tools/check_description.py
python tools/bom/summary.py

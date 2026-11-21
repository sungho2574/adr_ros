#!/bin/sh
set -eu

script_dir=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
exec python3 "$script_dir/msp_diagnostics.py" probe \
    --profile variant --hz 5 --seconds 180 "$@" --baud 500000

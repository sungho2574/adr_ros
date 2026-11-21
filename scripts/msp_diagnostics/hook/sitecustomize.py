"""Enable MSP capture in the msp_telemetry process launched by the diagnostic tool."""
import os
from pathlib import Path
import sys


directory = os.environ.get('ADR_MSP_CAPTURE_DIR')
if directory and Path(sys.argv[0]).name == 'msp_telemetry':
    try:
        from adr_betaflight import msp, node
        from capture import Capture, trace_ros_node

        capture = Capture(Path(directory), mode='live', msp_module=msp.__file__)
        trace_ros_node(node, msp, capture)
    except Exception as exc:
        print(f'MSP diagnostics hook failed: {exc}', file=sys.stderr, flush=True)

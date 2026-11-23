"""Find workspace-local runtime dependencies and configure launch children."""
import os
from pathlib import Path
import sys

from ament_index_python.packages import get_package_prefix
from launch.actions import SetEnvironmentVariable


def workspace_root():
    explicit = os.environ.get('ADR_ROOT')
    if explicit:
        return Path(explicit).expanduser().resolve()
    prefix = Path(get_package_prefix('adr_bringup')).resolve()
    return next((parent for parent in prefix.parents if (parent / 'src/adr_bringup').is_dir()),
                Path.cwd())


def runtime_environment(context, root):
    actions = [SetEnvironmentVariable('ADR_ROOT', str(root))]
    gst_lib = root / '.deps/smoke/gst/usr/lib/aarch64-linux-gnu'
    python_dir = root / f'.venv/lib/python{sys.version_info.major}.{sys.version_info.minor}/site-packages'
    for name, directory in (
        ('LD_LIBRARY_PATH', gst_lib),
        ('GST_PLUGIN_PATH', gst_lib / 'gstreamer-1.0'),
        ('PYTHONPATH', python_dir),
    ):
        if directory.is_dir():
            existing = context.environment.get(name)
            value = os.pathsep.join((str(directory), existing)) if existing else str(directory)
            actions.append(SetEnvironmentVariable(name, value))
    return actions

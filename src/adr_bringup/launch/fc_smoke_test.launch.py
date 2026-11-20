"""Check the camera pipeline and Betaflight telemetry together."""
from adr_bringup.smoke import smoke_launch_description


def generate_launch_description():
    return smoke_launch_description(with_fc=True)

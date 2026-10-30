"""Read backseat temperatures straight from the sysfs thermal zones.

psutil.sensors_temperatures() gives up on every zone when one of them fails in a
way it does not expect. On the Jetson Orin Nano the cv0, cv1 and cv2 zones answer
every read with EAGAIN; Python's open().read() returns None for EAGAIN, and
psutil's float(None) raises TypeError out of the call. Each zone is read here on
its own, so an unreadable zone is skipped instead of hiding the others.

Kept free of ROS imports so the parsing runs in plain unit tests.
"""

import os
import re
from pathlib import Path
from typing import NamedTuple

THERMAL_ROOT = Path('/sys/class/thermal')

# A sysfs attribute never holds more than one page.
_SYSFS_READ_BYTES = 4096
_ZONE_DIR = re.compile(r'thermal_zone(\d+)')


class ZoneReading(NamedTuple):
    """One thermal zone sample.

    Attributes:
        name: Zone type from sysfs, such as 'cpu-thermal', or the directory name
            when the type cannot be read.
        celsius: Temperature in degrees Celsius.
    """

    name: str
    celsius: float


def parse_millidegrees(raw: str) -> float | None:
    """Convert a sysfs temp value in millidegrees Celsius to degrees Celsius.

    Args:
        raw: Contents of a thermal_zone*/temp file, such as '46656'.

    Returns:
        The temperature in degrees Celsius, or None when the text is empty or not
        an integer.
    """
    text = raw.strip()
    if not text:
        return None
    try:
        return int(text) / 1000.0
    except ValueError:
        return None


def _read_sysfs(path: Path) -> str:
    """Return the text of a sysfs attribute.

    os.read is used instead of open().read(): it raises BlockingIOError on EAGAIN,
    where open().read() returns None.

    Raises:
        OSError: The attribute cannot be opened or read.
    """
    fd = os.open(path, os.O_RDONLY)
    try:
        data = os.read(fd, _SYSFS_READ_BYTES)
    finally:
        os.close(fd)
    return data.decode('ascii', errors='replace')


def read_zone(zone_dir: Path) -> ZoneReading | None:
    """Read one thermal zone.

    Args:
        zone_dir: A thermal_zone* directory.

    Returns:
        The reading, or None when the temperature cannot be read or parsed.
    """
    try:
        celsius = parse_millidegrees(_read_sysfs(zone_dir / 'temp'))
    except OSError:
        return None
    if celsius is None:
        return None
    try:
        name = _read_sysfs(zone_dir / 'type').strip()
    except OSError:
        name = ''
    return ZoneReading(name or zone_dir.name, celsius)


def read_thermal_zones(root: Path = THERMAL_ROOT) -> list[ZoneReading]:
    """Read every thermal zone under root that reports a temperature.

    Args:
        root: Directory holding the thermal_zone* entries.

    Returns:
        Readings in zone-number order. Zones that cannot be read or hold no
        number are left out; the list is empty when root cannot be listed.
    """
    try:
        names = os.listdir(root)
    except OSError:
        return []
    zones = sorted(
        (int(match.group(1)), root / name) for name in names if (match := _ZONE_DIR.fullmatch(name))
    )
    readings = (read_zone(zone_dir) for _, zone_dir in zones)
    return [reading for reading in readings if reading is not None]


def pick_cpu_zone(readings: list[ZoneReading]) -> ZoneReading | None:
    """Choose the reading published as the backseat temperature.

    Args:
        readings: Output of read_thermal_zones.

    Returns:
        The first zone whose type starts with 'cpu' (cpu-thermal on a Jetson,
        cpu_thermal on a Raspberry Pi), else the first reading, else None.
    """
    for reading in readings:
        if reading.name.startswith('cpu'):
            return reading
    return readings[0] if readings else None

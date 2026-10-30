"""Fleet link: exchange vehicle states as small UDP datagrams. Free of ROS.

Each vehicle has its own ROS domain, so peer states travel outside DDS. A lost datagram
costs nothing: the next one replaces it, and the receiver ages what it last heard. A forged
datagram can only make the helm steer away, because peer data never commands motion.
"""

from __future__ import annotations

import json
import math
import socket
from dataclasses import dataclass

VERSION = 1


@dataclass(frozen=True)
class PeerReport:
    """One state report from another vehicle."""

    vehicle: str
    lat: float
    lon: float
    heading_rad: float
    speed_mps: float


def encode(report: PeerReport) -> bytes:
    """Return the datagram for ``report``."""
    return json.dumps(
        {
            'v': VERSION,
            'vehicle': report.vehicle,
            'lat': report.lat,
            'lon': report.lon,
            'heading_rad': report.heading_rad,
            'speed_mps': report.speed_mps,
        }
    ).encode('utf-8')


def decode(data: bytes) -> PeerReport | None:
    """Return the report in ``data``, or None if it is not a valid one."""
    try:
        doc = json.loads(data.decode('utf-8'))
        if doc['v'] != VERSION:
            return None
        report = PeerReport(
            str(doc['vehicle']),
            float(doc['lat']),
            float(doc['lon']),
            float(doc['heading_rad']),
            float(doc['speed_mps']),
        )
    except (ValueError, KeyError, TypeError, AttributeError):
        return None
    numbers = (report.lat, report.lon, report.heading_rad, report.speed_mps)
    if not report.vehicle or not all(math.isfinite(n) for n in numbers):
        return None
    return report


def parse_peers(text: str, default_port: int) -> list[tuple[str, int]]:
    """Parse ``host`` or ``host:port`` entries separated by commas or spaces."""
    peers = []
    for entry in text.replace(',', ' ').split():
        host, _, port = entry.partition(':')
        peers.append((host, int(port) if port else default_port))
    return peers


class UdpLink:
    """Send to a fixed list of peers and collect whatever arrives."""

    def __init__(self, port: int, peers: list[tuple[str, int]]) -> None:
        self.peers = peers
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.bind(('0.0.0.0', port))
        self._sock.setblocking(False)

    def send(self, data: bytes) -> list[str]:
        """Send ``data`` to every peer; return the errors, one per peer that failed."""
        errors = []
        for peer in self.peers:
            try:
                self._sock.sendto(data, peer)
            except OSError as exc:
                errors.append(f'{peer[0]}: {exc}')
        return errors

    def receive(self) -> list[bytes]:
        """Return every datagram waiting."""
        datagrams = []
        while True:
            try:
                data, _ = self._sock.recvfrom(1024)
            except BlockingIOError:
                return datagrams
            except OSError:
                return datagrams
            datagrams.append(data)

    def close(self) -> None:
        """Close the socket."""
        self._sock.close()

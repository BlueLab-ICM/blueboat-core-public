"""Tests of the fleet link."""

import socket
import time

import pytest
from core_helm.fleet import PeerReport, UdpLink, decode, encode, parse_peers


def test_report_round_trip() -> None:
    report = PeerReport('boat1', 41.0, 2.0, 1.5, 0.8)
    assert decode(encode(report)) == report


@pytest.mark.parametrize(
    'data',
    [
        b'',
        b'not json',
        b'[]',
        b'{"v": 2, "vehicle": "a", "lat": 1, "lon": 1, "heading_rad": 0, "speed_mps": 0}',
        b'{"v": 1, "vehicle": "", "lat": 1, "lon": 1, "heading_rad": 0, "speed_mps": 0}',
        b'{"v": 1, "vehicle": "a", "lat": "x", "lon": 1, "heading_rad": 0, "speed_mps": 0}',
        b'{"v": 1, "vehicle": "a", "lat": NaN, "lon": 1, "heading_rad": 0, "speed_mps": 0}',
        b'{"v": 1, "vehicle": "a"}',
        b'\xff\xfe',
    ],
)
def test_bad_datagrams_are_dropped(data: bytes) -> None:
    assert decode(data) is None


def test_parse_peers() -> None:
    assert parse_peers('a, b:15000  c', 14660) == [('a', 14660), ('b', 15000), ('c', 14660)]
    assert parse_peers('', 14660) == []


def test_two_links_exchange_a_datagram() -> None:
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
        probe.bind(('127.0.0.1', 0))
        port = probe.getsockname()[1]
    other = port + 1
    a = UdpLink(port, [('127.0.0.1', other)])
    b = UdpLink(other, [('127.0.0.1', port)])
    try:
        assert a.send(b'hello') == []
        deadline = time.monotonic() + 2.0
        received: list[bytes] = []
        while not received and time.monotonic() < deadline:
            received = b.receive()
            time.sleep(0.01)
        assert received == [b'hello']
    finally:
        a.close()
        b.close()


def test_unreachable_peer_is_reported_not_raised() -> None:
    link = UdpLink(0, [('256.0.0.1', 1)])
    try:
        assert len(link.send(b'x')) == 1
    finally:
        link.close()

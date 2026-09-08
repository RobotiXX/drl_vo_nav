"""Tests for buffered nav2py transport framing."""

import socket

from nav2py.interfaces import _DelimitedMessageReader


def test_reader_handles_fragmented_large_message():
    """A large payload can arrive in chunks without per-byte reads."""
    receiving, sending = socket.socketpair()
    reader = _DelimitedMessageReader()
    payload = b'1.25,' * 20000
    try:
        sending.sendall(b'scan\x01' + payload[:40000])
        sending.sendall(payload[40000:] + b'\x03')
        name, content = reader.receive(receiving)
    finally:
        receiving.close()
        sending.close()
    assert name == 'scan'
    assert content == [payload]


def test_reader_retains_coalesced_following_message():
    """One socket read may contain more than one framed message."""
    receiving, sending = socket.socketpair()
    reader = _DelimitedMessageReader()
    try:
        sending.sendall(b'first\x01one\x03second\x01two\x01three\x03')
        assert reader.receive(receiving) == ('first', [b'one'])
        assert reader.receive(receiving) == ('second', [b'two', b'three'])
    finally:
        receiving.close()
        sending.close()

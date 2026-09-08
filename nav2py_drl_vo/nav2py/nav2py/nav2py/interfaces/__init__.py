"""
nav2py interfaces
"""

import abc
import socket
import struct
import sys
import typing


class _DelimitedMessageReader:
    """Read nav2py messages without one system call and copy per byte."""

    def __init__(self) -> None:
        self._buffer = bytearray()

    def receive(
        self, conn: socket.socket
    ) -> typing.Tuple[str, typing.List[bytes]]:
        """Return one delimiter-framed message and retain following bytes."""
        separator = bytes((1,))
        while True:
            end_index = self._buffer.find(3)
            if end_index >= 0:
                payload = bytes(self._buffer[:end_index])
                del self._buffer[:end_index + 1]
                name, *content = payload.split(separator)
                return name.decode(), content

            chunk = conn.recv(65536)
            if not chunk:
                raise ConnectionError('nav2py controller socket closed')
            self._buffer.extend(chunk)


class nav2py_costmap_controller(abc.ABC):
    """
    nav2py costmap controller base
    """

    _socket: socket.socket
    _conn: socket.socket
    __callbacks: typing.Dict[str, typing.Callable]

    def _register_callback(self, name: str, fn: typing.Callable):
        self.__callbacks[name] = fn

    def __init__(
        self,
        host: str,
        port: int
    ) -> None:

        self.__callbacks = {}

        self._socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._socket.bind((host, port))
        self._socket.listen()

        port = self._socket.getsockname()[1]
        port_message = struct.pack('!H', port)
        sys.stdout.buffer.write(port_message)
        sys.stdout.buffer.write(int.to_bytes(187, 1, 'big'))
        sys.stdout.buffer.write(int.to_bytes(201, 1, 'big'))
        sys.stdout.buffer.flush()

    def serve(self):
        conn, addr = self._socket.accept()
        self._conn = conn
        reader = _DelimitedMessageReader()

        with conn:
            while True:
                name, content = reader.receive(conn)
                if name in self.__callbacks:
                    self.__callbacks[name](content)

    def _send_cmd_vel(self, linear_x: float, angular_z: float):
        self._conn.sendall(struct.pack('!dd', linear_x, angular_z))

    def cleanup(self):
        self._socket.close()

"""Test fixtures: a fake serial port that stands in for `serial.Serial`."""

from __future__ import annotations

import time

import pytest


class FakeSerial:
    def __init__(self, *args, **kwargs):
        self.is_open = True
        self.timeout = kwargs.get("timeout", 1.0)
        self.written = bytearray()
        self._queue: list[bytes] = []
        # Optional auto-response programmed by tests (mimics real GRBL behavior):
        # whenever the controller sends '?', enqueue this status line.
        self._auto_status: bytes | None = None
        # Optional FIFO of pre-set responses; whenever a \n-terminated line is
        # written, the next one is enqueued (mimics GRBL ack'ing each line).
        self._line_responses: list[bytes] = []

    def readline(self) -> bytes:
        if self._queue:
            return self._queue.pop(0)
        # Yield CPU briefly so polling loops don't burn it in tests.
        time.sleep(0.005)
        return b""

    def write(self, data: bytes) -> int:
        data = bytes(data)
        self.written += data
        if data == b"?" and self._auto_status is not None:
            self._queue.append(self._auto_status)
        elif data.endswith(b"\n") and self._line_responses:
            self._queue.append(self._line_responses.pop(0))
        return len(data)

    def flush(self) -> None:
        pass

    def close(self) -> None:
        self.is_open = False


@pytest.fixture
def fake_serial(monkeypatch):
    """Replace `serial.Serial` with a FakeSerial factory.

    The returned dict has:
      - "queue": list of bytes lines pre-loaded into each new instance
                 (defaults to a GRBL banner so connect() succeeds quickly).
      - "instances": list of FakeSerial objects created during the test.

    Tests typically:
      - assert against `state["instances"][-1].written`,
      - enqueue further responses via `_queue.append(...)`,
      - or program auto-responses with `_auto_status` / `_line_responses`.
    """
    state: dict = {
        "queue": [b"Grbl 1.1f ['$' for help]\n"],
        "instances": [],
    }

    def factory(*args, **kwargs):
        s = FakeSerial(*args, **kwargs)
        s._queue = list(state["queue"])
        state["instances"].append(s)
        return s

    import serial as pyserial

    monkeypatch.setattr(pyserial, "Serial", factory)
    return state

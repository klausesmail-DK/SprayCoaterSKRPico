"""Test fixtures: a fake serial port that stands in for `serial.Serial`."""

from __future__ import annotations

import sys
import time
import types

import pytest

# The planner lives inside spray_gui, which imports Tkinter at module scope.
# None of the logic under test touches a widget, so on a machine without Tk -
# a CI runner, a headless box - a stub stands in rather than the whole suite
# being unrunnable. Where Tk is present this does nothing.
try:  # pragma: no cover
    import tkinter  # noqa: F401
except ImportError:  # pragma: no cover
    class _Stub:
        def __init__(self, *a, **k):
            pass

        def __getattr__(self, name):
            return _Stub()

        def __call__(self, *a, **k):
            return _Stub()

    class _StubModule(types.ModuleType):
        def __getattr__(self, name):
            return _Stub

    for _name in ("tkinter", "tkinter.ttk", "tkinter.filedialog",
                  "tkinter.messagebox", "tkinter.simpledialog"):
        sys.modules[_name] = _StubModule(_name)
    sys.modules["tkinter"].TclError = type("TclError", (Exception,), {})
    for _sub in ("ttk", "filedialog", "messagebox", "simpledialog"):
        setattr(sys.modules["tkinter"], _sub, sys.modules["tkinter." + _sub])


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
        # GRBL took its status request as a bare realtime byte; the stage
        # takes it as a line, like every other command. Accept both so the
        # fixture reflects the protocol actually in use.
        if data.strip() == b"?" and self._auto_status is not None:
            self._queue.append(self._auto_status)
        elif data.endswith(b"\n") and self._line_responses:
            self._queue.append(self._line_responses.pop(0))
        return len(data)

    def flush(self) -> None:
        pass

    def reset_input_buffer(self) -> None:
        """The transport clears the port before each transaction, so the
        fake has to be clearable too - otherwise a reply queued by a test
        would be thrown away before it could be read."""
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
    # connect() probes with a status request rather than waiting for a
    # banner, so what a fresh port must offer is a status line.
    # connect() probes with a status request rather than waiting for a
    # banner, and then reads the limits, so a fresh port has to offer both.
    # Without the limits reply every connect would sit out its timeout.
    state: dict = {
        "queue": [
            b"<Idle|MPos:6.000,6.000>\n",
            b"[LIM:X3.00..94.00,Y3.00..139.00]\n",
            b"[FEED:MAX2400,TRAVEL1800,MIN30]\n",
            b"[ACC:X800,Y800,V2.00]\n",
            b"ok\n",
        ],
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

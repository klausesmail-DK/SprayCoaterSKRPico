"""Pico XY spray stage controller, shaped like the GRBL one it replaces.

The spray program talks GRBL: `$H`, `$J=`, realtime `?`, `0x85`, `0x21`, and
G-code with a `G21 G90 M5` preamble. The stage firmware talks a much smaller
dialect of its own. Rather than rewrite the 2500-line GUI around a new
protocol, this class speaks GRBL upward and the stage's dialect downward, so
`spray_gui` keeps its existing call sites.

WHAT IS TRANSLATED, AND WHY EACH ONE NEEDS TRANSLATING

  ?, $H, G0, G1, G4      pass straight through; the firmware speaks these
  G21 G90 M3 M5 M8 M9    silently acked - units and absolute mode are the
                         only thing this machine does, and there is no
                         spindle or coolant to switch
  Z words                dropped. There is no Z axis: the standoff is fixed.
                         Status reports Z as 0 so the GUI's three-axis
                         plumbing and stored cup heights keep working.
  $J=G91 G21 X10 F600    rewritten as an absolute G1, because the firmware
                         has no jog and no relative mode. Needs the current
                         position, so this class tracks it.
  continuous $J=         run as a chain of short absolute moves on a worker
                         thread. GRBL's jog cancel (0x85) discards a queued
                         jog; here there is no queue, and aborting a move
                         mid-flight would lose the step count and therefore
                         the position. Chunking means a cancel is just "stop
                         sending the next chunk", and the machine stays homed.
  0x85                   stop the chunk chain (position preserved)
  0x21 / 0x18            '!' - a real abort. Position IS lost, which is
                         correct for STOP: after an abort nobody knows where
                         the carriages are, so the firmware goes to Alarm and
                         the GUI must re-home.
  $$                     parsed into limits, feeds and acceleration

THE SIGN FLIP
  Above this line, coordinates are NEGATIVE: zero is the home corner and the
  work area runs away from it into negative X and Y, exactly as on the
  Shapeoko. Below it, the firmware counts positive millimetres from home,
  which is the natural way to write soft limits and lead distances.

  Both conventions are defensible and neither is worth arguing about, so the
  translation happens here, in two methods, rather than being half-applied
  across a GUI and a planner. Everything the operator sees and types, and
  every recipe on disk, is in the negative frame they already know.

THE ONE THING THAT IS NOT A TRANSLATION
  GRBL acks a line when it has BUFFERED it, so the GUI polls status to find
  out when motion finished. This firmware acks when the move has actually
  COMPLETED - there is no look-ahead buffer. `send` therefore blocks for the
  duration of the move, and its default timeout has to cover the longest
  single move rather than the longest queueing delay. `_wait_done` in the GUI
  still works: by the time it asks, the machine is already Idle.
"""

from __future__ import annotations

import re
import threading
import time
from dataclasses import dataclass

import serial
from serial.tools import list_ports

_LIM_RE = re.compile(r"\[LIM:X([-\d.]+)\.\.([-\d.]+),Y([-\d.]+)\.\.([-\d.]+)\]")
_FEED_RE = re.compile(r"\[FEED:MAX([\d.]+),TRAVEL([\d.]+),MIN([\d.]+)\]")
_ACC_RE = re.compile(r"\[ACC:X([\d.]+),Y([\d.]+),V([\d.]+)\]")
_MPOS_RE = re.compile(r"<([^|]+)\|MPos:([-\d.]+),([-\d.]+)")
_WORD_RE = re.compile(r"([A-Z])\s*(-?\d*\.?\d+)")

# G-code the GUI emits that this machine has no use for but must not reject.
_NOOP_WORDS = {"G17", "G21", "G40", "G49", "G54", "G80", "G90", "G94",
               "M3", "M4", "M5", "M7", "M8", "M9", "M30"}

JOG_CHUNK_MM = 1.0        # continuous-jog granularity; also the stop latency
JOG_CHUNK_MIN = 0.2


class PicoError(Exception):
    """Raised when the stage answers `error: ...`.

    Carries a `code` so the GUI's GRBL-shaped except clauses keep working;
    this firmware reports reasons in words, so the code is always 1.
    """

    def __init__(self, message: str, code: int = 1):
        self.code = code
        self.response = message
        super().__init__(f"stage error: {message}")


@dataclass
class PortInfo:
    device: str
    description: str

    def __str__(self) -> str:
        return f"{self.device}\t{self.description}".rstrip()


@dataclass
class Limits:
    """Travel in HOST coordinates: negative, zero at the home corner.

    min_* is the far end of travel and max_* is as close to the switches as
    the machine goes, so both are negative and min < max. The firmware
    reports the mirror image of this and PicoController flips it on the way
    in; nothing above this class ever sees a positive coordinate.
    """

    min_x: float = -94.0
    max_x: float = -3.0
    min_y: float = -139.0
    max_y: float = -3.0
    max_feed: float = 2400.0
    travel_feed: float = 1800.0
    min_feed: float = 30.0
    accel_x: float = 800.0
    accel_y: float = 800.0
    junction: float = 2.0

    def accel(self, axis: str) -> float:
        return self.accel_x if axis.upper() == "X" else self.accel_y

    def lead(self, feed: float, axis: str) -> float:
        """Run-up needed to be at `feed` by the start of the coated span.

        Same v^2 = 2*a*d the firmware uses, plus half a millimetre so the
        span never begins exactly at the point the ramp ends.
        """
        v = feed / 60.0
        v0 = self.junction
        return max(0.0, (v * v - v0 * v0) / (2.0 * self.accel(axis))) + 0.5


def find_ports() -> list[PortInfo]:
    """Likely stage ports. Same USB CDC heuristic as the GRBL version."""
    keywords = ("usbmodem", "usbserial", "ttyacm", "ttyusb")
    out: list[PortInfo] = []
    for p in list_ports.comports():
        name = (p.device or "").lower()
        if any(k in name for k in keywords):
            out.append(PortInfo(device=p.device, description=p.description or ""))
    return out


@dataclass
class Status:
    """Deliberately the same shape as streamer.Status, so parse_status'
    consumers in the GUI need no changes."""

    state: str
    mpos: tuple[float, float, float] | None
    wpos: tuple[float, float, float] | None
    feed: float | None
    spindle: int | None
    raw: str


class PicoController:
    """Serial wrapper for the stage, with a GRBL-shaped surface."""

    def __init__(self, port: str, baudrate: int = 115200, timeout: float = 1.0):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self._serial: serial.Serial | None = None
        self.banner: str = ""
        self.limits = Limits()
        # Every port transaction goes through this. The GUI has its own lock
        # for the calls it makes, but the jog thread here is a second writer,
        # so the transport has to serialise on its own account.
        self._lock = threading.RLock()
        self._pos: tuple[float, float] | None = None
        self._jog_stop = threading.Event()
        self._jog_thread: threading.Thread | None = None
        self._last_feed = 0.0

    # -- lifecycle ---------------------------------------------------------
    def __enter__(self) -> PicoController:
        self.connect()
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()

    @property
    def is_open(self) -> bool:
        return self._serial is not None and self._serial.is_open

    def connect(self, boot_wait: float = 12.0) -> None:
        """Open the port and confirm the stage is answering.

        Waiting for the banner, the way this did at first, is a mistake: the
        banner is printed once when the board boots, and the board is
        usually ALREADY serving by the time the GUI opens. There is then no
        banner to wait for and the connect sits out its whole timeout before
        giving up - which is why connecting felt slower than GRBL, where the
        board resets on DTR and always greets you.

        So ask rather than listen. A status request answers immediately if
        the firmware is up; if the board happens to be mid-boot, the retry
        loop covers main.py's Ctrl-C window and the driver configuration.
        """
        if self.is_open:
            return
        self._serial = serial.Serial(port=self.port, baudrate=self.baudrate,
                                     timeout=self.timeout)
        # A board that has just been reset is still printing its driver
        # configuration, a dozen lines fed by slow UART reads. Give each
        # probe long enough to outlast that chatter and still find the
        # status line behind it - a short window would expire mid-flood,
        # and every retry would start the race again from scratch.
        deadline = time.monotonic() + boot_wait
        t0 = time.monotonic()
        while True:
            st = self.read_status(timeout=1.5)
            if st is not None:
                self.banner = "PicoStage (%s) in %.1f s" % (
                    st.state, time.monotonic() - t0)
                break
            if time.monotonic() >= deadline:
                # Opening the port is not connecting. A serial device that
                # never answers is a dead link, and reporting it as success
                # meant the GUI went on to show default travel limits
                # labelled "from board" and let the operator press HOME
                # against nothing.
                self.close()
                raise PicoError(
                    "no reply in %.0f s. Either serve() is not running on "
                    "the board (reset it, or check main.py), or another "
                    "program is holding %s - macOS lets two processes open "
                    "the same /dev/cu port and they each get half the bytes"
                    % (boot_wait, self.port))
            time.sleep(0.1)
        try:
            self.read_limits()
        except (PicoError, TimeoutError, RuntimeError):
            pass

    def close(self) -> None:
        self.stop_jog()
        if self._serial is None:
            return
        for fn in (self._serial.flush, self._serial.close):
            try:
                fn()
            except Exception:
                pass
        self._serial = None

    # -- raw io ------------------------------------------------------------
    def read_line(self, timeout: float | None = None) -> str:
        if self._serial is None:
            raise RuntimeError("Not connected")
        if timeout is None:
            raw = self._serial.readline()
        else:
            old = self._serial.timeout
            self._serial.timeout = timeout
            try:
                raw = self._serial.readline()
            finally:
                self._serial.timeout = old
        return raw.decode("utf-8", errors="replace").strip()

    def write_line(self, line: str) -> int:
        if self._serial is None:
            raise RuntimeError("Not connected")
        payload = (line.rstrip() + "\n").encode("ascii")
        self._serial.write(payload)
        self._serial.flush()
        return len(payload)

    def drain(self, quiet_seconds: float = 0.2) -> list[str]:
        out: list[str] = []
        with self._lock:
            if self._serial is None:
                raise RuntimeError("Not connected")
            deadline = time.monotonic() + quiet_seconds
            while time.monotonic() < deadline:
                line = self.read_line(timeout=quiet_seconds)
                if line:
                    out.append(line)
                    deadline = time.monotonic() + quiet_seconds
                else:
                    time.sleep(0.005)
        return out

    # -- the translating layer --------------------------------------------
    def send(self, line: str, response_timeout: float = 600.0) -> str:
        """Send one GRBL-ish line, translated, and wait for it to complete.

        The default timeout is long because an ack here means the motion has
        finished, not merely that it was queued - and a slow pass across the
        full Y travel at the minimum feed takes minutes.
        """
        payload = line.strip()
        if not payload:
            return ""
        translated = self._translate(payload)
        if translated is None:
            return "ok"          # a no-op the machine has no opinion about
        return self._transact(translated, response_timeout)

    def _transact(self, payload: str, response_timeout: float) -> str:
        with self._lock:
            if self._serial is None:
                raise RuntimeError("Not connected")
            self._serial.reset_input_buffer()
            self.write_line(payload)
            deadline = time.monotonic() + response_timeout
            collected: list[str] = []
            while time.monotonic() < deadline:
                resp = self.read_line(timeout=0.5)
                if not resp:
                    continue
                if resp == payload:
                    continue                  # echo, if the port is echoing
                if resp == "ok":
                    return "\n".join(collected) if collected else "ok"
                if resp.startswith("error:"):
                    raise PicoError(resp.split(":", 1)[1].strip(),
                                    )
                if resp.startswith("<"):
                    self._note_status(resp)
                    continue
                collected.append(resp)
            raise TimeoutError(
                f"No stage response within {response_timeout}s for: {payload!r}")

    def _translate(self, payload: str) -> str | None:
        """GRBL in, stage dialect out. None means "nothing to send".

        The no-op words are stripped token by token rather than matched at
        the start of the line, because the GUI writes modal prefixes:
        `G90 G1 Z-70 F0` begins with G90, and a startswith test on "G1"
        would miss it and hand the firmware a Z word and a zero feed.
        """
        up = payload.upper()
        if up.startswith("$J="):
            return self._translate_jog(up[3:])
        if up.startswith("$") or up in ("?", "!"):
            return up
        tokens = [t for t in up.split() if t not in _NOOP_WORDS]
        if not tokens:
            return None
        rest = " ".join(tokens)
        if rest.startswith("G4"):
            return rest
        if rest.startswith(("G0", "G1")):
            return self._translate_move(rest)
        if _WORD_RE.search(rest) and not rest.startswith("G"):
            # Modal motion: coordinates with no G word carry the last one.
            return self._translate_move("G1 " + rest)
        return rest

    def _translate_move(self, up: str) -> str | None:
        """Strip Z, keep XY/F, and carry E/I/K/Q through untouched.

        A Z-only move becomes nothing at all, which is what makes the GUI's
        cup logic work unchanged: plan_cup_move still emits its Z leg, and it
        simply evaporates here.
        """
        words = dict(_WORD_RE.findall(up))
        g = words.get("G", "1")
        keep = []
        for letter in ("X", "Y"):
            if letter in words:
                # The sign flip. Host coordinates are negative from the home
                # corner; the firmware counts positive millimetres from it.
                keep.append("%s%.3f" % (letter, -float(words[letter])))
        for letter in ("F", "E", "I", "K", "Q"):
            if letter in words:
                keep.append(f"{letter}{words[letter]}")
        if not any(k[0] in "XY" for k in keep):
            return None
        if "F" in words:
            self._last_feed = float(words["F"])
        elif g.startswith("1"):
            # The firmware refuses a G1 without a feed on purpose. The GUI
            # always supplies one; if a hand-typed line does not, fall back
            # to the last feed rather than failing, which is what GRBL does.
            keep.append(f"F{self._last_feed or self.limits.travel_feed:g}")
        return f"G{int(float(g))} " + " ".join(keep)

    def _translate_jog(self, body: str) -> str | None:
        """`G91 G21 X10.5 F600` -> an absolute G1 to where that lands."""
        words = dict(_WORD_RE.findall(body))
        if self._pos is None:
            raise PicoError("no position yet - home first")
        x, y = self._pos
        relative = "G91" in body
        feed = float(words.get("F", self.limits.travel_feed))
        tx, ty = x, y
        if "X" in words:
            tx = x + float(words["X"]) if relative else float(words["X"])
        if "Y" in words:
            ty = y + float(words["Y"]) if relative else float(words["Y"])
        if abs(tx - x) < 1e-4 and abs(ty - y) < 1e-4:
            return None
        # Still in host coordinates; _translate_move does the sign flip, so
        # the arithmetic above never has to think about which frame it is in.
        return self._translate_move(
            "G1 X%.3f Y%.3f F%g" % (tx, ty, min(feed, self.limits.max_feed)))

    # -- realtime ----------------------------------------------------------
    def send_realtime(self, byte: int) -> None:
        """The three realtime bytes the GUI uses.

        0x3F '?'   status
        0x85       jog cancel - stops the chunk chain, keeps the home reference
        0x21 '!'   feed hold, and 0x18 soft reset: both are STOP here, which
                   aborts the move and invalidates position. That is honest
                   rather than convenient - the step count of an interrupted
                   move is unknown, so the machine cannot claim to know where
                   it is until it homes again.
        """
        if self._serial is None:
            raise RuntimeError("Not connected")
        if byte == 0x3F:
            self.read_status()
            return
        if byte == 0x85:
            self.stop_jog()
            return
        self.stop_jog()
        self._pos = None
        with self._lock:
            self._serial.write(b"!\n")
            self._serial.flush()

    def read_status(self, timeout: float = 1.0) -> Status | None:
        with self._lock:
            if self._serial is None:
                raise RuntimeError("Not connected")
            self._serial.reset_input_buffer()
            self.write_line("?")
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                line = self.read_line(timeout=max(0.05, deadline - time.monotonic()))
                if not line:
                    continue
                st = self._note_status(line)
                if st is not None:
                    return st
        return None

    def _note_status(self, line: str) -> Status | None:
        """Parse a stage status line and remember the position.

        Z is reported as a constant 0.0 so everything downstream that expects
        three axes - the position readout, the cup, jog_target - keeps
        working without knowing this machine is flat.
        """
        if "MPos:none" in line:
            self._pos = None
            return Status(state="Alarm", mpos=None, wpos=None, feed=None,
                          spindle=None, raw=line)
        m = _MPOS_RE.match(line)
        if not m:
            return None
        state = m.group(1)
        x, y = -float(m.group(2)), -float(m.group(3))   # into host coordinates
        self._pos = (x, y)
        return Status(state=state, mpos=(x, y, 0.0), wpos=(x, y, 0.0),
                      feed=None, spindle=None, raw=line)

    # -- limits ------------------------------------------------------------
    def read_limits(self) -> Limits:
        text = self._transact("$$", 5.0)
        lim = Limits()
        m = _LIM_RE.search(text)
        if m:
            # Negated and swapped: the firmware's near limit is the host's
            # far one once the sign turns over.
            lim.min_x, lim.max_x = -float(m.group(2)), -float(m.group(1))
            lim.min_y, lim.max_y = -float(m.group(4)), -float(m.group(3))
        m = _FEED_RE.search(text)
        if m:
            lim.max_feed = float(m.group(1))
            lim.travel_feed = float(m.group(2))
            lim.min_feed = float(m.group(3))
        m = _ACC_RE.search(text)
        if m:
            lim.accel_x = float(m.group(1))
            lim.accel_y = float(m.group(2))
            lim.junction = float(m.group(3))
        self.limits = lim
        return lim

    # -- continuous jog ----------------------------------------------------
    def start_jog(self, axis: str, distance: float, feed: float) -> None:
        """Hold-to-move: one move toward the far end, cancelled on release.

        The move runs on a worker thread so the button press returns at once
        and the GUI stays live. `send` blocks until the firmware acks, which
        for this machine means the motion has ended - either at the limit or
        because stop_jog() asked it to.
        """
        self.stop_jog()
        self._jog_stop.clear()

        def run() -> None:
            try:
                self.send(f"$J=G91 G21 {axis}{distance:.3f} F{feed:g}",
                          response_timeout=600.0)
            except (PicoError, TimeoutError, RuntimeError, OSError):
                pass

        self._jog_thread = threading.Thread(target=run, daemon=True)
        self._jog_thread.start()

    def stop_jog(self) -> None:
        """Ask a running jog to decelerate and stop where it is.

        This is the one control that must NOT go through the lock: the jog
        thread is holding it for the whole move, and a button release that
        waited for the lock would be a release that did nothing. Writing a
        single byte straight at the port is safe precisely because the
        firmware is watching stdin for exactly this while it moves.

        '~' rather than '!' on purpose: the firmware ramps down and reads the
        step counters, so the machine still knows where it is afterwards. A
        '!' would stop faster and cost the home reference.
        """
        t = self._jog_thread
        if t is None:
            return
        self._jog_stop.set()
        if t.is_alive() and self._serial is not None:
            try:
                self._serial.write(b"~")
                self._serial.flush()
            except Exception:
                pass
            t.join(timeout=5.0)
        self._jog_thread = None


def parse_status(line: str) -> Status | None:
    """Module-level parser, for symmetry with the streamer it replaces."""
    if "MPos:none" in line:
        return Status(state="Alarm", mpos=None, wpos=None, feed=None,
                      spindle=None, raw=line)
    m = _MPOS_RE.match(line)
    if not m:
        return None
    x, y = -float(m.group(2)), -float(m.group(3))
    return Status(state=m.group(1), mpos=(x, y, 0.0), wpos=(x, y, 0.0),
                  feed=None, spindle=None, raw=line)

"""
XY spray stage on a BIGTREETECH SKR Pico v1.0  --  MicroPython

    Y (gantry)     TWO motors on the Y and Z driver channels, run in lockstep
    X (crossbeam)  one motor on the X driver channel

BOARD PIN MAP (SKR Pico v1.0, from the Klipper reference config)
    UART   TX GP8, RX GP9, shared by all four drivers (series resistor is
           on the board - no external 1k needed)
    X      STEP GP11  DIR GP10  EN GP12   endstop GP4   uart address 0
    Y      STEP GP6   DIR GP5   EN GP7    endstop GP3   uart address 2
    Z      STEP GP19  DIR GP28  EN GP2    endstop GP25  uart address 1
    E      STEP GP14  DIR GP13  EN GP15                 uart address 3

    The addresses are NOT in connector order: X=0, Z=1, Y=2, E=3.

GANTRY LOCKSTEP
    Each channel has its own STEP pin, so equal stepping is enforced in
    firmware: both state machines live on PIO0, are given the same clock
    divider, and are started and stopped by a single write to PIO0's CTRL
    register so they begin on the same clock edge. Nothing may ever enable
    or disable one without the other - which is why every start and stop
    goes through _sm_enable().

HOW MOTION WORKS HERE (and why it is not how the Shapeoko did it)
    GRBL holds one feed rate per move, so the spray program approximated a
    feed ramp by chopping each pass into 2 mm constant-feed segments. This
    firmware has no such limitation. The PIO program is handed an exact
    step count, and the step RATE is then swept continuously by writing the
    state machine's clock divider while it runs - a few hundred rate
    updates per second against GRBL's fifteen per pass. A gradient is one
    command and one genuinely smooth sweep.

    Two properties matter, and both fall out of the same design:
      - the step COUNT comes from a FIFO, so position never depends on
        timing and cannot drift no matter what the divider does;
      - the step RATE is free, so the speed profile is arbitrary.

    The mirrored return stroke keeps its exact-speed-at-every-point
    property, acceleration included: see pass_line().

MEASURED LIMITS (24 V, 1/16 ustep, 0.40 A, Tr6x4, 800 steps/mm)
    Both axes hold position to +-0.006 mm out to 40 mm/s and lose about
    0.157 mm at 50 mm/s. 40 mm/s (2400 mm/min) is the hard ceiling and is
    enforced in software; 30 mm/s (1800 mm/min) is the working traverse.
    Acceleration is per axis. X is measured clean to 800 mm/s2 over
    repeated full-speed reversals; Y has not been measured and is left at
    the old conservative default. Run accel_test('Y') before trusting it.
"""

import machine
import rp2
import select
import sys
import time
from machine import UART, Pin, mem32

# ---------------------------------------------------------------- config ---
UART_ID, TX_PIN, RX_PIN, BAUD = 1, 8, 9, 115200      # GP8/GP9 are UART1

MICROSTEPS = 16
IRUN_A, IHOLD_FRAC, RSENSE = 0.40, 0.5, 0.11
TPWMTHRS_VAL = 2000

# Feed limits, mm/min. MAX_FEED is the measured step-loss cliff and is a
# hard clamp: a move asking for more is REFUSED, not quietly slowed.
# Silently flattening the fast end of a gradient would corrupt the coating
# profile rather than merely the timing, so the host has to rescale instead.
MAX_FEED = 2400.0
TRAVEL_FEED = 1800.0
MIN_FEED = 30.0

START_MMS = 2.0            # junction speed: where a ramp starts and ends
ACCEL_MMSS = 50.0          # fallback for an axis with no measured value

AXES = (
    {
        "name": "Y",
        "motors": (
            {"sm": 0, "step": 6,  "dir": 5,  "en": 7, "addr": 2, "shaft": 0},
            {"sm": 1, "step": 19, "dir": 28, "en": 2, "addr": 1, "shaft": 0},
        ),
        "sw": 3, "sw_active": 0,
        "lead_mm": 4.0, "max_travel": 140.0,
        "home_dir": 1,
        "fast_hz": 8000, "slow_hz": 800,
        # Measured clean at every rate up to 1600 mm/s2 - 12 full-speed
        # reversals over 131 mm at F2400, nothing lost. 800 is adopted for
        # the margin, not because 1600 failed: the highest rate that passes
        # is a ceiling, not a setting, and none of this was measured with
        # the spray head and pressurised hoses fitted.
        "accel": 800.0,
    },
    {
        "name": "X",
        "motors": (
            {"sm": 2, "step": 11, "dir": 10, "en": 12, "addr": 0, "shaft": 0},
        ),
        "sw": 4, "sw_active": 0,
        "lead_mm": 4.0, "max_travel": 95.0,
        "home_dir": 1,
        "fast_hz": 8000, "slow_hz": 800,
        # Measured clean to 800 mm/s2 over 12 full-speed reversals at
        # F2400, with no position lost. 1600 also read clean but the
        # schedule of the day could not actually deliver it, so 800 is what
        # is honestly proven.
        "accel": 800.0,
    },
)

HOME_ORDER = ("X", "Y")
BACKOFF_MM, TRIALS = 2.0, 3
# MIN_POS is the soft limit; PARK_MM is where homing leaves the carriage.
# They are deliberately different: parking ON the limit means the operator
# cannot jog toward home at all after homing, which reads as a broken jog
# wheel rather than as the end of travel.
MIN_POS, PARK_MM = 3.0, 6.0
EDGE_MARGIN = 1.0          # stay this far off the far end of travel

GCONF, GSTAT, IFCNT, IOIN = 0x00, 0x01, 0x02, 0x06
IHOLD_IRUN, TPWMTHRS, TCOOLTHRS = 0x10, 0x13, 0x14
SGTHRS, SG_RESULT, CHOPCONF = 0x40, 0x41, 0x6C
MRES_MAP = {256: 0, 128: 1, 64: 2, 32: 3, 16: 4, 8: 5, 4: 6, 2: 7, 1: 8}

# PIO0 registers. Per-state-machine blocks are 0x18 apart.
PIO0_BASE = 0x50200000
PIO0_CTRL = PIO0_BASE + 0x000      # bits 0-3 enable SM0-SM3
PIO0_CLKDIV = PIO0_BASE + 0x0C8
PIO0_SHIFTCTRL = PIO0_BASE + 0x0D0
PIO0_INSTR = PIO0_BASE + 0x0D8     # write here to inject one instruction
SM_STRIDE = 0x18
PIO_SET_X_0 = 0xE020               # "set x, 0"
PIO_MOV_ISR_X = 0xA0C1             # "mov isr, x"
PIO_PUSH_NOBLOCK = 0x8000          # "push noblock"

SYS_HZ = machine.freq()
# The divider is 16 bits of integer, so step rate has a floor: about 477 Hz
# at 125 MHz with four PIO cycles per step.
MIN_HZ = int(SYS_HZ / (4 * 65535)) + 2

uart = UART(UART_ID, baudrate=BAUD, tx=Pin(TX_PIN), rx=Pin(RX_PIN), timeout=50)

WATCH_STDIN = False        # serve() arms this so '!' can abort a move


# ------------------------------------------------------------ UART layer ---
def crc8_atm(data):
    crc = 0
    for b in data:
        for _ in range(8):
            crc = (((crc << 1) ^ 0x07) & 0xFF) if ((crc >> 7) ^ (b & 1)) \
                else ((crc << 1) & 0xFF)
            b >>= 1
    return crc


def read_reg(reg, addr):
    req = bytearray([0x05, addr, reg & 0x7F, 0])
    req[3] = crc8_atm(req[:3])
    while uart.any():
        uart.read()
    uart.write(req)
    buf = b""
    end = time.ticks_add(time.ticks_ms(), 40)
    while len(buf) < 12 and time.ticks_diff(end, time.ticks_ms()) > 0:
        c = uart.read(12 - len(buf))
        if c:
            buf += c
    for i in range(len(buf) - 7):
        if buf[i] == 0x05 and buf[i+1] == 0xFF and buf[i+2] == (reg & 0x7F):
            p = buf[i:i+8]
            if crc8_atm(p[:7]) == p[7]:
                return (p[3] << 24) | (p[4] << 16) | (p[5] << 8) | p[6]
    return None


def write_reg(reg, val, addr):
    d = bytearray([0x05, addr, (reg & 0x7F) | 0x80,
                   (val >> 24) & 0xFF, (val >> 16) & 0xFF,
                   (val >> 8) & 0xFF, val & 0xFF, 0])
    d[7] = crc8_atm(d[:7])
    while uart.any():
        uart.read()
    uart.write(d)
    time.sleep_ms(5)
    while uart.any():
        uart.read()


def configure_driver(addr, shaft):
    g = read_reg(GCONF, addr)
    if g is None:
        raise RuntimeError("driver %d did not answer" % addr)
    g |= (1 << 7) | (1 << 6)          # mstep_reg_select, pdn_disable
    g &= ~(1 << 0)                    # digital current, ignore VREF
    g &= ~(1 << 2)                    # StealthChop
    g = (g | (1 << 3)) if shaft else (g & ~(1 << 3))
    write_reg(GCONF, g, addr)
    chop = read_reg(CHOPCONF, addr)
    chop = (chop & ~(0x0F << 24)) | (MRES_MAP[MICROSTEPS] << 24) | (1 << 28)
    if (chop & 0x0F) == 0:
        chop |= 3                     # TOFF
    write_reg(CHOPCONF, chop, addr)
    cs = max(0, min(31, int(round(
        IRUN_A * 1.41421 * 32 * (RSENSE + 0.02) / 0.325)) - 1))
    write_reg(IHOLD_IRUN, (2 << 16) | (cs << 8) | int(cs * IHOLD_FRAC), addr)
    write_reg(TPWMTHRS, TPWMTHRS_VAL, addr)
    write_reg(TCOOLTHRS, 0xFFFFF, addr)
    write_reg(SGTHRS, 0, addr)
    return cs


# ------------------------------------------------------ step generation ----
@rp2.asm_pio(set_init=rp2.PIO.OUT_LOW)
def _step_n():
    """Emit exactly N pulses, then report done.

    Four PIO cycles per pulse, so the state machine clock is 4x the step
    rate. The count arrives through the TX FIFO and the completion word
    goes back through the RX FIFO. Because the count is consumed from a
    FIFO rather than timed, the step total is exact no matter how the clock
    divider moves while the loop runs - which is what makes a continuous
    rate sweep safe.
    """
    wrap_target()
    pull(block)                  # N - 1
    mov(x, osr)
    label("tick")
    set(pins, 1)[1]              # 2 cycles high
    set(pins, 0)                 # 1
    jmp(x_dec, "tick")           # 1
    push(block)                  # move finished
    wrap()


def _div_word(hz):
    """PIO CLKDIV register value for a step rate."""
    div = SYS_HZ / (4.0 * hz)
    if div < 1.0:
        div = 1.0
    elif div > 65535.0:
        div = 65535.0
    i = int(div)
    return ((i & 0xFFFF) << 16) | ((int((div - i) * 256.0) & 0xFF) << 8)


def _set_div(sm_id, word):
    mem32[PIO0_CLKDIV + SM_STRIDE * sm_id] = word


def _sm_enable(ids, on):
    """Start or stop several PIO0 state machines in ONE register write.

    This is what keeps the two gantry motors in lockstep: calling
    sm.active(1) twice would start them microseconds apart, and although
    equal clock dividers mean the offset would never grow, starting them on
    the same edge costs nothing and removes the question entirely.
    """
    mask = 0
    for i in ids:
        mask |= 1 << i
    cur = mem32[PIO0_CTRL]
    mem32[PIO0_CTRL] = (cur | mask) if on else (cur & ~mask)


def _panic():
    """Has the host asked the move to end early? Only armed under serve().

    Returns 0 for no, 1 for a graceful stop, 2 for a hard abort.

    The difference matters entirely: a graceful stop decelerates and then
    reads the step count still sitting in the state machine, so position
    survives. A hard abort cuts the drivers and position is lost. Jog
    release wants the first, the STOP button wants the second.
    """
    if not WATCH_STDIN or _poll is None:
        return 0
    try:
        for _obj, ev in _poll.poll(0):
            if ev & select.POLLIN:
                return 2 if sys.stdin.read(1) in ("!", "\x18") else 1
            # POLLHUP or POLLERR: the host closed the port, or the link died.
            #
            # This used to test the poll result for truthiness alone and then
            # call a BLOCKING read(1). A hung-up port polls ready forever and
            # never delivers a byte, so the read never returned: the move
            # stopped here with the drivers live, printing nothing, reading
            # nothing, and the time bound in _sweep could not help because
            # control never got back to the loop. Only the power switch did.
            # Every GUI exit and every disconnect leaves exactly that state
            # behind, which is why it took a few runs to find.
            #
            # Nothing useful can be read from a dead link, so treat it as a
            # stop request: decelerate, keep the step count, keep the home
            # reference, and let serve() find out for itself when its own
            # readline returns.
            return 1
    except Exception:
        pass
    return 0


# ------------------------------------------------------ gradient profile ---
def gradient_feed(frac, f0, f1, mode):
    """Feed at fractional position `frac` along a pass. Ported unchanged
    from the Shapeoko program so both machines shape coatings identically.

    "velocity" interpolates the feed, so speed is linear and thickness,
    which goes as 1/v, comes out hyperbolic. "thickness" interpolates
    1/feed, so the coating is linear - usually what a calibration strip
    wants. Both hit f0 and f1 exactly at the ends.
    """
    if mode == "thickness":
        inv = (1.0 / f0) + ((1.0 / f1) - (1.0 / f0)) * frac
        return 1.0 / inv
    return f0 + (f1 - f0) * frac


def lead_needed(feed, accel):
    """How much run-up a pass needs before it is up to `feed`, in mm.

    v^2 = 2*a*d from the junction speed. This is the number that decides
    whether a gradient's fast end is real or merely requested, so it is
    worth printing rather than guessing.
    """
    v = feed / 60.0
    return max(0.0, (v * v - START_MMS * START_MMS) / (2.0 * accel))


def rate_at(s, length, f0, f1, mode, lead_in, lead_out, accel):
    """Commanded speed, mm/s, once the carriage has travelled `s` mm.

    The gradient is defined over the SPAN - what lies between the lead-in
    and lead-out - and holds flat at f0 before it and f1 after. That
    distinction is the whole point of the lead: starting and stopping take
    distance, and inside the span the coating has to see the feed the
    recipe asked for rather than whatever acceleration allowed. Put the
    lead zones off the slide and the accelerating is over by the time the
    paint matters.

    The commanded curve is then capped by what acceleration can deliver
    from a standstill at each end. Both caps are symmetric, so running the
    move backwards with f0/f1 and the two leads swapped gives the exact
    mirror-image speed profile, ramps included - which is what makes a
    Line/back deposit symmetrically.

    This is a function of DISTANCE, not of time. The run loop integrates
    distance as it goes rather than following a precomputed timetable,
    which matters more than it sounds: a table has a fixed number of
    points, so a short ramp falls between two of them and the machine
    quietly accelerates slower than asked. Here the resolution is the loop
    rate - a few hundred microseconds, tens of microns - so the requested
    acceleration is the delivered one.
    """
    span = length - lead_in - lead_out
    if span > 1e-9:
        u = (s - lead_in) / span
        if u < 0.0:
            u = 0.0
        elif u > 1.0:
            u = 1.0
    else:
        u = s / length if length > 1e-9 else 0.0
    v = gradient_feed(u, f0, f1, mode) / 60.0
    v0sq = START_MMS * START_MMS
    up = (v0sq + 2.0 * accel * (s if s > 0.0 else 0.0)) ** 0.5
    if v > up:
        v = up
    rem = length - s
    dn = (v0sq + 2.0 * accel * rem) ** 0.5 if rem > 0.0 else START_MMS
    if v > dn:
        v = dn
    return v


# ------------------------------------------------------------ machinery ---
class Motor:
    def __init__(self, cfg):
        self.cfg = cfg
        self.addr = cfg["addr"]
        self.shaft = cfg["shaft"]
        self.sm_id = cfg["sm"]
        self.step_pin = Pin(cfg["step"])
        self.dirn = Pin(cfg["dir"], Pin.OUT, value=0)
        self.en = Pin(cfg["en"], Pin.OUT, value=1)
        self.sm = rp2.StateMachine(self.sm_id, _step_n,
                                   freq=8000 * 4, set_base=self.step_pin)

    def arm(self, steps):
        """Load the step count. The state machine is still disabled, so the
        word waits in the FIFO until every axis is released together."""
        if steps < 1:
            return
        self.sm.put(steps - 1)

    def done(self):
        return self.sm.rx_fifo() > 0

    def steps_done(self, total):
        """How many pulses actually came out, for a move stopped early.

        The loop counter lives in the state machine's X register, so after
        stopping it the machine can be asked directly rather than guessed at
        from elapsed time. Injecting `mov isr, x` and a non-blocking push
        through the INSTR register lifts it out without disturbing anything.

        X holds total-1-k after k pulses, so k falls out of that. The answer
        can be one pulse stale if the machine stopped between emitting a
        pulse and decrementing - 1.25 um, which is below what the endstop
        itself repeats to.

        Only valid while the state machine is DISABLED and before reset(),
        which clears X. A completed move has already wrapped X, so callers
        must use `total` for that case rather than asking.
        """
        sm = self.sm_id
        while self.sm.rx_fifo():
            self.sm.get()
        base = PIO0_INSTR + SM_STRIDE * sm
        mem32[base] = PIO_MOV_ISR_X
        mem32[base] = PIO_PUSH_NOBLOCK
        if not self.sm.rx_fifo():
            return total
        x = self.sm.get()
        k = total - 1 - x
        return 0 if k < 0 else (total if k > total else k)

    def reset(self):
        """Return the state machine to a known idle state.

        A move that was stopped early leaves the program counter somewhere
        inside the pulse loop with a part-used count in X, and simply
        pushing the next count would make it finish the OLD move. So: zero
        X by injecting an instruction, let the loop unwind to its blocking
        pull, then clear both FIFOs.

        Unwinding can emit one more step pulse - 1.25 um, and only on paths
        where position is being re-established anyway. Zeroing X through
        the INSTR register avoids depending on where MicroPython happened
        to load the program, which is the one thing a jmp would need.
        """
        sm = self.sm_id
        _sm_enable((sm,), False)
        mem32[PIO0_INSTR + SM_STRIDE * sm] = PIO_SET_X_0
        _set_div(sm, _div_word(40000))
        _sm_enable((sm,), True)
        time.sleep_us(400)
        _sm_enable((sm,), False)
        shift = PIO0_SHIFTCTRL + SM_STRIDE * sm
        w = mem32[shift]
        mem32[shift] = w ^ (1 << 30)     # toggling FJOIN clears both FIFOs
        mem32[shift] = w


class Axis:
    def __init__(self, cfg):
        self.name = cfg["name"]
        self.cfg = cfg
        self.motors = [Motor(m) for m in cfg["motors"]]
        self.sm_ids = [m.sm_id for m in self.motors]
        self.sw = Pin(cfg["sw"], Pin.IN, Pin.PULL_UP)
        self.sw_active = cfg["sw_active"]
        self.home_dir = cfg["home_dir"]
        self.fast_hz = cfg["fast_hz"]
        self.slow_hz = cfg["slow_hz"]
        self.max_travel = cfg["max_travel"]
        self.accel = cfg.get("accel", ACCEL_MMSS)
        self.steps_per_mm = 200 * MICROSTEPS / cfg["lead_mm"]
        self.pos = None

    # -- drivers -----------------------------------------------------------
    def configure(self):
        for m in self.motors:
            v = read_reg(IOIN, m.addr)
            if v is None or ((v >> 24) & 0xFF) != 0x21:
                raise RuntimeError("%s: no TMC2209 at address %d"
                                   % (self.name, m.addr))
            cs = configure_driver(m.addr, m.shaft)
            print("  %s addr %d (sm%d): IRUN cs=%d shaft=%d"
                  % (self.name, m.addr, m.sm_id, cs, m.shaft))

    def verify(self):
        """Motors sharing an axis must advance the same distance per pulse.

        MRES is only obeyed when GCONF bit 7 is set; without it the driver
        follows its MS pins, which on this board are strapped per channel
        for addressing. A driver that missed its GCONF write therefore runs
        a different microstep mode while CHOPCONF still reads back what we
        wrote - so check both.
        """
        modes = []
        for m in self.motors:
            chop = read_reg(CHOPCONF, m.addr)
            g = read_reg(GCONF, m.addr)
            if chop is None or g is None:
                raise RuntimeError("%s addr %d stopped answering"
                                   % (self.name, m.addr))
            mres, reg_sel = (chop >> 24) & 0x0F, (g >> 7) & 1
            print("    addr %d: MRES 1/%-3d mstep_reg_select=%d shaft=%d"
                  % (m.addr, 256 >> mres, reg_sel, (g >> 3) & 1))
            if not reg_sel:
                raise RuntimeError(
                    "%s addr %d: mstep_reg_select CLEAR - following MS pins"
                    % (self.name, m.addr))
            modes.append(mres)
        if len(set(modes)) != 1:
            raise RuntimeError("%s: MICROSTEP MISMATCH - refusing to move"
                               % self.name)
        print("  %s: %.0f steps/mm, 1/%d ustep, %d motor(s), travel %.0f mm"
              % (self.name, self.steps_per_mm, 256 >> modes[0],
                 len(self.motors), self.max_travel))

    # -- low level ---------------------------------------------------------
    def triggered(self):
        return self.sw.value() == self.sw_active

    def set_dir(self, value):
        for m in self.motors:
            m.dirn.value(value)
        time.sleep_us(20)

    def enable(self, on):
        for m in self.motors:
            m.en.value(0 if on else 1)

    def halt(self):
        _sm_enable(self.sm_ids, False)
        for m in self.motors:
            m.reset()

    def set_rate(self, hz):
        if hz < MIN_HZ:
            hz = MIN_HZ
        w = _div_word(hz)
        for m in self.motors:
            _set_div(m.sm_id, w)

    def hz_of(self, feed_mm_min):
        return max(MIN_HZ, feed_mm_min / 60.0 * self.steps_per_mm)

    # -- unpositioned moves ------------------------------------------------
    def jog(self, mm, direction, feed=None):
        """Relative move that does not update `pos`. Used before the axis is
        homed, by homing itself, and by the speed tests - which is why it
        deliberately does NOT apply the MAX_FEED clamp: finding the ceiling
        requires being able to ask for more than it."""
        if mm <= 0:
            return
        feed = feed or TRAVEL_FEED
        toward = (direction == self.home_dir)
        steps = int(round(mm * self.steps_per_mm))
        if steps < 1:
            return
        for m in self.motors:
            m.reset()
            m.arm(steps)
        self.set_dir(direction)
        self.enable(True)
        try:
            _sweep(((self, 1.0),), mm, feed, feed, "velocity", 0.0, 0.0,
                   self.accel, watch=(self,) if toward else ())
        finally:
            self.halt()
            self.enable(False)

    def _seek_run(self, hz, max_mm):
        """Step toward the switch until it trips. Returns mm travelled.

        The rate is ramped by clock divider while the issued steps are
        integrated, so the returned distance is right even though the
        approach was not at a constant rate throughout. No UART in this
        loop: detection latency is one GPIO read.
        """
        steps_budget = int(max_mm * self.steps_per_mm)
        for m in self.motors:
            m.reset()
            m.arm(steps_budget)
        self.set_dir(self.home_dir)
        self.enable(True)
        issued = 0.0
        hit_ms = None
        start_hz = min(hz, self.hz_of(START_MMS * 60.0))
        try:
            self.set_rate(start_hz)
            _sm_enable(self.sm_ids, True)
            if hz > start_hz:
                ramp_ms = max(120, int(1000.0 * (hz - start_hz)
                                       / self.steps_per_mm / self.accel))
                t0 = time.ticks_ms()
                while True:
                    el = time.ticks_diff(time.ticks_ms(), t0)
                    if el >= ramp_ms:
                        break
                    if self.triggered():
                        return (start_hz * el / 1000.0) / self.steps_per_mm
                    self.set_rate(start_hz
                                  + (hz - start_hz) * el / ramp_ms)
                issued = (start_hz + hz) / 2.0 * ramp_ms / 1000.0
            self.set_rate(hz)
            t0 = time.ticks_ms()
            budget = (steps_budget - issued) / hz * 1000.0
            while True:
                if self.triggered():
                    hit_ms = time.ticks_diff(time.ticks_ms(), t0)
                    break
                if time.ticks_diff(time.ticks_ms(), t0) > budget:
                    break
                if _panic():
                    break
        finally:
            self.halt()
            self.enable(False)
        if hit_ms is None:
            return None
        time.sleep_ms(30)
        if not self.triggered():
            print("    %s: switch did not stay closed - bounce" % self.name)
            return None
        return (issued + hit_ms / 1000.0 * hz) / self.steps_per_mm

    def clear_switch(self, max_mm=8.0):
        """Retreat until the switch releases. A lever stays depressed well
        past its trip point, so a fixed backoff is not enough."""
        moved = 0.0
        while self.triggered() and moved < max_mm:
            self.jog(1.0, 1 - self.home_dir, 120.0)
            moved += 1.0
            time.sleep_ms(100)
        return not self.triggered()

    def home(self):
        """Two-stage home, then park clear of the switch.

        Position zero is the slow-approach trip point. The axis is left at
        PARK_MM, which is deliberately above the soft limit MIN_POS so there
        is somewhere to jog toward home afterwards; sitting on a closed
        switch is no place to start a run from either.
        """
        self.pos = None
        if not self.clear_switch():
            print("  %s: switch stuck on" % self.name)
            return None
        if self._seek_run(self.fast_hz, self.max_travel + 5) is None:
            print("  %s: fast seek found nothing" % self.name)
            return None
        if not self.clear_switch():
            print("  %s: could not release switch" % self.name)
            return None
        self.jog(BACKOFF_MM, 1 - self.home_dir, 120.0)
        time.sleep_ms(200)
        d = self._seek_run(self.slow_hz, BACKOFF_MM * 4 + 8.0)
        if d is None:
            print("  %s: slow re-approach found nothing" % self.name)
            return None
        self.pos = 0.0
        self.jog(PARK_MM, 1 - self.home_dir, 300.0)
        self.pos = PARK_MM
        return d

    # -- limits ------------------------------------------------------------
    def lo(self):
        return MIN_POS

    def hi(self):
        return self.max_travel - EDGE_MARGIN

    def check(self, mm):
        if self.pos is None:
            raise RuntimeError("%s not homed" % self.name)
        if mm < self.lo() - 1e-6 or mm > self.hi() + 1e-6:
            raise ValueError("%s %.3f outside [%.2f, %.2f]"
                             % (self.name, mm, self.lo(), self.hi()))


# --------------------------------------------------- coordinated motion ---
AX = {}


def _sweep(plan, length, f0, f1, mode, lead_in, lead_out, accel, watch=None):
    """Release already-armed state machines and steer the rate to the end.

    `plan` is a sequence of (axis, share), share being that axis's fraction
    of the path length - which is what turns a set of independent step
    generators into a straight line.

    Each pass round the loop measures how long the last one took,
    integrates that into distance travelled, and asks rate_at() what speed
    belongs at that distance. Distance is therefore tracked, not assumed;
    and because the step COUNT comes from the FIFO, an error in this
    integration can only shift the shape of the ramp slightly, never the
    position the move ends at. That separation is what makes it safe to
    compute the profile on the fly.

    `watch` is the axes heading toward their endstop. The soft limits should
    make hitting one impossible, so a trip here means the stored position was
    wrong - which is exactly when a backstop earns its keep, because the far
    end of travel has no switch at all and the near end is all there is.

    Returns 0 when the move completed, 1 when the host asked it to stop
    early (decelerated, position recoverable from the step counters), 2 when
    it was aborted or hit an endstop (position lost), and 3 when it overran
    the time its own profile needs - see the deadline below.

    A stop request does not slam the drivers off: at 30 mm/s that would lose
    steps and the position would be a guess again. It ramps down at the
    axis's own acceleration instead, which costs about half a millimetre and
    keeps the count exact.
    """
    s = 0.0
    v = START_MMS
    stopping = False
    for a, share in plan:
        a.set_rate(v * share * a.steps_per_mm)
    ids = []
    motors = []
    for a, _ in plan:
        ids += a.sm_ids
        motors += a.motors
    # A move that cannot finish must not be able to hang the board. Every
    # other exit from this loop depends on the machine: a motor reporting its
    # step count, or the host asking to stop. A motor that never reports -
    # a lost FIFO word, a state machine left mid-program - would spin here
    # forever with the drivers live, while the host sits in a blocking read
    # waiting for the ok that is never printed. Nothing times out on either
    # side and the only way back is the power switch.
    #
    # So: three times the time the commanded profile needs at its slowest
    # sustained speed, plus two seconds. The ramps are a few tens of
    # milliseconds at these accelerations, so that factor applies to
    # essentially the whole move and cannot be reached by one that is merely
    # slow. A travel move across the bed is bounded at about fifteen
    # seconds; the slowest legal gradient over full Y travel, minutes.
    v_min = min(f0, f1) / 60.0
    deadline = time.ticks_add(time.ticks_ms(),
                              int(3000.0 * length / v_min) + 2000)
    _sm_enable(ids, True)
    t_prev = time.ticks_us()
    while True:
        if watch:
            for a in watch:
                if a.triggered():
                    print("  %s hit its endstop mid-move - position was wrong"
                          % a.name)
                    return 2
        p = _panic()
        if p == 2:
            return 2
        if p == 1:
            stopping = True
        finished = True
        for m in motors:
            if not m.done():
                finished = False
                break
        if finished:
            return 0
        if time.ticks_diff(deadline, time.ticks_ms()) < 0:
            # No graceful ramp-down: that would read the same step counters
            # whose silence brought us here. Stop hard, lose the position,
            # make the host re-home.
            print("  %s: move overran its time bound - position lost"
                  % ",".join(a.name for a, _ in plan))
            return 3
        now = time.ticks_us()
        dt = time.ticks_diff(now, t_prev) / 1000000.0
        t_prev = now
        if stopping:
            v -= accel * dt
            if v <= START_MMS:
                return 1
        else:
            s += v * dt
            if s > length:
                s = length
            v = rate_at(s, length, f0, f1, mode, lead_in, lead_out, accel)
        for a, share in plan:
            a.set_rate(v * share * a.steps_per_mm)


def _clamp_feed(f, what):
    if f is None or f <= 0:
        raise ValueError("%s must be positive" % what)
    if f > MAX_FEED:
        raise ValueError("%s %.0f exceeds the %.0f mm/min ceiling"
                         % (what, f, MAX_FEED))
    return max(f, MIN_FEED)


def move(x=None, y=None, feed=None, feed_end=None, mode="velocity",
         lead_in=0.0, lead_out=0.0):
    """Absolute coordinated move, optionally sweeping the feed as it goes.

    feed_end turns the move into a gradient pass: the rate sweeps from feed
    to feed_end continuously. mode picks which quantity is interpolated -
    see gradient_feed(). lead_in and lead_out shrink the span the gradient
    is defined over, so the accelerating happens outside the part that
    matters - see rate_at().
    """
    f0 = _clamp_feed(feed or TRAVEL_FEED, "feed")
    f1 = _clamp_feed(feed_end, "end feed") if feed_end else f0

    want = []
    if x is not None:
        want.append(("X", float(x)))
    if y is not None:
        want.append(("Y", float(y)))
    if not want:
        return
    for n, v in want:
        AX[n].check(v)

    legs = []                       # (name, axis, target, delta)
    for n, v in want:
        a = AX[n]
        d = v - a.pos
        if abs(d) > 0.5 / a.steps_per_mm:
            legs.append((n, a, v, d))
    if not legs:
        return
    length = sum(d * d for _, _, _, d in legs) ** 0.5

    # A component much shorter than the path needs a proportionally lower
    # step rate, and below the divider's floor it would run too fast and bow
    # the line. Judge that at the move's CRUISE speed, not at the junction
    # speed: the junction speed applies only for the instant at each end of
    # the ramp, and testing against it declared almost every diagonal
    # unworkable. A near-axis-aligned travel move was then split into two
    # legs with a full stop between them - a dog-leg where a straight line
    # would do. At 40 mm/s a component down to 1.5% of the path stays
    # coordinated; below the floor the rate simply clamps, which costs a
    # fraction of a millimetre of straightness at the very ends of a travel
    # move and nothing at all where it matters.
    if len(legs) > 1:
        v_top = max(f0, f1) / 60.0
        floor = min(v_top * abs(d) / length * a.steps_per_mm
                    for _, a, _, d in legs)
        if floor < MIN_HZ:
            for n, _, v, _ in sorted(legs, key=lambda L: abs(L[3])):
                move(x=v if n == "X" else None,
                     y=v if n == "Y" else None, feed=f0)
            return

    sm_ids = []
    plan = []                       # (axis, share)
    # A coordinated move can only accelerate as hard as its slowest axis.
    accel = min(a.accel for _, a, _, _ in legs)
    for n, a, v, d in legs:
        a.set_dir(a.home_dir if d < 0 else 1 - a.home_dir)
        steps = int(round(abs(d) * a.steps_per_mm))
        for m in a.motors:
            m.reset()
            m.arm(steps)
        a.enable(True)
        sm_ids += a.sm_ids
        plan.append((a, abs(d) / length))

    status = 2
    partial = {}
    try:
        status = _sweep(plan, length, f0, f1, mode, lead_in, lead_out, accel,
                        watch=[a for _, a, _, d in legs if d < 0])
    finally:
        _sm_enable(sm_ids, False)
        if status == 1:
            # Read the counters BEFORE halt(), which resets them.
            for n, a, _v, d in legs:
                partial[n] = a.motors[0].steps_done(
                    int(round(abs(d) * a.steps_per_mm)))
        for a, _ in plan:
            a.halt()
            a.enable(False)
    if status == 0:
        for n, a, v, d in legs:
            a.pos = v
    elif status == 1:
        for n, a, v, d in legs:
            moved = partial.get(n, 0) / a.steps_per_mm
            a.pos = a.pos + (moved if d > 0 else -moved)
    else:
        for n, a, v, d in legs:
            a.pos = None             # aborted mid-move: position unknown
        raise RuntimeError("timed out" if status == 3 else "aborted")


def goto(axis, mm, feed=None):
    """Absolute single-axis move by name. Bounds-checked like any move()."""
    move(x=mm if axis == "X" else None, y=mm if axis == "Y" else None,
         feed=feed)


def pass_line(axis, start, end, feed, feed_end=None, mode="velocity",
              reps=1, pause_s=0.0, both_ways=True, lead=None):
    """One spray entry: travel to the start, then sweep back and forth.

    start and end bound the part that is meant to be coated. The carriage
    actually runs from `lead` before start to `lead` after end, so the
    ramps sit outside the coated span and every point inside it sees the
    feed the recipe asked for. Pass lead=0 to reproduce the Shapeoko's
    behaviour, where the ramps ate into the line.

    With lead unset it is worked out from the acceleration and the fastest
    feed in the pass. That is usually the honest number and often larger
    than expected - which is exactly why it is computed rather than
    assumed.

    On the return stroke the feed ramp is mirrored, so every point on the
    line is traversed at the same speed in both directions: the gradient
    shapes the coating instead of smearing it. Because the leads are equal
    and the acceleration caps symmetric, that identity survives the ramps.
    """
    a = AX[axis]
    if lead is None:
        lead = lead_needed(max(feed, feed_end or feed), a.accel) + 0.5
    d = 1.0 if end >= start else -1.0
    p0, p1 = start - d * lead, end + d * lead
    for p, what in ((p0, "lead-in"), (p1, "lead-out")):
        if p < a.lo() - 1e-6 or p > a.hi() + 1e-6:
            raise ValueError(
                "%s %s at %.2f leaves [%.2f, %.2f] - shorten the pass, move "
                "it, or measure a higher accel (accel_test)"
                % (axis, what, p, a.lo(), a.hi()))
    for r in range(reps):
        move(x=p0 if axis == "X" else None,
             y=p0 if axis == "Y" else None, feed=TRAVEL_FEED)
        move(x=p1 if axis == "X" else None,
             y=p1 if axis == "Y" else None,
             feed=feed, feed_end=feed_end, mode=mode,
             lead_in=lead, lead_out=lead)
        if both_ways:
            move(x=p0 if axis == "X" else None,
                 y=p0 if axis == "Y" else None,
                 feed=feed_end or feed,
                 feed_end=feed if feed_end else None, mode=mode,
                 lead_in=lead, lead_out=lead)
        if pause_s > 0 and r < reps - 1:
            time.sleep(pause_s)


# ------------------------------------------------------------ diagnostics --
def scan():
    """Which of the four driver channels answer, and what their pins read."""
    for a in range(4):
        v = read_reg(IOIN, a)
        if v is None:
            print("  addr %d: no reply" % a)
        else:
            print("  addr %d: version 0x%02X  ENN=%d MS1=%d MS2=%d DIAG=%d"
                  % (a, (v >> 24) & 0xFF, v & 1, (v >> 2) & 1,
                     (v >> 3) & 1, (v >> 4) & 1))


def switches(seconds=30):
    """Live endstop states - push the carriages by hand with drivers off."""
    stop()
    for _ in range(int(seconds * 4)):
        print("  " + "   ".join("%s %d %s" % (a.name, a.sw.value(),
                                              a.triggered())
                                for a in AX.values()))
        time.sleep_ms(250)


def selftest(name="X", n=30):
    """Hammer the state-machine reset path with nothing coupled to it.

    Every move stops the state machine and puts it back in a known state.
    If that recovery were wrong - a leftover count, a stale FIFO word - the
    error would show up as a move that goes the wrong distance, and it
    would show up intermittently, which is the worst way to find out. So
    check it directly: n short there-and-back moves, ending where it
    started. Measure the carriage before and after.
    """
    a = AX[name]
    if a.pos is None:
        print("  %s not homed - homing first" % name)
        if a.home() is None:
            return
    start = a.pos
    print("%s: %d reset cycles from %.3f mm - measure the carriage now"
          % (name, n, start))
    for i in range(n):
        goto(name, start + 5.0, 900.0)
        goto(name, start, 900.0)
    print("  back at %.3f mm by the firmware's own count" % a.pos)
    d = a._seek_run(a.slow_hz, start * 4 + 8.0)
    if d is None:
        print("  could not re-find the switch - something is wrong")
        return
    print("  switch says %.4f mm, expected %.4f, err %+.4f mm"
          % (d, start, d - start))
    a.home()


def speed_test(name, speeds=(8000, 12000, 16000, 24000, 32000, 40000),
               margin=5.0, dist=None):
    """Find the fastest rate this axis sustains without losing steps.

    Home, run out and back at the test speed but stop `margin` short, then
    creep in slowly. With nothing lost that creep is exactly `margin`; the
    difference is steps dropped on the fast legs. The endstop is the
    instrument, so no dial gauge is needed.
    """
    a = AX[name]
    if dist is None:
        dist = a.hi() - PARK_MM - 5.0
    if PARK_MM + dist > a.hi() + 1e-6:
        raise ValueError("%s: %.1f mm from the park point overruns %.1f"
                         % (name, dist, a.hi()))
    # home() leaves the carriage parked at PARK_MM, not on the switch, so
    # the expected creep is PARK_MM + margin rather than margin alone.
    expect = PARK_MM + margin
    print("%s: out-and-back %.0f mm, expect a %.1f mm creep"
          % (name, dist, expect))
    for hz in speeds:
        if a.home() is None:
            print("  home failed - stopping")
            return
        feed = hz * 60.0 / a.steps_per_mm
        a.jog(dist, 1 - a.home_dir, feed)
        a.jog(dist - margin, a.home_dir, feed)
        d = a._seek_run(a.slow_hz, expect * 3.0)
        mms = hz / a.steps_per_mm
        if d is None:
            print("  %6d Hz (%5.1f mm/s)  FAILED - missed the switch"
                  % (hz, mms))
            break
        err = d - expect
        flag = "ok" if abs(err) < 0.05 else ("MARGINAL" if abs(err) < 0.5
                                             else "LOSING STEPS")
        print("  %6d Hz (%5.1f mm/s)  creep %.3f mm  err %+.3f mm  %s"
              % (hz, mms, d, err, flag))
    a.home()


def accel_test(name, rates=(50, 100, 200, 400, 800, 1600), feed=None,
               margin=5.0, reps=6):
    """The same endstop-as-instrument trick, but for acceleration.

    Acceleration decides how much run-up a gradient pass needs before it
    is up to speed, and every millimetre it buys back is travel the pass no
    longer needs. Each axis gets its own value, because a two-motor gantry
    and a single-motor crossbeam have no reason to agree.

    Each candidate acceleration gets `reps` full-length strokes, so two
    hard accelerations per rep, at the top feed the machine is allowed.
    Then the switch says how much position was lost. The strokes are
    absolute moves between two fixed points, which makes the pattern
    position-neutral and keeps the soft limits in charge - an earlier
    relative version drifted outward every rep and ran into the end.

    Adopt the last CLEAN value, not the last MARGINAL one, then put it in
    that axis's config; this function restores what it found.
    """
    a = AX[name]
    keep = a.accel
    feed = feed or MAX_FEED
    lo, hi = PARK_MM + margin, a.hi()
    print("%s: %d strokes of %.1f mm (%.1f..%.1f) at F%.0f per rate"
          % (name, reps, hi - lo, lo, hi, feed))
    print("  accel     creep     err        run-up needed at F%.0f" % feed)
    try:
        for acc in rates:
            a.accel = float(acc)
            if a.home() is None:
                print("  home failed - stopping")
                return
            goto(name, lo, feed)
            for _ in range(reps):
                goto(name, hi, feed)
                goto(name, lo, feed)
            d = a._seek_run(a.slow_hz, lo * 3.0)
            if d is None:
                print("  %5d      FAILED - missed the switch" % acc)
                break
            err = d - lo
            flag = "ok" if abs(err) < 0.05 else ("MARGINAL" if abs(err) < 0.5
                                                 else "LOSING STEPS")
            print("  %5d    %7.3f  %+7.3f    %5.2f mm   %s"
                  % (acc, d, err, lead_needed(feed, a.accel), flag))
            if abs(err) >= 0.5:
                break
    finally:
        a.accel = keep
        a.home()
    print("  %s accel still %.0f - put a new value in that axis's config"
          % (name, a.accel))


def show_profile(axis="Y", span=30.0, f0=750.0, f1=2400.0, mode="thickness",
                 lead=None, dt=0.0004):
    """Simulate a pass and report it, without moving anything.

    This runs the same rate_at() the firmware runs, stepped at `dt` to
    imitate the loop rate, so what it prints is what the machine will do
    rather than an idealisation of it. The question it answers: does the
    coated span actually see the feeds the recipe asked for, or did
    acceleration quietly flatten the fast end? With lead=0 the answer is
    always no at the very ends, whatever the acceleration - a pass that
    stops on the span must decelerate on it. That is why lead exists.
    """
    a = AX[axis]
    if lead is None:
        lead = lead_needed(max(f0, f1), a.accel) + 0.5
    length = span + 2 * lead
    print("%s span %.1f mm + %.2f mm lead each end = %.2f mm of travel"
          % (axis, span, lead, length))
    print("F%.0f -> F%.0f, %s, accel %.0f mm/s2"
          % (f0, f1, mode, a.accel))
    s, t, worst = 0.0, 0.0, 0.0
    rows, next_row = [], 0.0
    while s < length:
        v = rate_at(s, length, f0, f1, mode, lead, lead, a.accel)
        u = (s - lead) / span if span > 1e-9 else 0.0
        if 0.0 <= u <= 1.0:
            want = gradient_feed(u, f0, f1, mode)
            short = (want - v * 60.0) / want
            if short > worst:
                worst = short
        if s >= next_row:
            rows.append((s - lead, t, v * 60.0,
                         gradient_feed(min(1.0, max(0.0, u)), f0, f1, mode),
                         0.0 <= u <= 1.0))
            next_row += length / 10.0
        s += v * dt
        t += dt
    for pos, tt, got, want, inside in rows:
        print("    %7.2f mm %s t=%5.2fs  want F%-6.0f got F%-6.0f%s"
              % (pos, "|" if inside else " ", tt, want, got,
                 "  <- short" if inside and got < want - 1 else ""))
    print("  %.2f s per pass, %.1f um between rate updates at full speed"
          % (t, max(r[2] for r in rows) / 60.0 * dt * 1000.0))
    if worst < 0.01:
        print("  coated span is faithful to within %.2f%%" % (worst * 100))
    else:
        print("  WORST SHORTFALL INSIDE THE SPAN: %.1f%% - the coating will "
              "not match the recipe.\n  Raise the lead, lower the top feed, "
              "or measure a higher accel with accel_test()." % (worst * 100))
    return t


def sync_test(name="Y", cycles=20, dist=30.0, feed=1800.0):
    """Mark both carriages, run this, and check the marks still line up.
    The only thing that can break firmware lockstep is a start or stop that
    touched one state machine and not the other."""
    a = AX[name]
    print("%s: %d round trips of %.0f mm at F%.0f - mark both carriages"
          % (name, cycles, dist, feed))
    for _ in range(cycles):
        a.jog(dist, 1 - a.home_dir, feed)
        a.jog(dist, a.home_dir, feed)
    print("done - re-measure both marks")


# ----------------------------------------------------------------- entry ---
def setup():
    AX.clear()
    print("configuring:")
    for cfg in AXES:
        a = Axis(cfg)
        a.configure()
        AX[a.name] = a
    print("verifying:")
    for a in AX.values():
        a.verify()
        print("  %s switch: %s" % (a.name,
              "TRIGGERED" if a.triggered() else "clear"))
    return AX


def stop():
    for cfg in AXES:
        for m in cfg["motors"]:
            Pin(m["en"], Pin.OUT, value=1)
    mem32[PIO0_CTRL] = mem32[PIO0_CTRL] & ~0x0F
    print("all drivers disabled")


def home_axis(name, trials=TRIALS):
    a = AX[name]
    print("homing %s:" % name)
    res = []
    for i in range(trials):
        d = a.home()
        if d is not None:
            res.append(d)
            print("  trial %d: %.4f mm" % (i + 1, d))
        time.sleep_ms(300)
    if len(res) >= 2:
        avg = sum(res) / len(res)
        sd = (sum((x - avg) ** 2 for x in res) / len(res)) ** 0.5
        print("  %s spread %.4f mm, sd %.4f mm"
              % (name, max(res) - min(res), sd))
    else:
        print("  %s: not enough successful homes to judge" % name)
    return res


def home_all():
    for name in HOME_ORDER:
        if AX[name].home() is None:
            raise RuntimeError("%s failed to home" % name)
    print("homed - X %.2f  Y %.2f" % (AX["X"].pos, AX["Y"].pos))


# ---------------------------------------------------------------- server ---
#
# A deliberately small G-code subset - enough for the spray program and
# nothing more. Every command answers exactly one line, so the host stays a
# simple request/response loop instead of tracking a planner buffer the way
# it had to with GRBL.
#
#   $H              home every axis, park at PARK_MM
#   $$              report limits
#   ?               status
#   G0 X.. Y..      rapid at TRAVEL_FEED
#   G1 X.. Y.. F..  feed move, F in mm/min
#   G1 .. F.. E..   gradient pass: sweep F -> E across the move.
#                   Q1 for thickness mode, Q0 (default) for velocity
#                   I.. lead-in mm, K.. lead-out mm: the gradient is
#                   defined over what is left between them, and holds flat
#                   across the leads so the ramps sit off the work
#   G4 P..          dwell, seconds
#   M17 / M18       enable / disable drivers
#   ~               stop the move in progress, decelerating; position is
#                   kept, so this is what a jog release sends
#   !               hard abort; position is LOST and the machine goes to
#                   Alarm, which is what the STOP button wants
#
# A feed above MAX_FEED is an error, never a silent clamp: the host is
# expected to rescale the recipe so the gradient keeps its shape.
#
# As with GRBL, a command is finished when a line beginning "ok" or
# "error:" comes back. Anything else - homing chatter, warnings - is
# informational and the host should log it and keep waiting.

_poll = None
BANNER = "PicoStage 1.0 ['?' for status, '$H' to home, '$$' for limits]"


def _words(line):
    out = {}
    key, num = None, ""
    for ch in line.upper() + " ":
        if "A" <= ch <= "Z":
            if key and num:
                out[key] = float(num)
            key, num = ch, ""
        elif ch in "0123456789.+-":
            num += ch
        else:
            if key and num:
                out[key] = float(num)
            key, num = None, ""
    return out


def _status():
    x, y = AX["X"].pos, AX["Y"].pos
    if x is None or y is None:
        return "<Alarm|MPos:none>"
    return "<Idle|MPos:%.3f,%.3f>" % (x, y)


def _execute(line):
    line = line.strip()
    if not line or line[0] in ";(":
        return "ok"
    if line == "?":
        return _status()
    if line in ("!", "\x18"):
        stop()
        AX["X"].pos = AX["Y"].pos = None
        return "ok"
    if line == "~":
        # A jog-cancel that arrived after the move already finished. The
        # release simply lost the race; there is nothing to stop.
        return "ok"
    if line == "$H":
        home_all()
        return "ok"
    if line == "$$":
        # The host needs the acceleration and junction speed to work out lead
        # distances that match what the firmware will actually do. Reporting
        # them keeps one source of truth instead of two constants that drift.
        return ("[LIM:X%.2f..%.2f,Y%.2f..%.2f]\n"
                "[FEED:MAX%.0f,TRAVEL%.0f,MIN%.0f]\n"
                "[ACC:X%.0f,Y%.0f,V%.2f]\nok") % (
            AX["X"].lo(), AX["X"].hi(), AX["Y"].lo(), AX["Y"].hi(),
            MAX_FEED, TRAVEL_FEED, MIN_FEED,
            AX["X"].accel, AX["Y"].accel, START_MMS)
    if line == "M17":
        for a in AX.values():
            a.enable(True)
        return "ok"
    if line == "M18":
        stop()
        return "ok"
    w = _words(line)
    if "G" in w:
        g = int(w["G"])
        if g == 4:
            time.sleep(w.get("P", 0.0))
            return "ok"
        if g in (0, 1):
            if g == 1 and "F" not in w:
                # No modal feed on purpose: inheriting a rate from an
                # earlier line is how a pass silently sprays at the wrong
                # speed, and every line the planner emits carries its own F.
                return "error: G1 needs an F word"
            move(x=w.get("X"), y=w.get("Y"),
                 feed=TRAVEL_FEED if g == 0 else w.get("F"),
                 feed_end=w.get("E"),
                 mode="thickness" if int(w.get("Q", 0)) else "velocity",
                 lead_in=w.get("I", 0.0), lead_out=w.get("K", 0.0))
            return "ok"
    return "error: unknown command %r" % line


def serve():
    """Take over the USB serial line and answer commands from the host.

    Ctrl-C returns to the REPL. Call this from main.py once the host
    program drives the machine; leave it alone while bringing things up by
    hand in Thonny, because it consumes everything typed at the prompt.
    """
    global WATCH_STDIN, _poll
    _poll = select.poll()
    _poll.register(sys.stdin, select.POLLIN)
    print(BANNER)
    try:
        while True:
            WATCH_STDIN = False
            line = sys.stdin.readline()
            if not line:
                continue
            WATCH_STDIN = True
            try:
                reply = _execute(line)
            except Exception as e:
                stop()
                reply = "error: %s" % e
            WATCH_STDIN = False
            print(reply)
    except KeyboardInterrupt:
        WATCH_STDIN = False
        stop()
        print("serve() stopped")


setup()
print("""
ready - nothing has moved. From the Shell:

  scan()                    which driver channels answer
  switches()                live endstop states, drivers off
  home_all()                home X then Y, park at %.1f mm

  selftest('X')             FIRST: 30 stop/restart cycles, check for drift
  show_profile('Y')         simulate a gradient pass, nothing moves
  accel_test('X')           re-measure acceleration (both axes at %.0f)

  move(x=20, y=40)          absolute travel move at F%.0f
  move(y=70, feed=900, feed_end=2400, mode='thickness')
  pass_line('Y', 10, 40, 750, 2400, 'thickness')
  speed_test('Y')           fastest rate that loses no steps
  sync_test('Y')            round trips - check the gantry marks
  serve()                   hand the serial line to the host program
  stop()                    disable everything

  limits  X %.1f..%.1f   Y %.1f..%.1f   feed <= %.0f mm/min
""" % (PARK_MM, AX["X"].accel, TRAVEL_FEED,
       AX["X"].lo(), AX["X"].hi(), AX["Y"].lo(), AX["Y"].hi(), MAX_FEED))

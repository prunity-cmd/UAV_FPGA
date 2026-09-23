"""
Adaptive Sonar Transmitter - Control, Model & Monitoring GUI   (merged v2)
==========================================================================

One window, two data sources:

  * SIMULATE (default, no hardware needed)
      A cycle-accurate Python model of the Verilog design
      (param_controller -> dds_waveform_gen -> adaptive_sonar_top) drives all
      plots and read-outs.

  * FPGA (UART)
      Slider values are sent to the FPGA, and the streamed sample_out /
      waveform_mode frames are drawn as a rolling scope.  The model keeps
      running alongside, so the GUI can show whether the hardware chose the
      SAME waveform mode the RTL spec says it should  (MATCH / MISMATCH).

PROTOCOL (must match the FPGA-side UART register interface):
  Host -> FPGA:  2 bytes  [register_id][value]
      register_id: 0x01 = depth        (0-255)
                   0x02 = salinity      (0-255)
                   0x03 = noise_level   (0-255)

  FPGA -> Host:  3-byte streaming frames, sent continuously
      [0xAA][waveform_mode (0-3)][sample_out (signed 8-bit, two's complement)]

  waveform_mode: 0 = CW, 1 = LFM up-chirp, 2 = LFM down-chirp   (see PROFILES)

If you change register IDs or the frame format on the FPGA side, update the
constants in the CONFIG section below.

Dependencies:
    pip install numpy matplotlib pyserial        (pyserial only for FPGA mode)

Run:
    python sonar_ctrl_gui.py
"""

import queue
import struct
import threading
import time
import math
from collections import deque

import tkinter as tk
from tkinter import ttk, messagebox

import numpy as np
import matplotlib
matplotlib.use("TkAgg")
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

try:
    import serial
    import serial.tools.list_ports
    PYSERIAL_AVAILABLE = True
except ImportError:
    PYSERIAL_AVAILABLE = False


# ============================== CONFIG ======================================
# --- UART link ---
BAUD_RATE = 115200
REG_DEPTH = 0x01
REG_SALINITY = 0x02
REG_NOISE_LEVEL = 0x03
FRAME_HEADER = 0xAA
FRAME_SIZE = 3                 # header + mode + sample
MAX_MODE = 3                   # waveform_mode is 2 bits
UART_WRITE_THROTTLE_MS = 30    # coalesce slider drags into fewer UART writes
HW_WINDOW_SAMPLES = 600        # rolling scope length in FPGA mode

# --- RTL constants (mirror the Verilog) ---
FREQ_WIDTH = 24
ROM_ADDR_BITS = 5
ROM_SIZE = 32
CLOCK_HZ = 100_000_000.0       # testbench clock.  1 cycle = 1/CLOCK_HZ seconds
PHASE_MOD = 1 << FREQ_WIDTH
PHASE_MASK = PHASE_MOD - 1

DEPTH_DEEP = 160               # param_controller thresholds
NOISE_HIGH = 150
SALINITY_HIGH = 180

# Paste the exact 32 signed values from your Verilog ROM here so the GUI trace
# matches GTKWave sample-for-sample.  None = compute round(127*sin()).
ROM_TABLE_OVERRIDE = [
    0, 25, 49, 71, 90, 106, 117, 125, 127, 125, 117, 106, 90, 71, 49, 25,
    0, -25, -49, -71, -90, -106, -117, -125, -127, -125, -117, -106, -90, -71, -49, -25
]

# The current RTL outputs the raw ROM value (no amplitude multiply), because the
# signed multiply-and-shift collapsed the output.  Set True to preview what
# amplitude scaling WOULD look like in the GUI (model only; FPGA is unaffected).
APPLY_AMPLITUDE = False
# =============================================================================


PROFILES = {
    "Deep Water": {
        "freq_word": 800_000, "chirp_rate": 0, "amplitude": 200,
        "pulse_width": 2000, "waveform_mode": "CW", "mode_code": 0,
        "reason": "Depth > 160: lower-frequency CW and longer pulse for range.",
        "short_reason": "DEEP WATER",
    },
    "High Noise": {
        "freq_word": 1_200_000, "chirp_rate": 400, "amplitude": 255,
        "pulse_width": 1200, "waveform_mode": "UP-CHIRP", "mode_code": 1,
        "reason": "Noise > 150: LFM up-chirp for pulse-compression gain.",
        "short_reason": "HIGH NOISE",
    },
    "High Salinity": {
        "freq_word": 1_500_000, "chirp_rate": -300, "amplitude": 230,
        "pulse_width": 800, "waveform_mode": "DOWN-CHIRP", "mode_code": 2,
        "reason": "Salinity > 180: shifted band with LFM down-chirp.",
        "short_reason": "HIGH SALINITY",
    },
    "Calm / Shallow": {
        "freq_word": 1_000_000, "chirp_rate": 0, "amplitude": 180,
        "pulse_width": 600, "waveform_mode": "CW", "mode_code": 0,
        "reason": "No adaptation threshold exceeded: default CW operation.",
        "short_reason": "BASELINE",
    },
}

MODE_NAMES = {0: "CW", 1: "UP-CHIRP", 2: "DOWN-CHIRP", 3: "RESERVED"}

# Same five back-to-back scenarios as tb_adaptive_sonar.v
SCENARIOS = [
    ("Calm / Shallow", 40, 60, 30),
    ("Deep Water", 180, 60, 30),
    ("High Noise", 40, 60, 200),
    ("High Salinity", 40, 220, 30),
    ("Baseline Again", 40, 60, 30),
]


# ============================================================================
#                      Pure-logic layer (no GUI, unit-testable)
# ============================================================================
def word_to_hz(word):
    """DDS tuning word -> Hz for the 24-bit phase accumulator."""
    return (word * CLOCK_HZ) / PHASE_MOD


def mode_from_inputs(depth, salinity, noise):
    """Exact priority order from param_controller.v: depth -> noise -> salinity."""
    if depth > DEPTH_DEEP:
        key = "Deep Water"
    elif noise > NOISE_HIGH:
        key = "High Noise"
    elif salinity > SALINITY_HIGH:
        key = "High Salinity"
    else:
        key = "Calm / Shallow"
    return key, PROFILES[key]


def _build_rom():
    if ROM_TABLE_OVERRIDE is not None:
        if len(ROM_TABLE_OVERRIDE) != ROM_SIZE:
            raise ValueError(f"ROM_TABLE_OVERRIDE needs {ROM_SIZE} entries")
        return np.array(ROM_TABLE_OVERRIDE, dtype=np.int16)
    return np.array(
        [int(round(127 * math.sin(2 * math.pi * i / ROM_SIZE)))
         for i in range(ROM_SIZE)],
        dtype=np.int16,
    )


ROM = _build_rom()


class SonarModel:
    """
    Cycle-accurate software model of the Verilog blocks.

    Per clock while pulse_counter < pulse_width (DDS enabled):
        rom_addr   = phase_acc[23:19]            (old phase)
        sample_out = ROM[rom_addr]
        phase_acc += freq_word_current
        freq_word_current += chirp_rate
    otherwise the DDS idles (phase = 0, freq = base word).
    pulse_counter counts 0..pulse_width then wraps (one idle cycle).

    Runs are computed in closed form with numpy (no per-cycle Python loop):
        freq[k]  = f0 + k*c
        phase[k] = p0 + k*f0 + c*k*(k-1)/2      (all mod 2^24)
    """

    def __init__(self):
        self.reset()

    # ---- state -----------------------------------------------------------
    def reset(self):
        self.depth = self.salinity = self.noise = 50
        self.active_key = "Calm / Shallow"
        self.params = dict(PROFILES[self.active_key])
        self._restart_dds()
        self.clock_count = 0
        self.total_pulses = 0
        self.rom_addr = 0
        self.sample_out = 0
        self.sample_valid = False

    def _restart_dds(self):
        self.phase_acc = 0
        self.freq_word_current = self.params["freq_word"]
        self.pulse_counter = 0

    def set_environment(self, depth, salinity, noise):
        """Apply new sensor inputs. Returns True if the adaptive profile changed."""
        self.depth = int(np.clip(depth, 0, 255))
        self.salinity = int(np.clip(salinity, 0, 255))
        self.noise = int(np.clip(noise, 0, 255))
        key, p = mode_from_inputs(self.depth, self.salinity, self.noise)
        changed = key != self.active_key
        if changed:
            self.active_key = key
            self.params = dict(p)
            self._restart_dds()      # new profile -> deterministic restart
        return changed

    @property
    def mode_code(self):
        return self.params["mode_code"]

    @property
    def current_frequency_hz(self):
        return word_to_hz(int(self.freq_word_current) & PHASE_MASK)

    @property
    def chirp_rate_hz_per_clock(self):
        return word_to_hz(self.params["chirp_rate"])

    # ---- core ------------------------------------------------------------
    def _run(self, n):
        p = self.params
        base, chirp, pw = p["freq_word"], p["chirp_rate"], p["pulse_width"]
        phase, freq, pc = int(self.phase_acc), int(self.freq_word_current), int(self.pulse_counter)
        pulses = 0

        samples = np.zeros(n, dtype=np.float64)
        addrs = np.zeros(n, dtype=np.int16)
        freq_words = np.full(n, float(base))
        valid = np.zeros(n, dtype=bool)

        i = 0
        while i < n:
            if pc < pw:                                   # DDS enabled
                run = min(n - i, pw - pc)
                k = np.arange(run, dtype=np.int64)
                f_seq = (freq + k * chirp) & PHASE_MASK
                p_seq = (phase + k * freq + (k * (k - 1) // 2) * chirp) & PHASE_MASK
                a = (p_seq >> (FREQ_WIDTH - ROM_ADDR_BITS)) & 0x1F
                samples[i:i + run] = ROM[a]
                addrs[i:i + run] = a
                freq_words[i:i + run] = f_seq
                valid[i:i + run] = True
                phase = (phase + run * freq + (run * (run - 1) // 2) * chirp) & PHASE_MASK
                freq = (freq + run * chirp) & PHASE_MASK
                pc += run
                i += run
            else:                                         # idle cycle, counter wraps
                freq, phase, pc = base, 0, 0
                pulses += 1
                i += 1

        if APPLY_AMPLITUDE:
            samples = samples * (p["amplitude"] / 255.0)
        return samples, addrs, freq_words, valid, (phase, freq, pc, pulses)

    def advance(self, n):
        """Advance the model n clock cycles (commits state)."""
        n = max(1, int(n))
        s, a, _, v, (phase, freq, pc, pulses) = self._run(n)
        self.phase_acc, self.freq_word_current, self.pulse_counter = phase, freq, pc
        self.total_pulses += pulses
        self.clock_count += n
        self.rom_addr, self.sample_out, self.sample_valid = int(a[-1]), int(round(s[-1])), bool(v[-1])

    def preview(self, n):
        """Trace of the next n cycles without changing model state."""
        s, a, f, v, _ = self._run(int(n))
        return s, a, f, v


class FrameParser:
    """Resynchronising parser for [0xAA][mode][sample] frames."""

    def __init__(self):
        self.buf = bytearray()

    def feed(self, data):
        self.buf.extend(data)
        out = []
        b = self.buf
        while len(b) >= FRAME_SIZE:
            # A sample byte can legitimately equal 0xAA, so validate the mode
            # field and (when possible) that the NEXT frame also starts with a
            # header before trusting this one.
            if b[0] != FRAME_HEADER or b[1] > MAX_MODE:
                del b[0]
                continue
            if len(b) >= 2 * FRAME_SIZE and b[FRAME_SIZE] != FRAME_HEADER:
                del b[0]
                continue
            sample = struct.unpack("b", bytes([b[2]]))[0]
            out.append((b[1], sample))
            del b[:FRAME_SIZE]
        return out


# ============================================================================
#                                    GUI
# ============================================================================
class SonarControlGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Adaptive Sonar Transmitter — Control, Model & Monitor")
        self.root.geometry("1440x940")
        self.root.minsize(1200, 800)

        self.model = SonarModel()

        # serial / rx
        self.serial_conn = None
        self.rx_thread = None
        self.rx_stop_event = threading.Event()
        self.sample_queue = queue.Queue()
        self.rx_error = None
        self.hw_data = deque([0] * HW_WINDOW_SAMPLES, maxlen=HW_WINDOW_SAMPLES)
        self.hw_mode = None
        self.last_rx_time = None
        self._pending_writes = {}
        self._flush_scheduled = False
        self._sent = {}

        # run / demo state
        self.running = True
        self.demo_running = False
        self.demo_index = 0
        self._demo_next = 0.0
        self.display_points = 1600
        self.frame_ms = 40                       # 25 GUI frames / s
        self._t0 = time.monotonic()
        self._last_env_change = 0.0

        # tk variables
        self.simulate_var = tk.BooleanVar(value=True)
        self.depth_var = tk.IntVar(value=50)
        self.salinity_var = tk.IntVar(value=50)
        self.noise_var = tk.IntVar(value=50)
        self.speed_var = tk.IntVar(value=250)
        self.demo_hold_var = tk.IntVar(value=4)

        self.status_var = tk.StringVar(value="READY")
        self.condition_var = tk.StringVar(value="BASELINE")
        self.mode_var = tk.StringVar(value="CW")
        self.latency_var = tk.StringVar(
            value=f"1 clock = {1e9 / CLOCK_HZ:.0f} ns")
        self.fpga_mode_var = tk.StringVar(value="—")
        self.check_var = tk.StringVar(value="MODEL ONLY")
        self.reason_var = tk.StringVar(value=PROFILES["Calm / Shallow"]["reason"])
        self.total_pulses_var = tk.StringVar(value="")

        self.mv = {k: tk.StringVar(value="—") for k in (
            "freq_word", "freq_hz", "chirp_word", "chirp_hz",
            "amp", "pw", "rom", "sample")}

        self._x_sim = np.arange(self.display_points)
        self._x_hw = np.arange(HW_WINDOW_SAMPLES)

        self._build_style()
        self._build_ui()
        self._refresh_ports()
        self._on_env_change()
        self._set_view_mode()
        self.root.after(self.frame_ms, self._tick)

    # ------------------------------------------------------------ styling --
    def _build_style(self):
        style = ttk.Style()
        try:
            style.theme_use("clam")
        except tk.TclError:
            pass

        self.bg = "#0b1220"
        self.panel = "#111b2e"
        self.panel2 = "#16233b"
        self.text = "#e9f1ff"
        self.muted = "#9bb0cc"
        self.accent = "#31d7ff"
        self.green = "#39e58c"
        self.orange = "#ffb347"
        self.red = "#ff6b7a"
        self.root.configure(bg=self.bg)

        style.configure("TFrame", background=self.bg)
        style.configure("Panel.TFrame", background=self.panel)
        style.configure("TLabel", background=self.bg, foreground=self.text,
                        font=("Segoe UI", 10))
        style.configure("Panel.TLabel", background=self.panel,
                        foreground=self.text, font=("Segoe UI", 10))
        style.configure("Muted.TLabel", background=self.panel,
                        foreground=self.muted, font=("Segoe UI", 9))
        style.configure("Title.TLabel", background=self.bg,
                        foreground=self.text, font=("Segoe UI", 22, "bold"))
        style.configure("Subtitle.TLabel", background=self.bg,
                        foreground=self.muted, font=("Segoe UI", 10))
        style.configure("PanelTitle.TLabel", background=self.panel,
                        foreground=self.text, font=("Segoe UI", 12, "bold"))
        style.configure("Metric.TLabel", background=self.panel2,
                        foreground=self.accent, font=("Consolas", 15, "bold"))
        style.configure("MetricCaption.TLabel", background=self.panel2,
                        foreground=self.muted, font=("Segoe UI", 9))
        style.configure("TButton", font=("Segoe UI", 10, "bold"), padding=6)
        style.configure("TCheckbutton", background=self.panel,
                        foreground=self.text, font=("Segoe UI", 10))
        style.map("TCheckbutton", background=[("active", self.panel)])
        style.configure("TCombobox", fieldbackground=self.panel2,
                        background=self.panel2, foreground=self.text,
                        arrowcolor=self.text)
        style.map("TCombobox", fieldbackground=[("readonly", self.panel2)],
                  foreground=[("readonly", self.text)])
        style.configure("TSpinbox", fieldbackground=self.panel2,
                        foreground=self.text, arrowcolor=self.text)

    def _panel(self, parent, title):
        frame = ttk.Frame(parent, style="Panel.TFrame", padding=12)
        ttk.Label(frame, text=title, style="PanelTitle.TLabel").pack(
            anchor="w", pady=(0, 8))
        return frame

    # ----------------------------------------------------------------- UI --
    def _build_ui(self):
        header = ttk.Frame(self.root, padding=(18, 12, 18, 6))
        header.pack(fill="x")
        ttk.Label(header, text="ADAPTIVE SONAR TRANSMITTER",
                  style="Title.TLabel").pack(anchor="w")
        ttk.Label(
            header,
            text="Control panel + cycle-accurate model of the Verilog DDS and "
                 "adaptive controller  •  standalone simulation or live FPGA over UART",
            style="Subtitle.TLabel").pack(anchor="w")

        body = ttk.Frame(self.root, padding=(18, 4, 18, 8))
        body.pack(fill="both", expand=True)
        body.columnconfigure(1, weight=1)
        body.rowconfigure(0, weight=1)

        left = ttk.Frame(body, width=330)
        left.grid(row=0, column=0, sticky="ns", padx=(0, 12))
        left.pack_propagate(False)

        right = ttk.Frame(body)
        right.grid(row=0, column=1, sticky="nsew")
        right.columnconfigure(0, weight=1)
        right.rowconfigure(1, weight=1)
        right.rowconfigure(2, weight=1)

        self._build_connection_panel(left)
        self._build_environment_panel(left)
        self._build_adaptation_panel(left)
        self._build_control_panel(left)
        self._build_log_panel(left)

        self._build_metrics(right)
        self._build_plots(right)
        self._build_status_bar()

    # -- left column --------------------------------------------------------
    def _build_connection_panel(self, parent):
        panel = self._panel(parent, "Data Source")
        panel.pack(fill="x", pady=(0, 8))

        row = ttk.Frame(panel, style="Panel.TFrame")
        row.pack(fill="x")
        ttk.Label(row, text="Port", style="Panel.TLabel").pack(side="left")
        self.port_combo = ttk.Combobox(row, width=13, state="readonly")
        self.port_combo.pack(side="left", padx=6)
        ttk.Button(row, text="↻", width=3,
                   command=self._refresh_ports).pack(side="left")

        self.connect_btn = ttk.Button(panel, text="Connect",
                                      command=self._toggle_connection)
        self.connect_btn.pack(fill="x", pady=(8, 2))

        self.conn_label = tk.Label(panel, text="Disconnected", bg=self.panel,
                                   fg=self.red, font=("Segoe UI", 9, "bold"),
                                   anchor="w")
        self.conn_label.pack(fill="x")

        ttk.Checkbutton(panel, text="Simulate (no hardware)",
                        variable=self.simulate_var,
                        command=self._on_simulate_toggle).pack(anchor="w",
                                                               pady=(4, 0))

    def _build_environment_panel(self, parent):
        panel = self._panel(parent, "Environmental Conditions")
        panel.pack(fill="x", pady=(0, 8))
        self._sensor_row(panel, "Depth", self.depth_var,
                         f"> {DEPTH_DEEP} → Deep Water")
        self._sensor_row(panel, "Salinity", self.salinity_var,
                         f"> {SALINITY_HIGH} → High Salinity")
        self._sensor_row(panel, "Noise", self.noise_var,
                         f"> {NOISE_HIGH} → High Noise")

    def _sensor_row(self, panel, name, var, hint):
        row = ttk.Frame(panel, style="Panel.TFrame")
        row.pack(fill="x", pady=3)
        top = ttk.Frame(row, style="Panel.TFrame")
        top.pack(fill="x")
        ttk.Label(top, text=name, style="Panel.TLabel").pack(side="left")
        ttk.Label(top, text=hint, style="Muted.TLabel").pack(side="left",
                                                             padx=8)
        ttk.Label(top, textvariable=var, style="Panel.TLabel",
                  width=4, anchor="e").pack(side="right")
        tk.Scale(row, from_=0, to=255, orient="horizontal", variable=var,
                 showvalue=False, resolution=1, bg=self.panel, fg=self.text,
                 activebackground=self.accent, highlightthickness=0,
                 troughcolor="#23334e", bd=0,
                 command=lambda _v: self._on_env_change()).pack(fill="x")

    def _build_adaptation_panel(self, parent):
        panel = self._panel(parent, "Adaptive Decision")
        panel.pack(fill="x", pady=(0, 8))

        grid = ttk.Frame(panel, style="Panel.TFrame")
        grid.pack(fill="x")
        grid.columnconfigure(1, weight=1)
        self.check_label = None
        rows = [
            ("ACTIVE CONDITION", self.condition_var),
            ("WAVEFORM MODE", self.mode_var),
            ("ADAPT LATENCY", self.latency_var),
            ("FPGA REPORTS", self.fpga_mode_var),
            ("MODEL vs FPGA", self.check_var),
        ]
        for i, (caption, var) in enumerate(rows):
            ttk.Label(grid, text=caption, style="Panel.TLabel").grid(
                row=i, column=0, sticky="w", pady=2)
            lbl = tk.Label(grid, textvariable=var, bg=self.panel,
                           fg=self.text, font=("Segoe UI", 10, "bold"),
                           anchor="e")
            lbl.grid(row=i, column=1, sticky="e", pady=2)
            if caption == "MODEL vs FPGA":
                self.check_label = lbl

        tk.Label(panel, textvariable=self.reason_var, bg=self.panel2,
                 fg=self.text, justify="left", anchor="w", wraplength=290,
                 padx=10, pady=8, font=("Segoe UI", 9)).pack(fill="x",
                                                             pady=(8, 0))

    def _build_control_panel(self, parent):
        panel = self._panel(parent, "Simulation Control")
        panel.pack(fill="x", pady=(0, 8))

        btns = ttk.Frame(panel, style="Panel.TFrame")
        btns.pack(fill="x")
        btns.columnconfigure((0, 1), weight=1)
        self.start_btn = ttk.Button(btns, text="❚❚ Pause",
                                    command=self.toggle_run)
        self.start_btn.grid(row=0, column=0, sticky="ew", padx=2, pady=2)
        ttk.Button(btns, text="↻ Reset", command=self.reset).grid(
            row=0, column=1, sticky="ew", padx=2, pady=2)
        ttk.Button(btns, text="▶ 5-Scenario Demo",
                   command=self.start_demo).grid(row=1, column=0,
                                                 sticky="ew", padx=2, pady=2)
        ttk.Button(btns, text="■ Stop Demo", command=self.stop_demo).grid(
            row=1, column=1, sticky="ew", padx=2, pady=2)

        ttk.Label(panel, text="Model clocks per frame", style="Muted.TLabel"
                  ).pack(anchor="w", pady=(8, 0))
        tk.Scale(panel, from_=25, to=1000, orient="horizontal",
                 variable=self.speed_var, resolution=25, bg=self.panel,
                 fg=self.text, activebackground=self.accent,
                 highlightthickness=0, troughcolor="#23334e", bd=0
                 ).pack(fill="x")

        hold = ttk.Frame(panel, style="Panel.TFrame")
        hold.pack(fill="x", pady=(4, 0))
        ttk.Label(hold, text="Demo: seconds per scenario",
                  style="Muted.TLabel").pack(side="left")
        ttk.Spinbox(hold, from_=1, to=15, width=4,
                    textvariable=self.demo_hold_var).pack(side="right")

        ttk.Label(panel, textvariable=self.total_pulses_var,
                  style="Muted.TLabel").pack(anchor="w", pady=(6, 0))

    def _build_log_panel(self, parent):
        panel = self._panel(parent, "Adaptation Log")
        panel.pack(fill="both", expand=True)
        self.log_text = tk.Text(panel, height=5, bg=self.panel2, fg=self.text,
                                font=("Consolas", 8), relief="flat", bd=0,
                                wrap="none", state="disabled")
        self.log_text.pack(fill="both", expand=True)

    # -- right column -------------------------------------------------------
    def _metric_card(self, parent, col, row, caption, var):
        box = tk.Frame(parent, bg=self.panel2, highlightthickness=1,
                       highlightbackground="#273956")
        box.grid(row=row, column=col, sticky="nsew", padx=4, pady=4)
        parent.columnconfigure(col, weight=1)
        ttk.Label(box, text=caption, style="MetricCaption.TLabel").pack(
            anchor="w", padx=9, pady=(8, 1))
        ttk.Label(box, textvariable=var, style="Metric.TLabel").pack(
            anchor="w", padx=9, pady=(0, 8))

    def _build_metrics(self, parent):
        m = ttk.Frame(parent, style="Panel.TFrame", padding=7)
        m.grid(row=0, column=0, sticky="ew", pady=(0, 8))
        cards = [
            ("DDS FREQUENCY WORD", "freq_word"), ("CURRENT FREQUENCY", "freq_hz"),
            ("CHIRP RATE (word/clk)", "chirp_word"), ("CHIRP RATE (Hz/clk)", "chirp_hz"),
            ("AMPLITUDE CONTROL", "amp"), ("PULSE WIDTH", "pw"),
            ("ROM ADDRESS", "rom"), ("SAMPLE_OUT / VALID", "sample"),
        ]
        for i, (cap, key) in enumerate(cards):
            self._metric_card(m, i % 4, i // 4, cap, self.mv[key])

    def _style_axes(self, ax, title, xlabel, ylabel):
        ax.set_facecolor("#0a1424")
        ax.set_title(title, color=self.text, fontsize=11, loc="left", pad=9)
        ax.set_xlabel(xlabel, color=self.muted)
        ax.set_ylabel(ylabel, color=self.muted)
        ax.tick_params(colors=self.muted, labelsize=8)
        ax.grid(True, alpha=0.14)
        for sp in ax.spines.values():
            sp.set_color("#2a3a55")

    def _build_plots(self, parent):
        # time-domain / live scope
        outer = ttk.Frame(parent, style="Panel.TFrame", padding=5)
        outer.grid(row=1, column=0, sticky="nsew", pady=(0, 8))
        self.wave_fig = Figure(figsize=(8, 3.3), dpi=100, facecolor=self.panel)
        self.wave_ax = self.wave_fig.add_subplot(111)
        self._style_axes(self.wave_ax, "Generated Sonar Ping",
                         "Clock samples", "Amplitude")
        self.wave_ax.set_ylim(-140, 140)
        (self.wave_line,) = self.wave_ax.plot([], [], color=self.accent,
                                              linewidth=1.15)
        self.wave_fig.tight_layout()
        self.wave_canvas = FigureCanvasTkAgg(self.wave_fig, master=outer)
        self.wave_canvas.get_tk_widget().pack(fill="both", expand=True)

        # instantaneous frequency (model / expected)
        outer2 = ttk.Frame(parent, style="Panel.TFrame", padding=5)
        outer2.grid(row=2, column=0, sticky="nsew")
        self.spec_fig = Figure(figsize=(8, 3.3), dpi=100, facecolor=self.panel)
        self.spec_ax = self.spec_fig.add_subplot(111)
        self._style_axes(self.spec_ax, "Instantaneous DDS Frequency",
                         "Clock samples", "Frequency (MHz)")
        self.spec_ax.set_xlim(0, self.display_points)
        (self.freq_line,) = self.spec_ax.plot([], [], color=self.green,
                                              linewidth=1.5)
        self.base_line = self.spec_ax.axhline(0, linestyle="--", linewidth=0.8,
                                              alpha=0.35, color=self.muted)
        self.spec_fig.tight_layout()
        self.spec_canvas = FigureCanvasTkAgg(self.spec_fig, master=outer2)
        self.spec_canvas.get_tk_widget().pack(fill="both", expand=True)

    def _build_status_bar(self):
        bar = tk.Frame(self.root, bg="#07101c", padx=14, pady=8)
        bar.pack(fill="x", side="bottom")
        self.status_label = tk.Label(bar, textvariable=self.status_var,
                                     bg="#07101c", fg=self.green,
                                     font=("Consolas", 10, "bold"))
        self.status_label.pack(side="left")
        tk.Label(
            bar,
            text=f"{CLOCK_HZ / 1e6:.0f} MHz clock  •  {FREQ_WIDTH}-bit DDS  •  "
                 f"{ROM_SIZE}-point sine ROM  •  priority: depth → noise → salinity",
            bg="#07101c", fg=self.muted, font=("Segoe UI", 8)
        ).pack(side="right")

    # ------------------------------------------------------ serial / UART --
    def _refresh_ports(self):
        if not PYSERIAL_AVAILABLE:
            self.port_combo["values"] = ["pyserial not installed"]
            return
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.port_combo["values"] = ports
        if ports:
            self.port_combo.current(0)

    def _toggle_connection(self):
        if self.serial_conn is not None:
            self._disconnect()
            return
        if not PYSERIAL_AVAILABLE:
            messagebox.showerror("pyserial not installed",
                                 "Install it with:  pip install pyserial")
            return
        port = self.port_combo.get()
        if not port:
            messagebox.showwarning("No port selected",
                                   "Choose a serial port first.")
            return
        try:
            self.serial_conn = serial.Serial(port, BAUD_RATE, timeout=0.1)
        except Exception as exc:
            messagebox.showerror("Connection failed", str(exc))
            self.serial_conn = None
            return

        self.simulate_var.set(False)
        self.conn_label.config(text=f"Connected: {port}", fg=self.green)
        self.connect_btn.config(text="Disconnect")
        self.hw_mode = None
        self.last_rx_time = None
        self.rx_error = None
        self.rx_stop_event.clear()
        self.rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self.rx_thread.start()

        self._sent.clear()
        self._push_env()             # sync FPGA registers to the sliders
        self._set_view_mode()

    def _disconnect(self):
        self.rx_stop_event.set()
        if self.rx_thread is not None:
            self.rx_thread.join(timeout=1.0)
            self.rx_thread = None
        if self.serial_conn is not None:
            try:
                self.serial_conn.close()
            except Exception:
                pass
        self.serial_conn = None
        self.hw_mode = None
        self._pending_writes.clear()
        self.conn_label.config(text="Disconnected", fg=self.red)
        self.connect_btn.config(text="Connect")
        self._set_view_mode()

    def _rx_loop(self):
        """Background thread: decode streaming frames -> queue for the UI."""
        parser = FrameParser()
        conn = self.serial_conn
        while not self.rx_stop_event.is_set():
            try:
                chunk = conn.read(64)
            except Exception as exc:
                self.rx_error = f"Serial read failed: {exc}"
                break
            if not chunk:
                continue
            for frame in parser.feed(chunk):
                self.sample_queue.put(frame)

    def _queue_uart_write(self, reg, value):
        if self.serial_conn is None or self._sent.get(reg) == value:
            return
        self._sent[reg] = value
        self._pending_writes[reg] = value
        if not self._flush_scheduled:
            self._flush_scheduled = True
            self.root.after(UART_WRITE_THROTTLE_MS, self._flush_uart_writes)

    def _flush_uart_writes(self):
        self._flush_scheduled = False
        pending, self._pending_writes = self._pending_writes, {}
        conn = self.serial_conn
        if conn is None:
            return
        try:
            for reg, val in pending.items():
                conn.write(bytes([reg, val]))
        except Exception as exc:
            self._disconnect()
            messagebox.showerror("Write failed", str(exc))

    def _push_env(self):
        for reg, var in ((REG_DEPTH, self.depth_var),
                         (REG_SALINITY, self.salinity_var),
                         (REG_NOISE_LEVEL, self.noise_var)):
            self._queue_uart_write(reg, int(var.get()))

    def _on_simulate_toggle(self):
        if self.simulate_var.get() and self.serial_conn is not None:
            self._disconnect()
        self._set_view_mode()

    # ---------------------------------------------------- model / display --
    def _on_env_change(self):
        d, s, n = (int(self.depth_var.get()), int(self.salinity_var.get()),
                   int(self.noise_var.get()))
        old = self.model.active_key
        changed = self.model.set_environment(d, s, n)
        self._last_env_change = time.monotonic()

        self._refresh_decision()
        if changed:
            self._log(f"{old} → {self.model.active_key}   (D{d} S{s} N{n})")
            self._set_status(
                f"ADAPTATION → {self.model.active_key.upper()} "
                f"(controller update: 1 clock = {1e9 / CLOCK_HZ:.0f} ns)",
                self.orange)
        self._push_env()

    def _refresh_decision(self):
        p = self.model.params
        self.condition_var.set(p["short_reason"])
        self.mode_var.set(p["waveform_mode"])
        self.reason_var.set(p["reason"])
        self.mv["freq_word"].set(f'{p["freq_word"]:,}')
        self.mv["chirp_word"].set(f'{p["chirp_rate"]:+d}')
        self.mv["chirp_hz"].set(f'{self.model.chirp_rate_hz_per_clock:+.1f}')
        self.mv["amp"].set(f'{p["amplitude"]} / 255')
        self.mv["pw"].set(f'{p["pulse_width"]:,} clk')

        # frequency axis: from base word to base + chirp*pulse_width
        base = p["freq_word"]
        end = base + p["chirp_rate"] * p["pulse_width"]
        lo, hi = word_to_hz(min(base, end)) / 1e6, word_to_hz(max(base, end)) / 1e6
        pad = max(0.15 * (hi - lo), 0.3)
        self.spec_ax.set_ylim(lo - pad, hi + pad)
        b = word_to_hz(base) / 1e6
        self.base_line.set_ydata([b, b])
        self._retitle()

    def _retitle(self):
        p = self.model.params
        if self.simulate_var.get():
            title = f"Generated Sonar Ping — {p['waveform_mode']}"
        else:
            name = MODE_NAMES.get(self.hw_mode, "waiting…") \
                if self.hw_mode is not None else "waiting…"
            title = f"FPGA sample_out — live UART stream ({name})"
        self.wave_ax.set_title(title, color=self.text, fontsize=11,
                               loc="left", pad=9)
        self.spec_ax.set_title(
            f"Expected DDS Frequency (model) • chirp = "
            f"{p['chirp_rate']:+d} word/clk",
            color=self.text, fontsize=11, loc="left", pad=9)
        self.wave_canvas.draw_idle()
        self.spec_canvas.draw_idle()

    def _set_view_mode(self):
        if self.simulate_var.get():
            self.wave_ax.set_xlim(0, self.display_points)
            self.wave_ax.set_xlabel("Modelled clock samples", color=self.muted)
        else:
            self.wave_ax.set_xlim(0, HW_WINDOW_SAMPLES)
            self.wave_ax.set_xlabel("Received frames", color=self.muted)
        self._retitle()

    def _set_status(self, text, color=None):
        self.status_var.set(text)
        self.status_label.config(fg=color or self.green)

    def _log(self, line):
        stamp = time.monotonic() - self._t0
        self.log_text.config(state="normal")
        self.log_text.insert("end", f"[{stamp:7.1f}s] {line}\n")
        if int(self.log_text.index("end-1c").split(".")[0]) > 60:
            self.log_text.delete("1.0", "2.0")
        self.log_text.see("end")
        self.log_text.config(state="disabled")

    # ------------------------------------------------------------ drawing --
    def _draw_model_view(self):
        s, a, f, v = self.model.preview(self.display_points)
        self.wave_line.set_data(self._x_sim, s)
        fmhz = np.where(v, f * CLOCK_HZ / PHASE_MOD / 1e6, np.nan)
        self.freq_line.set_data(self._x_sim, fmhz)
        self.wave_canvas.draw_idle()
        self.spec_canvas.draw_idle()

        m = self.model
        self.mv["freq_hz"].set(f"{m.current_frequency_hz / 1e6:.3f} MHz")
        self.mv["rom"].set(str(m.rom_addr))
        self.mv["sample"].set(f"{m.sample_out:+d} / {int(m.sample_valid)}")
        self.total_pulses_var.set(
            f"Pulses completed: {m.total_pulses:,}   counter: {m.pulse_counter:,}")

    def _drain_hw(self):
        got = False
        prev_mode = self.hw_mode
        while True:
            try:
                mode, sample = self.sample_queue.get_nowait()
            except queue.Empty:
                break
            self.hw_data.append(sample)
            self.hw_mode = mode
            got = True
        if got:
            self.last_rx_time = time.monotonic()
        if self.hw_mode != prev_mode:
            self._retitle()
        return got

    def _draw_hw_view(self, got):
        if got:
            self.wave_line.set_data(self._x_hw, list(self.hw_data))
            self.wave_canvas.draw_idle()
            self.mv["sample"].set(f"{self.hw_data[-1]:+d} (FPGA)")
        self.mv["rom"].set("—")
        p = self.model.params
        self.mv["freq_hz"].set(f'{word_to_hz(p["freq_word"]) / 1e6:.3f} MHz')
        # expected trace stays visible so judges see model vs hardware
        s, a, f, v = self.model.preview(self.display_points)
        fmhz = np.where(v, f * CLOCK_HZ / PHASE_MOD / 1e6, np.nan)
        self.freq_line.set_data(self._x_sim, fmhz)
        self.spec_canvas.draw_idle()
        self.total_pulses_var.set("Hardware mode — model shown as reference")

    def _update_check(self, now):
        if self.simulate_var.get():
            self.fpga_mode_var.set("—")
            self.check_var.set("MODEL ONLY")
            self.check_label.config(fg=self.muted)
            return
        if self.serial_conn is None:
            self.fpga_mode_var.set("—")
            self.check_var.set("NOT CONNECTED")
            self.check_label.config(fg=self.red)
            return
        if self.hw_mode is None:
            self.fpga_mode_var.set("—")
            self.check_var.set("WAITING FOR DATA")
            self.check_label.config(fg=self.orange)
            return
        self.fpga_mode_var.set(f"{self.hw_mode} • {MODE_NAMES.get(self.hw_mode, '?')}")
        if self.hw_mode == self.model.mode_code:
            self.check_var.set("MATCH ✓")
            self.check_label.config(fg=self.green)
        elif now - self._last_env_change < 0.5:
            self.check_var.set("SETTLING…")
            self.check_label.config(fg=self.orange)
        else:
            self.check_var.set(
                f"MISMATCH ✗ (expected {self.model.mode_code})")
            self.check_label.config(fg=self.red)

    # ------------------------------------------------------------ controls --
    def toggle_run(self):
        self.running = not self.running
        if self.running:
            self.start_btn.config(text="❚❚ Pause")
            self._set_status("RUNNING")
            if self.demo_running:
                self._demo_next = time.monotonic() + self._hold_s()
        else:
            self.start_btn.config(text="▶ Start")
            self._set_status("PAUSED", self.orange)

    def reset(self):
        self.demo_running = False
        self.demo_index = 0
        self.model.reset()
        self.depth_var.set(self.model.depth)
        self.salinity_var.set(self.model.salinity)
        self.noise_var.set(self.model.noise)
        self.hw_data.extend([0] * HW_WINDOW_SAMPLES)
        self.hw_mode = None
        self._on_env_change()
        self._set_status("RESET • system returned to baseline inputs")
        if not self.running:
            self._draw_model_view()

    def _hold_s(self):
        try:
            return max(1, int(self.demo_hold_var.get()))
        except (tk.TclError, ValueError):
            return 4

    def start_demo(self):
        self.running = True
        self.demo_running = True
        self.demo_index = 0
        self.start_btn.config(text="❚❚ Pause")
        self._advance_demo(time.monotonic())

    def stop_demo(self):
        self.demo_running = False
        self._set_status("DEMO STOPPED", self.orange)

    def _advance_demo(self, now):
        label, d, s, n = SCENARIOS[self.demo_index]
        self.depth_var.set(d)
        self.salinity_var.set(s)
        self.noise_var.set(n)
        self._on_env_change()
        self._set_status(
            f"DEMO • SCENARIO {self.demo_index + 1}/{len(SCENARIOS)}: "
            f"{label.upper()}", self.accent)
        self.demo_index = (self.demo_index + 1) % len(SCENARIOS)
        self._demo_next = now + self._hold_s()

    # ----------------------------------------------------------- main loop --
    def _tick(self):
        try:
            now = time.monotonic()
            if self.rx_error:
                msg, self.rx_error = self.rx_error, None
                self._disconnect()
                messagebox.showerror("Serial error", msg)

            if self.running:
                if self.demo_running and now >= self._demo_next:
                    self._advance_demo(now)
                if self.simulate_var.get():
                    self.model.advance(self.speed_var.get())
                    self._draw_model_view()
                else:
                    got = self._drain_hw()
                    self._draw_hw_view(got)
            self._update_check(now)
        except Exception as exc:
            self.running = False
            self.demo_running = False
            self.start_btn.config(text="▶ Start")
            messagebox.showerror("Simulation error",
                                 f"The GUI stopped because of:\n\n{exc}")
        self.root.after(self.frame_ms, self._tick)

    def on_close(self):
        self._disconnect()
        self.root.destroy()


def main():
    root = tk.Tk()
    app = SonarControlGUI(root)
    root.protocol("WM_DELETE_WINDOW", app.on_close)
    root.mainloop()


if __name__ == "__main__":
    main()

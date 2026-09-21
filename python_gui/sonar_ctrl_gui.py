"""
Adaptive Sonar Transmitter - Control & Monitoring GUI
======================================================

Talks to the FPGA over UART to set simulated environmental parameters
(depth, salinity, noise_level) and displays the live sample_out /
waveform_mode stream on a rolling oscilloscope-style plot.

PROTOCOL (must match the FPGA-side UART register interface):
  Host -> FPGA:  2 bytes  [register_id][value]
      register_id: 0x01 = depth        (0-255)
                   0x02 = salinity      (0-255)
                   0x03 = noise_level   (0-255)

  FPGA -> Host:  3-byte streaming frames, sent continuously
      [0xAA][waveform_mode (0-3)][sample_out (signed 8-bit, two's complement)]

If you change register IDs or the frame format on the FPGA side, update
the constants in the CONFIG section below to match.

Dependencies:
    pip install pyserial matplotlib

Run:
    python sonar_control_gui.py
"""

import threading
import queue
import time
import struct
import tkinter as tk
from tkinter import ttk, messagebox

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
BAUD_RATE = 115200
REG_DEPTH = 0x01
REG_SALINITY = 0x02
REG_NOISE_LEVEL = 0x03
FRAME_HEADER = 0xAA
FRAME_SIZE = 3          # header + mode + sample
PLOT_WINDOW_SAMPLES = 300
PLOT_REFRESH_MS = 40
# =============================================================================


class SonarControlGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Adaptive Sonar Transmitter - Control Panel")
        self.root.geometry("880x560")

        self.serial_conn = None
        self.rx_thread = None
        self.rx_stop_event = threading.Event()
        self.sample_queue = queue.Queue()

        self.simulate_var = tk.BooleanVar(value=True)
        self.depth_var = tk.IntVar(value=100)
        self.salinity_var = tk.IntVar(value=60)
        self.noise_var = tk.IntVar(value=30)

        self.plot_data = [0] * PLOT_WINDOW_SAMPLES
        self.current_mode = 0

        self._build_ui()
        self._start_simulation_source()  # harmless if simulate is off; it just idles
        self.root.after(PLOT_REFRESH_MS, self._refresh_plot)

    # ------------------------------------------------------------------ UI --
    def _build_ui(self):
        top = ttk.Frame(self.root, padding=10)
        top.pack(side=tk.TOP, fill=tk.X)

        # --- Connection row ---
        conn_frame = ttk.LabelFrame(top, text="Connection", padding=10)
        conn_frame.pack(side=tk.LEFT, fill=tk.Y, padx=(0, 10))

        ttk.Label(conn_frame, text="Port:").grid(row=0, column=0, sticky="w")
        self.port_combo = ttk.Combobox(conn_frame, width=14, state="readonly")
        self.port_combo.grid(row=0, column=1, padx=5)
        self._refresh_ports()

        ttk.Button(conn_frame, text="Refresh", command=self._refresh_ports).grid(
            row=0, column=2, padx=5
        )

        self.connect_btn = ttk.Button(
            conn_frame, text="Connect", command=self._toggle_connection
        )
        self.connect_btn.grid(row=1, column=0, columnspan=2, pady=(8, 0), sticky="we")

        self.status_label = ttk.Label(conn_frame, text="Disconnected", foreground="red")
        self.status_label.grid(row=2, column=0, columnspan=3, pady=(8, 0))

        sim_check = ttk.Checkbutton(
            conn_frame,
            text="Simulate (no hardware)",
            variable=self.simulate_var,
            command=self._on_simulate_toggle,
        )
        sim_check.grid(row=3, column=0, columnspan=3, pady=(8, 0), sticky="w")

        # --- Parameter sliders ---
        param_frame = ttk.LabelFrame(top, text="Environmental Parameters", padding=10)
        param_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        self._make_slider(param_frame, "Depth", self.depth_var, REG_DEPTH, row=0)
        self._make_slider(param_frame, "Salinity", self.salinity_var, REG_SALINITY, row=1)
        self._make_slider(param_frame, "Noise Level", self.noise_var, REG_NOISE_LEVEL, row=2)

        # --- Mode indicator ---
        mode_frame = ttk.LabelFrame(top, text="Live Status", padding=10)
        mode_frame.pack(side=tk.LEFT, fill=tk.Y, padx=(10, 0))
        ttk.Label(mode_frame, text="waveform_mode:").pack(anchor="w")
        self.mode_value_label = ttk.Label(
            mode_frame, text="--", font=("TkDefaultFont", 20, "bold")
        )
        self.mode_value_label.pack(anchor="w", pady=(0, 10))
        self.mode_desc_label = ttk.Label(mode_frame, text="")
        self.mode_desc_label.pack(anchor="w")

        # --- Plot ---
        plot_frame = ttk.Frame(self.root, padding=10)
        plot_frame.pack(side=tk.BOTTOM, fill=tk.BOTH, expand=True)

        self.fig = Figure(figsize=(8, 3.2), dpi=100)
        self.ax = self.fig.add_subplot(111)
        self.ax.set_title("sample_out (live)")
        self.ax.set_ylim(-140, 140)
        self.ax.set_xlim(0, PLOT_WINDOW_SAMPLES)
        (self.line,) = self.ax.plot(range(PLOT_WINDOW_SAMPLES), self.plot_data)

        self.canvas = FigureCanvasTkAgg(self.fig, master=plot_frame)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

    def _make_slider(self, parent, label, var, reg_id, row):
        ttk.Label(parent, text=label).grid(row=row, column=0, sticky="w", pady=6)
        slider = ttk.Scale(
            parent,
            from_=0,
            to=255,
            orient=tk.HORIZONTAL,
            variable=var,
            command=lambda v, r=reg_id, var=var: self._on_slider_change(r, var),
        )
        slider.grid(row=row, column=1, sticky="we", padx=10)
        value_label = ttk.Label(parent, textvariable=var, width=4)
        value_label.grid(row=row, column=2)
        parent.columnconfigure(1, weight=1)

    # ------------------------------------------------------------ Serial IO --
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
            messagebox.showerror(
                "pyserial not installed",
                "Install it with:  pip install pyserial",
            )
            return

        port = self.port_combo.get()
        if not port:
            messagebox.showwarning("No port selected", "Choose a serial port first.")
            return

        try:
            self.serial_conn = serial.Serial(port, BAUD_RATE, timeout=0.1)
        except Exception as exc:
            messagebox.showerror("Connection failed", str(exc))
            self.serial_conn = None
            return

        self.simulate_var.set(False)
        self.status_label.config(text=f"Connected: {port}", foreground="green")
        self.connect_btn.config(text="Disconnect")

        self.rx_stop_event.clear()
        self.rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self.rx_thread.start()

    def _disconnect(self):
        self.rx_stop_event.set()
        if self.rx_thread is not None:
            self.rx_thread.join(timeout=1.0)
        if self.serial_conn is not None:
            self.serial_conn.close()
        self.serial_conn = None
        self.status_label.config(text="Disconnected", foreground="red")
        self.connect_btn.config(text="Connect")

    def _rx_loop(self):
        """Background thread: read streaming frames from the FPGA and push
        decoded samples onto the queue for the UI thread to plot."""
        buf = bytearray()
        while not self.rx_stop_event.is_set():
            try:
                chunk = self.serial_conn.read(64)
            except Exception:
                break
            if not chunk:
                continue
            buf.extend(chunk)

            while len(buf) >= FRAME_SIZE:
                if buf[0] != FRAME_HEADER:
                    buf.pop(0)  # resync: drop bytes until we see a header
                    continue
                if len(buf) < FRAME_SIZE:
                    break
                _, mode, raw_sample = buf[:FRAME_SIZE]
                sample = struct.unpack("b", bytes([raw_sample]))[0]  # signed 8-bit
                self.sample_queue.put((mode, sample))
                del buf[:FRAME_SIZE]

    def _on_slider_change(self, register_id, var):
        value = int(var.get())
        if self.serial_conn is not None:
            try:
                self.serial_conn.write(bytes([register_id, value]))
            except Exception as exc:
                messagebox.showerror("Write failed", str(exc))
        # In simulate mode the slider values are read directly by the
        # synthetic waveform generator below, so no explicit send is needed.

    def _on_simulate_toggle(self):
        if self.simulate_var.get() and self.serial_conn is not None:
            self._disconnect()

    # -------------------------------------------------------- Simulate mode --
    def _start_simulation_source(self):
        self._sim_phase = 0.0
        self.root.after(20, self._simulation_tick)

    def _simulation_tick(self):
        if self.simulate_var.get():
            import math

            depth = self.depth_var.get()
            salinity = self.salinity_var.get()
            noise = self.noise_var.get()

            # Purely illustrative mapping so the GUI is demo-able without
            # hardware. Replace with real FPGA data once the UART link is up.
            freq = 0.05 + (depth / 255.0) * 0.6
            amplitude = 40 + (salinity / 255.0) * 80
            jitter = (noise / 255.0) * 15

            self._sim_phase += freq
            base = amplitude * math.sin(self._sim_phase)
            import random

            sample = int(max(-127, min(127, base + random.uniform(-jitter, jitter))))
            mode = 0 if depth < 85 else (1 if depth < 170 else 2)
            self.sample_queue.put((mode, sample))

        self.root.after(20, self._simulation_tick)

    # ------------------------------------------------------------- Plotting --
    def _refresh_plot(self):
        updated = False
        while not self.sample_queue.empty():
            mode, sample = self.sample_queue.get_nowait()
            self.plot_data.pop(0)
            self.plot_data.append(sample)
            self.current_mode = mode
            updated = True

        if updated:
            self.line.set_ydata(self.plot_data)
            self.canvas.draw_idle()

            mode_names = {0: "CW - Low Freq", 1: "CW - High Freq", 2: "LFM Chirp"}
            self.mode_value_label.config(text=str(self.current_mode))
            self.mode_desc_label.config(
                text=mode_names.get(self.current_mode, "Unknown")
            )

        self.root.after(PLOT_REFRESH_MS, self._refresh_plot)

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
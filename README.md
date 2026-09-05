# Adaptive Sonar Transmitter - RTL Prototype

## Files
- `rtl/param_controller.v` - adaptation decision logic (sensors -> waveform params)
- `rtl/dds_waveform_gen.v` - DDS phase accumulator + sine ROM waveform synthesis
- `rtl/adaptive_sonar_top.v` - top-level, ties both together + pulse timing
- `tb/tb_adaptive_sonar.v` - testbench, runs 5 scenarios in one simulation

## Build & run (Icarus Verilog)
Install once if you don't have it:
```
sudo apt-get install iverilog gtkwave
```

Compile and simulate:
```
cd sonar_prototype
iverilog -o sim.out rtl/param_controller.v rtl/dds_waveform_gen.v rtl/adaptive_sonar_top.v tb/tb_adaptive_sonar.v
vvp sim.out
```

This prints a `$monitor` log to the console and writes `adaptive_sonar.vcd`.

Open the waveform:
```
gtkwave adaptive_sonar.vcd
```
In GTKWave, drag these signals into the viewer (left panel -> tb_adaptive_sonar -> ...):
- `depth`, `salinity`, `noise_level` (the "sensor" inputs)
- `dut.waveform_mode` (0=CW, 1=up-chirp, 2=down-chirp)
- `sample_out` (the actual synthesized waveform - set its format to "Analog > Step" or "Analog > Interpolated" for a nice continuous look)

Save your view (File > Write Save File) so it reopens instantly next time -
important for not fumbling this live in front of judges.

## What to point at during the demo
Zoom into the transition points in the trace (every ~2000ns in this
testbench) and show: sensor input changes -> `waveform_mode` and `sample_out`
change within a handful of clock cycles (10ns period here). That's your
concrete "real-time adaptive" evidence - you can literally count clock edges
in GTKWave's cursor and quote the latency as a number.

## Note on where I generated this
I don't have network access in this sandbox, so I wasn't able to install
Icarus Verilog here to compile-check this code myself. Please run the
commands above yourself as the very first step, before building anything
on top (the Python GUI wrapper), so you catch any syntax issue early instead
of on the day of the demo.

## Next layer (not included here yet)
The Python dashboard should, on a button click:
1. Write `depth`/`salinity`/`noise_level` values into a small stimulus file
   (or directly into a copy of the testbench via string templating)
2. Call `iverilog` + `vvp` as subprocesses
3. Either auto-launch `gtkwave adaptive_sonar.vcd`, or parse the `.vcd` with
   the `vcdvcd` Python library and re-plot `sample_out` inline in the GUI
   as a backup in case GTKWave is hard to read on a projector.

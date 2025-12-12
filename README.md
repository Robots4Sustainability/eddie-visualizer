# Cartesian Error Visualizer

Real-time visualization of cartesian position and orientation errors from dual-arm robot manipulators.

## Quick Start

**Install dependencies:**

```bash
pip install -r requirements.txt
```

**Run the demo:**

```bash
# Terminal 1: Run the demo error publisher
python demo_error_publisher.py

# Terminal 2: Run the visualizer
python cartesian_error_visualizer.py
```

## What It Does

Subscribes to cartesian error topics and plots:

- **Linear errors** (X, Y, Z position) in meters
- **Angular errors** (Roll, Pitch, Yaw orientation) in radians

Topics:

- `right_arm/cartesian_error`
- `left_arm/cartesian_error`

## Demo Publisher

`demo_error_publisher.py` simulates realistic error variations:

- Converging trajectories
- Step responses with overshoot
- Oscillatory behavior
- Tracking errors
- Large movements

Cycles through scenarios every 5 seconds at 50 Hz.

# Plot Torque Log

`plot_torque_log.py` reads a torque log CSV file and plots the raw vs. smoothed torque for each joint found.
Optionally plots only the last N seconds of data.

**Run the plot**
```bash
python plot_torque_log.py ~/your_workspace/torque_log.csv --last-seconds 5
```
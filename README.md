# dafo-human

*[Versión en español](README.es.md)*

Minimal base to install, run, and understand how this MuJoCo simulator for Unitree robots is used.

## Documentation map

- [INSTALL.md](INSTALL.md): local installation and baseline validation.
- [RUNBOOK.md](RUNBOOK.md): day-to-day operation of the simulator, viewer, teleop, and demo.
- [WALKING.md](WALKING.md): current state of the walking controller and how to test it.
- [WORKSHOP.md](WORKSHOP.md): step-by-step walkthrough — manual control vs. Reinforcement Learning vs. objects in the scene.
- [REINFORCEMENT_LEARNING.md](REINFORCEMENT_LEARNING.md): how the RL policy that keeps the robot standing works internally.
- [FULL_BODY_INTEGRATION.md](FULL_BODY_INTEGRATION.md): what happened when combining the RL legs with the full arms/hands model, and how to actually fix it (retraining).

## What's in this repository

This project is oriented toward local execution. Today it does not include a remote deployment pipeline, containers, or infrastructure scripts. The real flow is:

1. Install Python dependencies.
2. Have `third_party/mujoco_menagerie` available.
3. Launch the simulator in `viewer` or `headless` mode.
4. Control it from another terminal over UDP with `teleop_unitree.py`.

## Requirements

- macOS or Linux with Python 3.
- A virtualenv in `.venv`.
- MuJoCo Python `3.2.7`.
- The `third_party/mujoco_menagerie` folder with the Unitree models.
- Optional: `ffmpeg` to export video in the grasp demo.

## Installation

The detailed guide is in [INSTALL.md](INSTALL.md).

Create the environment and install dependencies:

```bash
cd /path/to/repo/dafo-human
python3 -m venv .venv
.venv/bin/pip install -r requirements.txt
```

If the MuJoCo menagerie is missing, clone it inside `third_party` so paths like these exist:

- `third_party/mujoco_menagerie/unitree_h1/scene.xml`
- `third_party/mujoco_menagerie/unitree_g1/scene.xml`
- `third_party/mujoco_menagerie/unitree_g1/scene_with_hands.xml`

## Quick start

Headless test to validate that MuJoCo compiles the model:

```bash
cd /path/to/repo/dafo-human
.venv/bin/python simulate_unitree.py --robot g1-hands --mode headless --steps 300
```

Open the simulator with the viewer:

```bash
cd /path/to/repo/dafo-human
.venv/bin/mjpython simulate_unitree.py --robot g1-hands --mode viewer
```

Models supported by the launcher:

- `h1`
- `g1`
- `g1-hands`

## How this "deploys" today

In this repo, deployment means local execution of the simulator. There is no separate build/deploy stage to a server.

The entry point is [simulate_unitree.py](simulate_unitree.py), which:

- Resolves the robot's scene.
- Compiles the MuJoCo model.
- Applies the initial keyframe.
- Opens the viewer or runs a headless test.
- Can listen for external control over UDP on `127.0.0.1:47001`.

## Controlling the robot

Full operation is documented in [RUNBOOK.md](RUNBOOK.md).

The viewer runs the physics and listens for external commands over UDP. Teleop runs in another terminal:

```bash
cd /path/to/repo/dafo-human
.venv/bin/python teleop_unitree.py --host 127.0.0.1 --port 47001
```

There's also a wrapper:

```bash
cd /path/to/repo/dafo-human
.venv/bin/python teleop_unitree.pyw --host 127.0.0.1 --port 47001
```

Teleop controls:

- `W/S`: forward and backward
- `A/D`: turn
- `Space`: center joystick
- `R`: reset
- `P/O`: pause and resume
- `J/K`: decrease/increase amplitude
- `N/M`: decrease/increase frequency
- `Q`: quit

## Available demo

There's a reach-and-grasp demo for the G1 with hands in [reach_grasp_demo.py](reach_grasp_demo.py).

Run without video:

```bash
cd /path/to/repo/dafo-human
.venv/bin/python reach_grasp_demo.py --no-video
```

Export video:

```bash
cd /path/to/repo/dafo-human
.venv/bin/python reach_grasp_demo.py
```

The video is written by default to `artifacts/g1_reach_grasp.mp4`.

## Main files

- [simulate_unitree.py](simulate_unitree.py): main launcher.
- [interactive_unitree.py](interactive_unitree.py): viewer loop and interactive controller.
- [external_control.py](external_control.py): UDP transport.
- [teleop_unitree.py](teleop_unitree.py): raw-mode keyboard to send commands.
- [send_unitree_command.py](send_unitree_command.py): one-shot UDP command sending.
- [reach_grasp_demo.py](reach_grasp_demo.py): reach and grasp demo.

## Common issues

If the viewer doesn't open correctly:

- Use `.venv/bin/mjpython` instead of `.venv/bin/python` for `viewer` mode.
- Check that port `47001` isn't already in use by another instance.

If a "port in use" error appears:

```bash
lsof -nP -iUDP:47001
```

If the scene doesn't exist:

- Check that `third_party/mujoco_menagerie` is present.
- Or use `--xml /path/to/scene.xml` to pass a custom scene.

## Useful commands

Viewer with UDP control disabled:

```bash
cd /path/to/repo/dafo-human
.venv/bin/mjpython simulate_unitree.py --robot g1-hands --mode viewer --control-port 0
```

Viewer with automatic close to validate startup:

```bash
cd /path/to/repo/dafo-human
.venv/bin/mjpython simulate_unitree.py --robot g1-hands --mode viewer --max-seconds 10
```

Headless test with another robot:

```bash
cd /path/to/repo/dafo-human
.venv/bin/python simulate_unitree.py --robot h1 --mode headless --steps 300
```

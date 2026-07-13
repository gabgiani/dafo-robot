# Workshop: from manual control to walking with Reinforcement Learning

*[Versión en español](WORKSHOP.es.md)*

## Workshop objective

What we're trying to achieve, in one sentence: get a bipedal humanoid robot (Unitree
G1) to **stay standing** and, from there, **walk forward without falling** — first with
a hand-written controller and then with an already-trained Reinforcement Learning (RL)
policy.

By the end of this workshop, each participant will be able to:

- Explain why a standing bipedal robot is unstable and what it takes to keep it upright.
- Run a hand-written controller (fixed formulas per motor) and see under what
  conditions it falls.
- Run an already-trained RL policy and compare, with real numbers, how much better it
  holds balance than the manual controller on the same task.
- Add objects to the scene and check whether the robot stays standing when it hits
  something the original training didn't anticipate.
- Reproduce, step by step, the same tests documented here (with their real results and
  screenshots) to verify for themselves what we report.

## The problem we're solving

A standing humanoid robot is a tall, narrow tower resting on two small feet: any small
angle error in a leg, or any push, can knock it over. Keeping it standing — and on top
of that walking — requires constantly correcting balance, many times per second. There
are two ways to solve this that we're going to compare in this workshop:

1. **By hand**: a human writes fixed formulas (stride amplitude, knee flexion,
   correction forces) and tunes them by trial and error. It works up to a point, but
   it's fragile.
2. **With a learned policy (RL)**: a neural network learned on its own, in simulation
   and by trial and error, what torque to send to each motor to avoid falling, without
   a human writing the formulas by hand.

The detail of what Reinforcement Learning is and how the already-trained policy works
is in [REINFORCEMENT_LEARNING.md](REINFORCEMENT_LEARNING.md); in this workshop we focus
on the practical experience of running and comparing both approaches.

## Prerequisite

Follow [INSTALL.md](INSTALL.md) to have the environment (`.venv`) and
`third_party/mujoco_menagerie` installed.

## The 4 stages

Each stage has its own page with: the specific objective of that stage, how to run it,
what it consists of, what to look at, how the problem is solved, the real screenshots
from the tests we ran, and what problems we ran into building it — so you can reproduce
each test step by step and check whether you get the same result.

1. **[Heuristic control](workshop/01-control-heuristico.md)** — objective: make the
   robot walk with hand-written formulas per motor + remote control, and find the point
   where it falls.
2. **[Reinforcement Learning](workshop/02-reinforcement-learning.md)** — objective: the
   same task (walking without falling), replacing hand-written formulas with an
   already-trained policy.
3. **[Objects in the scene](workshop/03-objetos-en-el-escenario.md)** — objective: check
   whether the RL policy stays standing when it also has to deal with boxes and a shelf
   in its path.
4. **[SONIC whole-body control](workshop/04-control-cuerpo-completo-sonic.md)** —
  objective: walk physically with a 29-DOF policy and coordinate a carrying posture
  or natural arm swing from a safe graphical remote.

## Quick summary: what to run in each stage

If you just want the commands to try right now, here they are (the detail of each is
on that stage's page).

**Stage 1 — heuristic:**
```bash
./run_viewer.sh
```
```bash
.venv/bin/python send_unitree_command.py --advance 0.5
```

**Stage 2 — RL:**
```bash
./run_viewer_rl.sh
```
```bash
.venv/bin/python send_unitree_command.py --advance 0.6
```
(or directly `W/A/S/D` in the viewer window, no need for the second terminal)

**Stage 3 — RL + objects:**
```bash
.venv/bin/mjpython simulate_g1_rl.py --scene third_party/unitree_rl_gym/resources/robots/g1_description/g1_warehouse_scene.xml
```
```bash
.venv/bin/python send_unitree_command.py --advance 0.6
```

**Stage 4 — whole-body SONIC:**
```bash
# Run both commands in the same remote Linux desktop session:
export SONIC_ROOT="$PWD"
./dafo-human-sonic/sonic_desktop_control.sh sim
./dafo-human-sonic/sonic_desktop_control.sh control
```

Wait for `Policy: Listo`, press **Activar**, and do not move until the teleop reports
`Control conectado`. Stage 4 has additional prerequisites and an upstream patch;
follow its complete page before running these commands.

## To keep digging deeper

- [WALKING.md](WALKING.md): technical detail of the heuristic controller from stage 1.
- [REINFORCEMENT_LEARNING.md](REINFORCEMENT_LEARNING.md): how the stage 2 policy works internally (what RL is, how many motors, how many observation values, what the viewer panels mean).
- [RUNBOOK.md](RUNBOOK.md): day-to-day operation of the simulator in general.
- [FULL_BODY_INTEGRATION.md](FULL_BODY_INTEGRATION.md): why the earlier attempt to mix a
  leg policy with upper-body PD control was not physically stable.

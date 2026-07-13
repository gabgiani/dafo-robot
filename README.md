# DAFO-ROBOT Research

*[Versión en español](README.es.md)*

This repository documents the early research path for teaching a humanoid robot to do
useful physical work. The medium-term milestone is simple and concrete: the robot must
be able to take an object from one place and move it to another without falling,
losing the object, or requiring a human to hand-script every joint motion.

That pick-and-place milestone is not the final goal. It is the first measurable task
that forces the stack to solve the real problems behind physical work: balance,
locomotion, contact, upper-body coordination, perception of the scene, and action
selection over time. Once that foundation is reliable, the same approach can be pushed
toward other tasks, including manual handling flows and tasks derived from assembly
manuals.

## Research objective

The project asks a practical question: how do we turn operational know-how that today
exists in human operators and procedures into robot behavior that can be trained,
tested, reproduced, and improved?

Our working hypothesis is that the right path is not to start from a giant end-to-end
promise, but to decompose the problem into progressive capabilities:

1. keep the robot stable
2. make it move intentionally
3. make it survive contact with the world
4. coordinate the whole body for useful actions
5. connect those capabilities to simple task execution

This repository is the research notebook and implementation base for that path.

## What task are we pursuing?

The concrete task target is intentionally modest at first:

- detect or receive the location of an object
- approach it without falling
- reach it with the upper body
- grasp or carry it
- move it to a second location
- repeat the process under controlled variations

If the project cannot do that reliably in simulation, there is no technical basis for
claiming it can handle richer work such as repetitive handling or assembly manuals.
The simple task is therefore the test bench, not a toy example.

## Why split the research into stages?

Humanoid manipulation fails for many different reasons, and mixing all of them in one
experiment hides the real cause of failure. A staged program makes the failure mode
observable.

- A standing problem is not yet a grasping problem.
- A walking problem is not yet a perception problem.
- An arm-coordination problem is not yet a task-planning problem.
- A whole-body control problem should be measured before claiming task competence.

That is why the repository is organized as workshops and focused documents instead of a
single monolithic demo. Each stage isolates one capability, one comparison, or one
integration problem and records what worked, what failed, and why.

## Why not rely only on deterministic control?

Deterministic methods are still part of the toolbox. We use them for baselines,
instrumentation, safety envelopes, teleoperation, repeatable tests, and simple scripted
behaviors. But they are not enough by themselves for the target we care about.

For a humanoid robot, fully hand-designed control quickly becomes brittle because:

- the state space is high-dimensional
- balance corrections must happen continuously and fast
- contact with the world is hard to model with fixed rules
- upper and lower body decisions interact in ways that are expensive to hand-tune
- the same scripted behavior breaks when friction, timing, or geometry changes

In other words, a deterministic controller can demonstrate a narrow motion, but that is
not the same as a robust capability. This repository therefore compares explicit,
hand-written control against learned policies and then studies where whole-body learned
control becomes necessary.

## Technology stack

The current research stack combines the following pieces:

- **MuJoCo** as the main physics simulator for fast iteration and repeatable tests.
- **Unitree humanoid models** as the embodiment under study, especially G1.
- **Python control tooling** for launchers, teleoperation, experiments, and analysis.
- **Reinforcement Learning policies** for balance and locomotion where fixed formulas are too fragile.
- **Whole-body learned control with NVIDIA GEAR-SONIC** for 29-DOF coordinated motion, including locomotion plus upper-body posture control.
- **Teleoperation interfaces** to command and observe the robot safely while validating policies.
- **Scenario-based workshops** to document each research stage as a reproducible case.

The point is not to accumulate tools. The point is to use the minimum stack that can
answer a hard question with evidence.

The rationale for that stack is straightforward:

- MuJoCo gives fast resets, controlled variation, and reproducible contact experiments.
- Unitree G1 is a realistic humanoid morphology for studying balance plus manipulation.
- Python keeps the experiment loop cheap to modify while the control surfaces are still changing.
- RL is useful where the controller must continuously absorb variation instead of replaying a fixed script.
- SONIC becomes relevant when leg-only competence is no longer enough and the task requires coordinated upper-body behavior.

## Current research roadmap

The project is currently structured as four main stages:

1. **Heuristic control**: test how far hand-written rules can take walking.
2. **Reinforcement Learning locomotion**: compare a trained policy against the manual baseline.
3. **Objects in the scene**: verify whether locomotion survives contact and clutter.
4. **SONIC whole-body control**: move from leg-only competence to coordinated full-body control with upper-body behaviors such as carrying posture and arm swing.

Those stages do not yet mean the task is solved. They mean the foundation is becoming
credible enough to attempt simple object-moving behaviors and later more structured
manual workflows.

## How to read this repository

The README is the project overview. Installation, operation, and stage-by-stage
execution are intentionally separated.

- [INSTALL.md](INSTALL.md): local installation and baseline validation.
- [RUNBOOK.md](RUNBOOK.md): day-to-day operation of the simulator, viewer, teleop, and demo.
- [WORKSHOP.md](WORKSHOP.md): the staged research path and how each capability is tested.
- [WALKING.md](WALKING.md): current state of the hand-written walking controller.
- [REINFORCEMENT_LEARNING.md](REINFORCEMENT_LEARNING.md): what the RL policy is doing and why it helps.
- [FULL_BODY_INTEGRATION.md](FULL_BODY_INTEGRATION.md): why the earlier partial full-body attempt was unstable.

## Repository scope

Stages 1 through 3 are local to this repository. Stage 4 adds SSH-based interaction
with an external NVIDIA GR00T Whole-Body Control installation for SONIC experiments.
This repository documents and orchestrates those experiments; it does not redistribute
NVIDIA checkpoints or pretend that the entire stack is locally self-contained.

## What success looks like

The research direction is useful only if it leads to reproducible capability, not just
interesting demos. In practical terms, success means being able to show, measure, and
improve a robot that can:

- remain stable while idle and while moving
- accept commands safely
- coordinate locomotion and upper body
- interact with objects without immediate collapse
- execute a simple transport task end to end
- become trainable for additional tasks beyond the first scripted scenario

That is the reason for the current focus: not because walking is the final product, but
because walking, whole-body coordination, and object interaction are the minimum base
for any later manual or assembly-oriented task described by an operational procedure or
assembly manual.

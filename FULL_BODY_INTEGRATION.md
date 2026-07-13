# Full-body model (arms + hands): what happened, why, and how to actually fix it

*[Versión en español](FULL_BODY_INTEGRATION.es.md)*

This document records a real integration attempt: combining the RL walking policy (legs only)
with the fully articulated G1 model (arms + hands), what broke, why it broke, and the concrete
path to fix it properly (retraining with the new mass distribution).

## The idea

[simulate_g1_rl.py](simulate_g1_rl.py) drives a pretrained RL policy that only controls 12 leg
joints ([g1_12dof.xml](third_party/unitree_rl_gym/resources/robots/g1_description/g1_12dof.xml)).
That model has no arms/hands joints at all — they're fused, static meshes. To grasp objects we
need a model with real arm/hand joints.

The good news: `unitree_rl_gym` also ships
[g1_29dof_with_hand.xml](third_party/unitree_rl_gym/resources/robots/g1_description/g1_29dof_with_hand.xml),
from the **same source**, with:
- **Identical leg joint names** (`left_hip_pitch_joint`, `left_knee_joint`, etc.) in the same order.
- **Identical actuator type** for the legs (`<motor>`, torque-controlled) — matching exactly what
  the RL policy + our PD control loop expects.

So in principle, we could feed the RL policy's 12 leg torques into this bigger model's first 12
actuators, and control the remaining 31 actuators (waist, arms, hands) separately with our own PD
controller holding a neutral pose.

## What we implemented

1. New scene: [g1_warehouse_scene_full.xml](third_party/unitree_rl_gym/resources/robots/g1_description/g1_warehouse_scene_full.xml),
   including `g1_29dof_with_hand.xml` (reusing the box/shelf objects and camera from the existing
   warehouse scene).
2. Extended [simulate_g1_rl.py](simulate_g1_rl.py):
   - `self._num_upper = self.model.nu - 12` — detects how many actuators exist beyond the legs.
   - A second PD controller holds the upper body (waist/arms/hands) at a neutral target (all zeros),
     with per-joint-group gains:

     | Group | kp | kd |
     |---|---|---|
     | Waist (3 joints) | 80 | 2 |
     | Shoulder/elbow/wrist (14 joints) | 30 | 1 |
     | Fingers (14 joints) | 3 | 0.05 |

   - `_step_physics()` now writes `ctrl[:12]` from the RL policy and `ctrl[12:]` from this second
     PD controller.

This all **compiles and runs** — the wiring is correct.

## What broke, and why it's not a wiring problem

Total mass comparison:

```
legs-only model (what the policy was trained on): 32.11 kg
full model (with real arms + hands):                36.17 kg   (+4 kg, +12.6%)
```

Even standing perfectly still (`cmd = (0, 0, 0)`, no walking command at all), the robot **falls
within under one second**:

```
pelvis height over time: [0.793, 0.769, 0.508, 0.091, 0.141, 0.062, 0.161, ...]
```
(0.793 m is standing height; anything below ~0.4 m means it's on the ground.)

**Root cause:** an RL locomotion policy learns a balance strategy that is tightly coupled to the
exact mass and inertia distribution of the model it was trained on. The legs-only model's arms are
static meshes fused to the pelvis (their mass, if any, is lumped into the torso's inertia in a
simplified way). The full model has real, separately-articulated arm/hand links, each with their
own mass, hanging from the shoulders. This shifts the center of mass and changes the moment of
inertia enough that the policy's learned balance strategy no longer applies — it doesn't just walk
worse, it can't even stand still.

This is **not fixable by tuning the upper-body PD gains** — the leg policy itself doesn't know
how to react to a body it was never trained on.

## What it would take to actually fix this: retraining

### Framework: Isaac Lab, not Isaac Gym

NVIDIA's own page for Isaac Gym literally says **"Isaac Gym - Now Deprecated"** and recommends
migrating to **Isaac Lab**. `legged_gym` (the framework `unitree_rl_gym`'s training code is based
on) announced the same migration. So the correct modern path is Isaac Lab, not the legacy tool.

Isaac Lab already ships **native G1 locomotion environments** we could use as a reference or
starting point:
- `Isaac-Velocity-Flat-G1-v0`
- `Isaac-Velocity-Rough-G1-v0`

### What's needed

1. **Asset**: we already have it — `g1_29dof_with_hand.urdf` is in this repo
   (`third_party/unitree_rl_gym/resources/robots/g1_description/`). Isaac Lab can import URDF
   directly; the correct mass/inertia comes automatically from the file, no manual tuning needed.
2. **A training config** (reward terms), adapted from the G1 reference environment or from
   `unitree_rl_gym`'s own config, for the full-body asset instead of the legs-only one. Typical
   reward terms: track commanded linear/angular velocity, stay upright, minimize energy/jerk,
   penalize falling or excessive contact forces.
3. **Compute**: training runs thousands of parallel simulated environments on GPU for many
   iterations — this is normally **hours** of wall-clock time, not minutes, even on capable
   hardware.
4. **Export**: convert the trained checkpoint to TorchScript (same `.pt` format as
   `motion.pt`) so it can be loaded by a script like `simulate_g1_rl.py` without changes to the
   inference-side code.

### Hardware reality check (verified, not assumed)

Isaac Sim/Isaac Lab's own documented minimum requirements:

```
GPU VRAM: 16 GB or more
RAM:      32 GB or more
```

What we actually have available:

| Machine | VRAM | RAM | Disk free | Meets minimum? |
|---|---|---|---|---|
| RTX 3060 (`192.168.0.60`) | 6 GB | 23 GB | 30 GB (94% used) | ❌ No — well below every requirement |
| Tesla T4 (`10.8.0.3`) | 16 GB | 27 GB | 305 GB | ⚠️ Meets VRAM exactly (no margin), and already shares that GPU with a live TTS service (`orpheus_ollama`) that has already crashed another model once due to resource contention |

**Conclusion**: the RTX 3060 cannot run Isaac Lab at all — this isn't a "slower" scenario, it's
below the documented minimum on every axis. The T4 is borderline-viable but shares its (already
tight) VRAM with a production service.

## Practical alternative (no retraining required)

Instead of forcing one model to do both jobs at once, switch models **sequentially**:
1. Walk to the target using the light legs-only RL model (already proven stable).
2. Once close enough, switch to the full-body model, controlled by the heuristic/position-based
   approach (`interactive_unitree.py`-style), only for the static reach/grasp phase.

This sidesteps the mass-mismatch problem entirely, because the RL policy is never asked to balance
while also carrying arm mass — it only ever runs on the exact model it was trained for.

## Summary

| | Status |
|---|---|
| Same joint names/actuator types between models | ✅ Confirmed, not an issue |
| Upper-body PD hold wiring | ✅ Implemented, compiles and runs |
| Standing balance with the full model | ❌ Falls in under 1 second — mass/inertia mismatch |
| Proper fix (retrain policy) | Requires Isaac Lab + new reward config + GPU with 16GB+ VRAM / 32GB+ RAM |
| Available hardware for that | RTX 3060 fails minimums; T4 borderline but shared with a live service |
| Working alternative today | Sequential model switch (walk with RL, then switch model to grasp) |

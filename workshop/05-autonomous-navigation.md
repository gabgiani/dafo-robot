# Stage 5 — Autonomous RGB-D navigation with obstacle avoidance

*[Versión en español](05-navegacion-autonoma.es.md)*

## Objective of this stage

Make the Unitree G1 travel autonomously from point A to point B without colliding with
the obstacles in between. The robot must perceive the scene through its head camera,
build a traversability map, plan a route, and continuously translate that route into
whole-body SONIC commands while preserving the safety behavior from stage 4.

This stage runs only on the remote Linux machine. macOS remains the development and
control workstation; MuJoCo rendering and the SONIC policy stay on the Linux GPU host.

## How to run it

From `/home/nvidia/GR00T-WholeBodyControl` on the remote Linux desktop session:

```bash
export DISPLAY=:0
export XAUTHORITY=/run/user/$(id -u)/gdm/Xauthority
dafo-human-sonic/sonic_navigation_stack.sh \
  --goal-x 4.5 --goal-y 0 \
  --speed 0.1 --goal-tolerance 0.35 --timeout 90
```

The launcher installs the navigation scene, starts the simulator, physical-pose relay,
SONIC policy, and autonomous controller, then stops all of them together. Full logs are
written to `/tmp/sonic_navigation_*.log`.

## What it consists of

The scene in
[sonic_navigation_scene.xml](../sonic_navigation_scene.xml) places two fixed obstacles
between the start `(0, 0)` and goal `(4.5, 0)`. The complete stack is split into four
small components:

- [sonic_navigation_sim.py](../sonic_navigation_sim.py) keeps the RGB JPEG stream on
  port `5555` and publishes metric depth plus camera calibration on port `5559`.
- [sonic_navigation_planner.py](../sonic_navigation_planner.py) projects depth into
  world coordinates, filters points by height, rasterizes and inflates obstacles, runs
  A*, and selects a lookahead waypoint.
- [sonic_autonomous_navigation.py](../sonic_autonomous_navigation.py) consumes depth,
  physical pose on `5558`, and SONIC telemetry on `5557`, then publishes global
  `movement` and `facing` commands on `5556`.
- [sonic_navigation_stack.sh](../sonic_navigation_stack.sh) owns the Linux-only process
  lifecycle and coordinated shutdown.

For `WALK=2`, both `movement` and `facing` are expressed in global coordinates. The
movement vector carries the bounded requested speed and the planner `speed` field stays
at the SONIC sentinel value `-1.0`.

## Perception and planning pipeline

1. Decode the compressed `(480, 640)` floating-point depth image and its camera
   intrinsics/extrinsics.
2. Back-project sampled pixels into a 3D point cloud and transform it into world space.
3. Keep obstacle-height points, remove returns inside the G1's `0.35 m` body footprint,
   and rasterize the remainder at `0.1 m` resolution.
4. Inflate occupied cells by `0.45 m`, leaving enough room for the robot body.
5. Run eight-connected A* from the current physical pose to the goal.
6. Hold a safe `0.6 m` lookahead waypoint until it is reached or becomes blocked. This
   prevents symmetric routes from switching sides at every depth frame.
7. Rotate progressively toward that waypoint, then walk using global movement and
   facing vectors.

## Safety behavior

Motion does not begin merely because a port exists. The controller waits for a real
depth frame, physical pose, and post-activation SONIC telemetry. During navigation it
publishes a stable `IDLE` and stops if any input becomes stale, the pelvis indicates a
fall, clearance becomes critical, A* has no safe route, a subscriber fails, or the
timeout expires. The launcher then terminates the policy, relay, and simulator as one
unit so the robot is never left without its controller.

## Real remote result

The complete route was validated on the Linux host with `--speed 0.1`:

```text
start:                 (0.00, 0.00)
goal:                  (4.50, 0.00)
maximum lateral detour y: approximately 2.17 m
final reported pose:   approximately (3.83, 0.47)
final goal distance:   0.326 m
fall warnings:         0
policy after exit:     inactive
simulator after exit:  inactive
```

The initial RGB-D frame produced 7,352 obstacle points. The first planned path contained
48 grid cells and detoured around obstacle A instead of crossing it. The physical robot
then passed both obstacles, returned toward the goal axis, reached the configured
`0.35 m` tolerance, and shut down cleanly.

## Problems we ran into

- **A/D changed the requested direction but did not physically turn the robot.** SONIC
  `WALK=2` expects progressive global facing, not a local yaw-rate shortcut. A bounded
  probe measured a requested `+90°` turn as `+88.08°` with no fall.
- **The short smoke goal intermittently had no path.** A goal at `x=0.8` sat on the edge
  of the first obstacle's inflated region. Moving the smoke goal to `x=0.5` separated
  locomotion validation from obstacle avoidance; it reached a final distance of
  `0.135 m` with zero falls.
- **A* alternated between equally valid sides of a symmetric obstacle.** Holding the
  current waypoint until reached or blocked removed the steering oscillation.
- **The robot's moving body appeared in its own depth image.** Those near-base returns
  trapped the A* start and triggered false critical-clearance stops. A bounded body
  self-filter removes points inside `0.35 m`; the larger `0.45 m` map inflation still
  keeps external geometry outside the robot footprint.
- **Independent teardown could make a standing robot fall.** The launcher owns all
  process IDs, publishes stable `IDLE`, and shuts the policy and simulator down together.

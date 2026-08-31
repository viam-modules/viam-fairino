# Module viam-fairino

Viam `arm` component module for the Fairino FR5-WML collaborative robot.

The module connects to the controller's status port (TCP 8083, fixed) for
feedback and the command port (TCP 8080, fixed) for motion, and has a real
kinematic model (`Kinematics`/`Geometries`/`Get3DModels`, including 3D-scene
rendering in the Viam app). **The kinematic model's link lengths are
CAD-corrected but its meshes and mass/inertia are still known-inaccurate
placeholders**, pending the correct source files for this exact arm variant
(FR5-WML). `Stop` remains unimplemented.

## Cartesian motion (`MoveToPosition`) requires a `"frame"` config block

`MoveToPosition` delegates to a `motion.Service` (the robot's default
`"builtin"` one, unless the `motion` config attribute names another) rather
than planning IK inside this module — see that method's doc comment in
`module.go` for why (short version: keeps the `libnlopt`/cgo dependency that
motion planning needs out of this module's own binary; it lives in
viam-server's own `builtin` motion service instead, which Viam ships
pre-built with that already resolved).

**This means the arm's component entry in the robot's JSON config must
include a `"frame"` block**, at minimum a `parent`:

```json
{
  "name": "fairino",
  "api": "rdk:component:arm",
  "model": "viam-soleng:viam-fairino:fairino",
  "attributes": { "host": "192.168.58.2" },
  "frame": { "parent": "world" }
}
```

Without it, this component is never added to the robot's frame system, and
`MoveToPosition` fails with `component named <name> not found in robot frame
system`. This is a robot-config change, not something the module supplies on
its own — `MoveToJointPositions`/`MoveThroughJointPositions`/`GoToInputs`
don't need it, only `MoveToPosition`.

## Models

This module provides the following model(s):

- [`viam-soleng:viam-fairino:fairino`](3a94d2f9082b44ad8569a9d8a1330050_viam-fairino_fairino.md) - Fairino FR5-WML arm

## Configuration

```json
{
  "host": "192.168.58.2"
}
```

| Attribute | Type | Required | Description |
|---|---|---|---|
| `host` | string | no | Controller IP address. Defaults to `192.168.58.2` (the Fairino factory default). |
| `tool_num` / `workpiece_num` | int | no | Active tool/workpiece coordinate frame, 0-14 each. Default 0 (no tool, base workpiece). |
| `joint_speed_percent` | int | no | `MoveJ`'s speed field, 1-100. Default 20 (matches `fairino-python-sdk`'s own `MoveJ` default). |
| `use_urdf` | bool | no | Use the on-disk URDF (`arm/fr5wml.urdf`) as the kinematics source instead of the default embedded JSON. Requires `VIAM_MODULE_ROOT` to be set (true for a normal deployment). JSON is the default because it's the only one the Viam app's joint-limit sliders currently parse correctly. |
| `mesh_decimation_ratios` | []float | no | Only used when `use_urdf` is true; per-collision-mesh decimation ratios (document order). |
| `motion` | string | no | Name of the `motion.Service` `MoveToPosition` delegates to. Defaults to this robot's `"builtin"` motion service if available (optional dependency); naming one explicitly makes it a hard dependency instead. See the Cartesian motion section above. |

The controller's ports are fixed by firmware (status 8083, commands 8080) and
are not configurable.

## Supported methods

| Method | Status |
|---|---|
| `JointPositions` / `CurrentInputs` | ✅ live from the 8083 status stream |
| `IsMoving` | ✅ from `robot_motion_done` (may not register pendant/drag motion yet) |
| `Status` | ✅ connection health + decoded controller state |
| `DoCommand` | ✅ `{"command": "status"}` (see below) |
| `MoveToJointPositions` | ✅ single blocking `MoveJ` |
| `MoveThroughJointPositions` | ✅ loops `MoveToJointPositions`' underlying mechanics per waypoint |
| `MoveToPosition` | ✅ via a `motion.Service` (see the "frame" config requirement above) — IK-solved, self-collision-checked joint trajectory, never a direct Cartesian command to the controller |
| `GoToInputs` | ✅ delegates to `MoveThroughJointPositions` (this is what the motion service's plan executor actually calls) |
| `Kinematics` / `Geometries` / `Get3DModels` | ✅ real kinematic model (see the mesh/mass-inertia accuracy caveat above) |
| `EndPosition` | ✅ via forward kinematics — see its doc comment for a still-open design question (device-reported pose vs. FK) |
| `Stop` | ❌ not implemented |

If the status connection drops, the implemented methods return the connection
error rather than stale data, and `Status` reports the resource unhealthy. The
reader reconnects automatically with backoff.

## DoCommand

`{"command": "status"}` returns the full decoded status frame — program
state/name/lines, robot mode, fault code (human-readable), joint
positions/torques, device-reported TCP pose, digital IO, force/torque sensor
data, e-stop, motion-done, and frame diagnostics (`payload_len`,
`frame_count`). Useful for bring-up and debugging.

## Notes

- The controller broadcasts status every 100 ms by default; the period is
  configurable to 8–100 ms on the teach pendant under System → Maintenance
  ("Sampling period of robot state", port 8083).
- A bring-up harness lives at `cmd/cli`: `go run ./cmd/cli [host]` prints the
  decoded status once per second without needing viam-server.

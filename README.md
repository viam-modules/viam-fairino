# Module viam-fairino

Viam `arm` component module for Fairino collaborative robots. Two robot
models are supported: the standard **FR5** and the long-reach **FR5-WML**.

The module connects to the controller's status port (TCP 8083, fixed) for
feedback and the command port (TCP 8080, fixed) for motion, and has a real
kinematic model (`Kinematics`/`Geometries`/`Get3DModels`, including 3D-scene
rendering in the Viam app). **The kinematic model's link lengths are
CAD-corrected but its meshes and mass/inertia are still known-inaccurate
placeholders**, pending the correct source files for this exact arm variant
(FR5-WML).

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

## Waypoint blending (`blend_time_ms`)

By default, a multi-waypoint move (`MoveThroughJointPositions`, and
therefore `GoToInputs`/`MoveToPosition` too, since both delegate to it)
decelerates to a full stop at every waypoint before starting the next —
this can look jerky. Setting `blend_time_ms` (1-500) makes every waypoint
but the last blend into the next instead of stopping, using the
controller's own motion queue. The last waypoint always stops, so the overall call
still only returns once the arm is physically at rest at its final target.

**This applies uniformly, including to IK-planned trajectories.** There is
no separate code path for `GoToInputs`/`MoveToPosition` — if `blend_time_ms`
is set, it blends through those waypoints too. For a motion-service plan
that was computed to be collision-aware, blending can "cut the corner" at
an intermediate waypoint instead of passing through it exactly, which may
not preserve the clearances the plan was computed for. If that matters for
your workcell, leave `blend_time_ms` at 0 for motion-service-driven moves,
or otherwise account for the corner-cutting in how the plan's waypoints are
spaced.

## Models

This module provides the following model(s):

- `viam-soleng:viam-fairino:fairino` - Fairino arm (FR5 or FR5-WML, auto-selected or selected by the `model` attribute)
- `viam-soleng:viam-fairino:gripper` - Fairino-configured gripper (generic, percentage-based)

## Configuration

```json
{
  "host": "192.168.58.2",
  "model": "fr5-wml"
}
```

| Attribute | Type | Required | Description |
|---|---|---|---|
| `host` | string | no | Controller IP address. Defaults to `192.168.58.2` (the Fairino factory default). |
| `model` | string | no | Which robot's kinematics/meshes to load: `"fr5"` (standard FR5) or `"fr5-wml"` (long-reach FR5-WML). Defaults to auto-detection via the controller's `GetSoftwareVersion`, which requires the robot to be reachable at startup. |
| `tool_num` / `workpiece_num` | int | no | Active tool/workpiece coordinate frame, 0-14 each. Default 0 (no tool, base workpiece). |
| `joint_speed_percent` | int | no | `MoveJ`'s speed field, 1-100. Default 20 (matches `fairino-python-sdk`'s own `MoveJ` default). |
| `use_urdf` | bool | no | Use the selected model's on-disk URDF (`arm/fr5.urdf` or `arm/fr5wml.urdf`) as the kinematics source instead of the default embedded JSON. Requires `VIAM_MODULE_ROOT` to be set (true for a normal deployment). JSON is the default because it's the only one the Viam app's joint-limit sliders currently parse correctly. |
| `mesh_decimation_ratios` | []float | no | Only used when `use_urdf` is true; per-collision-mesh decimation ratios (document order). |
| `motion` | string | no | Name of the `motion.Service` `MoveToPosition` delegates to. Defaults to this robot's `"builtin"` motion service if available (optional dependency); naming one explicitly makes it a hard dependency instead. See the Cartesian motion section above. |
| `blend_time_ms` | int | no | `MoveJ`'s non-blocking `blendT` field, 0-500. Default 0 (disabled): every waypoint decelerates to a full stop before the next, the original behavior. A nonzero value blends through every waypoint but the last in a multi-waypoint move (`MoveThroughJointPositions`, and therefore `GoToInputs`/`MoveToPosition` too, since both delegate to it) instead of stopping at each — see the blending caveat below. |

The controller's ports are fixed by firmware (status 8083, commands 8080) and
are not configurable. Both the arm and the gripper speak only these two
documented ports.

### Gripper configuration

```json
{
  "host": "192.168.58.2"
}
```

| Attribute | Type | Required | Description |
|---|---|---|---|
| `host` | string | no | Controller IP. Defaults to `192.168.58.2`. |
| `configure` | bool | no | Push `SetGripperConfig` at startup from the `company`/`device`/`software_version`/`bus` fields. Default false — the gripper is normally configured once on the controller. |
| `company` / `device` / `software_version` / `bus` | int | no | Gripper identity codes, used only when `configure` is true. Read the current values with `DoCommand {"command": "gripper_config"}`. |
| `activate` | bool | no | Call `ActGripper` at startup. Default true. |
| `activate_wait_ms` | int | no | Wait after activation for its homing motion to finish before a move is accepted. Default 3000. |
| `speed_percent` / `force_percent` | int | no | `MoveGripper` speed/torque, 0-100. Default 50 each. |
| `max_time_ms` | int | no | `MoveGripper` timeout. Default 30000. |
| `open_position` / `grab_position` | int | no | Positions used by `Open()`/`Grab()`, 0-100. Defaults 100 and 0. |
| `stroke_mm` | float | no | Total finger travel, used to size the kinematic model. Default 145 (AG-105-145). |

### Gripper bring-up on the controller

The module cannot enable a gripper by itself — `SetGripperConfig` only records
the vendor selection, it does not bring up the end-tool bus. Before any gripper
command works, once per controller:

1. WebApp → Peripherals → Gripper: select the vendor (for a DH Robotics gripper
   choose **`dahuan`**; the dropdown order is robotiq, hitbot, tianji, dahuan,
   zhixing, jodell, mt, rm).
2. Enable the end-tool RS-485 peripheral protocol and **Activate**. A healthy DH
   gripper performs a visible full open/close self-calibration when it
   activates.
3. If it reports "please reset the activated gripper" and software resets
   (`ActGripper 0/1`, i.e. `DoCommand {"command": "activate"}`) don't clear it,
   **power-cycle the gripper** — that latched fault needs a physical cycle.

Until this is done, moves fail with controller errcode 73 ("gripper movement
error"). The module retries once with a reset+activate before surfacing it.

Diagnose with the bring-up CLI, which needs no running viam-server:

```
go run ./cmd/fairino-test -host <ip> gripper diag
go run ./cmd/fairino-test -host <ip> gripper activate
go run ./cmd/fairino-test -host <ip> gripper open|close|move <0-100>
```

## Supported methods

| Method | Status |
|---|---|
| `JointPositions` / `CurrentInputs` | ✅ live from the 8083 status stream |
| `IsMoving` | ✅ from `robot_motion_done` (may not register pendant/drag motion yet) |
| `Status` | ✅ connection health + decoded controller state |
| `DoCommand` | ✅ `{"command": "status"}`, `"enter_drag_mode"`, `"exit_drag_mode"` (see below) |
| `MoveToJointPositions` | ✅ single blocking `MoveJ` |
| `MoveThroughJointPositions` | ✅ loops `MoveToJointPositions`' underlying mechanics per waypoint; blends through intermediate waypoints when `blend_time_ms` is set (see below) |
| `MoveToPosition` | ✅ via a `motion.Service` (see the "frame" config requirement above) — IK-solved, self-collision-checked joint trajectory, never a direct Cartesian command to the controller |
| `GoToInputs` | ✅ delegates to `MoveThroughJointPositions` (this is what the motion service's plan executor actually calls) |
| `Kinematics` / `Geometries` / `Get3DModels` | ✅ real kinematic model (see the mesh/mass-inertia accuracy caveat above) |
| `EndPosition` | ✅ via forward kinematics — see its doc comment for a still-open design question (device-reported pose vs. FK) |
| `Stop` | ✅ cancels any in-flight move and sends `STOP` |

If the status connection drops, the implemented methods return the connection
error rather than stale data, and `Status` reports the resource unhealthy. The
reader reconnects automatically with backoff.

## DoCommand

`{"command": "status"}` returns the full decoded status frame — program
state/name/lines, robot mode, fault code (human-readable), joint
positions/torques, device-reported TCP pose, digital IO, force/torque sensor
data, e-stop, motion-done, and frame diagnostics (`payload_len`,
`frame_count`). Useful for bring-up and debugging.

`{"command": "enter_drag_mode"}` / `{"command": "exit_drag_mode"}` toggle the
controller's drag-teach (free-drive) mode. This uses an undocumented command

## Gripper methods

| Method | Status |
|---|---|
| `Open` / `Grab` | ✅ blocking `MoveGripper` to the configured open/grab position |
| `IsHoldingSomething` | ✅ from the status feed's `gripper_motion_done` (DH encoding: 2 = clamped, 3 = clamped then dropped) |
| `CurrentInputs` / `Kinematics` / `Geometries` | ✅ fingers render at the live gripper position |
| `GoToInputs` | ✅ maps finger travel back to a percentage |
| `IsMoving` | ✅ true while an `Open`/`Grab` is in flight |
| `Stop` | ⚠️ cancels the in-flight call; the controller has no gripper-stop command |
| `DoCommand` | ✅ `gripper_config`, `status`, `activate`, `deactivate`, `set_config`, `move`, `diag` |

Gripper *commands* go to port 8080 (manual chapter 7.1: `SetGripperConfig`,
`GetGripperConfig`, `ActGripper`, `MoveGripper`); gripper *state* comes from the
8083 status feed (Table 2-2 rows 41 and 56-63), which is where the controller
publishes position, activation, fault, and motion-done. There is deliberately no
gripper readback over the command port — it doesn't document any.

## Notes

- The controller broadcasts status every 100 ms by default; the period is
  configurable to 8–100 ms on the teach pendant under System → Maintenance
  ("Sampling period of robot state", port 8083).
- A bring-up harness lives at `cmd/cli`: `go run ./cmd/cli [host]` prints the
  decoded status once per second without needing viam-server.

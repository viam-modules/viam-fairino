# Module viam-fairino

Viam `arm` component module for the Fairino FR5-WML collaborative robot.

**Current phase: status feedback only.** The module connects to the
controller's status port (TCP 8083, fixed) and exposes the read-only parts of
the arm API. Motion (via the command port, TCP 8080), kinematics, and geometry
are planned — see [PLAN.md](PLAN.md) for the phase breakdown.

## Models

This module provides the following model(s):

- [`3a94d2f9082b44ad8569a9d8a1330050:viam-fairino:fairino`](3a94d2f9082b44ad8569a9d8a1330050_viam-fairino_fairino.md) - Fairino FR5-WML arm (status feedback only)

## Configuration

```json
{
  "host": "192.168.58.2"
}
```

| Attribute | Type   | Required | Description |
|-----------|--------|----------|-------------|
| `host`    | string | no       | Controller IP address. Defaults to `192.168.58.2` (the Fairino factory default). |

The controller's ports are fixed by firmware (status 8083, commands 8080) and
are not configurable.

## Supported methods

| Method | Status |
|---|---|
| `JointPositions` / `CurrentInputs` | ✅ live from the 8083 status stream |
| `IsMoving` | ✅ from `robot_motion_done` (may not register pendant/drag motion yet) |
| `Status` | ✅ connection health + decoded controller state |
| `DoCommand` | ✅ `{"command": "status"}` (see below) |
| `EndPosition` | ❌ phase 3 (kinematics-model FK) |
| `MoveToPosition` / `MoveToJointPositions` / `MoveThroughJointPositions` / `GoToInputs` / `Stop` | ❌ phase 2 (command port) |
| `Kinematics` / `Geometries` / `Get3DModels` | ❌ phase 3 |

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

// fairino-test is a hardware bring-up CLI for driving a real Fairino controller
// directly, without a running viam-server. It exercises the same code paths as
// the module (arm MoveJ, gripper XML-RPC) so behavior seen here matches the
// deployed component.
//
// Usage:
//
//	go run ./cmd/fairino-test [-host IP] <command> [args]
//
// Commands:
//
//	status                       decoded arm status
//	joints                       current joint positions (degrees)
//	jog <joint 1-6> <deltaDeg>   move one joint by delta degrees (blocking MoveJ)
//
//	gripper status               config + current position + motion state
//	gripper config               configured company/device/software/bus
//	gripper activate             reset + activate + wait
//	gripper open                 open (MoveGripper to open position)
//	gripper close                close (MoveGripper to grab position)
//	gripper move <pos 0-100>     move to an explicit position
//
// Example:
//
//	go run ./cmd/fairino-test -host 10.1.7.1 gripper open
//	go run ./cmd/fairino-test -host 10.1.7.1 jog 6 15
package main

import (
	"context"
	"flag"
	"fmt"
	"os"
	"strconv"
	"time"

	"viamfairino"

	arm "go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/components/gripper"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/resource"
	rdkutils "go.viam.com/rdk/utils"
)

func main() {
	host := flag.String("host", "192.168.58.2", "controller IP")
	flag.Parse()
	args := flag.Args()
	if len(args) == 0 {
		flag.Usage()
		fmt.Fprintln(os.Stderr, "\nsee the file header for the command list")
		os.Exit(2)
	}
	if err := run(*host, args); err != nil {
		fmt.Fprintln(os.Stderr, "ERROR:", err)
		os.Exit(1)
	}
}

func run(host string, args []string) error {
	ctx, cancel := context.WithTimeout(context.Background(), 90*time.Second)
	defer cancel()
	logger := logging.NewLogger("fairino-test")

	switch args[0] {
	case "status", "joints", "jog":
		return runArm(ctx, host, logger, args)
	case "gripper":
		return runGripper(ctx, host, logger, args[1:])
	default:
		return fmt.Errorf("unknown command %q", args[0])
	}
}

func runArm(ctx context.Context, host string, logger logging.Logger, args []string) error {
	a, err := viamfairino.NewFairino(ctx, resource.Dependencies{}, arm.Named("fairino"),
		&viamfairino.Config{Host: host}, logger)
	if err != nil {
		return err
	}
	defer a.Close(ctx)

	switch args[0] {
	case "status":
		st, err := a.DoCommand(ctx, map[string]any{"command": "status"})
		if err != nil {
			return err
		}
		for _, k := range []string{"program_state", "robot_mode", "fault", "emergency_stop",
			"robot_motion_done", "jt_cur_pos_deg", "tl_cur_pos"} {
			fmt.Printf("%-20s %v\n", k, st[k])
		}
		return nil

	case "joints":
		return printJoints(ctx, a)

	case "jog":
		if len(args) != 3 {
			return fmt.Errorf("usage: jog <joint 1-6> <deltaDeg>")
		}
		joint, err := strconv.Atoi(args[1])
		if err != nil || joint < 1 || joint > 6 {
			return fmt.Errorf("joint must be 1-6, got %q", args[1])
		}
		delta, err := strconv.ParseFloat(args[2], 64)
		if err != nil {
			return fmt.Errorf("bad deltaDeg %q: %w", args[2], err)
		}
		cur, err := a.JointPositions(ctx, nil)
		if err != nil {
			return err
		}
		fmt.Printf("before: %s\n", jointsDeg(cur))
		target := make([]referenceframe.Input, len(cur))
		copy(target, cur)
		target[joint-1] = cur[joint-1] + rdkutils.DegToRad(delta)
		fmt.Printf("moving joint %d by %+.1f deg -> %s\n", joint, delta, jointsDeg(target))
		if err := a.MoveToJointPositions(ctx, target, nil); err != nil {
			return err
		}
		return printJoints(ctx, a)
	}
	return fmt.Errorf("unknown arm command %q", args[0])
}

func printJoints(ctx context.Context, a arm.Arm) error {
	j, err := a.JointPositions(ctx, nil)
	if err != nil {
		return err
	}
	fmt.Printf("joints: %s\n", jointsDeg(j))
	return nil
}

func jointsDeg(in []referenceframe.Input) string {
	s := "["
	for i, v := range in {
		if i > 0 {
			s += " "
		}
		s += fmt.Sprintf("%.2f", rdkutils.RadToDeg(v))
	}
	return s + "] deg"
}

func runGripper(ctx context.Context, host string, logger logging.Logger, args []string) error {
	if len(args) == 0 {
		return fmt.Errorf("usage: gripper <status|config|activate|open|close|move> [args]")
	}
	// Skip auto-activate on construct; each command controls activation itself.
	no := false
	g, err := viamfairino.NewFairinoGripper(ctx, gripper.Named("gripper"),
		&viamfairino.GripperConfig{Host: host, Activate: &no}, logger)
	if err != nil {
		return err
	}
	defer g.Close(ctx)

	switch args[0] {
	case "status":
		st, err := g.DoCommand(ctx, map[string]any{"command": "status"})
		if err != nil {
			return err
		}
		fmt.Printf("%v\n", st)
		return nil
	case "config":
		st, err := g.DoCommand(ctx, map[string]any{"command": "gripper_config"})
		if err != nil {
			return err
		}
		fmt.Printf("%v\n", st)
		return nil
	case "diag":
		st, err := g.DoCommand(ctx, map[string]any{"command": "diag"})
		if err != nil {
			return err
		}
		for k, v := range st {
			fmt.Printf("%-24s %v\n", k, v)
		}
		return nil
	case "activate":
		st, err := g.DoCommand(ctx, map[string]any{"command": "activate"})
		if err != nil {
			return err
		}
		fmt.Printf("activated: %v\n", st)
		return nil
	case "deactivate":
		if _, err := g.DoCommand(ctx, map[string]any{"command": "deactivate"}); err != nil {
			return err
		}
		fmt.Println("deactivated (ActGripper 0 / reset)")
		return nil
	case "reset":
		if _, err := g.DoCommand(ctx, map[string]any{"command": "reset_error"}); err != nil {
			return err
		}
		fmt.Println("ResetAllError ok; activating...")
		st, err := g.DoCommand(ctx, map[string]any{"command": "activate"})
		if err != nil {
			return err
		}
		fmt.Printf("activated: %v\n", st)
		return nil
	case "open":
		fmt.Println("opening...")
		if err := g.Open(ctx, nil); err != nil {
			return err
		}
		fmt.Println("open done")
		return nil
	case "close":
		fmt.Println("closing...")
		held, err := g.Grab(ctx, nil)
		if err != nil {
			return err
		}
		fmt.Printf("close done, holding=%v\n", held)
		return nil
	case "setconfig":
		if len(args) != 3 {
			return fmt.Errorf("usage: gripper setconfig <company> <device>")
		}
		company, err := strconv.Atoi(args[1])
		if err != nil {
			return fmt.Errorf("bad company %q: %w", args[1], err)
		}
		device, err := strconv.Atoi(args[2])
		if err != nil {
			return fmt.Errorf("bad device %q: %w", args[2], err)
		}
		st, err := g.DoCommand(ctx, map[string]any{
			"command": "set_config", "company": company, "device": device,
		})
		if err != nil {
			return err
		}
		fmt.Printf("set_config ok: %v\n", st)
		return nil
	case "move":
		if len(args) != 2 {
			return fmt.Errorf("usage: gripper move <pos 0-100>")
		}
		pos, err := strconv.Atoi(args[1])
		if err != nil {
			return fmt.Errorf("bad pos %q: %w", args[1], err)
		}
		st, err := g.DoCommand(ctx, map[string]any{"command": "move", "position": pos})
		if err != nil {
			return err
		}
		fmt.Printf("%v\n", st)
		return nil
	}
	return fmt.Errorf("unknown gripper command %q", args[0])
}

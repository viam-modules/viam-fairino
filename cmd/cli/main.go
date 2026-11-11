// Bring-up harness: connects to the controller's status port and prints the
// decoded status once per second, mirroring fairino_test/test.py. Usage:
//
//	go run ./cmd/cli [host]
//
// host defaults to the Fairino factory address (192.168.58.2).
package main

import (
	"context"
	"fmt"
	"os"
	"os/signal"
	"time"

	"viamfairino"

	arm "go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/resource"
)

func main() {
	if err := realMain(); err != nil {
		panic(err)
	}
}

func realMain() error {
	ctx, stop := signal.NotifyContext(context.Background(), os.Interrupt)
	defer stop()
	logger := logging.NewLogger("cli")

	deps := resource.Dependencies{}
	cfg := viamfairino.Config{}
	if len(os.Args) > 1 {
		cfg.Host = os.Args[1]
	}

	thing, err := viamfairino.NewFairino(ctx, deps, arm.Named("fairino"), &cfg, logger)
	if err != nil {
		return err
	}
	defer func() { _ = thing.Close(ctx) }()

	for {
		select {
		case <-ctx.Done():
			return nil
		case <-time.After(time.Second):
		}

		status, err := thing.DoCommand(ctx, map[string]interface{}{"command": "status"})
		if err != nil {
			logger.Errorw("status unavailable", "err", err)
			continue
		}
		fmt.Println("--- Robot Status ---")
		for _, k := range []string{
			"program_state", "robot_mode", "fault", "emergency_stop",
			"robot_motion_done", "jt_cur_pos_deg", "tl_cur_pos",
			"program_name", "payload_len", "frame_count",
		} {
			fmt.Printf("%s: %v\n", k, status[k])
		}
	}
}

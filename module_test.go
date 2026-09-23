package viamfairino

import (
	"context"
	"errors"
	"testing"
	"time"

	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/services/motion"
	"go.viam.com/rdk/spatialmath"
)

func TestValidateBlendTimeMs(t *testing.T) {
	base := func(blendTimeMs int) *Config {
		return &Config{BlendTimeMs: blendTimeMs}
	}

	for _, v := range []int{0, 1, 200, 500} {
		if _, _, err := base(v).Validate("path"); err != nil {
			t.Errorf("blend_time_ms=%d: expected no error, got %v", v, err)
		}
	}

	for _, v := range []int{-1, 501, 1000} {
		if _, _, err := base(v).Validate("path"); err == nil {
			t.Errorf("blend_time_ms=%d: expected an error, got nil", v)
		}
	}
}

func TestMoveToJointPositionsOnceRejectsZeroBlendT(t *testing.T) {
	s := &viamFairinoFairino{}
	positions := make([]referenceframe.Input, 6)
	if _, err := s.moveToJointPositionsOnce(context.Background(), positions, 0, defaultJointSpeedPercent, 0); err == nil {
		t.Error("blendT=0: expected an error, got nil")
	}
}

func TestProperties(t *testing.T) {
	props, err := (&viamFairinoFairino{}).Properties(context.Background(), nil)
	if err != nil {
		t.Fatal(err)
	}
	if props.SupportCartesianCommands {
		t.Error("SupportCartesianCommands reported with no motion service: MoveToPosition would always fail")
	}
	if props.SupportManualMode {
		t.Error("SupportManualMode must stay false until SetManualMode is implemented")
	}
	props, err = (&viamFairinoFairino{motion: &planningMotion{}}).Properties(context.Background(), nil)
	if err != nil || !props.SupportCartesianCommands {
		t.Errorf("got %+v, %v; want Cartesian commands with a motion service", props, err)
	}
}

// planningMotion is a motion service whose Move plans until it is cancelled.
type planningMotion struct {
	motion.Service
	started chan struct{}
}

func (m *planningMotion) Move(ctx context.Context, _ motion.MoveReq) (bool, error) {
	close(m.started)
	<-ctx.Done()
	return false, ctx.Err()
}

// A MoveToPosition still planning holds no operation, yet Stop and Close must
// still cancel it: a plan finishing after a reconfigure would otherwise run
// on the rebuilt arm.
func TestStopAndCloseCancelPlanning(t *testing.T) {
	for name, halt := range map[string]func(*viamFairinoFairino) error{
		"Stop":  func(s *viamFairinoFairino) error { return s.Stop(context.Background(), nil) },
		"Close": func(s *viamFairinoFairino) error { return s.Close(context.Background()) },
	} {
		t.Run(name, func(t *testing.T) {
			s, _ := newControllerTestArm(t, newFakeController(t), restingFrame([6]float64{}))
			m := &planningMotion{started: make(chan struct{})}
			s.motion = m
			errCh := make(chan error, 1)
			go func() { errCh <- s.MoveToPosition(context.Background(), spatialmath.NewZeroPose(), nil) }()
			<-m.started
			if err := halt(s); err != nil {
				t.Fatal(err)
			}
			select {
			case err := <-errCh:
				if !errors.Is(err, context.Canceled) {
					t.Errorf("got %v, want context.Canceled", err)
				}
			case <-time.After(5 * time.Second):
				t.Fatal("planning wasn't cancelled")
			}
		})
	}
}

func TestValidateCollisionLevels(t *testing.T) {
	for _, cfg := range []Config{
		{CollisionLevel: 11},
		{CollisionLevel: -1},
		{CollisionLevels: []int{1, 2, 3}},
		{CollisionLevels: []int{1, 2, 3, 4, 5, 0}},
		{CollisionLevel: 3, CollisionLevels: []int{1, 1, 1, 1, 1, 1}},
	} {
		if _, _, err := cfg.Validate("path"); err == nil {
			t.Errorf("%+v: expected an error", cfg)
		}
	}
	for _, cfg := range []Config{{CollisionLevel: 1}, {CollisionLevels: []int{1, 2, 3, 4, 5, 10}}} {
		if _, _, err := cfg.Validate("path"); err != nil {
			t.Errorf("%+v: %v", cfg, err)
		}
	}
}

func TestCollisionLevelsResolve(t *testing.T) {
	if _, ok := (&Config{}).collisionLevels(); ok {
		t.Error("unset config should leave the controller's levels alone")
	}
	if got, _ := (&Config{CollisionLevel: 4}).collisionLevels(); got != [6]int{4, 4, 4, 4, 4, 4} {
		t.Errorf("collision_level: got %v", got)
	}
	if got, _ := (&Config{CollisionLevels: []int{1, 2, 3, 4, 5, 6}}).collisionLevels(); got != [6]int{1, 2, 3, 4, 5, 6} {
		t.Errorf("collision_levels: got %v", got)
	}
}

// The wire format must match the manual's example shape (Table 5-4-4).
func TestSetAnticollisionFormat(t *testing.T) {
	fc := newFakeController(t)
	s, _ := newControllerTestArm(t, fc, restingFrame([6]float64{}))
	if err := s.cmd.setAnticollision(context.Background(), [6]int{5, 5, 5, 5, 5, 3}); err != nil {
		t.Fatal(err)
	}
	fc.mu.Lock()
	defer fc.mu.Unlock()
	if want := "SetAnticollision(0,{5,5,5,5,5,3},0)"; len(fc.commands) != 1 || fc.commands[0] != want {
		t.Errorf("got %v want [%s]", fc.commands, want)
	}
}

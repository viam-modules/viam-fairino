package viamfairino

import (
	"context"
	"math"
	"strconv"
	"strings"
	"testing"

	arm "go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/referenceframe"
	rdkutils "go.viam.com/rdk/utils"
)

func TestMoveSpeedPercent(t *testing.T) {
	s, _ := newControllerTestArm(t, newFakeController(t), restingFrame([6]float64{}))
	rad := func(deg float64) float64 { return rdkutils.DegToRad(deg) }
	for name, tc := range map[string]struct {
		cfg   Config
		opts  *arm.MoveOptions
		extra map[string]interface{}
		want  float64
	}{
		"default":                {want: defaultJointSpeedPercent},
		"joint_speed_percent":    {cfg: Config{JointSpeedPercent: 40}, want: 40},
		"speed_degs_per_sec":     {cfg: Config{SpeedDegsPerSec: 90}, want: 50},
		"MaxVelRads":             {opts: &arm.MoveOptions{MaxVelRads: rad(36)}, want: 20},
		"slowest per-joint":      {opts: &arm.MoveOptions{MaxVelRadsJoints: []float64{rad(90), 0, rad(18)}}, want: 10},
		"extra speed_d wins":     {opts: &arm.MoveOptions{MaxVelRads: rad(36)}, extra: map[string]interface{}{"speed_d": 144.0}, want: 80},
		"extra speed_r":          {extra: map[string]interface{}{"speed_r": rad(45)}, want: 25},
		"clamped to the maximum": {extra: map[string]interface{}{"speed_d": 1000.0}, want: maxSpeedPercent},
		"exactly the minimum":    {extra: map[string]interface{}{"speed_d": minSpeedDegsPerSec}, want: minSpeedPercent},
		"minimum in rad/s":       {extra: map[string]interface{}{"speed_r": rad(minSpeedDegsPerSec)}, want: minSpeedPercent},
	} {
		t.Run(name, func(t *testing.T) {
			cfg := tc.cfg
			s.cfg = &cfg
			got, err := s.moveSpeedPercent(tc.opts, tc.extra)
			if err != nil || math.Abs(got-tc.want) > 1e-9 {
				t.Errorf("got %v, %v want %v", got, err, tc.want)
			}
		})
	}
}

// A move can't go slower than MoveJ's 1%, and must not be quietly sped up to
// it either.
func TestMoveSpeedBelowMinimumRejected(t *testing.T) {
	s, _ := newControllerTestArm(t, newFakeController(t), restingFrame([6]float64{}))
	for name, tc := range map[string]struct {
		opts  *arm.MoveOptions
		extra map[string]interface{}
	}{
		"speed_d tiny":     {extra: map[string]interface{}{"speed_d": 0.5}},
		"speed_d zero":     {extra: map[string]interface{}{"speed_d": 0.0}},
		"speed_d negative": {extra: map[string]interface{}{"speed_d": -10.0}},
		"MaxVelRads tiny":  {opts: &arm.MoveOptions{MaxVelRads: rdkutils.DegToRad(1)}},
		"per-joint tiny":   {opts: &arm.MoveOptions{MaxVelRadsJoints: []float64{1, rdkutils.DegToRad(0.1)}}},
	} {
		if pct, err := s.moveSpeedPercent(tc.opts, tc.extra); err == nil {
			t.Errorf("%s: got %v%%, want an error", name, pct)
		}
	}
}

// NaN compares false against both clamp bounds, so it has to be caught before
// it reaches the controller as the text "NaN".
func TestMoveSpeedRejectsNonFinite(t *testing.T) {
	s, _ := newControllerTestArm(t, newFakeController(t), restingFrame([6]float64{}))
	for name, tc := range map[string]struct {
		opts  *arm.MoveOptions
		extra map[string]interface{}
	}{
		"speed_d NaN":    {extra: map[string]interface{}{"speed_d": math.NaN()}},
		"speed_r +Inf":   {extra: map[string]interface{}{"speed_r": math.Inf(1)}},
		"MaxVelRads NaN": {opts: &arm.MoveOptions{MaxVelRads: math.NaN()}},
		"per-joint +Inf": {opts: &arm.MoveOptions{MaxVelRadsJoints: []float64{1, math.Inf(1)}}},
	} {
		if _, err := s.moveSpeedPercent(tc.opts, tc.extra); err == nil {
			t.Errorf("%s: expected an error", name)
		}
	}
	if err := s.MoveToJointPositions(context.Background(), make([]referenceframe.Input, 6),
		map[string]interface{}{"speed_d": math.NaN()}); err == nil {
		t.Error("MoveToJointPositions accepted a NaN speed")
	}
}

func TestSetAndGetSpeedDoCommand(t *testing.T) {
	s, _ := newControllerTestArm(t, newFakeController(t), restingFrame([6]float64{}))
	if _, err := s.DoCommand(context.Background(), map[string]interface{}{"command": "set_speed", "speed_degs_per_sec": 45.0}); err != nil {
		t.Fatal(err)
	}
	resp, err := s.DoCommand(context.Background(), map[string]interface{}{"command": "get_speed"})
	if err != nil {
		t.Fatal(err)
	}
	if got := resp["speed_degs_per_sec"]; got != 45.0 {
		t.Errorf("get_speed: got %v want 45", got)
	}
	for _, bad := range []interface{}{0.0, -5.0, 1.0, 181.0, "fast", math.NaN(), math.Inf(1)} {
		if _, err := s.DoCommand(context.Background(), map[string]interface{}{"command": "set_speed", "speed_degs_per_sec": bad}); err == nil {
			t.Errorf("set_speed %v: expected an error", bad)
		}
	}
}

func TestValidateSpeed(t *testing.T) {
	for _, cfg := range []Config{{SpeedDegsPerSec: -1}, {SpeedDegsPerSec: 0.5}, {SpeedDegsPerSec: 181}, {SpeedDegsPerSec: 30, JointSpeedPercent: 20}} {
		if _, _, err := cfg.Validate("path"); err == nil {
			t.Errorf("%+v: expected an error", cfg)
		}
	}
	if _, _, err := (&Config{SpeedDegsPerSec: 180}).Validate("path"); err != nil {
		t.Error(err)
	}
}

// The resolved speed must actually reach MoveJ's speed field (Table 2-1, 15th
// argument).
func TestMoveJCarriesRequestedSpeed(t *testing.T) {
	fc := newFakeController(t)
	fc.setReply(fakeFKReply)
	target := testTarget()
	var deg [6]float64
	for i := range deg {
		deg[i] = rdkutils.RadToDeg(target[i])
	}
	s, _ := newControllerTestArm(t, fc, restingFrame(deg))

	if err := s.MoveToJointPositions(context.Background(), target, map[string]interface{}{"speed_d": 90.0}); err != nil {
		t.Fatal(err)
	}
	if got := moveJSpeed(t, fc); got != 50 {
		t.Errorf("MoveJ speed: got %v want 50", got)
	}
}

func moveJSpeed(t *testing.T, fc *fakeController) float64 {
	t.Helper()
	fc.mu.Lock()
	defer fc.mu.Unlock()
	for i := len(fc.commands) - 1; i >= 0; i-- {
		args, ok := strings.CutPrefix(fc.commands[i], "MoveJ(")
		if !ok {
			continue
		}
		fields := strings.Split(strings.TrimSuffix(args, ")"), ",")
		v, err := strconv.ParseFloat(fields[14], 64)
		if err != nil {
			t.Fatal(err)
		}
		return v
	}
	t.Fatal("no MoveJ sent")
	return 0
}

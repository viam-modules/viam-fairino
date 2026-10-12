package viamfairino

import (
	"context"
	"testing"

	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/referenceframe"
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

// A nil ToolNum (the "unset" sentinel, see Config.ToolNum's doc comment)
// must pass validation untouched — only an explicitly-set out-of-range value
// is an error.
func TestValidateToolNum(t *testing.T) {
	if _, _, err := (&Config{}).Validate("path"); err != nil {
		t.Errorf("unset tool_num: expected no error, got %v", err)
	}
	for _, v := range []int{0, 1, 14} {
		if _, _, err := (&Config{ToolNum: &v}).Validate("path"); err != nil {
			t.Errorf("tool_num=%d: expected no error, got %v", v, err)
		}
	}
	for _, v := range []int{-1, 15} {
		if _, _, err := (&Config{ToolNum: &v}).Validate("path"); err == nil {
			t.Errorf("tool_num=%d: expected an error, got nil", v)
		}
	}
}

// resolveToolNum must prefer an explicitly-set Config.ToolNum without
// touching the status feed at all (so it works even with no status client
// configured, as in this test), and otherwise fall back to the controller's
// live active-tool index from the status feed.
func TestResolveToolNum(t *testing.T) {
	explicit := 7
	s := &viamFairinoFairino{cfg: &Config{ToolNum: &explicit}}
	got, err := s.resolveToolNum(context.Background())
	if err != nil {
		t.Fatalf("explicit tool_num: unexpected error %v", err)
	}
	if got != explicit {
		t.Errorf("explicit tool_num: got %d want %d", got, explicit)
	}

	sc := newStatusClient("test", statusPort, logging.NewTestLogger(t))
	sc.frame = statusFrame{ToolNum: 3}
	sc.have = true
	s = &viamFairinoFairino{cfg: &Config{}, status: sc}
	got, err = s.resolveToolNum(context.Background())
	if err != nil {
		t.Fatalf("unset tool_num: unexpected error %v", err)
	}
	if got != 3 {
		t.Errorf("unset tool_num: got %d want 3 (from the live status frame)", got)
	}
}

func TestMoveToJointPositionsOnceRejectsZeroBlendT(t *testing.T) {
	s := &viamFairinoFairino{}
	positions := make([]referenceframe.Input, 6)
	if err := s.moveToJointPositionsOnce(context.Background(), positions, 0); err == nil {
		t.Error("blendT=0: expected an error, got nil")
	}
}

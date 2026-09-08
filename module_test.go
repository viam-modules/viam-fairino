package viamfairino

import (
	"context"
	"testing"

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

func TestMoveToJointPositionsOnceRejectsZeroBlendT(t *testing.T) {
	s := &viamFairinoFairino{}
	positions := make([]referenceframe.Input, 6)
	if err := s.moveToJointPositionsOnce(context.Background(), positions, 0); err == nil {
		t.Error("blendT=0: expected an error, got nil")
	}
}

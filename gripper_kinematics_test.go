package viamfairino

import (
	"math"
	"testing"
)

func TestMakeGripperModelDoF(t *testing.T) {
	m, err := makeGripperModel("g", gripperStrokeMM)
	if err != nil {
		t.Fatalf("makeGripperModel: %v", err)
	}
	if dof := len(m.DoF()); dof != 2 {
		t.Fatalf("expected 2 DoF (two fingers), got %d", dof)
	}
}

func TestPositionToInputs(t *testing.T) {
	half := gripperStrokeMM / 2
	// inputs are [d, 2d]; d = percent/100 * half.
	tests := []struct {
		percent float64
		wantD   float64
	}{
		{0, 0},
		{100, half},
		{50, half / 2},
		{-10, 0},    // clamp low
		{150, half}, // clamp high
	}
	for _, tt := range tests {
		in := positionToInputs(tt.percent, gripperStrokeMM)
		if len(in) != 2 {
			t.Fatalf("percent %v: expected 2 inputs, got %d", tt.percent, len(in))
		}
		if math.Abs(float64(in[0])-tt.wantD) > 1e-9 || math.Abs(float64(in[1])-2*tt.wantD) > 1e-9 {
			t.Errorf("positionToInputs(%v) = %v, want [%v %v]", tt.percent, in, tt.wantD, 2*tt.wantD)
		}
	}
}

func TestInputsToPercentRoundTrip(t *testing.T) {
	for _, p := range []float64{0, 25, 50, 75, 100} {
		got := inputsToPercent(positionToInputs(p, gripperStrokeMM), gripperStrokeMM)
		if math.Abs(got-p) > 1e-6 {
			t.Errorf("round trip %v -> %v", p, got)
		}
	}
}

// TestGripperFingersMove proves the fingers actually change pose between closed
// and open — i.e. the model reflects position, which is the whole point.
func TestGripperFingersMove(t *testing.T) {
	m, err := makeGripperModel("g", gripperStrokeMM)
	if err != nil {
		t.Fatal(err)
	}
	closed, err := m.Geometries(positionToInputs(0, gripperStrokeMM))
	if err != nil {
		t.Fatal(err)
	}
	open, err := m.Geometries(positionToInputs(100, gripperStrokeMM))
	if err != nil {
		t.Fatal(err)
	}
	cg, og := closed.Geometries(), open.Geometries()
	if len(cg) != len(og) || len(cg) == 0 {
		t.Fatalf("geometry count mismatch/empty: closed=%d open=%d", len(cg), len(og))
	}

	// Find a finger geometry and confirm its Y position differs by ~half stroke.
	moved := false
	for i := range cg {
		if !strings_HasSuffix(cg[i].Label(), "finger") {
			continue
		}
		dy := math.Abs(og[i].Pose().Point().Y - cg[i].Pose().Point().Y)
		if math.Abs(dy-gripperStrokeMM/2) < 1e-6 {
			moved = true
		}
	}
	if !moved {
		t.Error("no finger geometry moved by half-stroke between closed and open")
	}
}

func strings_HasSuffix(s, suffix string) bool {
	return len(s) >= len(suffix) && s[len(s)-len(suffix):] == suffix
}

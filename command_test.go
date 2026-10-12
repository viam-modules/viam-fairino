package viamfairino

import (
	"testing"
)

// formatMoveL must reproduce the field order/count of the manual's own
// worked example (Table 2-6), using that example's exact values. The
// expected string is re-derived at this module's own fixed %.4f precision
// rather than pasted verbatim from the manual — the manual's own worked
// examples use a variable number of decimals (e.g. "3.100", "100"), while
// every float field here is always formatted to 4 decimals — so a literal
// copy-paste would never match regardless of correctness.
func TestFormatMoveLMatchesManualValues(t *testing.T) {
	p := moveLParams{
		Joints:       [6]float64{3.580, -83.769, 132.494, -138.725, -90.000, 156.101},
		Pose:         [6]float64{-423.534, -185.807, 290.307, -180.000, -0.000, -62.521},
		ToolNum:      0,
		WorkPieceNum: 0,
		SpeedPercent: 100,
		AccelPercent: 100,
		OverridePct:  100,
		BlendR:       -1,
		BlendMode:    0,
		SearchFlag:   0,
		OffsetFlag:   0,
		OAcc:         0,
		VelAccMode:   0,
	}
	got := formatMoveL(p)
	want := "MoveL(3.5800,-83.7690,132.4940,-138.7250,-90.0000,156.1010," +
		"-423.5340,-185.8070,290.3070,-180.0000,0.0000,-62.5210," +
		"0,0," +
		"100.0000,100.0000,100," +
		"-1.0000,0," +
		"0.0000,0.0000,0.0000,0.0000," +
		"0,0," +
		"0.0000,0.0000,0.0000,0.0000,0.0000,0.0000," +
		"0.0000,0)"
	if got != want {
		t.Errorf("got  %q\nwant %q", got, want)
	}
}

// Same treatment for formatSetToolCoord against Table 5-3-1's example,
// reformatted at our own %.4f precision.
func TestFormatSetToolCoordMatchesManualValues(t *testing.T) {
	got := formatSetToolCoord(0, [6]float64{3.1, 0, 2.1, 0, 1.1, 0}, 0, 0, 0)
	want := "SetToolCoord(0,3.1000,0.0000,2.1000,0.0000,1.1000,0.0000,0,0,0,0)"
	if got != want {
		t.Errorf("got %q want %q", got, want)
	}
}

// GetToolCoordWithID's reply is the same bare "x,y,z,rx,ry,rz" shape
// GetForwardKin uses, so it reuses parsePose6 — confirm that against this
// command's own worked example (Table 6-2-7).
func TestParsePose6MatchesGetToolCoordWithIDExample(t *testing.T) {
	got, err := parsePose6("0.000000,0.000000,136.000000,0.000000,0.000000,135.000000")
	if err != nil {
		t.Fatalf("unexpected error: %v", err)
	}
	want := [6]float64{0, 0, 136, 0, 0, 135}
	if got != want {
		t.Errorf("got %v want %v", got, want)
	}
}

func TestAxisOffset(t *testing.T) {
	for _, tc := range []struct {
		axis                   string
		wantRx, wantRy, wantRz float64
	}{
		{"x", 30, 0, 0},
		{"X", 30, 0, 0},
		{"y", 0, 30, 0},
		{"Y", 0, 30, 0},
		{"z", 0, 0, 30},
		{"Z", 0, 0, 30},
	} {
		rx, ry, rz, err := axisOffset(tc.axis, 30)
		if err != nil {
			t.Errorf("axis=%q: unexpected error %v", tc.axis, err)
			continue
		}
		if rx != tc.wantRx || ry != tc.wantRy || rz != tc.wantRz {
			t.Errorf("axis=%q: got (%v,%v,%v) want (%v,%v,%v)", tc.axis, rx, ry, rz, tc.wantRx, tc.wantRy, tc.wantRz)
		}
	}

	for _, bad := range []string{"", "w", "xyz", "0"} {
		if _, _, _, err := axisOffset(bad, 30); err == nil {
			t.Errorf("axis=%q: expected an error, got nil", bad)
		}
	}
}

func TestFloatFromCmd(t *testing.T) {
	cmd := map[string]any{
		"degrees": 30.5,
		"zero":    0.0,
		"intval":  7,
		"str":     "not a number",
	}
	for _, tc := range []struct {
		key    string
		want   float64
		wantOk bool
	}{
		{"degrees", 30.5, true},
		{"zero", 0, true}, // 0 is a legitimate value, not treated as absent
		{"intval", 7, true},
		{"str", 0, false},
		{"missing", 0, false},
	} {
		got, ok := floatFromCmd(cmd, tc.key)
		if ok != tc.wantOk || (ok && got != tc.want) {
			t.Errorf("key=%q: got (%v,%v) want (%v,%v)", tc.key, got, ok, tc.want, tc.wantOk)
		}
	}
}

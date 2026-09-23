package viamfairino

import (
	"math"
	"testing"

	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/spatialmath"
)

// testModels is every model in modelRegistry — the JSON↔URDF agreement tests
// run against all of them.
var testModels = []string{ModelFR5, ModelFR5WML}

// buildBothModels returns (jsonModel, urdfModel) for the given model — the
// default embedded-JSON source and the on-disk URDF source, both derived from
// the same URDF (see PLAN.md Step 10).
func buildBothModels(t *testing.T, model string) (referenceframe.Model, referenceframe.Model) {
	t.Helper()
	t.Setenv("VIAM_MODULE_ROOT", ".")

	jsonModel, err := makeModelFrame("fairino", model, false, nil)
	if err != nil {
		t.Fatalf("makeModelFrame (json, %s): %v", model, err)
	}
	urdfModel, err := makeModelFrame("fairino", model, true, nil)
	if err != nil {
		t.Fatalf("makeModelFrame (urdf, %s): %v", model, err)
	}
	return jsonModel, urdfModel
}

// degToInputs converts a slice of joint angles in degrees to radians-based
// referenceframe.Input (a float64 alias), matching this module's own
// DegToRad convention at the arm.Arm boundary (module.go).
func degToInputs(deg []float64) []referenceframe.Input {
	inputs := make([]referenceframe.Input, len(deg))
	for i, d := range deg {
		inputs[i] = d * math.Pi / 180
	}
	return inputs
}

// TestJSONURDFJointLimitsAgree guards, for every model, that its embedded
// kinematics JSON (the default source) reports the same per-joint limits as
// its URDF (source of truth) — the JSON is generated FROM the URDF
// (cmd/gen-kinematics-json), but nothing stops it from silently drifting if
// the URDF is edited without regenerating.
func TestJSONURDFJointLimitsAgree(t *testing.T) {
	for _, model := range testModels {
		t.Run(model, func(t *testing.T) {
			jsonModel, urdfModel := buildBothModels(t, model)

			jsonLimits := jsonModel.DoF()
			urdfLimits := urdfModel.DoF()
			if len(jsonLimits) != len(urdfLimits) {
				t.Fatalf("DoF count mismatch: json=%d urdf=%d", len(jsonLimits), len(urdfLimits))
			}

			const epsilon = 1e-4 // radians; JSON round-trips through degrees, so allow float rounding
			for i := range jsonLimits {
				if math.Abs(jsonLimits[i].Min-urdfLimits[i].Min) > epsilon {
					t.Errorf("joint %d: Min mismatch: json=%.6f urdf=%.6f", i, jsonLimits[i].Min, urdfLimits[i].Min)
				}
				if math.Abs(jsonLimits[i].Max-urdfLimits[i].Max) > epsilon {
					t.Errorf("joint %d: Max mismatch: json=%.6f urdf=%.6f", i, jsonLimits[i].Max, urdfLimits[i].Max)
				}
			}
		})
	}
}

// TestJSONURDFForwardKinematicsAgree guards, for every model, that both
// kinematics sources produce the same tool pose for the same joint inputs, at
// the zero pose and a handful of non-trivial poses safely within every joint's
// limits (+/-20deg per joint has comfortable margin on both models — the
// tightest limit is FR5's j3 at +/-162deg).
func TestJSONURDFForwardKinematicsAgree(t *testing.T) {
	poses := [][]float64{
		{0, 0, 0, 0, 0, 0},
		{20, -15, 10, -20, 15, -10},
		{-20, 15, -10, 20, -15, 10},
	}

	for _, model := range testModels {
		t.Run(model, func(t *testing.T) {
			jsonModel, urdfModel := buildBothModels(t, model)

			for _, deg := range poses {
				inputs := degToInputs(deg)
				jsonPose, err := jsonModel.Transform(inputs)
				if err != nil {
					t.Fatalf("json model Transform(%v): %v", deg, err)
				}
				urdfPose, err := urdfModel.Transform(inputs)
				if err != nil {
					t.Fatalf("urdf model Transform(%v): %v", deg, err)
				}
				if !spatialmath.PoseAlmostEqual(jsonPose, urdfPose) {
					t.Errorf("pose mismatch at %v deg:\n  json: %v\n  urdf: %v", deg, jsonPose, urdfPose)
				}
			}
		})
	}
}

// The joint speed must reach the kinematics document, since that document is
// what the motion service and clients see.
func TestJointSpeedPublishedInKinematics(t *testing.T) {
	m, err := makeModelFrame("test", ModelFR5, false, nil)
	if err != nil {
		t.Fatal(err)
	}
	m, err = applyJointLimitOverrides(m, "test", jointLimitOverrides{maxVelocityDegsPerSec: 36})
	if err != nil {
		t.Fatal(err)
	}
	reparsed, err := referenceframe.UnmarshalModelJSON(m.ModelConfig().OriginalFile.Bytes, "test")
	if err != nil {
		t.Fatal(err)
	}
	for _, joint := range reparsed.ModelConfig().Joints {
		if joint.MaxVelocity == nil || *joint.MaxVelocity != 36 {
			t.Errorf("joint %s: max_velocity %v, want 36", joint.ID, joint.MaxVelocity)
		}
	}
	if len(m.DoF()) != 6 {
		t.Errorf("patched model has %d DoF, want 6", len(m.DoF()))
	}
}

func TestBadJointLockPublished(t *testing.T) {
	m, err := makeModelFrame("test", ModelFR5, false, nil)
	if err != nil {
		t.Fatal(err)
	}
	m, err = applyJointLimitOverrides(m, "test", jointLimitOverrides{lockedDeg: map[int]float64{2: 30}})
	if err != nil {
		t.Fatal(err)
	}
	reparsed, err := referenceframe.UnmarshalModelJSON(m.ModelConfig().OriginalFile.Bytes, "test")
	if err != nil {
		t.Fatal(err)
	}
	for i, limit := range reparsed.DoF() {
		width := limit.Max - limit.Min
		if i == 2 {
			if math.Abs(width-2*math.Pi/180) > 1e-9 || math.Abs((limit.Max+limit.Min)/2-30*math.Pi/180) > 1e-9 {
				t.Errorf("joint 2 not locked to 30±1°: [%v, %v] rad", limit.Min, limit.Max)
			}
		} else if width < math.Pi {
			t.Errorf("joint %d narrowed to [%v, %v] rad but wasn't locked", i, limit.Min, limit.Max)
		}
	}
}

// With use_urdf the lock can't be published, but it must still shape the
// model this module checks targets against.
func TestBadJointLockAppliesToURDFModel(t *testing.T) {
	t.Setenv("VIAM_MODULE_ROOT", ".")
	m, err := makeModelFrame("test", ModelFR5, true, nil)
	if err != nil {
		t.Fatal(err)
	}
	m, err = applyJointLimitOverrides(m, "test", jointLimitOverrides{lockedDeg: map[int]float64{0: 10}, maxVelocityDegsPerSec: 36})
	if err != nil {
		t.Fatal(err)
	}
	if limit := m.DoF()[0]; limit.Max-limit.Min > 2.01*math.Pi/180 {
		t.Errorf("joint 0 not locked: [%v, %v] rad", limit.Min, limit.Max)
	}
}

func TestLockedJointRangeDeg(t *testing.T) {
	joint := referenceframe.JointConfig{Min: -175, Max: 175}
	for _, tc := range []struct{ now, lo, hi float64 }{
		{0, -1, 1},
		{174.5, 173.5, 175}, // clamped to the joint's own stop
		{176, 175, 177},     // already outside its range: left where it is
	} {
		lo, hi := lockedJointRangeDeg(tc.now, joint)
		if lo != tc.lo || hi != tc.hi {
			t.Errorf("now=%v: got [%v, %v] want [%v, %v]", tc.now, lo, hi, tc.lo, tc.hi)
		}
	}
}

func TestBadJointsRejectsBadIndex(t *testing.T) {
	m, err := makeModelFrame("test", ModelFR5, false, nil)
	if err != nil {
		t.Fatal(err)
	}
	if _, err := applyJointLimitOverrides(m, "test", jointLimitOverrides{lockedDeg: map[int]float64{6: 0}}); err == nil {
		t.Error("expected an error for joint index 6")
	}
	if _, _, err := (&Config{BadJoints: []int{-1}}).Validate("path"); err == nil {
		t.Error("Validate: expected an error for joint index -1")
	}
}

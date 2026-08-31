package viamfairino

import (
	"math"
	"testing"

	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/spatialmath"
)

// buildBothModels returns (jsonModel, urdfModel) — the default embedded-JSON
// source and the on-disk URDF source, both derived from the same
// arm/fr5wml.urdf (see PLAN.md Step 10).
func buildBothModels(t *testing.T) (referenceframe.Model, referenceframe.Model) {
	t.Helper()
	t.Setenv("VIAM_MODULE_ROOT", ".")

	jsonModel, err := makeModelFrame("fairino", false, nil)
	if err != nil {
		t.Fatalf("makeModelFrame (json): %v", err)
	}
	urdfModel, err := makeModelFrame("fairino", true, nil)
	if err != nil {
		t.Fatalf("makeModelFrame (urdf): %v", err)
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

// TestJSONURDFJointLimitsAgree guards that arm/fr5wml_kinematics.json (the
// default kinematics source) reports the same per-joint limits as
// arm/fr5wml.urdf (source of truth) — the JSON is generated FROM the URDF
// (cmd/gen-kinematics-json), but nothing stops it from silently drifting if
// the URDF is edited without regenerating.
func TestJSONURDFJointLimitsAgree(t *testing.T) {
	jsonModel, urdfModel := buildBothModels(t)

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
}

// TestJSONURDFForwardKinematicsAgree guards that both kinematics sources
// produce the same tool pose for the same joint inputs, at the zero pose
// and a handful of non-trivial poses safely within every joint's limits
// (see the joint limit table in PLAN.md Step 1: the tightest is j3 at
// +/-138deg, so +/-20deg per joint here has comfortable margin everywhere).
func TestJSONURDFForwardKinematicsAgree(t *testing.T) {
	jsonModel, urdfModel := buildBothModels(t)

	poses := [][]float64{
		{0, 0, 0, 0, 0, 0},
		{20, -15, 10, -20, 15, -10},
		{-20, 15, -10, 20, -15, 10},
	}

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
}

package viamfairino

import (
	"fmt"

	"go.viam.com/rdk/referenceframe"
)

// gripperStrokeMM is the AG-105-145's total finger travel (fully closed to
// fully open, both fingers combined). Each of the two fingers moves half of
// this. Exposed via GripperConfig.StrokeMM so other Fairino/DH grippers can
// override it.
const gripperStrokeMM = 145.0

// Approximate AG-105-145 body/finger dimensions in mm (matching the arm
// kinematics' mm convention). These are for visualization/collision only — the
// point is that the fingers visibly open and close in the 3D scene, not exact
// CAD. Z is the approach axis (out of the arm flange); Y is the opening axis.
const (
	gripperBodyX = 90.0
	gripperBodyY = 146.0
	gripperBodyZ = 100.0

	gripperFingerX = 25.0
	gripperFingerY = 15.0
	gripperFingerZ = 90.0

	// gripperFingerZOffset places the finger boxes protruding beyond the body
	// (body spans Z 0..100; fingers centered at 145 span ~100..190).
	gripperFingerZOffset = 145.0

	// gripperFingerYRest is each finger's inner offset at the closed position
	// (input 0), so the two fingers nearly meet at the center when closed.
	gripperFingerYRest = 10.0
)

// makeGripperModel builds a 2-DoF kinematic model for a two-finger parallel
// gripper. The finger geometry rides prismatic joints so posing the model at
// the live gripper position (see fairinoGripper.CurrentInputs) makes the
// fingers open and close in the 3D scene.
//
// RDK models must have exactly one end effector (one leaf frame), so the two
// fingers can't be independent children of the base — they're a single chain:
//
//	world -> gripper_base -> left_slide -> left_finger -> right_slide -> right_finger(leaf)
//
// Because right_slide is chained under left_slide, its motion accumulates on
// top of the left finger's. To keep the fingers symmetric about the center,
// the joints are driven [d, 2d] (see positionToInputs): the left finger sits at
// +d, and the right finger at (d - 2d) = -d. d ranges 0..strokeMM/2; 0 =
// closed, strokeMM/2 = fully open.
func makeGripperModel(name string, strokeMM float64) (referenceframe.Model, error) {
	half := strokeMM / 2

	jsonStr := fmt.Sprintf(`{
  "name": %q,
  "kinematic_param_type": "SVA",
  "links": [
    {
      "id": "gripper_base",
      "parent": "world",
      "translation": {"X": 0, "Y": 0, "Z": 0},
      "geometry": {"type": "box", "x": %g, "y": %g, "z": %g, "translation": {"X": 0, "Y": 0, "Z": %g}, "label": "gripper_body"}
    },
    {
      "id": "left_finger",
      "parent": "left_slide",
      "translation": {"X": 0, "Y": 0, "Z": 0},
      "geometry": {"type": "box", "x": %g, "y": %g, "z": %g, "translation": {"X": 0, "Y": %g, "Z": %g}, "label": "left_finger"}
    },
    {
      "id": "right_finger",
      "parent": "right_slide",
      "translation": {"X": 0, "Y": 0, "Z": 0},
      "geometry": {"type": "box", "x": %g, "y": %g, "z": %g, "translation": {"X": 0, "Y": %g, "Z": %g}, "label": "right_finger"}
    }
  ],
  "joints": [
    {"id": "left_slide",  "type": "prismatic", "parent": "gripper_base", "axis": {"X": 0, "Y": 1,  "Z": 0}, "min": 0, "max": %g},
    {"id": "right_slide", "type": "prismatic", "parent": "left_finger",  "axis": {"X": 0, "Y": -1, "Z": 0}, "min": 0, "max": %g}
  ]
}`,
		name,
		gripperBodyX, gripperBodyY, gripperBodyZ, gripperBodyZ/2,
		gripperFingerX, gripperFingerY, gripperFingerZ, gripperFingerYRest, gripperFingerZOffset,
		gripperFingerX, gripperFingerY, gripperFingerZ, -gripperFingerYRest, gripperFingerZOffset,
		half, strokeMM,
	)

	model, err := referenceframe.UnmarshalModelJSON([]byte(jsonStr), name)
	if err != nil {
		return nil, fmt.Errorf("building gripper kinematic model: %w", err)
	}
	return model, nil
}

// positionToInputs maps a gripper open-percentage (0..100, as reported by
// GetGripperCurPosition / passed to MoveGripper) to the model's two prismatic
// joint inputs (mm). 100% = fully open, 0% = closed. The joints are [d, 2d]
// because right_slide is chained under left_slide (see makeGripperModel): this
// keeps both fingers symmetric about the center.
func positionToInputs(percent float64, strokeMM float64) []referenceframe.Input {
	if percent < 0 {
		percent = 0
	}
	if percent > 100 {
		percent = 100
	}
	d := percent / 100 * (strokeMM / 2)
	return []referenceframe.Input{referenceframe.Input(d), referenceframe.Input(2 * d)}
}

// inputsToPercent is the inverse of positionToInputs: it maps the left finger's
// travel back to an open-percentage for MoveGripper (used by GoToInputs).
func inputsToPercent(inputs []referenceframe.Input, strokeMM float64) float64 {
	if len(inputs) == 0 || strokeMM <= 0 {
		return 0
	}
	percent := float64(inputs[0]) / (strokeMM / 2) * 100
	if percent < 0 {
		percent = 0
	}
	if percent > 100 {
		percent = 100
	}
	return percent
}

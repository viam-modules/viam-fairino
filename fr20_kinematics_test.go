package viamfairino

import (
	"math"
	"testing"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/spatialmath"
	rdkutils "go.viam.com/rdk/utils"
)

// fr20PublishedDH is FAIRINO's published FR20 DH table (User Manual 3.9.4,
// Table 2.4-9) as an RDK DH model: a/d in mm, alpha in degrees.
const fr20PublishedDH = `{
  "name": "fr20_dh",
  "kinematic_param_type": "DH",
  "dhParams": [
    {"id": "l1", "parent": "world", "a": 0,     "d": 215, "alpha": 90,  "min": -175, "max": 175},
    {"id": "l2", "parent": "l1",    "a": -1000, "d": 0,   "alpha": 0,   "min": -265, "max": 85},
    {"id": "l3", "parent": "l2",    "a": -716,  "d": 0,   "alpha": 0,   "min": -160, "max": 160},
    {"id": "l4", "parent": "l3",    "a": 0,     "d": 166, "alpha": 90,  "min": -265, "max": 85},
    {"id": "l5", "parent": "l4",    "a": 0,     "d": 138, "alpha": -90, "min": -175, "max": 175},
    {"id": "l6", "parent": "l5",    "a": 0,     "d": 120, "alpha": 0,   "min": -175, "max": 175}
  ]
}`

type fr20ControllerSample struct {
	jointsDeg []float64
	pose      [6]float64
}

// fr20ControllerSamples are joint positions (deg) and the pose (x,y,z mm,
// rx,ry,rz deg) the sanding cell's FR20 (controller V3.9.33-QX, tool 0)
// reported for them, captured read-only: the first from tl_cur_pos at rest,
// the rest from GetForwardKin (computation only).
var fr20ControllerSamples = []fr20ControllerSample{
	{[]float64{-103.0015, 4.9963, -159.0024, -120.1088, 0.0161, 0}, [6]float64{-228.9392, 273.0542, 432.0015, 90.2238, -85.8850, -103.2259}},
	{[]float64{0, -90, 0, -90, 0, 0}, [6]float64{0, -284.5009, 2069.5740, -90, 0, 180}},
	{[]float64{30, -60, 45, -100, 20, 10}, [6]float64{-987.1772, -890.1736, 1361.7236, -40.8790, 65.7971, -94.7564}},
	{[]float64{-45, -120, 90, -60, -30, 45}, [6]float64{-372.6986, -7.0997, 1379.8208, 129.2315, 37.7612, -18.4349}},
	{[]float64{90, -30, -60, -150, 60, -90}, [6]float64{225.0009, -695.0095, 1411.2972, 150, -30, 90}},
	{[]float64{-120, -150, 120, -30, 90, 120}, [6]float64{-176.5845, 25.1485, 1107.3312, -16.1021, -25.6589, -86.3099}},
	{[]float64{150, -200, 30, -200, -120, -150}, [6]float64{-1439.0673, 953.2452, -156.4234, -100.5246, 34.5827, 84.3617}},
	{[]float64{10, -80, 140, -240, 160, 170}, [6]float64{-474.4314, -138.1606, 717.3357, 90, 10, 170}},
	{[]float64{-170, -10, -150, 60, -170, -170}, [6]float64{435.8021, 125.8977, 637.3351, 116.7402, 67.6621, 36.7402}},
}

// poseError returns the translation (mm) and rotation (deg) between two poses.
func poseError(want, got spatialmath.Pose) (float64, float64) {
	delta := spatialmath.PoseBetween(want, got)
	return delta.Point().Norm(), rdkutils.RadToDeg(math.Abs(delta.Orientation().AxisAngles().Theta))
}

// TestFR20MatchesPublishedDH checks both FR20 model sources, flange included,
// against the published DH table. The tolerance covers the vendor URDF's
// rounding (d4 = 166.01 mm, alpha = 1.5708 rad).
func TestFR20MatchesPublishedDH(t *testing.T) {
	dh, err := referenceframe.UnmarshalModelJSON([]byte(fr20PublishedDH), "fr20_dh")
	if err != nil {
		t.Fatalf("parsing published DH model: %v", err)
	}
	jsonModel, urdfModel := buildBothModels(t, ModelFR20)

	for _, s := range fr20ControllerSamples {
		inputs := degToInputs(s.jointsDeg)
		want, err := dh.Transform(inputs)
		if err != nil {
			t.Fatalf("DH Transform(%v): %v", s.jointsDeg, err)
		}
		for name, m := range map[string]referenceframe.Model{"json": jsonModel, "urdf": urdfModel} {
			got, err := m.Transform(inputs)
			if err != nil {
				t.Fatalf("%s Transform(%v): %v", name, s.jointsDeg, err)
			}
			if mm, deg := poseError(want, got); mm > 0.05 || deg > 0.01 {
				t.Errorf("%s at %v deg: off published DH by %.3f mm / %.4f deg", name, s.jointsDeg, mm, deg)
			}
		}
	}
}

// TestFR20MatchesController checks the FR20 model against poses the real
// controller reported, pinning what the DH table alone can't: base frame
// alignment, the 120 mm flange frame, and tl_cur_pos's rx,ry,rz being
// spatialmath.EulerAngles. The 2.5 mm tolerance covers the controller's
// calibrated DH lengths (fitted: a3 -716.52, d4 165.50, d6 119.00 mm), which
// differ slightly from the published values this module ships.
func TestFR20MatchesController(t *testing.T) {
	jsonModel, _ := buildBothModels(t, ModelFR20)

	for _, s := range fr20ControllerSamples {
		got, err := jsonModel.Transform(degToInputs(s.jointsDeg))
		if err != nil {
			t.Fatalf("Transform(%v): %v", s.jointsDeg, err)
		}
		want := spatialmath.NewPose(
			r3.Vector{X: s.pose[0], Y: s.pose[1], Z: s.pose[2]},
			&spatialmath.EulerAngles{
				Roll:  rdkutils.DegToRad(s.pose[3]),
				Pitch: rdkutils.DegToRad(s.pose[4]),
				Yaw:   rdkutils.DegToRad(s.pose[5]),
			},
		)
		if mm, deg := poseError(want, got); mm > 2.5 || deg > 0.01 {
			t.Errorf("at %v deg: off the controller's pose by %.3f mm / %.4f deg", s.jointsDeg, mm, deg)
		}
	}
}

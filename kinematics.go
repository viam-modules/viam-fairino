package viamfairino

import (
	_ "embed"
	"fmt"
	"os"
	"path/filepath"

	"go.viam.com/rdk/referenceframe"
)

// urdfRelPath is arm/fr5wm.urdf's location relative to the module's root
// directory (VIAM_MODULE_ROOT, set by viam-server after unpacking the
// module tarball). See PLAN.md Steps 0-4 for how that URDF was derived from
// Fairino's own ROS description (fairino_description/urdf/FR5WM.urdf),
// cleaned up, and laid out under arm/ alongside its meshes.
const urdfRelPath = "arm/fr5wm.urdf"

// fr5wmKinematicsJSON is the embedded default kinematics source — generated
// once from the validated arm/fr5wm.urdf by cmd/gen-kinematics-json (PLAN.md
// Step 10) and committed, not regenerated at build/run time. Used unless
// Config.UseURDF selects the on-disk URDF path instead.
//
//go:embed arm/fr5wm_kinematics.json
var fr5wmKinematicsJSON []byte

// makeModelFrame builds the FR5-WML's kinematic model. useURDF selects the
// source: false (the default) parses the embedded JSON
// (fr5wmKinematicsJSON); true parses the URDF shipped in the module tarball
// instead, in which case meshDecimationRatios is forwarded as-is to
// ParseModelXMLFile (nil means no decimation — see that function's doc
// comment for the per-collision-mesh ordering it expects). Both sources are
// derived from, and validated against, the same arm/fr5wm.urdf — see
// TestJSONURDFForwardKinematicsAgree / TestJSONURDFJointLimitsAgree.
func makeModelFrame(name string, useURDF bool, meshDecimationRatios []float64) (referenceframe.Model, error) {
	if !useURDF {
		model, err := referenceframe.UnmarshalModelJSON(fr5wmKinematicsJSON, name)
		if err != nil {
			return nil, fmt.Errorf("failed to parse embedded kinematics JSON: %w", err)
		}
		return model, nil
	}

	root := os.Getenv("VIAM_MODULE_ROOT")
	if root == "" {
		return nil, fmt.Errorf("VIAM_MODULE_ROOT is unset; can't locate %s", urdfRelPath)
	}
	path := filepath.Join(root, urdfRelPath)
	model, err := referenceframe.ParseModelXMLFile(path, name, meshDecimationRatios)
	if err != nil {
		return nil, fmt.Errorf("failed to parse kinematics URDF %s: %w", path, err)
	}
	return model, nil
}

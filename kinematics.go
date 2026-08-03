package viamfairino

import (
	_ "embed"
	"fmt"
	"os"
	"path/filepath"

	"go.viam.com/rdk/referenceframe"
)

// urdfRelPath is arm/fr5wml.urdf's location relative to the module's root
// directory (VIAM_MODULE_ROOT, set by viam-server after unpacking the
// module tarball). See PLAN.md STATUS/Step 1 for where this URDF actually
// came from: a manually CAD-corrected copy of Fairino's fairino5_v6.urdf
// (link lengths fixed; mass/inertia and meshes are still fairino5_v6's
// originals and known to be inaccurate for now) — not FR5WM.urdf, which an
// earlier pass discovered didn't match the physical FR5-WML at all.
const urdfRelPath = "arm/fr5wml.urdf"

// fr5wmlKinematicsJSON is the embedded default kinematics source — generated
// once from the validated arm/fr5wml.urdf by cmd/gen-kinematics-json (PLAN.md
// Step 10) and committed, not regenerated at build/run time. Used unless
// Config.UseURDF selects the on-disk URDF path instead.
//
//go:embed arm/fr5wml_kinematics.json
var fr5wmlKinematicsJSON []byte

// makeModelFrame builds the FR5-WML's kinematic model. useURDF selects the
// source: false (the default) parses the embedded JSON
// (fr5wmlKinematicsJSON); true parses the URDF shipped in the module tarball
// instead, in which case meshDecimationRatios is forwarded as-is to
// ParseModelXMLFile (nil means no decimation — see that function's doc
// comment for the per-collision-mesh ordering it expects). Both sources are
// derived from, and validated against, the same arm/fr5wml.urdf — see
// TestJSONURDFForwardKinematicsAgree / TestJSONURDFJointLimitsAgree.
func makeModelFrame(name string, useURDF bool, meshDecimationRatios []float64) (referenceframe.Model, error) {
	if !useURDF {
		model, err := referenceframe.UnmarshalModelJSON(fr5wmlKinematicsJSON, name)
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

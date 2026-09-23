package viamfairino

import (
	_ "embed"
	"fmt"
	"os"
	"path/filepath"
	"sort"
	"strings"

	"go.viam.com/rdk/referenceframe"
)

// Supported robot models. The controller does not report its own model over
// the status/command protocol, so the model must be selected explicitly via
// Config.Model (see module.go) — there is no reliable auto-detection.
const (
	ModelFR5    = "fr5"     // standard FAIRINO FR5, ~922 mm reach
	ModelFR5WML = "fr5-wml" // long-reach FR5-WML, ~1.9 m reach
)

// fr5wmlKinematicsJSON / fr5KinematicsJSON are the embedded default kinematics
// sources — generated once from the committed URDFs by cmd/gen-kinematics-json
// and committed, not regenerated at build/run time. Used unless Config.UseURDF
// selects the on-disk URDF path instead.
//
//go:embed arm/fr5wml_kinematics.json
var fr5wmlKinematicsJSON []byte

//go:embed arm/fr5_kinematics.json
var fr5KinematicsJSON []byte

// modelAssets bundles everything that differs between robot models: the
// on-disk URDF path (relative to VIAM_MODULE_ROOT), the embedded kinematics
// JSON, and the visual-mesh directory Get3DModels serves from. numCollisionMeshes
// is the count of <collision><mesh> elements in the URDF, in document order —
// used to size cmd/gen-kinematics-json's decimation-ratio slice (both models
// happen to have 7: base_link plus six links).
type modelAssets struct {
	urdfRelPath        string
	kinematicsJSON     []byte
	meshVisualDir      string
	numCollisionMeshes int
}

// modelRegistry maps a Config.Model value to its assets. Adding a model is a
// matter of adding its URDF/JSON/meshes and one entry here.
var modelRegistry = map[string]modelAssets{
	ModelFR5WML: {
		// FAIRINO's official FR5-WML URDF with real CAD meshes, adapted for
		// this module — see arm/fr5wml_SOURCES.md.
		urdfRelPath:        "arm/fr5wml.urdf",
		kinematicsJSON:     fr5wmlKinematicsJSON,
		meshVisualDir:      "arm/meshes/fr5wml/visual",
		numCollisionMeshes: 7,
	},
	ModelFR5: {
		// The vendor's official fairino5_v6 SolidWorks export with real CAD
		// meshes — see arm/fr5_SOURCES.md.
		urdfRelPath:        "arm/fr5.urdf",
		kinematicsJSON:     fr5KinematicsJSON,
		meshVisualDir:      "arm/meshes/fr5/visual",
		numCollisionMeshes: 7,
	},
}

// modelFromRobotModel maps the controller's reported robotModel string (from
// GetSoftwareVersion — e.g. "FR5-V1-002(V6.0)" for a standard FR5, and
// presumably "FR5-WML-..." for the long-reach variant) to a modelRegistry key.
// WML is checked first since an FR5-WML string also contains "FR5". Returns
// ("", false) if unrecognized, so the caller can fall back or error.
func modelFromRobotModel(robotModel string) (string, bool) {
	u := strings.ToUpper(robotModel)
	switch {
	case strings.Contains(u, "FR5") && strings.Contains(u, "WML"):
		return ModelFR5WML, true
	case strings.Contains(u, "FR5"):
		return ModelFR5, true
	default:
		return "", false
	}
}

// supportedModels returns the known model names, sorted, for error messages.
func supportedModels() string {
	names := make([]string, 0, len(modelRegistry))
	for name := range modelRegistry {
		names = append(names, name)
	}
	sort.Strings(names)
	return strings.Join(names, ", ")
}

// makeModelFrame builds the named robot's kinematic model for the given model
// (ModelFR5 / ModelFR5WML). useURDF selects the source: false (the default)
// parses the model's embedded JSON; true parses the on-disk URDF shipped in
// the module tarball instead, in which case meshDecimationRatios is forwarded
// as-is to ParseModelXMLFile (nil means no decimation — see that function's
// doc comment for the per-collision-mesh ordering it expects). Both sources
// are derived from, and validated against, the same URDF — see
// TestJSONURDFForwardKinematicsAgree / TestJSONURDFJointLimitsAgree.
func makeModelFrame(name, model string, useURDF bool, meshDecimationRatios []float64) (referenceframe.Model, error) {
	assets, ok := modelRegistry[model]
	if !ok {
		return nil, fmt.Errorf("unsupported model %q; supported models: %s", model, supportedModels())
	}

	if !useURDF {
		m, err := referenceframe.UnmarshalModelJSON(assets.kinematicsJSON, name)
		if err != nil {
			return nil, fmt.Errorf("failed to parse embedded kinematics JSON for model %q: %w", model, err)
		}
		return m, nil
	}

	root := os.Getenv("VIAM_MODULE_ROOT")
	if root == "" {
		return nil, fmt.Errorf("VIAM_MODULE_ROOT is unset; can't locate %s", assets.urdfRelPath)
	}
	path := filepath.Join(root, assets.urdfRelPath)
	m, err := referenceframe.ParseModelXMLFile(path, name, meshDecimationRatios)
	if err != nil {
		return nil, fmt.Errorf("failed to parse kinematics URDF %s: %w", path, err)
	}
	return m, nil
}

// jointLimitOverrides are the limits this module writes into its kinematics on
// top of the model's own position limits.
type jointLimitOverrides struct {
	// maxVelocityDegsPerSec is published as every joint's max_velocity, so the
	// motion service plans at the speed the arm will actually move. 0 leaves
	// the document's velocity limits alone.
	maxVelocityDegsPerSec float64
}

// applyJointLimitOverrides writes o into m's kinematics document and reparses
// it. Writing into the document is the only way limits reach the motion service
// and clients: RDK sends the document bytes, not the parsed model. Only the SVA
// JSON source can carry them (URDF limits are RSDK-14232), so for a URDF model
// this leaves m unchanged.
func applyJointLimitOverrides(m referenceframe.Model, name string, o jointLimitOverrides) (referenceframe.Model, error) {
	cfg := m.ModelConfig()
	if o.maxVelocityDegsPerSec <= 0 || cfg.OriginalFile != nil && cfg.OriginalFile.Extension == "urdf" {
		return m, nil
	}
	v := o.maxVelocityDegsPerSec
	limits := make(map[string]referenceframe.JointLimits, len(cfg.Joints))
	for _, joint := range cfg.Joints {
		if joint.Mimic != nil {
			continue
		}
		limits[joint.ID] = referenceframe.JointLimits{MaxVelocity: &v}
	}
	patched, err := referenceframe.SetJointLimits(cfg, limits)
	if err != nil {
		return nil, fmt.Errorf("setting joint limits: %w", err)
	}
	return patched.ParseConfig(name)
}

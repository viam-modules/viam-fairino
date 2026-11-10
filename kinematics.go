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

// Supported robot models, selected via Config.Model (see module.go) or, when
// that is empty, auto-detected from the controller's reported robotModel
// (modelFromRobotModel).
const (
	ModelFR5    = "fr5"     // standard FAIRINO FR5, ~922 mm reach
	ModelFR5WML = "fr5-wml" // long-reach FR5-WML, ~1.9 m reach
	ModelFR20   = "fr20"    // FAIRINO FR20, 20 kg rated payload
)

// The fr*KinematicsJSON vars are the embedded default kinematics
// sources — generated once from the committed URDFs by cmd/gen-kinematics-json
// and committed, not regenerated at build/run time. Used unless Config.UseURDF
// selects the on-disk URDF path instead.
//
//go:embed arm/fr5wml_kinematics.json
var fr5wmlKinematicsJSON []byte

//go:embed arm/fr5_kinematics.json
var fr5KinematicsJSON []byte

//go:embed arm/fr20_kinematics.json
var fr20KinematicsJSON []byte

// modelAssets bundles everything that differs between robot models: the
// on-disk URDF path (relative to VIAM_MODULE_ROOT), the embedded kinematics
// JSON, and the visual-mesh directory Get3DModels serves from. numCollisionMeshes
// is the count of <collision><mesh> elements in the URDF, in document order —
// used to size cmd/gen-kinematics-json's decimation-ratio slice (every model
// happens to have 7: base_link plus six links).
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
	ModelFR20: {
		// The vendor's official fairino20_v6 SolidWorks export with real CAD
		// meshes, plus the flange frame it omits — see arm/fr20_SOURCES.md.
		urdfRelPath:        "arm/fr20.urdf",
		kinematicsJSON:     fr20KinematicsJSON,
		meshVisualDir:      "arm/meshes/fr20/visual",
		numCollisionMeshes: 7,
	},
}

// modelFromRobotModel maps the controller's reported robotModel string (from
// GetSoftwareVersion — e.g. "FR5-V1-002(V6.0)", "FRCustom(8)-V1-002(FR5-WML)",
// "FR20-V1-001(V6.0)") to a modelRegistry key. FR20 matches on the leading
// model token, so a longer "FR200..." name wouldn't. WML is checked before
// plain FR5 since an FR5-WML string also contains "FR5". Returns ("", false)
// if unrecognized, so the caller can fall back or error.
func modelFromRobotModel(robotModel string) (string, bool) {
	u := strings.ToUpper(robotModel)
	mainModel, _, _ := strings.Cut(u, "-")
	switch {
	case mainModel == "FR20":
		return ModelFR20, true
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
// (a modelRegistry key). useURDF selects the source: false (the default)
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

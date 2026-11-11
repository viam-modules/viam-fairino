// Command gen-kinematics-json is a one-off generator, run manually and its
// output committed — not run at build or module-start time. For each supported
// robot model it parses the validated URDF and dumps the resulting SVA-style
// kinematics config to the embedded JSON kinematics.go go:embeds as that
// model's default (non-URDF) kinematics source (PLAN.md Step 10).
//
// Run from the repo root: go run ./cmd/gen-kinematics-json
package main

import (
	"encoding/json"
	"fmt"
	"os"

	"go.viam.com/rdk/referenceframe"
)

// genModel describes one robot model to generate a kinematics JSON for.
// numCollisionMeshes is the count of <collision><mesh> elements in the URDF,
// in document order (base_link, then the 6 links) — one per link, no
// flange/tool frame. meshDecimationRatios needs exactly one entry per mesh,
// not per joint. See referenceframe.ParseModelXMLFile's doc comment.
type genModel struct {
	urdfPath           string
	jsonPath           string
	numCollisionMeshes int
}

// models must stay in sync with viamfairino's modelRegistry (kinematics.go).
var models = []genModel{
	{"arm/fr5wml.urdf", "arm/fr5wml_kinematics.json", 7},
	{"arm/fr5.urdf", "arm/fr5_kinematics.json", 7},
	{"arm/fr20.urdf", "arm/fr20_kinematics.json", 7},
}

// decimationRatio is the fraction of triangles kept per mesh — applied here
// (unlike dobot/xarm's URDF-mode decimation) because this file is embedded
// directly into the binary (go:embed) as the default kinematics source, so
// its size and parse cost matter in a way the on-disk, opt-in URDF path's
// don't. Undecimated, the FR5-WML file was 21.5MB (the full STL triangle data
// re-encoded as bloated ASCII JSON). Tune this and regenerate if the result
// is still too large/slow, or too coarse for collision purposes.
const decimationRatio = 0.1

func main() {
	if err := run(); err != nil {
		fmt.Fprintln(os.Stderr, err)
		os.Exit(1)
	}
}

func run() error {
	for _, m := range models {
		if err := genOne(m); err != nil {
			return err
		}
	}
	return nil
}

func genOne(m genModel) error {
	ratios := make([]float64, m.numCollisionMeshes)
	for i := range ratios {
		ratios[i] = decimationRatio
	}

	model, err := referenceframe.ParseModelXMLFile(m.urdfPath, "fairino", ratios)
	if err != nil {
		return fmt.Errorf("parsing %s: %w", m.urdfPath, err)
	}

	cfg := model.ModelConfig()
	// Must be nil, not just left as the URDF's OriginalFile: this field is a
	// real, marshaled JSON key ("original_file"). If the generated file
	// still had the URDF's bytes/extension embedded in it,
	// referenceframe.UnmarshalModelJSON's json.Unmarshal call would let that
	// overwrite its own pre-set OriginalFile{Extension: "json"} on load —
	// silently defeating the entire point of this generator (GetKinematics
	// would still tag the wire format URDF, not SVA, and the app's
	// joint-limit sliders would still break — see PLAN.md Step 10).
	cfg.OriginalFile = nil

	data, err := json.MarshalIndent(cfg, "", "  ")
	if err != nil {
		return fmt.Errorf("marshaling kinematics config: %w", err)
	}
	data = append(data, '\n')

	if err := os.WriteFile(m.jsonPath, data, 0o644); err != nil {
		return fmt.Errorf("writing %s: %w", m.jsonPath, err)
	}
	fmt.Printf("wrote %s (%d bytes)\n", m.jsonPath, len(data))
	return nil
}

// Command gen-kinematics-json is a one-off generator, run manually and its
// output committed — not run at build or module-start time. It parses the
// validated arm/fr5wm.urdf and dumps the resulting SVA-style kinematics
// config to arm/fr5wm_kinematics.json, which kinematics.go go:embeds as the
// default (non-URDF) kinematics source (PLAN.md Step 10).
//
// Run from the repo root: go run ./cmd/gen-kinematics-json
package main

import (
	"encoding/json"
	"fmt"
	"os"

	"go.viam.com/rdk/referenceframe"
)

const (
	urdfPath = "arm/fr5wm.urdf"
	jsonPath = "arm/fr5wm_kinematics.json"

	// numCollisionMeshes is the count of <collision><mesh> elements in
	// arm/fr5wm.urdf, in document order (base_link, then the 6 links) —
	// meshDecimationRatios below needs exactly one entry per mesh, not per
	// joint (flange has no collision geometry, so it's 7, not 8 or 6). See
	// referenceframe.ParseModelXMLFile's doc comment for this ordering.
	numCollisionMeshes = 7

	// decimationRatio is the fraction of triangles kept per mesh — applied
	// here (unlike dobot/xarm's URDF-mode decimation) because this file gets
	// go:embedded directly into the binary as the default kinematics
	// source, so its size and parse cost matter in a way the on-disk,
	// opt-in URDF path's don't. Undecimated, this file was 21.5MB (the full
	// STL triangle data re-encoded as bloated ASCII JSON). Tune this and
	// regenerate if the result is still too large/slow, or too coarse for
	// collision purposes.
	decimationRatio = 0.1
)

func main() {
	if err := run(); err != nil {
		fmt.Fprintln(os.Stderr, err)
		os.Exit(1)
	}
}

func run() error {
	ratios := make([]float64, numCollisionMeshes)
	for i := range ratios {
		ratios[i] = decimationRatio
	}

	model, err := referenceframe.ParseModelXMLFile(urdfPath, "fairino", ratios)
	if err != nil {
		return fmt.Errorf("parsing %s: %w", urdfPath, err)
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

	if err := os.WriteFile(jsonPath, data, 0o644); err != nil {
		return fmt.Errorf("writing %s: %w", jsonPath, err)
	}
	fmt.Printf("wrote %s (%d bytes)\n", jsonPath, len(data))
	return nil
}

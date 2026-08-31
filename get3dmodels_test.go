package viamfairino

import (
	"context"
	"testing"

	"go.viam.com/rdk/logging"
)

// wantLinkNames are the 7 links in arm/fr5wml.urdf (no flange/tool frame in
// this URDF, unlike the earlier FR5WM.urdf-derived one — see PLAN.md
// STATUS).
var wantLinkNames = []string{
	"base_link",
	"shoulder_link",
	"upperarm_link",
	"forearm_link",
	"wrist1_link",
	"wrist2_link",
	"wrist3_link",
}

func TestGet3DModelsUnsetModuleRoot(t *testing.T) {
	t.Setenv("VIAM_MODULE_ROOT", "")
	s := &viamFairinoFairino{logger: logging.NewTestLogger(t)}

	got, err := s.Get3DModels(context.Background(), nil)
	if err != nil {
		t.Fatalf("Get3DModels: %v", err)
	}
	if len(got) != 0 {
		t.Errorf("expected empty map with VIAM_MODULE_ROOT unset, got %d entries", len(got))
	}
}

func TestGet3DModelsReturnsAllLinkMeshes(t *testing.T) {
	t.Setenv("VIAM_MODULE_ROOT", ".")
	s := &viamFairinoFairino{logger: logging.NewTestLogger(t)}

	got, err := s.Get3DModels(context.Background(), nil)
	if err != nil {
		t.Fatalf("Get3DModels: %v", err)
	}
	if len(got) != len(wantLinkNames) {
		t.Errorf("got %d meshes, want %d", len(got), len(wantLinkNames))
	}

	for _, name := range wantLinkNames {
		mesh, ok := got[name]
		if !ok {
			t.Errorf("missing mesh for link %q", name)
			continue
		}
		if mesh.ContentType != "model/gltf-binary" {
			t.Errorf("%s: content type = %q, want %q", name, mesh.ContentType, "model/gltf-binary")
		}
		if len(mesh.Mesh) < 4 {
			t.Errorf("%s: mesh bytes too short (%d bytes) to be a valid glb", name, len(mesh.Mesh))
			continue
		}
		// glTF binary files start with the magic 4-byte ASCII string "glTF".
		if magic := string(mesh.Mesh[:4]); magic != "glTF" {
			t.Errorf("%s: missing glTF magic header, got %q", name, magic)
		}
	}
}

func TestGet3DModelsCaches(t *testing.T) {
	t.Setenv("VIAM_MODULE_ROOT", ".")
	s := &viamFairinoFairino{logger: logging.NewTestLogger(t)}

	first, err := s.Get3DModels(context.Background(), nil)
	if err != nil {
		t.Fatalf("Get3DModels (first call): %v", err)
	}
	if s.meshCache == nil {
		t.Fatal("expected meshCache to be populated after first call")
	}

	second, err := s.Get3DModels(context.Background(), nil)
	if err != nil {
		t.Fatalf("Get3DModels (second call): %v", err)
	}
	if len(first) != len(second) {
		t.Errorf("cached call returned %d meshes, first call returned %d", len(second), len(first))
	}

	// The returned maps must be independent copies of the cache (cloneMeshMap) —
	// mutating one must not affect the other or the cache itself.
	for name := range first {
		delete(first, name)
		break
	}
	if len(first) == len(second) {
		t.Error("mutating a returned map affected another returned map; Get3DModels must return independent copies")
	}
}

package viamfairino

import (
	"context"
	"testing"

	"go.viam.com/rdk/logging"
)

// wantLinkNames maps each model to the 7 links in its URDF that carry a visual
// mesh. Every model's URDF uses the vendor's descriptive link names.
// Get3DModels keys meshes by these names.
var wantLinkNames = map[string][]string{
	ModelFR5WML: {"base_link", "shoulder_link", "upperarm_link", "forearm_link", "wrist1_link", "wrist2_link", "wrist3_link"},
	ModelFR5:    {"base_link", "shoulder_link", "upperarm_link", "forearm_link", "wrist1_link", "wrist2_link", "wrist3_link"},
	ModelFR20:   {"base_link", "shoulder_link", "upperarm_link", "forearm_link", "wrist1_link", "wrist2_link", "wrist3_link"},
}

// newTestArm builds a bare arm struct wired with the given model's visual-mesh
// directory — enough to exercise Get3DModels without a full construction.
func newTestArm(t *testing.T, model string) *viamFairinoFairino {
	t.Helper()
	return &viamFairinoFairino{
		logger:        logging.NewTestLogger(t),
		meshVisualDir: modelRegistry[model].meshVisualDir,
	}
}

func TestGet3DModelsUnsetModuleRoot(t *testing.T) {
	t.Setenv("VIAM_MODULE_ROOT", "")
	s := newTestArm(t, ModelFR5WML)

	got, err := s.Get3DModels(context.Background(), nil)
	if err != nil {
		t.Fatalf("Get3DModels: %v", err)
	}
	if len(got) != 0 {
		t.Errorf("expected empty map with VIAM_MODULE_ROOT unset, got %d entries", len(got))
	}
}

func TestGet3DModelsReturnsAllLinkMeshes(t *testing.T) {
	for _, model := range testModels {
		t.Run(model, func(t *testing.T) {
			t.Setenv("VIAM_MODULE_ROOT", ".")
			s := newTestArm(t, model)

			got, err := s.Get3DModels(context.Background(), nil)
			if err != nil {
				t.Fatalf("Get3DModels: %v", err)
			}
			want := wantLinkNames[model]
			if len(got) != len(want) {
				t.Errorf("got %d meshes, want %d", len(got), len(want))
			}

			for _, name := range want {
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
		})
	}
}

func TestGet3DModelsCaches(t *testing.T) {
	t.Setenv("VIAM_MODULE_ROOT", ".")
	s := newTestArm(t, ModelFR5WML)

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

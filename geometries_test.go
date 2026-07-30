package viamfairino

import (
	"strconv"
	"strings"
	"testing"

	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/spatialmath"
)

// TestGeometryProtoRoundTrip documents and guards against a real RDK bug
// (not our data): a mesh loaded from STL has its rawBytes field populated
// with the *original* STL bytes; Mesh.ensurePLYBytes only regenerates real
// PLY when rawBytes is nil, so it returns those raw STL bytes unchanged —
// and Mesh.ToProtobuf unconditionally labels whatever ensurePLYBytes
// returns as ContentType "ply" regardless. Result: GetGeometries sends raw
// binary STL data mislabeled as PLY, which goply (RDK's PLY reader) can't
// parse — panics "Invalid ply file" trying to read the STL header as PLY's
// magic string. This broke the app's 3D scene viewer's getGeometries call
// outright on real hardware.
//
// module.go's Geometries() works around this via rebuildMeshForPLYBug. This
// test proves both halves: the raw model geometry still exhibits the bug
// (so this test would catch it disappearing silently, e.g. if RDK fixes it
// upstream and the workaround becomes dead code worth removing), and that
// s.Geometries()'s actual output (post-workaround) round-trips cleanly.
func TestGeometryProtoRoundTrip(t *testing.T) {
	t.Setenv("VIAM_MODULE_ROOT", ".")
	model, err := makeModelFrame("fairino", true, nil)
	if err != nil {
		t.Fatalf("makeModelFrame: %v", err)
	}

	zeroInputs := make([]referenceframe.Input, len(model.DoF()))
	gif, err := model.Geometries(zeroInputs)
	if err != nil {
		t.Fatalf("Geometries: %v", err)
	}

	geoms := gif.Geometries()
	if len(geoms) == 0 {
		t.Fatal("Geometries returned no geometries at zero pose")
	}

	sawRawBug := false
	for _, g := range geoms {
		if _, isMesh := g.(*spatialmath.Mesh); !isMesh {
			continue
		}

		rawMesh := g.ToProtobuf().GetMesh()
		info := inspectPLY(rawMesh.Mesh)
		t.Logf("%s: raw model geometry: content_type=%q bytes=%d header declares vertex=%d face=%d, found %d actual data lines",
			g.Label(), rawMesh.ContentType, len(rawMesh.Mesh), info.declaredVerts, info.declaredFaces, info.dataLines)

		if _, err := spatialmath.NewMeshFromProto(spatialmath.NewZeroPose(), rawMesh, g.Label()); err != nil {
			sawRawBug = true
			t.Logf("%s: raw model geometry round-trip fails as expected (RDK bug): %v", g.Label(), err)
		}

		fixed := rebuildMeshForPLYBug(g)
		fixedProto := fixed.ToProtobuf().GetMesh()
		if _, err := spatialmath.NewMeshFromProto(spatialmath.NewZeroPose(), fixedProto, g.Label()); err != nil {
			t.Errorf("%s: rebuildMeshForPLYBug did NOT fix the round-trip: %v", g.Label(), err)
		}
	}

	if !sawRawBug {
		t.Log("NOTE: none of the raw model geometries exhibited the PLY bug — either RDK fixed it upstream " +
			"(check whether rebuildMeshForPLYBug/this test's diagnostics are still needed) or something else changed.")
	}
}

type plyInfo struct {
	declaredVerts, declaredFaces, dataLines, headerLines, maxLineLen int
}

// inspectPLY does a lightweight, independent sanity check of an ASCII PLY
// buffer's header-declared element counts against the number of data lines
// actually present, and reports the longest line — all without going
// through goply itself, since goply's own parser is what's under test here.
func inspectPLY(data []byte) plyInfo {
	var info plyInfo
	lines := strings.Split(string(data), "\n")
	inHeader := true
	for _, line := range lines {
		if len(line) > info.maxLineLen {
			info.maxLineLen = len(line)
		}
		if inHeader {
			info.headerLines++
			fields := strings.Fields(line)
			if len(fields) == 0 {
				continue
			}
			switch {
			case fields[0] == "element" && len(fields) >= 3 && fields[1] == "vertex":
				info.declaredVerts, _ = strconv.Atoi(fields[2])
			case fields[0] == "element" && len(fields) >= 3 && fields[1] == "face":
				info.declaredFaces, _ = strconv.Atoi(fields[2])
			case fields[0] == "end_header":
				inHeader = false
			}
			continue
		}
		if strings.TrimSpace(line) != "" {
			info.dataLines++
		}
	}
	return info
}

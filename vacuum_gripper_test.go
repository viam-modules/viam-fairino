package viamfairino

import (
	"context"
	"testing"

	"go.viam.com/rdk/components/gripper"
	"go.viam.com/rdk/logging"
)

func intPtr(v int) *int { return &v }

func newTestVacuum(t *testing.T, fc *fakeController, cfg *VacuumGripperConfig, fr statusFrame) (*fairinoVacuumGripper, *statusClient) {
	t.Helper()
	logger := logging.NewTestLogger(t)
	model, err := makeVacuumModel("vac", cfg.LengthMM)
	if err != nil {
		t.Fatal(err)
	}
	sc := newStatusClient("127.0.0.1", statusPort, logger)
	setTestFrame(sc, fr)
	cmd := newCommandClient("127.0.0.1", fc.port(), logger)
	t.Cleanup(func() { _ = cmd.close() })
	return &fairinoVacuumGripper{name: gripper.Named("vac"), logger: logger, cfg: cfg, cmd: cmd, status: sc, model: model}, sc
}

// ioFrame is a status frame with the given tool and box IO bytes.
func ioFrame(t *testing.T, toolOut, toolIn, boxOutL, boxOutH, boxInL, boxInH byte) statusFrame {
	t.Helper()
	p := buildTestPayload()
	p[offTlDgtOutputL], p[offTlDgtInputL] = toolOut, toolIn
	p[offClDgtOutputL], p[offClDgtOutputH] = boxOutL, boxOutH
	p[offClDgtInputL], p[offClDgtInputH] = boxInL, boxInH
	fr, ok := parseStatus(p, 1)
	if !ok {
		t.Fatal("failed to build test frame")
	}
	return fr
}

func TestVacuumGrabAndOpenDriveBothPins(t *testing.T) {
	fc := newFakeController(t)
	g, _ := newTestVacuum(t, fc, &VacuumGripperConfig{SuctionOutput: 0, ReleaseOutput: intPtr(1)}, ioFrame(t, 0, 0, 0, 0, 0, 0))
	if holding, err := g.Grab(context.Background(), nil); err != nil || !holding {
		t.Fatalf("Grab: %v, %v", holding, err)
	}
	if err := g.Open(context.Background(), nil); err != nil {
		t.Fatal(err)
	}
	fc.mu.Lock()
	got := append([]string(nil), fc.commands...)
	fc.mu.Unlock()
	// Release closes before suction starts, and opens only after it stops.
	want := []string{"SetToolDO(1,0,0,0)", "SetToolDO(0,1,0,0)", "SetToolDO(0,0,0,0)", "SetToolDO(1,1,0,0)"}
	if len(got) != len(want) {
		t.Fatalf("got %v want %v", got, want)
	}
	for i := range want {
		if got[i] != want[i] {
			t.Fatalf("got %v want %v", got, want)
		}
	}
}

func TestVacuumHoldingFromSensorOrOutput(t *testing.T) {
	fc := newFakeController(t)
	// Box IO: suction on output 9 (high byte bit 1), sensor on input 3.
	cfg := &VacuumGripperConfig{IO: "box", SuctionOutput: 9, SensorInput: intPtr(3)}
	g, sc := newTestVacuum(t, fc, cfg, ioFrame(t, 0, 0, 0, 0x02, 0x08, 0))
	st, err := g.IsHoldingSomething(context.Background(), nil)
	if err != nil || !st.IsHoldingSomething || st.Meta["source"] != "sensor_input" {
		t.Errorf("sensor on: got %+v, %v", st, err)
	}
	setTestFrame(sc, ioFrame(t, 0, 0, 0, 0x02, 0, 0))
	if st, _ := g.IsHoldingSomething(context.Background(), nil); st.IsHoldingSomething {
		t.Error("sensor off should not report holding")
	}

	// Without a sensor, holding follows the suction output.
	cfg.SensorInput = nil
	st, err = g.IsHoldingSomething(context.Background(), nil)
	if err != nil || !st.IsHoldingSomething || st.Meta["source"] != "suction_output" {
		t.Errorf("suction on, no sensor: got %+v, %v", st, err)
	}
}

func TestVacuumGeometry(t *testing.T) {
	g, _ := newTestVacuum(t, newFakeController(t), &VacuumGripperConfig{LengthMM: 50}, ioFrame(t, 0, 0, 0, 0, 0, 0))
	geoms, err := g.Geometries(context.Background(), nil)
	if err != nil || len(geoms) != 1 {
		t.Fatalf("got %v, %v", geoms, err)
	}
}

func TestValidateVacuumConfig(t *testing.T) {
	for _, cfg := range []VacuumGripperConfig{
		{IO: "wifi"},
		{SuctionOutput: 2},
		{IO: "box", SuctionOutput: 16},
		{SuctionOutput: 1, ReleaseOutput: intPtr(1)},
		{SensorInput: intPtr(-1)},
		{LengthMM: -1},
	} {
		if _, _, err := cfg.Validate("path"); err == nil {
			t.Errorf("%+v: expected an error", cfg)
		}
	}
	if _, _, err := (&VacuumGripperConfig{IO: "box", SuctionOutput: 15, ReleaseOutput: intPtr(14)}).Validate("path"); err != nil {
		t.Error(err)
	}
}

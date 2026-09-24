package viamfairino

import (
	"context"
	"errors"
	"fmt"
	"testing"

	"go.viam.com/rdk/components/gripper"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/operation"
	"go.viam.com/rdk/resource"
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
	return &fairinoVacuumGripper{
		name: gripper.Named("vac"), logger: logger, cfg: cfg, cmd: cmd, status: sc, model: model,
		opMgr: operation.NewSingleOperationManager(),
	}, sc
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
	want := []string{"SetToolDO(1,0,0,1)", "SetToolDO(0,1,0,1)", "SetToolDO(0,0,0,1)", "SetToolDO(1,1,0,1)"}
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

// With a vacuum switch, Grab reports only what frames captured after suction
// came on say: a sensor reading from before (a part left over, a stale
// frame) must not count as this grab's hold.
func TestVacuumGrabReadsOnlyFreshFrames(t *testing.T) {
	fc := newFakeController(t)
	cfg := &VacuumGripperConfig{SuctionOutput: 0, SensorInput: intPtr(1)}
	sensorOn := ioFrame(t, 0x01, 0x02, 0, 0, 0, 0)
	g, sc := newTestVacuum(t, fc, cfg, sensorOn) // a stale frame, never refreshed
	if holding, err := g.Grab(context.Background(), nil); err == nil || holding {
		t.Errorf("stalled feed: got %v, %v; want an error", holding, err)
	}
	liveFeed(t, sc, sensorOn)
	if holding, err := g.Grab(context.Background(), nil); err != nil || !holding {
		t.Errorf("live feed with the switch closed: got %v, %v", holding, err)
	}
}

// Grab and Open are one operation at a time: an Open supersedes a Grab that
// is still waiting for the vacuum switch, and nothing of the Grab's goes out
// after the Open's.
func TestVacuumOpenSupersedesGrab(t *testing.T) {
	fc := newFakeController(t)
	cfg := &VacuumGripperConfig{SuctionOutput: 0, ReleaseOutput: intPtr(1), SensorInput: intPtr(1)}
	g, sc := newTestVacuum(t, fc, cfg, ioFrame(t, 0, 0, 0, 0, 0, 0))
	liveFeed(t, sc, ioFrame(t, 0, 0, 0, 0, 0, 0)) // the switch never closes
	grabbed := make(chan error, 1)
	go func() { _, err := g.Grab(context.Background(), nil); grabbed <- err }()
	waitFor(t, "Grab to wait for the switch", func() bool { return fc.count("SetToolDO") == 2 })
	if moving, _ := g.IsMoving(context.Background()); !moving {
		t.Error("IsMoving false while Grab waits for the vacuum switch")
	}
	if err := g.Open(context.Background(), nil); err != nil {
		t.Fatal(err)
	}
	if err := <-grabbed; err == nil {
		t.Error("superseded Grab returned no error")
	}
	fc.mu.Lock()
	got := append([]string(nil), fc.commands...)
	fc.mu.Unlock()
	want := []string{"SetToolDO(1,0,0,1)", "SetToolDO(0,1,0,1)", "SetToolDO(0,0,0,1)", "SetToolDO(1,1,0,1)"}
	if fmt.Sprint(got) != fmt.Sprint(want) {
		t.Errorf("got %v want %v", got, want)
	}
}

func TestVacuumDoCommandUnimplemented(t *testing.T) {
	g, _ := newTestVacuum(t, newFakeController(t), &VacuumGripperConfig{}, ioFrame(t, 0, 0, 0, 0, 0, 0))
	if _, err := g.DoCommand(context.Background(), map[string]any{"command": "x"}); !errors.Is(err, resource.ErrDoUnimplemented) {
		t.Errorf("got %v", err)
	}
}

package viamfairino

import (
	"context"
	"encoding/binary"
	"math"
	"testing"

	"go.viam.com/rdk/components/sensor"
	"go.viam.com/rdk/logging"
)

func newTestFTSensor(t *testing.T, fc *fakeController, activated bool) *fairinoFTSensor {
	t.Helper()
	p := buildTestPayload()
	for i := 0; i < 6; i++ {
		binary.LittleEndian.PutUint64(p[offFTData+i*8:], math.Float64bits(float64(i+1)*1.5))
	}
	if activated {
		p[offFTActStatus] = 1
	}
	fr, ok := parseStatus(p, 1)
	if !ok {
		t.Fatal("failed to build test frame")
	}
	logger := logging.NewTestLogger(t)
	sc := newStatusClient("127.0.0.1", statusPort, logger)
	setTestFrame(sc, fr)
	cmd := newCommandClient("127.0.0.1", fc.port(), logger)
	t.Cleanup(func() { _ = cmd.close() })
	return &fairinoFTSensor{name: sensor.Named("ft"), cmd: cmd, status: sc}
}

func TestFTSensorReadings(t *testing.T) {
	s := newTestFTSensor(t, newFakeController(t), true)
	got, err := s.Readings(context.Background(), nil)
	if err != nil {
		t.Fatal(err)
	}
	for i, key := range ftReadingKeys {
		if want := float64(i+1) * 1.5; got[key] != want {
			t.Errorf("%s: got %v want %v", key, got[key], want)
		}
	}
}

// An inactive sensor reports zeros; that must not pass for "no load".
func TestFTSensorReadingsRequireActivation(t *testing.T) {
	s := newTestFTSensor(t, newFakeController(t), false)
	if _, err := s.Readings(context.Background(), nil); err == nil {
		t.Error("expected an error from an inactive sensor")
	}
	st, err := s.Status(context.Background())
	if err != nil || st["activated"] != false {
		t.Errorf("Status: got %v, %v", st, err)
	}
}

func TestFTSensorDoCommand(t *testing.T) {
	fc := newFakeController(t)
	s := newTestFTSensor(t, fc, true)
	for _, command := range []string{"tare", "clear_tare", "activate", "deactivate"} {
		if _, err := s.DoCommand(context.Background(), map[string]interface{}{"command": command}); err != nil {
			t.Fatalf("%s: %v", command, err)
		}
	}
	fc.mu.Lock()
	got := append([]string(nil), fc.commands...)
	fc.mu.Unlock()
	want := []string{"FT_SetZero(1,0)", "FT_SetZero(0,0)", "FT_Activate(1)", "FT_Activate(0)"}
	for i := range want {
		if i >= len(got) || got[i] != want[i] {
			t.Fatalf("got %v want %v", got, want)
		}
	}
	if _, err := s.DoCommand(context.Background(), map[string]interface{}{"command": "nope"}); err == nil {
		t.Error("expected an error for an unknown command")
	}
}

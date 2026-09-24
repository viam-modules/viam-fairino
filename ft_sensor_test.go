package viamfairino

import (
	"context"
	"encoding/binary"
	"math"
	"net"
	"strconv"
	"testing"

	"go.viam.com/rdk/components/sensor"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/operation"
	"go.viam.com/rdk/resource"
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

// Every resource on a controller shares its connections by reference count,
// so a repeated Close must not drop a sibling's reference.
func TestWrapperClosesAreIdempotent(t *testing.T) {
	const host = "203.0.113.4" // TEST-NET-3: the shared client connects lazily, and never does here
	for name, closer := range map[string]resource.Resource{
		"gripper":        &fairinoGripper{host: host, opMgr: operation.NewSingleOperationManager()},
		"vacuum_gripper": &fairinoVacuumGripper{host: host, opMgr: operation.NewSingleOperationManager()},
		"ft_sensor":      &fairinoFTSensor{host: host},
	} {
		t.Run(name, func(t *testing.T) {
			logger := logging.NewTestLogger(t)
			acquireCommandClient(host, commandPort, logger) // this resource's
			acquireCommandClient(host, commandPort, logger) // a sibling's
			defer releaseCommandClient(host, commandPort)
			for range 2 {
				if err := closer.Close(context.Background()); err != nil {
					t.Fatal(err)
				}
			}
			sharedCommandMu.Lock()
			e, ok := sharedCommandClients[net.JoinHostPort(host, strconv.Itoa(commandPort))]
			sharedCommandMu.Unlock()
			if !ok || e.refs != 1 {
				t.Error("a second Close released the sibling's reference")
			}
		})
	}
}

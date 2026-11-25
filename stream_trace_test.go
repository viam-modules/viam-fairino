package viamfairino

import (
	"testing"
	"time"

	"go.viam.com/rdk/logging"
)

func TestStreamTraceLag(t *testing.T) {
	logger, logs := logging.NewObservedTestLogger(t)
	anchor := time.Now()
	tr := newStreamTrace(logger, anchor, [6]float64{})
	tr.sent(100*time.Millisecond, [6]float64{10}, 2*time.Millisecond, 3*time.Millisecond)

	// Commanded at 100°/s, so 5° at 50 ms; the arm is at 4°, 10 ms behind.
	tr.observe(statusFrame{FrameCount: 7, ReceivedAt: anchor.Add(50 * time.Millisecond), JointDegs: [6]float64{4}})
	tr.observe(statusFrame{FrameCount: 7, ReceivedAt: anchor.Add(60 * time.Millisecond), JointDegs: [6]float64{5}})

	entries := logs.FilterMessage("stream trace").All()
	if len(entries) != 1 {
		t.Fatalf("got %d trace lines, want 1 (a repeated frame is not logged again)", len(entries))
	}
	f := entries[0].ContextMap()
	for key, want := range map[string]float64{"t_ms": 50, "lag_ms": 10, "err_deg": 1, "late_ms": 2, "servoj_rtt_ms": 3} {
		if got, ok := f[key].(float64); !ok || got != want {
			t.Errorf("%s = %v, want %v", key, f[key], want)
		}
	}
}

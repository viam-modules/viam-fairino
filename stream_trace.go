package viamfairino

import (
	"math"
	"time"

	"go.viam.com/rdk/logging"
)

const (
	// Enough ServoJ targets to cover a frame from a 100 ms feed.
	maxTraceKnots = 64
	// Slower than this, a joint's position says too little about time to estimate lag.
	traceMinSpeedDegPerSec = 0.5
)

type traceKnot struct {
	at  time.Duration
	deg [6]float64
}

// streamTrace logs each new status frame during a stream next to where the arm was commanded
// to be when the frame arrived, so a run shows how far behind its trajectory the arm is.
type streamTrace struct {
	logger    logging.Logger
	anchor    time.Time
	knots     []traceKnot
	lastCount uint8
	logged    bool
	maxLate   time.Duration
	maxRTT    time.Duration
}

func newStreamTrace(logger logging.Logger, anchor time.Time, start [6]float64) *streamTrace {
	logger.Infow("stream trace: started", "anchor_unix_us", anchor.UnixMicro())
	return &streamTrace{logger: logger, anchor: anchor, knots: []traceKnot{{0, start}}}
}

// sent records a ServoJ toward deg due at `at`, which went out late past its slot and took rtt to be acknowledged.
func (tr *streamTrace) sent(at time.Duration, deg [6]float64, late, rtt time.Duration) {
	tr.knots = append(tr.knots, traceKnot{at, deg})
	if len(tr.knots) > maxTraceKnots {
		tr.knots = append(tr.knots[:0], tr.knots[1:]...)
	}
	tr.maxLate = max(tr.maxLate, late)
	tr.maxRTT = max(tr.maxRTT, rtt)
}

func (tr *streamTrace) observe(fr statusFrame) {
	if tr.logged && fr.FrameCount == tr.lastCount {
		return
	}
	tr.logged, tr.lastCount = true, fr.FrameCount

	at := fr.ReceivedAt.Sub(tr.anchor)
	fields := []any{
		"t_ms", toMs(at), "frame", fr.FrameCount, "act_deg", roundDegs(fr.JointDegs),
		"late_ms", toMs(tr.maxLate), "servoj_rtt_ms", toMs(tr.maxRTT),
	}
	tr.maxLate, tr.maxRTT = 0, 0

	if a, b, ok := tr.bracket(at); ok {
		frac := float64(at-a.at) / float64(b.at-a.at)
		var cmd [6]float64
		var errDeg float64
		dom := 0
		for i := range cmd {
			cmd[i] = a.deg[i] + (b.deg[i]-a.deg[i])*frac
			errDeg = max(errDeg, math.Abs(cmd[i]-fr.JointDegs[i]))
			if math.Abs(b.deg[i]-a.deg[i]) > math.Abs(b.deg[dom]-a.deg[dom]) {
				dom = i
			}
		}
		fields = append(fields, "cmd_deg", roundDegs(cmd), "err_deg", round(errDeg, 3))
		if v := (b.deg[dom] - a.deg[dom]) / (b.at - a.at).Seconds(); math.Abs(v) >= traceMinSpeedDegPerSec {
			fields = append(fields, "lag_ms", round((cmd[dom]-fr.JointDegs[dom])/v*1000, 1))
		}
	}
	tr.logger.Infow("stream trace", fields...)
}

// bracket returns the commanded targets either side of at.
func (tr *streamTrace) bracket(at time.Duration) (traceKnot, traceKnot, bool) {
	for i := len(tr.knots) - 1; i > 0; i-- {
		if tr.knots[i-1].at <= at && at <= tr.knots[i].at {
			return tr.knots[i-1], tr.knots[i], true
		}
	}
	return traceKnot{}, traceKnot{}, false
}

func toMs(d time.Duration) float64 {
	return round(float64(d)/float64(time.Millisecond), 2)
}

func roundDegs(d [6]float64) [6]float64 {
	for i := range d {
		d[i] = round(d[i], 3)
	}
	return d
}

func round(x float64, places int) float64 {
	p := math.Pow(10, float64(places))
	return math.Round(x*p) / p
}

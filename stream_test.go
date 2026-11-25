package viamfairino

import (
	"context"
	"errors"
	"slices"
	"strings"
	"testing"
	"time"

	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/referenceframe"
	rdkutils "go.viam.com/rdk/utils"
)

func (f *fakeCommandPort) servoJs() ([]string, []time.Time) {
	f.mu.Lock()
	defer f.mu.Unlock()
	var data []string
	var at []time.Time
	for i, id := range f.received {
		if id == cmdIDServoJ {
			data = append(data, f.data[i])
			at = append(at, f.at[i])
		}
	}
	return data, at
}

func degPoint(t time.Duration, deg ...float64) arm.TrajectoryPoint {
	in := make([]referenceframe.Input, len(deg))
	for i, d := range deg {
		in[i] = rdkutils.DegToRad(d)
	}
	return arm.TrajectoryPoint{Time: t, Positions: in}
}

func joint1At(t time.Duration, deg float64) arm.TrajectoryPoint {
	return degPoint(t, deg, 0, 0, 0, 0, 0)
}

func restingPoints(n int) []arm.TrajectoryPoint {
	points := make([]arm.TrajectoryPoint, n)
	for k := range points {
		points[k] = joint1At(time.Duration(k)*8*time.Millisecond, 0)
	}
	return points
}

func startStream(s *viamFairinoFairino, points []arm.TrajectoryPoint) (<-chan error, chan arm.Response) {
	return startStreamExtra(s, points, nil)
}

func startStreamExtra(s *viamFairinoFairino, points []arm.TrajectoryPoint, extra map[string]interface{}) (<-chan error, chan arm.Response) {
	batches := make(chan []arm.TrajectoryPoint, 1)
	responses := make(chan arm.Response, 16)
	errCh := make(chan error, 1)
	go func() { errCh <- s.MoveThroughJointPositionsStreamed(context.Background(), batches, responses, extra) }()
	batches <- points
	close(batches)
	return errCh, responses
}

func TestFormatServoJ(t *testing.T) {
	got := formatServoJ([6]float64{-47.195, -173.82, 104.056, -110.23, 137.199, 134.998}, 8*time.Millisecond)
	want := "ServoJ(-47.1950,-173.8200,104.0560,-110.2300,137.1990,134.9980,0.000,0.000,0.000,0.000,0,0,0.0080,0,0)"
	if got != want {
		t.Errorf("got  %s\nwant %s", got, want)
	}
}

func TestStreamSendsOneServoJPerPointOnSchedule(t *testing.T) {
	f := newFakeCommandPort(t)
	s := newMotionTestArm(t, f, atRestFrame, Config{})

	errCh, responses := startStream(s, restingPoints(11))
	if err := waitErr(t, errCh); err != nil {
		t.Fatalf("stream: %v", err)
	}

	data, at := f.servoJs()
	if len(data) != 10 {
		t.Fatalf("sent %d ServoJs, want 10 (every point but t=0)", len(data))
	}
	for _, d := range data {
		if !strings.HasSuffix(d, ",0.0080,0,0)") {
			t.Errorf("ServoJ %q does not carry the 8 ms gap as its interval", d)
		}
	}
	if span := at[9].Sub(at[0]); span < 70*time.Millisecond || span > 150*time.Millisecond {
		t.Errorf("ServoJs spanned %v, want about 72 ms (the trajectory's own pace)", span)
	}
	if n := len(responses); n != 1 {
		t.Errorf("got %d acks, want 1 per batch", n)
	}
	if f.count(cmdIDStop) != 0 {
		t.Errorf("a clean stream sent STOP: %v", f.commands())
	}
}

func TestStreamWaitsForStartAt(t *testing.T) {
	f := newFakeCommandPort(t)
	s := newMotionTestArm(t, f, atRestFrame, Config{})

	startAt := time.Now().Add(150 * time.Millisecond)
	errCh, _ := startStreamExtra(s, restingPoints(3), map[string]interface{}{StreamStartAtKey: float64(startAt.UnixMicro())})
	if err := waitErr(t, errCh); err != nil {
		t.Fatalf("stream: %v", err)
	}

	_, at := f.servoJs()
	if len(at) != 2 {
		t.Fatalf("sent %d ServoJs, want 2", len(at))
	}
	if early := startAt.Sub(at[0]); early > time.Millisecond {
		t.Errorf("first ServoJ went out %v before start_at", early)
	}
	if late := at[0].Sub(startAt); late > 20*time.Millisecond {
		t.Errorf("first ServoJ went out %v after start_at", late)
	}
}

func TestStreamRefusesMissedStartAt(t *testing.T) {
	f := newFakeCommandPort(t)
	s := newMotionTestArm(t, f, atRestFrame, Config{})

	startAt := time.Now().Add(-50 * time.Millisecond)
	errCh, _ := startStreamExtra(s, restingPoints(3), map[string]interface{}{StreamStartAtKey: float64(startAt.UnixMicro())})
	err := waitErr(t, errCh)
	if err == nil || !strings.Contains(err.Error(), StreamStartAtKey) {
		t.Fatalf("got %v, want an error about the missed %s", err, StreamStartAtKey)
	}
	if data, _ := f.servoJs(); len(data) != 0 {
		t.Errorf("sent %d ServoJs after missing start_at, want none", len(data))
	}
}

func TestStreamSplitsLongGaps(t *testing.T) {
	f := newFakeCommandPort(t)
	s := newMotionTestArm(t, f, atRestFrame, Config{})

	errCh, _ := startStream(s, []arm.TrajectoryPoint{joint1At(0, 0), joint1At(20*time.Millisecond, 0.2), joint1At(40*time.Millisecond, 0)})
	if err := waitErr(t, errCh); err != nil {
		t.Fatalf("stream: %v", err)
	}

	data, _ := f.servoJs()
	if len(data) != 6 {
		t.Fatalf("sent %d ServoJs, want 3 per 20 ms gap", len(data))
	}
	for i, prefix := range []string{"ServoJ(0.0667,", "ServoJ(0.1333,", "ServoJ(0.2000,"} {
		if !strings.HasPrefix(data[i], prefix) || !strings.HasSuffix(data[i], ",0.0067,0,0)") {
			t.Errorf("ServoJ %d = %q, want %s... with a 6.7 ms interval", i, data[i], prefix)
		}
	}
}

func TestStreamRejectsBadPointsBeforeMoving(t *testing.T) {
	moving := joint1At(0, 0)
	moving.Constraints = &arm.KinematicConstraints{Velocities: []float64{0.1, 0, 0, 0, 0, 0}}

	for _, tc := range []struct {
		name   string
		points []arm.TrajectoryPoint
		want   string
	}{
		{"first point after zero", []arm.TrajectoryPoint{joint1At(8*time.Millisecond, 0)}, "time zero"},
		{"first point moving", []arm.TrajectoryPoint{moving}, "at rest"},
		{"wrong joint count", []arm.TrajectoryPoint{degPoint(0, 0, 0, 0, 0, 0)}, "5 joint positions"},
		{"outside joint limits", []arm.TrajectoryPoint{joint1At(0, 180)}, "outside"},
		{"not where the arm is", []arm.TrajectoryPoint{joint1At(0, 1)}, "not where the arm is"},
		{"time not increasing", []arm.TrajectoryPoint{joint1At(0, 0), joint1At(0, 0)}, "strictly increase"},
		{"jump", []arm.TrajectoryPoint{joint1At(0, 0), joint1At(8*time.Millisecond, 2)}, "faster than"},
	} {
		t.Run(tc.name, func(t *testing.T) {
			f := newFakeCommandPort(t)
			s := newMotionTestArm(t, f, atRestFrame, Config{})

			errCh, _ := startStream(s, tc.points)
			if err := waitErr(t, errCh); err == nil || !strings.Contains(err.Error(), tc.want) {
				t.Errorf("got %v, want an error containing %q", err, tc.want)
			}
			if got := f.commands(); len(got) != 0 {
				t.Errorf("controller received %v, want nothing", got)
			}
		})
	}
}

func TestStreamBadPointMidwaySendsStop(t *testing.T) {
	f := newFakeCommandPort(t)
	s := newMotionTestArm(t, f, atRestFrame, Config{})

	errCh, _ := startStream(s, []arm.TrajectoryPoint{joint1At(0, 0), joint1At(8*time.Millisecond, 0), joint1At(16*time.Millisecond, 2)})
	if err := waitErr(t, errCh); err == nil {
		t.Fatal("expected the jump to be rejected")
	}
	if got, want := f.commands(), []int{cmdIDServoJ, cmdIDStop}; !slices.Equal(got, want) {
		t.Errorf("got commands %v, want %v", got, want)
	}
}

func TestStreamReportsArmNotReachingLastPoint(t *testing.T) {
	f := newFakeCommandPort(t)
	s := newMotionTestArm(t, f, atRestFrame, Config{})

	errCh, _ := startStream(s, []arm.TrajectoryPoint{joint1At(0, 0), joint1At(8*time.Millisecond, 0.8)})
	if err := waitErr(t, errCh); err == nil {
		t.Fatal("expected an error: the status feed never shows the arm at 0.8°")
	}
	if cmds := f.commands(); cmds[len(cmds)-1] != cmdIDStop {
		t.Errorf("last command was %d, want STOP; got %v", cmds[len(cmds)-1], cmds)
	}
}

func TestStopCancelsStream(t *testing.T) {
	f := newFakeCommandPort(t)
	s := newMotionTestArm(t, f, atRestFrame, Config{})

	errCh, _ := startStream(s, restingPoints(500))
	f.waitFor(t, cmdIDServoJ)
	if err := s.Stop(context.Background(), nil); err != nil {
		t.Fatalf("Stop: %v", err)
	}
	if err := waitErr(t, errCh); !errors.Is(err, context.Canceled) {
		t.Errorf("stream returned %v, want context.Canceled", err)
	}
	cmds := f.commands()
	if cmds[len(cmds)-1] != cmdIDStop {
		t.Errorf("last command was %d, want STOP; got %v", cmds[len(cmds)-1], cmds)
	}
	if n := f.count(cmdIDServoJ); n >= 499 {
		t.Errorf("sent %d ServoJs; Stop did not cut the stream short", n)
	}
}

func TestStreamAbortsOnEStop(t *testing.T) {
	f := newFakeCommandPort(t)
	s := newMotionTestArm(t, f, atRestFrame, Config{})

	errCh, _ := startStream(s, restingPoints(500))
	f.waitFor(t, cmdIDServoJ)
	s.status.mu.Lock()
	s.status.frame.EmergencyStop = true
	s.status.mu.Unlock()

	if err := waitErr(t, errCh); err == nil || !strings.Contains(err.Error(), "e-stopped") {
		t.Errorf("stream returned %v, want an e-stop error", err)
	}
	cmds := f.commands()
	if cmds[len(cmds)-1] != cmdIDStop {
		t.Errorf("last command was %d, want STOP; got %v", cmds[len(cmds)-1], cmds)
	}
}

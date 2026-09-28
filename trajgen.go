package viamfairino

import (
	"context"
	"errors"
	"fmt"
	"math"
	"time"

	arm "go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/ml"
	"go.viam.com/rdk/referenceframe"
	rdkutils "go.viam.com/rdk/utils"
	"gorgonia.org/tensor"
)

// TrajGenConfig routes MoveThroughJointPositions (and so GoToInputs and
// MoveToPosition) through an ML model service that generates time-optimal
// trajectories, such as trajex, executed as a ServoJ stream. Same inputs and
// defaults as viam-ufactory-xarm's trajectory_generator.
type TrajGenConfig struct {
	Service                            string   `json:"service"`
	PathToleranceDeltaRads             *float64 `json:"path_tolerance_delta_rads,omitempty"`
	PathColinearizationRatio           *float64 `json:"path_colinearization_ratio,omitempty"`
	WaypointDeduplicationToleranceRads *float64 `json:"waypoint_deduplication_tolerance_rads,omitempty"`

	// MaxAccelerationDegsPerSec2 bounds joint acceleration in the generated
	// trajectory. ServoJ follows positions directly, so this is the only
	// acceleration limit the motion gets. Default 180 (full speed in 1 s).
	MaxAccelerationDegsPerSec2 float64 `json:"max_acceleration_degs_per_sec_per_sec,omitempty"`
}

const (
	defaultTrajGenPathToleranceDeltaRads     = 0.1
	defaultTrajGenWaypointDedupToleranceRads = 1e-3
	defaultTrajGenMaxAccelerationDegsPerSec2 = 180.0
	trajGenSampleHz                          = 125 // one ServoJ every 8 ms, within maxServoIntervalSec
)

func (c *TrajGenConfig) validate(path string) error {
	if c.Service == "" {
		return fmt.Errorf("%s: trajectory_generator.service is required", path)
	}
	if c.MaxAccelerationDegsPerSec2 < 0 {
		return fmt.Errorf("%s: trajectory_generator.max_acceleration_degs_per_sec_per_sec cannot be negative", path)
	}
	return nil
}

func floatOr(p *float64, def float64) float64 {
	if p == nil {
		return def
	}
	return *p
}

func scalar(v float64) *tensor.Dense {
	return tensor.New(tensor.WithShape(1), tensor.WithBacking([]float64{v}))
}

// generateTrajectory asks the trajectory generator for a trajectory from the
// current position through waypoints, sampled at trajGenSampleHz, with each
// point timed by the generator's own sample_times_sec. It returns no points
// when there is nowhere to go: trajex refuses a path without two distinct
// waypoints, so waypoints within the deduplication tolerance of the one before
// (starting from the current position) are dropped here first, the same way.
func (s *viamFairinoFairino) generateTrajectory(
	ctx context.Context, waypoints [][]referenceframe.Input, speedPct float64,
) ([]arm.TrajectoryPoint, error) {
	current, err := s.JointPositions(ctx, nil)
	if err != nil {
		return nil, err
	}
	c := s.cfg.TrajGen
	dedupTol := floatOr(c.WaypointDeduplicationToleranceRads, defaultTrajGenWaypointDedupToleranceRads)
	path := [][]referenceframe.Input{current}
	for _, wp := range waypoints {
		if referenceframe.InputsLinfDistance(path[len(path)-1], wp) > dedupTol {
			path = append(path, wp)
		}
	}
	if len(path) < 2 {
		return nil, nil // already there
	}

	const dof = 6
	flat := make([]float64, 0, len(path)*dof)
	for _, wp := range path {
		flat = append(flat, wp...)
	}
	vel := rdkutils.DegToRad(speedDegsFromPercent(speedPct))
	acc := rdkutils.DegToRad(c.MaxAccelerationDegsPerSec2)
	if acc == 0 {
		acc = rdkutils.DegToRad(defaultTrajGenMaxAccelerationDegsPerSec2)
	}
	velLimits, accLimits := make([]float64, dof), make([]float64, dof)
	for i := range velLimits {
		velLimits[i], accLimits[i] = vel, acc
	}

	// Every scalar is float64: trajex rejects an input of the wrong dtype.
	out, err := s.trajGen.Infer(ctx, ml.Tensors{
		"waypoints_rads":                        tensor.New(tensor.WithShape(len(path), dof), tensor.WithBacking(flat)),
		"velocity_limits_rads_per_sec":          tensor.New(tensor.WithShape(dof), tensor.WithBacking(velLimits)),
		"acceleration_limits_rads_per_sec2":     tensor.New(tensor.WithShape(dof), tensor.WithBacking(accLimits)),
		"path_tolerance_delta_rads":             scalar(floatOr(c.PathToleranceDeltaRads, defaultTrajGenPathToleranceDeltaRads)),
		"path_colinearization_ratio":            scalar(floatOr(c.PathColinearizationRatio, 0)),
		"waypoint_deduplication_tolerance_rads": scalar(dedupTol),
		"trajectory_sampling_freq_hz":           scalar(float64(trajGenSampleHz)),
	})
	if err != nil {
		return nil, fmt.Errorf("trajectory generator: %w", err)
	}
	configs, okConfigs := out["configurations_rads"]
	times, okTimes := out["sample_times_sec"]
	if !okConfigs || !okTimes {
		return nil, errors.New("trajectory generator returned no configurations_rads/sample_times_sec")
	}
	data, okData := configs.Data().([]float64)
	timeData, okTimeData := times.Data().([]float64)
	if !okData || len(configs.Shape()) != 2 || configs.Shape()[1] != dof || !okTimeData || len(timeData) != configs.Shape()[0] {
		return nil, fmt.Errorf("trajectory generator returned configurations_rads of shape %v and sample_times_sec of shape %v, "+
			"want (N, %d) and (N) float64", configs.Shape(), times.Shape(), dof)
	}
	points := make([]arm.TrajectoryPoint, 0, len(timeData))
	for i, t := range timeData {
		p := arm.TrajectoryPoint{
			Time:      time.Duration(math.Round(t * float64(time.Second))),
			Positions: data[i*dof : (i+1)*dof],
		}
		points = foldCloseSample(points, p)
	}
	return points, nil
}

// foldCloseSample appends p to points unless it comes less than
// minServoIntervalSec after the last one, which ServoJ can't be given. The
// generator's last sample is the final waypoint at the trajectory's exact
// duration, rarely a whole sample period after the one before, so this is
// routinely hit there. Such a sample replaces the previous one instead, whose
// interval it takes over (at most one sample period plus 1 ms, well within
// maxServoIntervalSec), keeping the final waypoint and its time; after the
// start, which must stay, it is pushed back to the minimum instead.
func foldCloseSample(points []arm.TrajectoryPoint, p arm.TrajectoryPoint) []arm.TrajectoryPoint {
	minGap := time.Duration(minServoIntervalSec * float64(time.Second))
	n := len(points)
	switch {
	case n == 0 || p.Time-points[n-1].Time >= minGap:
		return append(points, p)
	case n == 1:
		p.Time = points[0].Time + minGap
		return append(points, p)
	default:
		points[n-1] = p
		return points
	}
}

// moveWithTrajGen executes waypoints as a generated trajectory, every sample
// a ServoJ setpoint. The whole trajectory exists before the arm moves, so all
// of it is checked first (limits, start, step speed, spacing — see
// servoStreamer.check): one that breaks a limit partway is refused outright
// rather than cut off mid-motion. The caller owns the operation.
func (s *viamFairinoFairino) moveWithTrajGen(ctx context.Context, waypoints [][]referenceframe.Input, speedPct float64) error {
	points, err := s.generateTrajectory(ctx, waypoints, speedPct)
	if err != nil || len(points) == 0 {
		return err
	}
	dryRun := &servoStreamer{s: s}
	for _, p := range points {
		if err := dryRun.check(ctx, p); err != nil {
			return fmt.Errorf("generated trajectory: %w", err)
		}
	}
	st := &servoStreamer{s: s}
	err = func() error {
		for _, p := range points {
			if err := st.send(ctx, p); err != nil {
				return err
			}
		}
		return st.finish(ctx)
	}()
	if err != nil && st.sent {
		s.stopDetached()
	}
	return err
}

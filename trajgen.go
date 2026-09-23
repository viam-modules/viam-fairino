package viamfairino

import (
	"context"
	"fmt"
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

func scalar[T float64 | int64](v T) *tensor.Dense {
	return tensor.New(tensor.WithShape(1), tensor.WithBacking([]T{v}))
}

// generateTrajectory asks the trajectory generator for samples from the current
// position through waypoints, at trajGenSampleHz. It returns nil when the
// generator reports nothing to do (fewer than two distinct waypoints).
func (s *viamFairinoFairino) generateTrajectory(
	ctx context.Context, waypoints [][]referenceframe.Input, speedPct float64,
) ([][]float64, error) {
	current, err := s.JointPositions(ctx, nil)
	if err != nil {
		return nil, err
	}
	const dof = 6
	flat := make([]float64, 0, (len(waypoints)+1)*dof)
	flat = append(flat, current...)
	for _, wp := range waypoints {
		flat = append(flat, wp...)
	}
	vel := rdkutils.DegToRad(speedDegsFromPercent(speedPct))
	acc := rdkutils.DegToRad(s.cfg.TrajGen.MaxAccelerationDegsPerSec2)
	if acc == 0 {
		acc = rdkutils.DegToRad(defaultTrajGenMaxAccelerationDegsPerSec2)
	}
	velLimits, accLimits := make([]float64, dof), make([]float64, dof)
	for i := range velLimits {
		velLimits[i], accLimits[i] = vel, acc
	}

	c := s.cfg.TrajGen
	out, err := s.trajGen.Infer(ctx, ml.Tensors{
		"waypoints_rads":                        tensor.New(tensor.WithShape(len(waypoints)+1, dof), tensor.WithBacking(flat)),
		"velocity_limits_rads_per_sec":          tensor.New(tensor.WithShape(dof), tensor.WithBacking(velLimits)),
		"acceleration_limits_rads_per_sec2":     tensor.New(tensor.WithShape(dof), tensor.WithBacking(accLimits)),
		"path_tolerance_delta_rads":             scalar(floatOr(c.PathToleranceDeltaRads, defaultTrajGenPathToleranceDeltaRads)),
		"path_colinearization_ratio":            scalar(floatOr(c.PathColinearizationRatio, 0)),
		"waypoint_deduplication_tolerance_rads": scalar(floatOr(c.WaypointDeduplicationToleranceRads, defaultTrajGenWaypointDedupToleranceRads)),
		"trajectory_sampling_freq_hz":           scalar(int64(trajGenSampleHz)),
	})
	if err != nil {
		return nil, fmt.Errorf("trajectory generator: %w", err)
	}
	configs, ok := out["configurations_rads"]
	if !ok {
		return nil, nil // already at the goal
	}
	data, ok := configs.Data().([]float64)
	if !ok || len(configs.Shape()) != 2 || configs.Shape()[1] != dof {
		return nil, fmt.Errorf("trajectory generator returned configurations_rads of shape %v, want (N, %d) float64", configs.Shape(), dof)
	}
	samples := make([][]float64, configs.Shape()[0])
	for i := range samples {
		samples[i] = data[i*dof : (i+1)*dof]
	}
	return samples, nil
}

// moveWithTrajGen executes waypoints as a generated trajectory: every sample
// becomes a ServoJ point, 1/trajGenSampleHz apart, each checked against the
// joint limits as it goes out. The caller owns the operation.
func (s *viamFairinoFairino) moveWithTrajGen(ctx context.Context, waypoints [][]referenceframe.Input, speedPct float64) error {
	samples, err := s.generateTrajectory(ctx, waypoints, speedPct)
	if err != nil || len(samples) == 0 {
		return err
	}
	period := time.Second / trajGenSampleHz
	points := make([]arm.TrajectoryPoint, len(samples))
	for i, sample := range samples {
		points[i] = arm.TrajectoryPoint{Time: time.Duration(i) * period, Positions: sample}
	}
	batches := make(chan []arm.TrajectoryPoint, 1)
	batches <- points
	close(batches)
	return s.runServoStream(ctx, batches, make(chan arm.Response, 1))
}

package viamfairino

import (
	"fmt"

	arm "go.viam.com/rdk/components/arm"
	rdkutils "go.viam.com/rdk/utils"
)

// MoveJ's speed field is a percentage, 0-100; 0 would never move.
const (
	minSpeedPercent = 1.0
	maxSpeedPercent = 100.0
)

func speedPercentFromDegs(degsPerSec float64) float64 {
	return degsPerSec / maxJointSpeedDegsPerSec * 100
}

func speedDegsFromPercent(pct float64) float64 {
	return pct / 100 * maxJointSpeedDegsPerSec
}

// speedPercent is the configured default speed: speed_degs_per_sec when set,
// else joint_speed_percent, else defaultJointSpeedPercent.
func (cfg *Config) speedPercent() float64 {
	if cfg.SpeedDegsPerSec > 0 {
		return speedPercentFromDegs(cfg.SpeedDegsPerSec)
	}
	return float64(orDefaultInt(cfg.JointSpeedPercent, defaultJointSpeedPercent))
}

// currentSpeedPercent is the speed moves use unless a call overrides it: the
// configured one, or whatever set_speed last changed it to.
func (s *viamFairinoFairino) currentSpeedPercent() float64 {
	s.speedMu.Lock()
	defer s.speedMu.Unlock()
	if s.speedPct > 0 {
		return s.speedPct
	}
	return s.cfg.speedPercent()
}

func (s *viamFairinoFairino) setSpeedDegsPerSec(degsPerSec float64) error {
	if degsPerSec <= 0 || degsPerSec > maxJointSpeedDegsPerSec {
		return fmt.Errorf("speed must be in (0, %v] degrees per second, got %v", maxJointSpeedDegsPerSec, degsPerSec)
	}
	s.speedMu.Lock()
	defer s.speedMu.Unlock()
	s.speedPct = speedPercentFromDegs(degsPerSec)
	return nil
}

// moveSpeedPercent resolves one move's MoveJ speed. Per-call settings override
// the current speed, in the same order viam-ufactory-xarm applies them:
// MoveOptions first, then extra's speed_r/speed_d. MoveJ takes a single speed
// for all joints, so per-joint limits are honored by moving at the slowest.
func (s *viamFairinoFairino) moveSpeedPercent(opts *arm.MoveOptions, extra map[string]interface{}) float64 {
	pct := s.currentSpeedPercent()
	if opts != nil {
		if opts.MaxVelRads > 0 {
			pct = speedPercentFromDegs(rdkutils.RadToDeg(opts.MaxVelRads))
		}
		slowest := 0.0
		for _, v := range opts.MaxVelRadsJoints {
			if v > 0 && (slowest == 0 || v < slowest) {
				slowest = v
			}
		}
		if slowest > 0 {
			pct = speedPercentFromDegs(rdkutils.RadToDeg(slowest))
		}
	}
	if v, ok := floatArg(extra, "speed_r"); ok {
		pct = speedPercentFromDegs(rdkutils.RadToDeg(v))
	}
	if v, ok := floatArg(extra, "speed_d"); ok {
		pct = speedPercentFromDegs(v)
	}

	switch {
	case pct < minSpeedPercent:
		s.logger.Warnf("requested joint speed %.2f%% is below the minimum, using %.0f%%", pct, minSpeedPercent)
		return minSpeedPercent
	case pct > maxSpeedPercent:
		s.logger.Warnf("requested joint speed %.2f%% exceeds the maximum, using %.0f%%", pct, maxSpeedPercent)
		return maxSpeedPercent
	}
	return pct
}

// floatArg reads a numeric value from a DoCommand/extra map, which arrive as
// float64 over the wire but may be ints when built in Go.
func floatArg(m map[string]interface{}, key string) (float64, bool) {
	switch v := m[key].(type) {
	case float64:
		return v, true
	case int:
		return float64(v), true
	default:
		return 0, false
	}
}

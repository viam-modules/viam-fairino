package main

import (
	arm "go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/components/gripper"
	"go.viam.com/rdk/components/sensor"
	"go.viam.com/rdk/module"
	"go.viam.com/rdk/resource"
	"viamfairino"
)

func main() {
	// ModularMain can take multiple APIModel arguments, if your module implements multiple models.
	module.ModularMain(
		resource.APIModel{API: arm.API, Model: viamfairino.Fairino},
		resource.APIModel{API: gripper.API, Model: viamfairino.Gripper},
		resource.APIModel{API: sensor.API, Model: viamfairino.FTSensor},
	)
}

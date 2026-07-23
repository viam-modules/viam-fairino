package main

import (
	"viamfairino"
	"go.viam.com/rdk/module"
	"go.viam.com/rdk/resource"
	arm "go.viam.com/rdk/components/arm"
)

func main() {
	// ModularMain can take multiple APIModel arguments, if your module implements multiple models.
	module.ModularMain(resource.APIModel{ arm.API, viamfairino.Fairino})
}

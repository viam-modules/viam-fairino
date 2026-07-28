package main

import (
	arm "go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/module"
	"go.viam.com/rdk/resource"
	"viamfairino"
)

func main() {
	// ModularMain can take multiple APIModel arguments, if your module implements multiple models.
	module.ModularMain(resource.APIModel{arm.API, viamfairino.Fairino})
}

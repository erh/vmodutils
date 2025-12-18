package touch

import (
	"context"
	"testing"

	toggleswitch "go.viam.com/rdk/components/switch"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/robot/framesystem"
	"go.viam.com/rdk/testutils/inject"
	injectMotion "go.viam.com/rdk/testutils/inject/motion"
	"go.viam.com/test"

	"github.com/erh/vmodutils"
)

func TestMultiArmPositionSwitchValidate(t *testing.T) {
	const path = "components.0"
	const armDep = "armDep"
	const motionDep = "motionDep"
	const visionDep1 = "visionDep1"
	const visionDep2 = "visionDep2"

	makeValidConfig := func() *MultiArmPositionSwitchConfig {
		return &MultiArmPositionSwitchConfig{
			Arm: armDep,
			JointsList: [][]float64{
				{0.0, 0.0, 0.0},
				{1.0, 1.0, 1.0},
			},
			Motion:         motionDep,
			VisionServices: []string{visionDep1, visionDep2},
		}
	}

	t.Run("succeeds with basic config", func(t *testing.T) {
		cfg := makeValidConfig()
		reqDeps, optDeps, err := cfg.Validate(path)
		test.That(t, err, test.ShouldBeNil)
		test.That(t, reqDeps, test.ShouldResemble, []string{armDep, motionDep, visionDep1, visionDep2})
		test.That(t, optDeps, test.ShouldBeNil)
	})

	type testCase struct {
		name      string
		modify    func(*MultiArmPositionSwitchConfig)
		expectErr error
	}

	testCases := []testCase{
		{
			name: "missing arm",
			modify: func(c *MultiArmPositionSwitchConfig) {
				c.Arm = ""
			},
			expectErr: resource.NewConfigValidationFieldRequiredError(path, "arm"),
		},
		{
			name: "extra contains 'goal_state'",
			modify: func(c *MultiArmPositionSwitchConfig) {
				c.Extra = map[string]any{extraParamsKeyGoalState: "some_value"}
			},
			expectErr: ErrCannotSpecifyGoalStateInExtra,
		},
	}

	for _, tc := range testCases {
		t.Run("testing config: "+tc.name, func(t *testing.T) {
			cfg := makeValidConfig()
			tc.modify(cfg)
			_, _, err := cfg.Validate(path)
			if tc.expectErr == nil {
				test.That(t, err, test.ShouldBeNil)
			} else {
				test.That(t, err, test.ShouldNotBeNil)
				test.That(t, err, test.ShouldBeError, tc.expectErr)
			}
		})
	}
}

func TestMuiltiArmPositionSwitchConstructor(t *testing.T) {
	const path = "components.0"
	ctx := context.Background()
	logger := logging.NewTestLogger(t)

	// create injected resources with distinct labels
	fakeArm := inject.NewArm("arm")
	fakeMotion := injectMotion.NewMotionService("builtin")
	fakeVision1 := inject.NewVisionService("vision1")
	fakeVision2 := inject.NewVisionService("vision2")
	fakeFsSvc := inject.NewFrameSystemService(framesystem.PublicServiceName.Name)

	allDeps := resource.Dependencies{
		fakeArm.Name():     fakeArm,
		fakeMotion.Name():  fakeMotion,
		fakeVision1.Name(): fakeVision1,
		fakeVision2.Name(): fakeVision2,
		fakeFsSvc.Name():   fakeFsSvc,
	}
	baseConfig := &MultiArmPositionSwitchConfig{
		Arm:            "arm",
		Motion:         "builtin",
		VisionServices: []string{"vision1", "vision2"},
	}
	_, _, err := baseConfig.Validate(path)
	test.That(t, err, test.ShouldBeNil)

	cfg := resource.Config{
		Name: "multi_arm_position_switch",
		API:  toggleswitch.API,
		Model: resource.Model{
			Family: vmodutils.NamespaceFamily,
		},
		ConvertedAttributes: baseConfig,
	}

	t.Run("succeeds with basic config", func(t *testing.T) {
		res, err := newMultiArmPositionSwitch(ctx, allDeps, cfg, logger)
		test.That(t, err, test.ShouldBeNil)
		test.That(t, res, test.ShouldNotBeNil)

		s, ok := res.(*MultiArmPositionSwitch)
		test.That(t, ok, test.ShouldBeTrue)

		test.That(t, s.cfg.Motion, test.ShouldEqual, resource.DefaultServiceName)
	})

	/*
		withConfig := func(mod func(*MultiArmPositionSwitchConfig)) resource.Config {
			c := *baseConfig
			mod(&c)
			newCfg := cfg
			newCfg.ConvertedAttributes = &c
			return newCfg
		}
	*/

}

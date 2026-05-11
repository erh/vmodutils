package touch

import (
	"context"
	"errors"
	"fmt"
	"math"
	"testing"

	"github.com/golang/geo/r3"
	toggleswitch "go.viam.com/rdk/components/switch"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/robot/framesystem"
	"go.viam.com/rdk/services/motion"
	"go.viam.com/rdk/spatialmath"
	"go.viam.com/rdk/testutils/inject"
	injectMotion "go.viam.com/rdk/testutils/inject/motion"
	"go.viam.com/rdk/vision"
	"go.viam.com/test"

	"github.com/erh/vmodutils"
)

var dummyErr = errors.New("dummy")

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
		{
			name: "no joint positions specified",
			modify: func(c *MultiArmPositionSwitchConfig) {
				c.JointsList = [][]float64{}
			},
			expectErr: ErrMustSpecifyAtLeastOneJointPosition,
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

func TestMultiArmPositionSwitchSetPositionAndGetPosition(t *testing.T) {
	const path = "components.0"
	ctx := context.Background()
	logger := logging.NewTestLogger(t)

	t.Run("SetPosition uses motion.Move and vision services when motion service is configured", func(t *testing.T) {
		fakeArm := inject.NewArm("arm")
		fakeArmMoveToJointPositionsCallCount := 0
		fakeArm.MoveToJointPositionsFunc = func(ctx context.Context, joints []float64, extra map[string]any) error {
			fakeArmMoveToJointPositionsCallCount++
			return nil
		}

		fakeMotion := injectMotion.NewMotionService("builtin")
		fakeMotionMoveCallCount := 0
		fakeMotion.MoveFunc = func(ctx context.Context, req motion.MoveReq) (bool, error) {
			fakeMotionMoveCallCount++
			return true, nil
		}

		fakeVision1 := inject.NewVisionService("vision1")
		fakeVision1GetObjectPointCloudsCallCount := 0
		fakeVision1.GetObjectPointCloudsFunc = func(ctx context.Context, cameraName string, extra map[string]interface{}) ([]*vision.Object, error) {
			fakeVision1GetObjectPointCloudsCallCount++
			return nil, nil
		}

		fakeVision2 := inject.NewVisionService("vision2")
		fakeVision2GetObjectPointCloudsCallCount := 0
		fakeVision2.GetObjectPointCloudsFunc = func(ctx context.Context, cameraName string, extra map[string]interface{}) ([]*vision.Object, error) {
			fakeVision2GetObjectPointCloudsCallCount++
			return nil, nil
		}

		fakeFsSvc := inject.NewFrameSystemService(framesystem.PublicServiceName.Name)
		fakeFsSvc.FrameSystemConfigFunc = func(ctx context.Context) (*framesystem.Config, error) {
			return mustTestFSConfigArm3DOF(t), nil
		}

		allDeps := resource.Dependencies{
			fakeArm.Name():     fakeArm,
			fakeMotion.Name():  fakeMotion,
			fakeVision1.Name(): fakeVision1,
			fakeVision2.Name(): fakeVision2,
			fakeFsSvc.Name():   fakeFsSvc,
		}

		baseConfig := &MultiArmPositionSwitchConfig{
			Arm:            "arm",
			JointsList:     [][]float64{{0.0, 0.0, 0.0}, {1.0, 1.0, 1.0}},
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

		res, err := newMultiArmPositionSwitch(ctx, allDeps, cfg, logger)
		test.That(t, err, test.ShouldBeNil)
		s, ok := res.(*MultiArmPositionSwitch)
		test.That(t, ok, test.ShouldBeTrue)
		test.That(t, s, test.ShouldNotBeNil)

		err = s.SetPosition(ctx, 0, nil)
		test.That(t, err, test.ShouldBeNil)
		test.That(t, fakeArmMoveToJointPositionsCallCount, test.ShouldEqual, 0)
		test.That(t, fakeMotionMoveCallCount, test.ShouldEqual, 1)
		test.That(t, fakeVision1GetObjectPointCloudsCallCount, test.ShouldEqual, 1)
		test.That(t, fakeVision2GetObjectPointCloudsCallCount, test.ShouldEqual, 1)

		position, err := s.GetPosition(ctx, nil)
		test.That(t, err, test.ShouldBeNil)
		test.That(t, position, test.ShouldEqual, 0)

		err = s.SetPosition(ctx, 1, nil)
		test.That(t, err, test.ShouldBeNil)
		test.That(t, fakeArmMoveToJointPositionsCallCount, test.ShouldEqual, 0)
		test.That(t, fakeMotionMoveCallCount, test.ShouldEqual, 2)
		test.That(t, fakeVision1GetObjectPointCloudsCallCount, test.ShouldEqual, 2)
		test.That(t, fakeVision2GetObjectPointCloudsCallCount, test.ShouldEqual, 2)

		position, err = s.GetPosition(ctx, nil)
		test.That(t, err, test.ShouldBeNil)
		test.That(t, position, test.ShouldEqual, 1)

		// Now make SetPosition fail and confirm GetPosition still shows the attempt to set a different position
		fakeMotion.MoveFunc = func(ctx context.Context, req motion.MoveReq) (bool, error) {
			fakeMotionMoveCallCount++
			return false, dummyErr
		}
		err = s.SetPosition(ctx, 0, nil)
		test.That(t, err, test.ShouldBeError, dummyErr)
		test.That(t, fakeArmMoveToJointPositionsCallCount, test.ShouldEqual, 0)
		test.That(t, fakeMotionMoveCallCount, test.ShouldEqual, 3)
		test.That(t, fakeVision1GetObjectPointCloudsCallCount, test.ShouldEqual, 3)
		test.That(t, fakeVision2GetObjectPointCloudsCallCount, test.ShouldEqual, 3)
		position, err = s.GetPosition(ctx, nil)
		test.That(t, err, test.ShouldBeNil)
		test.That(t, position, test.ShouldEqual, 0)
	})

	t.Run("SetPosition uses only arm.MoveToJointPositions when motion service is not configured", func(t *testing.T) {
		fakeArm := inject.NewArm("arm")
		fakeArmMoveToJointPositionsCallCount := 0
		fakeArm.MoveToJointPositionsFunc = func(ctx context.Context, joints []float64, extra map[string]any) error {
			fakeArmMoveToJointPositionsCallCount++

			if fakeArmMoveToJointPositionsCallCount == 1 {
				test.That(t, joints, test.ShouldResemble, []float64{0.0, 0.0, 0.0})
			} else if fakeArmMoveToJointPositionsCallCount == 2 {
				test.That(t, joints, test.ShouldResemble, []float64{1.0, 1.0, 1.0})
			}
			return nil
		}

		fakeFsSvc := inject.NewFrameSystemService(framesystem.PublicServiceName.Name)
		fakeFsSvc.FrameSystemConfigFunc = func(ctx context.Context) (*framesystem.Config, error) {
			return mustTestFSConfigArm3DOF(t), nil
		}

		allDeps := resource.Dependencies{
			fakeArm.Name():   fakeArm,
			fakeFsSvc.Name(): fakeFsSvc,
		}

		baseConfig := &MultiArmPositionSwitchConfig{
			Arm:        "arm",
			JointsList: [][]float64{{0.0, 0.0, 0.0}, {1.0, 1.0, 1.0}},
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

		res, err := newMultiArmPositionSwitch(ctx, allDeps, cfg, logger)
		test.That(t, err, test.ShouldBeNil)
		s, ok := res.(*MultiArmPositionSwitch)
		test.That(t, ok, test.ShouldBeTrue)
		test.That(t, s, test.ShouldNotBeNil)

		err = s.SetPosition(ctx, 0, nil)
		test.That(t, err, test.ShouldBeNil)
		test.That(t, fakeArmMoveToJointPositionsCallCount, test.ShouldEqual, 1)

		position, err := s.GetPosition(ctx, nil)
		test.That(t, err, test.ShouldBeNil)
		test.That(t, position, test.ShouldEqual, 0)

		err = s.SetPosition(ctx, 1, nil)
		test.That(t, err, test.ShouldBeNil)
		test.That(t, fakeArmMoveToJointPositionsCallCount, test.ShouldEqual, 2)

		position, err = s.GetPosition(ctx, nil)
		test.That(t, err, test.ShouldBeNil)
		test.That(t, position, test.ShouldEqual, 1)

		// Now make SetPosition fail and confirm GetPosition still shows the attempt to set a different position
		fakeArm.MoveToJointPositionsFunc = func(ctx context.Context, joints []float64, extra map[string]any) error {
			fakeArmMoveToJointPositionsCallCount++
			return dummyErr
		}
		err = s.SetPosition(ctx, 0, nil)
		test.That(t, err, test.ShouldBeError, dummyErr)
		test.That(t, fakeArmMoveToJointPositionsCallCount, test.ShouldEqual, 3)
		position, err = s.GetPosition(ctx, nil)
		test.That(t, err, test.ShouldBeNil)
		test.That(t, position, test.ShouldEqual, 0)
	})
}

func mustTestFSConfigArm3DOF(t *testing.T) *framesystem.Config {
	t.Helper()
	internal := referenceframe.NewEmptyFrameSystem("internal")
	var parent referenceframe.Frame = internal.World()
	var last string
	for i := 0; i < 3; i++ {
		nm := fmt.Sprintf("j%d", i)
		j, err := referenceframe.NewRotationalFrame(
			nm,
			spatialmath.R4AA{RX: 1},
			referenceframe.Limit{Min: -math.Pi, Max: math.Pi},
		)
		test.That(t, err, test.ShouldBeNil)
		test.That(t, internal.AddFrame(j, parent), test.ShouldBeNil)
		parent = j
		last = nm
	}
	m, err := referenceframe.NewModel("arm", internal, last)
	test.That(t, err, test.ShouldBeNil)
	lif := referenceframe.NewLinkInFrame(referenceframe.World, spatialmath.NewZeroPose(), "arm", nil)
	return &framesystem.Config{Parts: []*referenceframe.FrameSystemPart{{FrameConfig: lif, ModelFrame: m}}}
}

func mustTestFSConfigArmUnderGantry(t *testing.T) *framesystem.Config {
	t.Helper()
	gInternal := referenceframe.NewEmptyFrameSystem("g")
	gf, err := referenceframe.NewTranslationalFrame(
		"axis",
		r3.Vector{X: 1},
		referenceframe.Limit{Min: 0, Max: 2},
	)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, gInternal.AddFrame(gf, gInternal.World()), test.ShouldBeNil)
	gantryModel, err := referenceframe.NewModel("gantry", gInternal, "axis")
	test.That(t, err, test.ShouldBeNil)
	gantryLink := referenceframe.NewLinkInFrame(referenceframe.World, spatialmath.NewZeroPose(), "gantry", nil)

	internal := referenceframe.NewEmptyFrameSystem("internal")
	var parent referenceframe.Frame = internal.World()
	var last string
	for i := 0; i < 2; i++ {
		nm := fmt.Sprintf("j%d", i)
		j, err := referenceframe.NewRotationalFrame(
			nm,
			spatialmath.R4AA{RX: 1},
			referenceframe.Limit{Min: -math.Pi, Max: math.Pi},
		)
		test.That(t, err, test.ShouldBeNil)
		test.That(t, internal.AddFrame(j, parent), test.ShouldBeNil)
		parent = j
		last = nm
	}
	armModel, err := referenceframe.NewModel("arm", internal, last)
	test.That(t, err, test.ShouldBeNil)
	armLink := referenceframe.NewLinkInFrame("gantry", spatialmath.NewZeroPose(), "arm", nil)

	return &framesystem.Config{Parts: []*referenceframe.FrameSystemPart{
		{FrameConfig: gantryLink, ModelFrame: gantryModel},
		{FrameConfig: armLink, ModelFrame: armModel},
	}}
}

func TestMovableAncestors(t *testing.T) {
	t.Run("arm only (3 DoF model)", func(t *testing.T) {
		cfg := mustTestFSConfigArm3DOF(t)
		fs, err := referenceframe.NewFrameSystem(framesystem.LocalFrameSystemName, cfg.Parts, nil)
		test.That(t, err, test.ShouldBeNil)

		chain, err := movableAncestors(fs, "arm")
		test.That(t, err, test.ShouldBeNil)
		test.That(t, len(chain), test.ShouldEqual, 1)
		test.That(t, chain[0].Name(), test.ShouldEqual, "arm")
		test.That(t, len(chain[0].DoF()), test.ShouldEqual, 3)
	})

	t.Run("gantry + arm", func(t *testing.T) {
		cfg := mustTestFSConfigArmUnderGantry(t)
		fs, err := referenceframe.NewFrameSystem(framesystem.LocalFrameSystemName, cfg.Parts, nil)
		test.That(t, err, test.ShouldBeNil)

		chain, err := movableAncestors(fs, "arm")
		test.That(t, err, test.ShouldBeNil)
		test.That(t, len(chain), test.ShouldEqual, 2)
		test.That(t, chain[0].Name(), test.ShouldEqual, "arm")
		test.That(t, chain[1].Name(), test.ShouldEqual, "gantry")
	})

	t.Run("static ancestor skipped (table has 0 DoF)", func(t *testing.T) {
		table := referenceframe.NewLinkInFrame(referenceframe.World, spatialmath.NewZeroPose(), "table", nil)
		base := mustTestFSConfigArmUnderGantry(t)
		gantryPart := base.Parts[0]
		gantryPart.FrameConfig = referenceframe.NewLinkInFrame("table", spatialmath.NewZeroPose(), "gantry", nil)
		parts := []*referenceframe.FrameSystemPart{
			{FrameConfig: table, ModelFrame: nil},
			gantryPart,
			base.Parts[1],
		}
		fs, err := referenceframe.NewFrameSystem(framesystem.LocalFrameSystemName, parts, nil)
		test.That(t, err, test.ShouldBeNil)

		chain, err := movableAncestors(fs, "arm")
		test.That(t, err, test.ShouldBeNil)
		test.That(t, len(chain), test.ShouldEqual, 2)
		test.That(t, chain[1].Name(), test.ShouldEqual, "gantry")
	})
}

func TestFlatJointsToGoalInputs(t *testing.T) {
	cfg := mustTestFSConfigArmUnderGantry(t)
	fs, err := referenceframe.NewFrameSystem(framesystem.LocalFrameSystemName, cfg.Parts, nil)
	test.That(t, err, test.ShouldBeNil)
	chain, err := movableAncestors(fs, "arm")
	test.That(t, err, test.ShouldBeNil)

	got, err := flatJointsToGoalInputs([]float64{1, 2, 3}, chain)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, len(got), test.ShouldEqual, 2)
	test.That(t, []float64(got["arm"]), test.ShouldResemble, []float64{1, 2})
	test.That(t, []float64(got["gantry"]), test.ShouldResemble, []float64{3})

	_, err = flatJointsToGoalInputs([]float64{1, 2}, chain)
	test.That(t, err, test.ShouldNotBeNil)
}

func TestMultiArmPositionSwitchValidateUniformLength(t *testing.T) {
	path := "components.0"
	cfg := &MultiArmPositionSwitchConfig{
		Arm: "arm",
		JointsList: [][]float64{
			{0, 0, 0},
			{1, 1},
		},
	}
	_, _, err := cfg.Validate(path)
	test.That(t, err, test.ShouldNotBeNil)
	test.That(t, err.Error(), test.ShouldContainSubstring, "same inner length")
}

package touch

import (
	"context"
	"errors"
	"fmt"
	"math"
	"strings"
	"sync"
	"sync/atomic"
	"time"

	"go.viam.com/rdk/components/arm"
	toggleswitch "go.viam.com/rdk/components/switch"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/motionplan"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/robot/framesystem"
	"go.viam.com/rdk/services/motion"
	"go.viam.com/rdk/services/vision"

	"github.com/erh/vmodutils"
	"github.com/erh/vmodutils/file_utils"
)

const extraParamsKeyGoalState = "goal_state"

var MultiArmPositionSwitchModel = vmodutils.NamespaceFamily.WithModel("multi-arm-position-switch")

func init() {
	resource.RegisterComponent(
		toggleswitch.API,
		MultiArmPositionSwitchModel,
		resource.Registration[toggleswitch.Switch, *MultiArmPositionSwitchConfig]{
			Constructor: newMultiArmPositionSwitch,
		})
}

type MultiArmPositionSwitchConfig struct {
	Arm                          string                  `json:"arm,omitempty"`
	JointsList                   [][]float64             `json:"joints_list,omitempty"`
	Motion                       string                  `json:"motion,omitempty"`
	VisionServices               []string                `json:"vision_services,omitempty"`
	Extra                        map[string]any          `json:"extra,omitempty"`
	Constraints                  *motionplan.Constraints `json:"constraints,omitempty"`
	WriteFilesToCaptureDirectory bool                    `json:"write_files_to_capture_directory,omitempty"`

	MaxSpeedDegsPerSec         float64 `json:"max_speed_degs_per_sec,omitempty"`
	MaxAccelerationDegsPerSec2 float64 `json:"max_acceleration_degs_per_sec2,omitempty"`
}

func (c *MultiArmPositionSwitchConfig) moveOptions() *arm.MoveOptions {
	if c.MaxSpeedDegsPerSec <= 0 && c.MaxAccelerationDegsPerSec2 <= 0 {
		return nil
	}
	opts := &arm.MoveOptions{}
	if c.MaxSpeedDegsPerSec > 0 {
		opts.MaxVelRads = c.MaxSpeedDegsPerSec * math.Pi / 180.0
	}
	if c.MaxAccelerationDegsPerSec2 > 0 {
		opts.MaxAccRads = c.MaxAccelerationDegsPerSec2 * math.Pi / 180.0
	}
	return opts
}

func (c *MultiArmPositionSwitchConfig) Validate(path string) ([]string, []string, error) {
	reqDeps := []string{}

	if c.Arm == "" {
		return nil, nil, resource.NewConfigValidationFieldRequiredError(path, "arm")
	}
	reqDeps = append(reqDeps, c.Arm)

	if c.Motion != "" {
		if c.Motion == "builtin" {
			reqDeps = append(reqDeps, motion.Named("builtin").String())
		} else {
			reqDeps = append(reqDeps, c.Motion)
		}
	}

	reqDeps = append(reqDeps, c.VisionServices...)

	if len(c.JointsList) == 0 {
		return nil, nil, ErrMustSpecifyAtLeastOneJointPosition
	}

	firstLen := len(c.JointsList[0])
	for i := 1; i < len(c.JointsList); i++ {
		if len(c.JointsList[i]) != firstLen {
			return nil, nil, fmt.Errorf(
				"joints_list entries must all have the same inner length; entry 0 has length %d but entry %d has length %d",
				firstLen, i, len(c.JointsList[i]),
			)
		}
	}

	if c.Extra != nil && c.Extra[extraParamsKeyGoalState] != nil {
		return nil, nil, ErrCannotSpecifyGoalStateInExtra
	}

	if c.MaxSpeedDegsPerSec < 0 {
		return nil, nil, fmt.Errorf("%s.max_speed_degs_per_sec must be non-negative, got %v", path, c.MaxSpeedDegsPerSec)
	}
	if c.MaxAccelerationDegsPerSec2 < 0 {
		return nil, nil, fmt.Errorf("%s.max_acceleration_degs_per_sec2 must be non-negative, got %v", path, c.MaxAccelerationDegsPerSec2)
	}

	return reqDeps, nil, nil
}

func newMultiArmPositionSwitch(ctx context.Context, deps resource.Dependencies, config resource.Config, logger logging.Logger) (toggleswitch.Switch, error) {
	newConf, err := resource.NativeConfig[*MultiArmPositionSwitchConfig](config)
	if err != nil {
		return nil, err
	}

	arm, err := arm.FromProvider(deps, newConf.Arm)
	if err != nil {
		return nil, err
	}

	maps := &MultiArmPositionSwitch{
		name:   config.ResourceName(),
		cfg:    newConf,
		logger: logger,
		arm:    arm,
	}

	if newConf.Motion != "" {
		maps.motion, err = motion.FromProvider(deps, newConf.Motion)
		if err != nil {
			return nil, err
		}
	}

	for _, name := range newConf.VisionServices {
		v, err := vision.FromProvider(deps, name)
		if err != nil {
			return nil, err
		}
		maps.visionServices = append(maps.visionServices, v)
	}

	maps.fsSvc, err = framesystem.FromDependencies(deps)
	if err != nil {
		return nil, err
	}

	fs, err := framesystem.NewFromService(ctx, maps.fsSvc, nil)
	if err != nil {
		return nil, fmt.Errorf("multi-arm-position-switch: building frame system: %w", err)
	}
	maps.inputChain, err = movableAncestors(fs, arm.Name().Name)
	if err != nil {
		return nil, err
	}

	dofMappingSummary := formatDoFMapping(maps.inputChain)

	maps.logger.Infof("multi-arm-position-switch joint chain (arm first, toward world): %s",
		dofMappingSummary)

	wantLen := totalDoF(maps.inputChain)
	for i, row := range newConf.JointsList {
		if len(row) != wantLen {
			return nil, fmt.Errorf("joints_list[%d]: length is %d but expected %d (%s)",
				i, len(row), wantLen, dofMappingSummary)
		}
	}
	if newConf.Motion == "" && len(maps.inputChain) > 1 {
		return nil, fmt.Errorf(
			"multi-arm-position-switch: motion service is required when the arm has input-enabled ancestors (%s); "+
				"configure \"motion\" or use MoveToJointPositions only with an arm-only frame chain",
			dofMappingSummary,
		)
	}

	return maps, nil
}

type MultiArmPositionSwitch struct {
	resource.AlwaysRebuild
	resource.TriviallyCloseable

	name   resource.Name
	cfg    *MultiArmPositionSwitchConfig
	logger logging.Logger

	arm            arm.Arm
	motion         motion.Service
	visionServices []vision.Service
	fsSvc          framesystem.Service
	inputChain     []referenceframe.Frame

	// 'mu' protects access to 'position'
	mu       sync.Mutex
	position uint32

	// 'executing' ensures only one goToPosition call is active at a time
	executing atomic.Bool
}

func (maps *MultiArmPositionSwitch) Name() resource.Name {
	return maps.name
}

func (maps *MultiArmPositionSwitch) Status(ctx context.Context) (map[string]interface{}, error) {
	return map[string]interface{}{}, nil
}

func (maps *MultiArmPositionSwitch) DoCommand(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	return nil, resource.ErrDoUnimplemented
}

func (maps *MultiArmPositionSwitch) updatePosition(position uint32) {
	maps.mu.Lock()
	defer maps.mu.Unlock()
	maps.position = position
}

func (maps *MultiArmPositionSwitch) SetPosition(ctx context.Context, position uint32, extra map[string]interface{}) error {
	if position > uint32(len(maps.cfg.JointsList))-1 {
		return fmt.Errorf("requested position %d is greater than highest possible position %d", position, len(maps.cfg.JointsList)-1)
	}

	err := maps.goToPosition(ctx, position)
	if err != nil {
		return err
	}

	return nil
}

func (maps *MultiArmPositionSwitch) GetPosition(ctx context.Context, extra map[string]interface{}) (uint32, error) {
	maps.mu.Lock()
	defer maps.mu.Unlock()
	return maps.position, nil
}

func (maps *MultiArmPositionSwitch) GetNumberOfPositions(ctx context.Context, extra map[string]interface{}) (uint32, []string, error) {
	var positionStrs []string
	for i := range maps.cfg.JointsList {
		positionStrs = append(positionStrs, fmt.Sprintf("go to %d", i))
	}
	return uint32(len(maps.cfg.JointsList)), positionStrs, nil
}

func (maps *MultiArmPositionSwitch) goToPosition(ctx context.Context, position uint32) error {
	if !maps.executing.CompareAndSwap(false, true) {
		return errors.New("switch is currently executing")
	}
	defer maps.executing.Store(false)

	traceID := getTraceID(ctx)
	dirPath := file_utils.GetPathInCaptureDir(fmt.Sprintf("tag=%s", traceID))
	if traceID == "" && maps.cfg.WriteFilesToCaptureDirectory {
		maps.logger.Warnf("no traceID set, will write files directly to capture directory")
	}

	maps.updatePosition(position)

	joints := maps.cfg.JointsList[position]

	if maps.cfg.WriteFilesToCaptureDirectory {
		// Write the config
		fileName := fmt.Sprintf("%s_config.json", maps.name.Name)
		if err := file_utils.SaveJsonFile(maps.cfg, dirPath, fileName, time.Now()); err != nil {
			return err
		}

		// Write the goal joint position
		goal_filename := fmt.Sprintf("%s_joint_position_goal_%d.json", maps.name.Name, position)
		if err := file_utils.SaveJsonFile(joints, dirPath, goal_filename, time.Now()); err != nil {
			return err
		}
	}

	var moveErr error
	if maps.motion != nil {
		goalInputs, err := flatJointsToGoalInputs(joints, maps.inputChain)
		if err != nil {
			return err
		}
		moveErr = goToPositionUsingJointToJointMotion(ctx, goalInputs, maps.arm.Name().Name, maps.motion, maps.visionServices, maps.cfg.Extra, maps.cfg.Constraints, maps.logger)
	} else {
		moveErr = goToPositionUsingMoveToJointPositions(ctx, joints, maps.arm, maps.cfg.moveOptions(), maps.cfg.Extra, maps.logger)
	}

	if maps.cfg.WriteFilesToCaptureDirectory {
		// Write the actual joint position we ended up at
		curInputs, err := maps.arm.CurrentInputs(ctx)
		if err != nil {
			return errors.Join(moveErr, err)
		}
		actual_position_filename := fmt.Sprintf("%s_joint_position_actual_%d.json", maps.name.Name, position)
		if err := file_utils.SaveJsonFile(curInputs, dirPath, actual_position_filename, time.Now()); err != nil {
			return errors.Join(moveErr, err)
		}
	}

	return moveErr
}

func movableAncestors(fs *referenceframe.FrameSystem, armFrameName string) ([]referenceframe.Frame, error) {
	armFrame := fs.Frame(armFrameName)
	if armFrame == nil {
		return nil, fmt.Errorf(
			"multi-arm-position-switch: arm %q is not in the frame system; check the arm's name matches the frame system",
			armFrameName,
		)
	}
	path, err := fs.TracebackFrame(armFrame)
	if err != nil {
		return nil, fmt.Errorf("multi-arm-position-switch: tracing frame %q toward world: %w", armFrameName, err)
	}

	var chain []referenceframe.Frame
	for _, fr := range path {
		if fr.Name() == referenceframe.World {
			continue
		}
		if len(fr.DoF()) > 0 {
			chain = append(chain, fr)
		}
	}
	return chain, nil
}

func totalDoF(frames []referenceframe.Frame) int {
	n := 0
	for _, fr := range frames {
		n += len(fr.DoF())
	}
	return n
}

func formatDoFMapping(frames []referenceframe.Frame) string {
	var b strings.Builder
	offset := 0
	for i, fr := range frames {
		if i > 0 {
			b.WriteString(", ")
		}
		dof := len(fr.DoF())
		if dof == 1 {
			fmt.Fprintf(&b, "%s[%d]", fr.Name(), offset)
		} else {
			fmt.Fprintf(&b, "%s[%d:%d]", fr.Name(), offset, offset+dof)
		}
		offset += dof
	}
	return b.String()
}

func flatJointsToGoalInputs(joints []float64, chain []referenceframe.Frame) (referenceframe.FrameSystemInputs, error) {
	want := totalDoF(chain)
	if len(joints) != want {
		return nil, fmt.Errorf("internal error: got %d joint values for %d expected (mapping: %s)",
			len(joints), want, formatDoFMapping(chain))
	}
	out := make(referenceframe.FrameSystemInputs)
	off := 0
	for _, fr := range chain {
		d := len(fr.DoF())
		slice := joints[off : off+d]
		out[fr.Name()] = floatsToInputs(slice)
		off += d
	}
	return out, nil
}

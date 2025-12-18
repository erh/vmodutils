package touch

import (
	"errors"
)

var (
	ErrCannotSpecifyGoalStateInExtra      = errors.New("cannot specify 'goal_state' in 'extra', should be specified via 'joints'")
	ErrMustSpecifyAtLeastOneJointPosition = errors.New("must specify at least one joint position")
)

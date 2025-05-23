package vmodutils

import (
	"context"
	"fmt"
	"os"

	"go.viam.com/rdk/app"
	"go.viam.com/rdk/cli"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/robot"
	"go.viam.com/rdk/robot/client"
	"go.viam.com/rdk/utils"
	"go.viam.com/utils/rpc"
)

func MachineToDependencies(client *client.RobotClient) (resource.Dependencies, error) {
	deps := resource.Dependencies{}

	names := client.ResourceNames()
	for _, n := range names {
		r, err := client.ResourceByName(n)
		if err != nil {
			return nil, err
		}
		deps[n] = r
	}

	return deps, nil
}

func ConnectToMachineFromEnv(ctx context.Context, logger logging.Logger) (robot.Robot, error) {
	params := []string{}
	for _, pp := range []string{utils.MachineFQDNEnvVar, utils.APIKeyIDEnvVar, utils.APIKeyEnvVar} {
		x := os.Getenv(pp)
		if x == "" {
			return nil, fmt.Errorf("no environment variable for %s", pp)
		}
		params = append(params, x)
	}
	return ConnectToMachine(ctx, logger, params[0], params[1], params[2])
}

func ConnectToMachine(ctx context.Context, logger logging.Logger, host, apiKeyId, apiKey string) (robot.Robot, error) {
	return client.New(
		ctx,
		host,
		logger,
		client.WithDialOptions(rpc.WithEntityCredentials(
			apiKeyId,
			rpc.Credentials{
				Type:    rpc.CredentialsTypeAPIKey,
				Payload: apiKey,
			},
		)),
	)
}

// ConnectToHostFromCLIToken uses the viam cli token to login to a machine with just a hostname.
// use "viam login" to setup the token.
func ConnectToHostFromCLIToken(ctx context.Context, host string, logger logging.Logger) (robot.Robot, error) {
	c, err := cli.ConfigFromCache(nil)
	if err != nil {
		return nil, err
	}

	dopts, err := c.DialOptions()
	if err != nil {
		return nil, err
	}

	return client.New(
		ctx,
		host,
		logger,
		client.WithDialOptions(dopts...),
	)
}

func UpdateComponentCloudAttributesFromModuleEnv(ctx context.Context, name resource.Name, newAttr utils.AttributeMap, logger logging.Logger) error {
	id := os.Getenv(utils.MachinePartIDEnvVar)
	if id == "" {
		return fmt.Errorf("no %s in env", utils.MachinePartIDEnvVar)
	}

	c, err := app.CreateViamClientFromEnvVars(ctx, nil, logger)
	if err != nil {
		return err
	}
	defer c.Close()

	return UpdateComponentCloudAttributes(ctx, c.AppClient(), id, name, newAttr)

}

func UpdateComponentCloudAttributes(ctx context.Context, c *app.AppClient, id string, name resource.Name, newAttr utils.AttributeMap) error {
	part, _, err := c.GetRobotPart(ctx, id)
	if err != nil {
		return err
	}

	cs, ok := part.RobotConfig["components"].([]interface{})
	if !ok {
		return fmt.Errorf("no components %T", part.RobotConfig["components"])
	}
	services, ok := part.RobotConfig["services"].([]interface{})
	if ok {
		cs = append(cs, services...)
	}

	found := false

	for idx, cc := range cs {
		ccc, ok := cc.(map[string]interface{})
		if !ok {
			return fmt.Errorf("config bad %d: %T", idx, cc)
		}
		if ccc["name"] != name.ShortName() {
			continue
		}
		fmt.Printf("c %d %v %T\n", idx, ccc, ccc)
		ccc["attributes"] = newAttr
		fmt.Printf("c %d %v %T\n", idx, ccc, ccc)
		cs[idx] = ccc
		found = true
	}

	if !found {
		return fmt.Errorf("didn't find component with name %v", name.ShortName())
	}

	_, err = c.UpdateRobotPart(ctx, id, part.Name, part.RobotConfig)
	return err
}

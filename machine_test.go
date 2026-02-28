package vmodutils

import (
	"context"
	"errors"
	"fmt"
	"testing"

	"go.viam.com/rdk/app"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/utils"
	"go.viam.com/test"
)

type MockAppClient struct {
	machineConfigs map[string]interface{}
	fragments      map[string]interface{}
}

func helperMachineConfig(componentNames, serviceNames, fragmentIDs []string) map[string]interface{} {
	components := []interface{}{}
	for _, c := range componentNames {
		components = append(components, map[string]interface{}{"name": c})
	}
	services := []interface{}{}
	for _, s := range serviceNames {
		services = append(services, map[string]interface{}{"name": s})
	}

	fragments := []interface{}{}
	for _, f := range fragmentIDs {
		fragments = append(fragments, f)
	}
	return map[string]interface{}{"components": components,
		"services":  services,
		"fragments": fragments}
}

func (c *MockAppClient) GetFragment(ctx context.Context, id, version string) (*app.Fragment, error) {
	// ignoring versions for tests
	frag, ok := c.fragments[id].(map[string]interface{})
	if !ok {
		return nil, fmt.Errorf("fragment %v not found", id)
	}

	return &app.Fragment{ID: id, Fragment: frag}, nil
}

// simple helper function for tests
func getAttrFromConfigForTests(machine map[string]interface{}, name string) interface{} {
	cs, ok := machine["components"].([]interface{})
	if !ok {
		return nil
	}
	services, ok := machine["services"].([]interface{})
	if ok {
		cs = append(cs, services...)
	}

	for _, cc := range cs {
		ccc, ok := cc.(map[string]interface{})
		if !ok {
			return nil
		}
		if ccc["name"] != name {
			continue
		}

		return ccc["attributes"]
	}
	return nil
}

func TestFindComponentInFragment(t *testing.T) {
	f0 := helperMachineConfig([]string{"c1", "c2"}, []string{"s1", "s2"}, []string{"f1", "f2"})
	f1 := helperMachineConfig([]string{"c3", "c4"}, []string{"s3", "s4"}, []string{"f3"})
	f2 := helperMachineConfig([]string{"c5", "c6"}, []string{"s5", "s6"}, []string{})
	f3 := helperMachineConfig([]string{"c7", "c8"}, []string{"s7", "s8"}, []string{})
	myMock := MockAppClient{fragments: map[string]interface{}{"f0": f0, "f1": f1, "f2": f2, "f3": f3}}
	cases := []struct {
		description           string
		componentName         string
		expectedFragModString string
		expectedErr           error
	}{
		{
			description:           "the component is in the first fragment",
			componentName:         "c1",
			expectedFragModString: "components.c1.attributes",
		},
		{
			description:           "the service is in the first fragment",
			componentName:         "s2",
			expectedFragModString: "services.s2.attributes",
		},
		{
			description:           "the component is in a nested fragment",
			componentName:         "c5",
			expectedFragModString: "components.c5.attributes",
		},
		{
			description:           "the service is in a nested nested fragment",
			componentName:         "s7",
			expectedFragModString: "services.s7.attributes",
		},
		{
			description:           "the component cannot be found",
			componentName:         "not-here",
			expectedFragModString: "",
		},
	}
	for _, tt := range cases {
		t.Run(tt.description, func(t *testing.T) {
			// make a machine for the test
			name := resource.NewName(resource.APINamespaceRDK.WithComponentType("test"), tt.componentName)
			fragModString, err := findComponentInFragment(context.Background(), myMock.GetFragment, "f0", "", name)
			test.That(t, err, test.ShouldResemble, tt.expectedErr)
			test.That(t, tt.expectedFragModString, test.ShouldEqual, fragModString)
		})
	}
}

func TestUpdateComponentAttributesInPlace(t *testing.T) {
	newAttr := utils.AttributeMap{"attr1": true}
	f1 := helperMachineConfig([]string{"c3", "c4"}, []string{"s3", "s4"}, []string{"f3"})
	f2 := helperMachineConfig([]string{"c5", "c6"}, []string{"s5", "s6"}, []string{})
	f3 := helperMachineConfig([]string{"c7", "c8"}, []string{"s7", "s8"}, []string{})
	myMock := MockAppClient{fragments: map[string]interface{}{"f1": f1, "f2": f2, "f3": f3}}
	cases := []struct {
		description   string
		componentName string
		expectedErr   error
		machineConfig map[string]interface{}
	}{
		{
			description:   "the component is in the machine",
			componentName: "c1",
			machineConfig: helperMachineConfig([]string{"c1", "c2"}, []string{"s1", "s2"}, []string{"f1", "f2"}),
		},
		{
			description:   "the service is in the machine",
			componentName: "s2",
			machineConfig: helperMachineConfig([]string{"c1", "c2"}, []string{"s1", "s2"}, []string{"f1", "f2"}),
		},
		{
			description:   "the component cannot be found",
			componentName: "not-here",
			expectedErr:   errors.New("didn't find component with name not-here"),
			machineConfig: helperMachineConfig([]string{"c1", "c2"}, []string{"s1", "s2"}, []string{"f1", "f2"}),
		},
		{
			description:   "really bad machine config",
			componentName: "c1",
			machineConfig: nil,
			expectedErr:   fmt.Errorf("didn't find component with name c1"),
		},
	}
	for _, tt := range cases {
		t.Run(tt.description, func(t *testing.T) {
			// make a machine for the test
			name := resource.NewName(resource.APINamespaceRDK.WithComponentType("test"), tt.componentName)
			err := updateComponentAttributesInPlace(context.Background(), tt.machineConfig, myMock.GetFragment, name, newAttr)
			test.That(t, err, test.ShouldResemble, tt.expectedErr)
			if tt.expectedErr == nil {
				updatedAttrs := getAttrFromConfigForTests(tt.machineConfig, tt.componentName)
				test.That(t, updatedAttrs, test.ShouldResemble, newAttr)
			}
		})
	}
}

func TestUpdateComponentOrServiceConfig(t *testing.T) {
	newAttr := utils.AttributeMap{"attr1": true}
	cases := []struct {
		description   string
		componentName string
		shouldBeFound bool
		expectedErr   error
		machineConfig map[string]interface{}
	}{
		{
			description:   "the component is in the machine",
			componentName: "c1",
			shouldBeFound: true,
			machineConfig: helperMachineConfig([]string{"c1", "c2"}, []string{"s1", "s2"}, []string{"f1", "f2"}),
		},
		{
			description:   "the service is in the machine",
			componentName: "s2",
			shouldBeFound: true,
			machineConfig: helperMachineConfig([]string{"c1", "c2"}, []string{"s1", "s2"}, []string{"f1", "f2"}),
		},
		{
			description:   "the component cannot be found",
			componentName: "not-here",
			shouldBeFound: false,
			machineConfig: helperMachineConfig([]string{"c1", "c2"}, []string{"s1", "s2"}, []string{"f1", "f2"}),
		},
		{
			description:   "really bad machine config",
			componentName: "c1",
			shouldBeFound: false,
			machineConfig: nil,
			expectedErr:   nil,
		},
	}
	for _, tt := range cases {
		t.Run(tt.description, func(t *testing.T) {
			// make a machine for the test
			name := resource.NewName(resource.APINamespaceRDK.WithComponentType("test"), tt.componentName)
			found, err := updateComponentOrServiceConfig(tt.machineConfig, name, newAttr)

			test.That(t, found, test.ShouldEqual, tt.shouldBeFound)
			test.That(t, err, test.ShouldResemble, tt.expectedErr)
			if tt.shouldBeFound {
				updatedAttrs := getAttrFromConfigForTests(tt.machineConfig, tt.componentName)
				test.That(t, updatedAttrs, test.ShouldResemble, newAttr)
			}
		})
	}
}

func TestGetFragmentId(t *testing.T) {
	id := "id"
	version := "version"
	reallyBadFrag := 25
	cases := []struct {
		description     string
		frag            interface{}
		expectedID      string
		expectedVersion string
		expectedErr     error
	}{
		{
			description: "the fragment is just the id string",
			frag:        id,
			expectedID:  id,
		},
		{
			description: "fragment is a map with no version",
			frag:        map[string]interface{}{"id": id},
			expectedID:  id,
		},
		{
			description:     "fragment is a map with a version",
			frag:            map[string]interface{}{"id": id, "version": version},
			expectedID:      id,
			expectedVersion: version,
		},
		{
			description: "fragment has no id",
			frag:        map[string]interface{}{"version": version},
			expectedID:  "",
			expectedErr: fmt.Errorf("fragment is missing an id: %v", map[string]interface{}{"version": version}),
		},
		{
			description: "fragment is not an expected fragment config",
			frag:        reallyBadFrag,
			expectedID:  "",
			expectedErr: fmt.Errorf("fragment config does not match expected interface: %T", reallyBadFrag),
		},
	}
	for _, tt := range cases {
		t.Run(tt.description, func(t *testing.T) {
			idOut, versionOut, err := getFragmentId(tt.frag)
			test.That(t, idOut, test.ShouldEqual, tt.expectedID)
			test.That(t, versionOut, test.ShouldResemble, tt.expectedVersion)
			test.That(t, err, test.ShouldResemble, tt.expectedErr)

		})
	}
}

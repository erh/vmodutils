
GO_BUILD_ENV   :=
GO_BUILD_FLAGS :=
MODULE_BINARY  := bin/vmodutils

ifeq ($(VIAM_TARGET_OS), windows)
	GO_BUILD_ENV += GOOS=windows GOARCH=amd64
	GO_BUILD_FLAGS := -tags no_cgo
	MODULE_BINARY = bin/vmodutils.exe
endif

$(MODULE_BINARY): bin *.go cmd/module/*.go *.mod touch/*.go meta.json app-arm-control/index.html
	$(GO_BUILD_ENV) go build $(GO_BUILD_FLAGS) -o $(MODULE_BINARY) cmd/module/cmd.go

# tests run on the host, not the cross-compile target
test:
	GOOS=$(shell go env GOHOSTOS) GOARCH=$(shell go env GOHOSTARCH) CGO_ENABLED=1 go test ./...

lint:
	gofmt -w -s .

update:
	go get go.viam.com/rdk@latest
	go mod tidy

module: $(MODULE_BINARY) test
ifeq ($(VIAM_TARGET_OS), windows)
	cp meta.json meta.json.orig
	jq '.entrypoint = "./bin/vmodutils.exe"' meta.json.orig > meta.json
endif
	tar czf module.tar.gz $(MODULE_BINARY) meta.json app-arm-control
ifeq ($(VIAM_TARGET_OS), windows)
	mv meta.json.orig meta.json
endif

bin:
	-mkdir bin

setup:
	brew install nlopt-static || sudo apt install -y libnlopt-dev 


bin/vmodutils: bin *.go cmd/module/*.go *.mod touch/*.go meta.json
	go build -o bin/vmodutils cmd/module/cmd.go

test:
	go test ./...

lint:
	gofmt -w -s .

update:
	go get go.viam.com/rdk@latest
	go mod tidy

module: bin/vmodutils test
	tar czf module.tar.gz bin/vmodutils meta.json

bin:
	-mkdir bin

setup:
	brew install nlopt-static || sudo apt install -y libnlopt-dev 

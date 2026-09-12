# arm-web one-click build (web assets embedded via go:embed).
# Run `make` from the arm-web directory.

GO ?= go
OUT := arm-web
ifeq ($(OS),Windows_NT)
OUT := arm-web.exe
endif

VET_ENV := GOFLAGS=-mod=mod GOPROXY=off GOSUMDB=off

.PHONY: all build vet test clean

all: build

vet:
	$(VET_ENV) $(GO) vet ./...

build: vet
	$(VET_ENV) $(GO) build -o $(OUT) .

test:
	$(VET_ENV) $(GO) test ./...

clean:
	rm -f arm-web arm-web.exe 2>/dev/null || true

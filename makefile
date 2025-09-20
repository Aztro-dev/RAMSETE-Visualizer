# Detect OS
UNAME := $(shell uname)

# Source and output
SRC := ./src/main.cpp
OUT_DIR := ./build
OUT := $(OUT_DIR)/main
ifeq ($(UNAME), Windows_NT)
    OUT := $(OUT).exe
endif

# Optional: user override for raylib paths
RAYLIB_DIR ?= ./raylib         # Default relative location
RAYLIB_INCLUDE ?= $(RAYLIB_DIR)
RAYLIB_EXTERNAL ?= $(RAYLIB_DIR)

# Ensure build dir exists
$(shell mkdir -p $(OUT_DIR))

# Compiler and flags
CXX := g++
CXXFLAGS := -g -O2 -Wall -Wno-narrowing -Wno-missing-braces
CPPFLAGS := -I. -I$(RAYLIB_INCLUDE) -I$(RAYLIB_EXTERNAL)

ifeq ($(UNAME), Linux)
	LIBS := -L$(RAYLIB_DIR) -lraylib -lGL -lm -lpthread -ldl -lrt -lX11 -latomic
	DEFINES := -DPLATFORM_DESKTOP -DPLATFORM_DESKTOP_GLFW -D_DEFAULT_SOURCE
else
	CPPFLAGS += -I$(RAYLIB_DIR)
	LIBS := -L$(RAYLIB_DIR) -lraylib -lgdi32 -lwinmm
	DEFINES := -DPLATFORM_DESKTOP
endif

# Build target
main: $(SRC)
	$(CXX) $(CXXFLAGS) $(CPPFLAGS) $(DEFINES) -o $(OUT) $^ $(LIBS)

# Run target
run: main
	$(OUT)

# Clean build artifacts
clean:
	rm -rf $(OUT_DIR)/*

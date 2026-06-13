# mirage - head-tracked virtual displays for Hyprland
#
#   make            build everything (mirage + mirage-posedump)
#   make posedump   build just the pose test tool (no wayland/GL needed)
#   make protocols  generate wayland protocol glue only
#   make clean
#
# mirage's own sources are C++ (gnu++23); the wayland-scanner-generated protocol
# glue stays C (its headers self-guard with extern "C", so the C++ TUs link to
# the C-compiled interface symbols cleanly). The rayneo bridge is a separate C
# tool, built untouched.

CC      ?= cc
CXX     ?= c++
SCANNER ?= wayland-scanner
PKGCONF ?= pkg-config

SYS_PROTO := /usr/share/wayland-protocols

# Wayland protocols we bind. Names map to <name>.xml found via vpath below.
PROTO_NAMES := \
	xdg-shell \
	linux-dmabuf-unstable-v1 \
	wlr-virtual-pointer-unstable-v1 \
	pointer-constraints-unstable-v1 \
	relative-pointer-unstable-v1 \
	ext-foreign-toplevel-list-v1 \
	ext-image-capture-source-v1 \
	ext-image-copy-capture-v1

vpath %.xml protocol:$(SYS_PROTO)/stable/xdg-shell:$(SYS_PROTO)/unstable/linux-dmabuf:$(SYS_PROTO)/unstable/pointer-constraints:$(SYS_PROTO)/unstable/relative-pointer:$(SYS_PROTO)/staging/ext-foreign-toplevel-list:$(SYS_PROTO)/staging/ext-image-capture-source:$(SYS_PROTO)/staging/ext-image-copy-capture

PROTO_HDR := $(PROTO_NAMES:%=build/proto/%-client-protocol.h)
PROTO_SRC := $(PROTO_NAMES:%=build/proto/%-protocol.c)
PROTO_OBJ := $(PROTO_SRC:build/proto/%.c=build/obj/%.o)

RENDER_PKGS := wayland-client wayland-egl wayland-cursor egl glesv2 gbm libdrm libinput
PKG_CFLAGS  := $(shell $(PKGCONF) --cflags $(RENDER_PKGS))
OPT     ?= -O2 -g
WARN    := -Wall -Wextra -Wno-missing-field-initializers
INC     := -Isrc -Ibuild/proto
COMMON  := $(OPT) -D_GNU_SOURCE $(WARN) $(INC) -pthread $(PKG_CFLAGS)

# C++ flags for mirage's own sources; C flags for the generated protocol glue.
CXXFLAGS ?= -std=gnu++23
CXXFLAGS += $(COMMON)
CFLAGS   ?= -std=c11
CFLAGS   += $(COMMON)
LDLIBS   := $(shell $(PKGCONF) --libs $(RENDER_PKGS)) -lm -pthread -lrt

# ---- core mirage objects (wayland + GL), now C++ ----
MIRAGE_SRC := src/main.cpp src/pose.cpp src/capture.cpp src/render.cpp \
              src/layout.cpp src/grab.cpp src/config.cpp src/layouts.cpp \
              src/profile.cpp src/calib.cpp src/stb_truetype_impl.cpp
MIRAGE_OBJ := $(MIRAGE_SRC:src/%.cpp=build/obj/%.o) $(PROTO_OBJ)

# ---- pose test tool (no wayland/GL) ----
POSEDUMP_OBJ := build/obj/tool_posedump.o build/obj/pose.o

.PHONY: all posedump protocols bridge clean
all: mirage mirage-posedump
posedump: mirage-posedump
bridge: rayneo-bridge
protocols: $(PROTO_HDR) $(PROTO_SRC)

rayneo-bridge: src/rayneo_bridge.c src/rayneo.c src/magcal.c src/rayneo.h src/magcal.h
	$(CC) -O2 -g -std=c11 -D_GNU_SOURCE -Wall -Wextra -Isrc \
	    -o $@ src/rayneo_bridge.c src/rayneo.c src/magcal.c -lm

mirage: $(MIRAGE_OBJ)
	$(CXX) $(CXXFLAGS) -o $@ $^ $(LDLIBS)

mirage-posedump: $(POSEDUMP_OBJ)
	$(CXX) $(CXXFLAGS) -o $@ $^ -lm -pthread

# protocol codegen
build/proto/%-client-protocol.h: %.xml | build/proto
	$(SCANNER) client-header $< $@
build/proto/%-protocol.c: %.xml | build/proto
	$(SCANNER) private-code $< $@

# compile generated protocol code (stays C)
build/obj/%.o: build/proto/%.c | build/obj
	$(CC) $(CFLAGS) -c -o $@ $<

# compile our sources (C++); depend on generated headers so codegen runs first,
# and on our own headers so a struct-layout change (e.g. mirage.h) rebuilds
# everything that includes it - mismatched object layouts corrupt memory.
SRC_HDR := $(wildcard src/*.h) $(wildcard src/*.hpp)
build/obj/%.o: src/%.cpp $(PROTO_HDR) $(SRC_HDR) | build/obj
	$(CXX) $(CXXFLAGS) -c -o $@ $<

build/proto build/obj:
	mkdir -p $@

clean:
	rm -rf build mirage mirage-posedump

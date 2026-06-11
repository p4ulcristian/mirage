# mirage - head-tracked virtual displays for Hyprland
#
#   make            build everything (mirage + mirage-posedump)
#   make posedump   build just the pose test tool (no wayland/GL needed)
#   make protocols  generate wayland protocol glue only
#   make clean

CC      ?= cc
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
CFLAGS  ?= -O2 -g
CFLAGS  += -std=c11 -D_GNU_SOURCE -Wall -Wextra -Isrc -Ibuild/proto -pthread \
           $(shell $(PKGCONF) --cflags $(RENDER_PKGS))
LDLIBS  := $(shell $(PKGCONF) --libs $(RENDER_PKGS)) -lm -pthread -lrt

# ---- core mirage objects (wayland + GL) ----
MIRAGE_SRC := src/main.c src/pose.c src/capture.c src/render.c \
              src/layout.c src/grab.c src/config.c src/layouts.c
MIRAGE_OBJ := $(MIRAGE_SRC:src/%.c=build/obj/%.o) $(PROTO_OBJ)

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
	$(CC) $(CFLAGS) -o $@ $^ $(LDLIBS)

mirage-posedump: $(POSEDUMP_OBJ)
	$(CC) $(CFLAGS) -o $@ $^ -lm -pthread

# protocol codegen
build/proto/%-client-protocol.h: %.xml | build/proto
	$(SCANNER) client-header $< $@
build/proto/%-protocol.c: %.xml | build/proto
	$(SCANNER) private-code $< $@

# compile generated protocol code
build/obj/%.o: build/proto/%.c | build/obj
	$(CC) $(CFLAGS) -c -o $@ $<

# compile our sources; depend on generated headers so codegen runs first, and on
# our own headers so a struct-layout change (e.g. mirage.h) rebuilds everything
# that includes it - mismatched object layouts cause silent memory corruption.
SRC_HDR := $(wildcard src/*.h)
build/obj/%.o: src/%.c $(PROTO_HDR) $(SRC_HDR) | build/obj
	$(CC) $(CFLAGS) -c -o $@ $<

build/proto build/obj:
	mkdir -p $@

clean:
	rm -rf build mirage mirage-posedump

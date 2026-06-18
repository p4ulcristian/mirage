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
              src/profile.cpp src/calib.cpp src/stb_truetype_impl.cpp src/diag.cpp \
              src/camera.cpp src/worldvio.cpp
MIRAGE_OBJ := $(MIRAGE_SRC:src/%.cpp=build/obj/%.o) $(PROTO_OBJ)
# camera passthrough decodes MJPEG with libturbojpeg
MIRAGE_LIBS := -lturbojpeg

# ---- pose test tool (no wayland/GL) ----
POSEDUMP_OBJ := build/obj/tool_posedump.o build/obj/pose.o build/obj/diag.o

# facecam 6DoF bridge (webcam head-position tracker) - separate target so OpenCV is
# only required when you actually build it, like the rayneo bridge.
OPENCV_CFLAGS := $(shell $(PKGCONF) --cflags opencv4)
# Link only the modules we use - pkg-config --libs opencv4 pulls in viz/hdf (VTK,
# HDF5) which over-link and fail. FaceDetectorYN lives in objdetect over dnn.
OPENCV_LIBS   := -lopencv_core -lopencv_imgproc -lopencv_videoio \
                 -lopencv_objdetect -lopencv_dnn

# mirage's worldvio (6DoF-lite) optional OpenCV LK backend: needs core/imgproc/video.
# Headers go to every TU (just -I, harmless); only worldvio.o uses the symbols.
CXXFLAGS    += $(OPENCV_CFLAGS)
MIRAGE_LIBS += -lopencv_core -lopencv_imgproc -lopencv_video -lopencv_calib3d

.PHONY: all posedump protocols bridge viture viture-vio facecam clean
all: mirage mirage-posedump
posedump: mirage-posedump
bridge: rayneo-bridge
viture: viture-bridge
viture-vio: viture-vio-bin
facecam: facecam-bridge
protocols: $(PROTO_HDR) $(PROTO_SRC)

# VITURE Carina VIO (OpenVINS) bring-up harness. Links libcarina_vio.so directly
# (the carina_vio_* exports are C-linkage but take libstdc++ types by ref, matching
# our g++ __cxx11 ABI). rpath so it finds the .so next to the repo at runtime.
SDK_DIR ?= viture-sdk
viture-vio-bin: src/viture_vio.cpp src/camera.cpp src/camera.h
	$(CXX) -O2 -g -std=gnu++23 -Wall -Wextra -Isrc -o viture-vio \
	    src/viture_vio.cpp src/camera.cpp \
	    -L$(SDK_DIR) -lcarina_vio -Wl,-rpath,'$$ORIGIN/$(SDK_DIR)' \
	    -lturbojpeg -ldl -pthread

rayneo-bridge: src/rayneo_bridge.c src/rayneo.c src/magcal.c src/rayneo.h src/magcal.h
	$(CC) -O2 -g -std=c11 -D_GNU_SOURCE -Wall -Wextra -Isrc \
	    -o $@ src/rayneo_bridge.c src/rayneo.c src/magcal.c -lm

# VITURE (Beast) head-tracking bridge. dlopen()s VITURE's v2.0.0 aarch64 SDK
# (libglasses.so) at runtime (so it builds with no SDK present, only -ldl) and fuses
# the Beast's raw IMU with VQF (src/vqf, C++) - with the RayNeo Madgwick kept as an
# A/B fallback. C sources compiled with $(CC), VQF + shim with $(CXX); linked with $(CXX).
VB_C_OBJ   := build/obj/vb_bridge.o build/obj/vb_rayneo.o build/obj/vb_magcal.o
VB_CPP_OBJ := build/obj/vb_vqf.o build/obj/vb_vqf_shim.o
VB_CFLAGS  := -O2 -g -std=c11 -D_GNU_SOURCE -Wall -Wextra -Isrc
VB_CXXFLAGS:= -O2 -g -std=gnu++17 -Wall -Wextra -Isrc

build/obj/vb_bridge.o: src/viture_bridge.c src/rayneo.h src/magcal.h src/vqf_shim.h | build/obj
	$(CC) $(VB_CFLAGS) -c -o $@ $<
build/obj/vb_rayneo.o: src/rayneo.c src/rayneo.h | build/obj
	$(CC) $(VB_CFLAGS) -c -o $@ $<
build/obj/vb_magcal.o: src/magcal.c src/magcal.h | build/obj
	$(CC) $(VB_CFLAGS) -c -o $@ $<
build/obj/vb_vqf.o: src/vqf/vqf.cpp src/vqf/vqf.hpp | build/obj
	$(CXX) $(VB_CXXFLAGS) -c -o $@ $<
build/obj/vb_vqf_shim.o: src/vqf_shim.cpp src/vqf_shim.h src/vqf/vqf.hpp | build/obj
	$(CXX) $(VB_CXXFLAGS) -c -o $@ $<

viture-bridge: $(VB_C_OBJ) $(VB_CPP_OBJ)
	$(CXX) -O2 -g -o $@ $^ -ldl -lm

facecam-bridge: src/facecam_bridge.cpp
	$(CXX) -O2 -g -std=gnu++23 -Wall -Wextra $(OPENCV_CFLAGS) \
	    -o $@ $< $(OPENCV_LIBS)

mirage: $(MIRAGE_OBJ)
	$(CXX) $(CXXFLAGS) -o $@ $^ $(LDLIBS) $(MIRAGE_LIBS)

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
	rm -rf build mirage mirage-posedump rayneo-bridge viture-bridge facecam-bridge

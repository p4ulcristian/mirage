# Vendored: VQF (Versatile Quaternion-based Filter)

`vqf.cpp` / `vqf.hpp` are vendored verbatim from the official reference
implementation:

- Upstream: https://github.com/dlaidig/vqf  (`vqf/cpp/`)
- Author:   Daniel Laidig <laidig@control.tu-berlin.de>
- License:  MIT (see the SPDX headers in each file)
- Paper:    D. Laidig and T. Seel, "VQF: Highly Accurate IMU Orientation
            Estimation with Bias Estimation and Magnetic Disturbance Rejection,"
            Information Fusion, 2023. https://arxiv.org/abs/2203.17024

Used by the VITURE Beast bridge (`src/viture_bridge.c` via `src/vqf_shim.*`) as the
3DoF AHRS: runtime gyro-bias estimation + filtered tilt correction + (9-axis)
magnetic-disturbance rejection. Replaces the older RayNeo Madgwick filter.

MIT License, Copyright (c) 2021 Daniel Laidig. Permission is hereby granted, free
of charge, to any person obtaining a copy of this software to deal in it without
restriction; see https://github.com/dlaidig/vqf/blob/main/LICENSE-SPDX/MIT.txt

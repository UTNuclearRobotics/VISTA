#!/usr/bin/env python3
"""Generate a 200x200m procedural seabed mesh as an OBJ file."""

import os
import numpy as np

# ── Parameters ────────────────────────────────────────────────────────────────
SIZE        = 200       # metres (square)
RESOLUTION  = 1.0       # metres between vertices
AMPLITUDE   = 2.5       # metres, gentle rolling relief
NUM_HILLS   = 6         # scattered mounds and depressions
HILL_SIGMA  = 28.0      # metres, wide/fat for rolling character
ROUGHNESS   = 0.25      # metres, small-scale seabed texture
SEED        = 42
OUTPUT      = os.path.join(os.path.dirname(os.path.abspath(__file__)), "seabed_200x200m.obj")
# ──────────────────────────────────────────────────────────────────────────────

rng = np.random.default_rng(SEED)

n = int(SIZE / RESOLUTION) + 1
xs = np.linspace(0, SIZE, n)
ys = np.linspace(0, SIZE, n)
xx, yy = np.meshgrid(xs, ys)
zz = np.zeros((n, n))

# Scattered rolling hills and valleys
for _ in range(NUM_HILLS):
    cx = rng.uniform(20, SIZE - 20)
    cy = rng.uniform(20, SIZE - 20)
    a  = rng.uniform(0.5, 1.0) * AMPLITUDE * rng.choice([-1, 1])
    zz += a * np.exp(-((xx - cx)**2 + (yy - cy)**2) / (2 * HILL_SIGMA**2))

# Small-scale roughness via summed sine waves
for _ in range(20):
    fx    = rng.uniform(0.02, 0.12)
    fy    = rng.uniform(0.02, 0.12)
    phase = rng.uniform(0, 2 * np.pi)
    amp   = rng.uniform(0.3, 1.0) * ROUGHNESS
    zz   += amp * np.sin(2 * np.pi * fx * xx + 2 * np.pi * fy * yy + phase)

# Write OBJ
with open(OUTPUT, "w") as f:
    f.write(f"# Procedural seabed {SIZE}x{SIZE}m\n")
    for j in range(n):
        for i in range(n):
            f.write(f"v {xx[j,i]:.3f} {yy[j,i]:.3f} {zz[j,i]:.3f}\n")
    for j in range(n - 1):
        for i in range(n - 1):
            v00 = j * n + i + 1
            v10 = j * n + (i + 1) + 1
            v01 = (j + 1) * n + i + 1
            v11 = (j + 1) * n + (i + 1) + 1
            f.write(f"f {v00} {v10} {v11}\n")
            f.write(f"f {v00} {v11} {v01}\n")

print(f"Written {OUTPUT}  ({n}x{n} vertices, {2*(n-1)**2} triangles)")

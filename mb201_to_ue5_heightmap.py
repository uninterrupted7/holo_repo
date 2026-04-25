#!/usr/bin/env python3
"""
mb201_to_ue5_heightmap.py
=========================
Converts .mb201 multibeam sonar data → 16-bit grayscale PNG
suitable for Unreal Engine 5.5 heightmap import.

Pipeline:
  .mb201  →  mbgrid (MB-System)  →  NetCDF/GMT grid  →  16-bit PNG

Requirements:
  - MB-System  (https://www.mbari.org/technology/mb-system/)
  - Python 3.8+
  - pip install numpy pillow scipy rasterio

Usage:
  python3 mb201_to_ue5_heightmap.py input.mb201 [options]

Examples:
  python3 mb201_to_ue5_heightmap.py survey.mb201
  python3 mb201_to_ue5_heightmap.py survey.mb201 --size 2017 --output heightmap.png
  python3 mb201_to_ue5_heightmap.py survey.mb201 --size 4033 --invert --smooth 2
"""

import argparse
import subprocess
import sys
import os
import struct
import tempfile
import shutil
from pathlib import Path

import numpy as np
from PIL import Image
from scipy.ndimage import gaussian_filter, zoom
from scipy.interpolate import griddata

# ---------------------------------------------------------------------------
# Unreal Engine 5 valid heightmap dimensions (must be (n*2^k)+1)
# ---------------------------------------------------------------------------
UE5_VALID_SIZES = [
    127, 253, 505, 1009, 2017, 4033, 8129,   # square
    255, 511, 1023, 2047, 4095, 8191,         # power-of-2 (also accepted)
]
UE5_RECOMMENDED = {
    "small":   505,
    "medium":  1009,
    "large":   2017,
    "xlarge":  4033,
    "xxlarge": 8129,
}


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def check_mbsystem():
    """Verify MB-System is installed."""
    for tool in ["mbinfo", "mbgrid"]:
        if not shutil.which(tool):
            print(f"[ERROR] '{tool}' not found. Install MB-System first:")
            print("        https://www.mbari.org/technology/mb-system/")
            print("        Ubuntu/Debian: sudo apt install mbsystem")
            print("        macOS (brew):  brew install mbsystem")
            sys.exit(1)
    print("[OK] MB-System found")


def run(cmd, desc=""):
    """Run a shell command and stream output."""
    print(f"[RUN] {' '.join(cmd)}" + (f"  # {desc}" if desc else ""))
    result = subprocess.run(cmd, capture_output=True, text=True)
    if result.returncode != 0:
        print(f"[STDERR] {result.stderr.strip()}")
        raise RuntimeError(f"Command failed: {' '.join(cmd)}")
    return result.stdout


def mbgrid_to_numpy(input_file: str, resolution: float, workdir: str):
    """
    Grid the .mb201 file with mbgrid and return a NumPy array.
    Returns (grid_array, nodata_value).
    """
    grid_stem = os.path.join(workdir, "bath_grid")

    # Build mbgrid command
    # -I  : input file (or datalist)
    # -O  : output file stem
    # -R  : region (auto-determined from data → use mbinfo first)
    # -E  : resolution in meters
    # -A2 : grid bathymetry (mode 2 = mean)
    # -G3 : output as NetCDF (COARDS)
    # -F1 : tension parameter for spline
    # -N  : no NaN masking
    cmd_grid = [
        "mbgrid",
        "-I", input_file,
        "-O", grid_stem,
        f"-E{resolution}/{resolution}!",  # resolution in metres
        "-A2",   # mean bathymetry
        "-G3",   # NetCDF output
        "-F1",   # spline interpolation
        "-clipmode=0",
    ]
    run(cmd_grid, "gridding bathymetry")

    # mbgrid writes <stem>.grd
    grd_file = grid_stem + ".grd"
    if not os.path.exists(grd_file):
        # Some MB-System builds use .nc extension
        grd_file = grid_stem + ".nc"
    if not os.path.exists(grd_file):
        raise FileNotFoundError(f"mbgrid output not found at {grid_stem}.grd / .nc")

    print(f"[OK] Grid written to {grd_file}")

    # Read the NetCDF grid with rasterio
    try:
        import rasterio
        with rasterio.open(grd_file) as ds:
            arr = ds.read(1).astype(np.float32)
            nodata = ds.nodata if ds.nodata is not None else -9999.0
    except Exception:
        # Fallback: try scipy netcdf reader
        from scipy.io import netcdf_file
        with netcdf_file(grd_file, "r") as nc:
            # Variable name is usually 'z' or 'Band1'
            for var in ["z", "Band1", "elevation", "depth"]:
                if var in nc.variables:
                    arr = np.array(nc.variables[var][:], dtype=np.float32)
                    break
            else:
                raise KeyError(f"Cannot find depth variable in {grd_file}. Vars: {list(nc.variables.keys())}")
            nodata = -9999.0

    return arr, nodata


def load_xyz_fallback(input_file: str, workdir: str, grid_pixels: int):
    """
    Fallback when MB-System is unavailable: export to XYZ with mblist,
    then interpolate onto a regular grid.
    """
    xyz_file = os.path.join(workdir, "points.xyz")
    run(["mblist", "-I", input_file, "-OXY+Z", "-R"], "export XYZ points")

    print("[INFO] Loading XYZ points …")
    pts = np.loadtxt(xyz_file)
    lon, lat, z = pts[:, 0], pts[:, 1], pts[:, 2]

    lon_grid = np.linspace(lon.min(), lon.max(), grid_pixels)
    lat_grid = np.linspace(lat.min(), lat.max(), grid_pixels)
    gx, gy = np.meshgrid(lon_grid, lat_grid)

    print("[INFO] Interpolating grid (may take a moment) …")
    arr = griddata((lon, lat), z, (gx, gy), method="linear").astype(np.float32)
    return arr, np.nan


# ---------------------------------------------------------------------------
# Grid processing
# ---------------------------------------------------------------------------

def fill_nodata(arr: np.ndarray, nodata) -> np.ndarray:
    """Replace nodata / NaN with nearest-neighbour interpolation."""
    if np.isnan(nodata):
        mask = np.isnan(arr)
    else:
        mask = (arr == nodata) | np.isnan(arr)

    if not mask.any():
        return arr

    print(f"[INFO] Filling {mask.sum():,} nodata cells …")
    # Use nearest-neighbour via scipy griddata
    rows, cols = np.mgrid[0:arr.shape[0], 0:arr.shape[1]]
    valid = ~mask
    filled = griddata(
        (rows[valid], cols[valid]),
        arr[valid],
        (rows, cols),
        method="nearest"
    )
    arr[mask] = filled[mask]
    return arr


def smooth_grid(arr: np.ndarray, sigma: float) -> np.ndarray:
    if sigma > 0:
        print(f"[INFO] Applying Gaussian smooth σ={sigma} …")
        arr = gaussian_filter(arr, sigma=sigma)
    return arr


def resize_to_ue5(arr: np.ndarray, target: int) -> np.ndarray:
    """Resize array to target × target using zoom."""
    if arr.shape[0] == target and arr.shape[1] == target:
        return arr
    print(f"[INFO] Resizing {arr.shape[0]}×{arr.shape[1]} → {target}×{target} …")
    zy = target / arr.shape[0]
    zx = target / arr.shape[1]
    return zoom(arr, (zy, zx), order=3).astype(np.float32)


def normalize_to_uint16(arr: np.ndarray, invert: bool,
                         z_min_override=None, z_max_override=None) -> np.ndarray:
    """
    Normalise depth/elevation array to uint16 [0, 65535].

    UE5 heightmap convention:
      - 32768 = sea level / base height
      - lower values → below base, higher → above base
    For bathymetry (negative Z values) invert=True maps:
      deepest point → 0, shallowest → 65535
    """
    z_min = float(z_min_override) if z_min_override is not None else float(arr.min())
    z_max = float(z_max_override) if z_max_override is not None else float(arr.max())

    if z_max == z_min:
        raise ValueError("Flat data – all depth values are identical. Check your input.")

    print(f"[INFO] Depth range: {z_min:.2f} m  →  {z_max:.2f} m")
    print(f"[INFO] Vertical span: {z_max - z_min:.2f} m")

    norm = (arr - z_min) / (z_max - z_min)   # 0.0 … 1.0

    if invert:
        norm = 1.0 - norm   # flip so deep = dark / low
        print("[INFO] Z inverted (bathymetry mode: deep=0, shallow=65535)")

    u16 = (norm * 65535.0).clip(0, 65535).astype(np.uint16)
    return u16


def save_png16(arr: np.ndarray, path: str):
    """Save uint16 array as 16-bit grayscale PNG (UE5-compatible)."""
    img = Image.fromarray(arr, mode="I;16")
    img.save(path, format="PNG")
    size_mb = os.path.getsize(path) / (1024 * 1024)
    print(f"[OK] Saved {path}  ({arr.shape[1]}×{arr.shape[0]} px, {size_mb:.1f} MB)")


def write_ue5_info(arr_raw, target_size, output_path, invert, smooth, resolution):
    """Print an UE5 import guide."""
    z_min = arr_raw.min()
    z_max = arr_raw.max()
    span_m = abs(z_max - z_min)

    # UE5 Z scale: 65536 units = 512 cm by default per UE docs
    # Real-world scale (cm per unit) = (span_m * 100) / 65536
    z_scale = (span_m * 100.0) / 65535.0 * 100.0  # → scale factor for UE5 dialog

    print("\n" + "=" * 60)
    print("  UNREAL ENGINE 5.5 IMPORT SETTINGS")
    print("=" * 60)
    print(f"  File            : {os.path.basename(output_path)}")
    print(f"  Dimensions      : {target_size} × {target_size}  (verify in import dialog)")
    print(f"  Depth range     : {z_min:.1f} m  →  {z_max:.1f} m")
    print(f"  Vertical span   : {span_m:.1f} m")
    print(f"\n  UE5 Landscape Import Dialog:")
    print(f"    Heightmap File  → select the PNG above")
    print(f"    Scale X / Y     → set to your survey coverage in cm")
    print(f"       e.g. if survey is {resolution*target_size:.0f} m wide → X/Y = {resolution*target_size*100:.0f}")
    print(f"    Scale Z         → {z_scale:.4f}  (preserves real depth in cm)")
    print(f"    Invert          → {'YES (bathymetry: deep=black)' if invert else 'NO'}")
    print("=" * 60)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description="Convert .mb201 multibeam sonar data to UE5 16-bit PNG heightmap"
    )
    parser.add_argument("input", help="Input .mb201 file (or MB-System datalist)")
    parser.add_argument("-o", "--output", default=None,
                        help="Output PNG path (default: <input>_heightmap.png)")
    parser.add_argument("-s", "--size", type=int, default=1009,
                        help=f"Target square size in pixels. UE5 valid: {UE5_VALID_SIZES}. Default: 1009")
    parser.add_argument("-r", "--resolution", type=float, default=5.0,
                        help="mbgrid resolution in metres (default: 5.0)")
    parser.add_argument("--smooth", type=float, default=0.5,
                        help="Gaussian smoothing sigma (0=off, default: 0.5)")
    parser.add_argument("--invert", action="store_true", default=True,
                        help="Invert Z so deep water = low (default: ON for bathymetry)")
    parser.add_argument("--no-invert", dest="invert", action="store_false",
                        help="Disable Z inversion (for elevation/topo data)")
    parser.add_argument("--zmin", type=float, default=None,
                        help="Override minimum depth for normalisation")
    parser.add_argument("--zmax", type=float, default=None,
                        help="Override maximum depth for normalisation")
    parser.add_argument("--workdir", default=None,
                        help="Temp working directory (auto-created if omitted)")
    args = parser.parse_args()

    # ── Validate input ────────────────────────────────────────────────────
    if not os.path.exists(args.input):
        print(f"[ERROR] File not found: {args.input}")
        sys.exit(1)

    if args.size not in UE5_VALID_SIZES and args.size not in [
        2**k + 1 for k in range(6, 14)
    ]:
        print(f"[WARN] {args.size} is not a standard UE5 heightmap size.")
        print(f"       Recommended sizes: {UE5_VALID_SIZES}")

    output_path = args.output or str(
        Path(args.input).with_suffix("").with_name(
            Path(args.input).stem + "_heightmap.png"
        )
    )

    # ── Setup working directory ───────────────────────────────────────────
    cleanup_workdir = False
    workdir = args.workdir
    if workdir is None:
        workdir = tempfile.mkdtemp(prefix="mb201_conv_")
        cleanup_workdir = True

    try:
        print(f"\n{'='*60}")
        print(f"  MB201 → UE5 Heightmap Converter")
        print(f"{'='*60}")
        print(f"  Input      : {args.input}")
        print(f"  Output     : {output_path}")
        print(f"  UE5 size   : {args.size} × {args.size} px")
        print(f"  Resolution : {args.resolution} m/px (gridding)")
        print(f"  Smooth σ   : {args.smooth}")
        print(f"  Invert Z   : {args.invert}")
        print()

        # ── Step 1: Check MB-System ───────────────────────────────────────
        check_mbsystem()

        # ── Step 2: Grid with mbgrid ──────────────────────────────────────
        arr_raw, nodata = mbgrid_to_numpy(args.input, args.resolution, workdir)
        print(f"[INFO] Raw grid shape: {arr_raw.shape[0]} × {arr_raw.shape[1]}")

        # ── Step 3: Fill nodata / NaNs ────────────────────────────────────
        arr = fill_nodata(arr_raw, nodata)

        # ── Step 4: Smooth ────────────────────────────────────────────────
        arr = smooth_grid(arr, args.smooth)

        # ── Step 5: Resize to UE5 target ─────────────────────────────────
        arr = resize_to_ue5(arr, args.size)

        # Keep raw for UE5 info printout
        arr_for_info = arr.copy()

        # ── Step 6: Normalise → uint16 ────────────────────────────────────
        u16 = normalize_to_uint16(arr, args.invert, args.zmin, args.zmax)

        # ── Step 7: Save PNG ──────────────────────────────────────────────
        save_png16(u16, output_path)

        # ── Step 8: Print UE5 import guide ───────────────────────────────
        write_ue5_info(arr_for_info, args.size, output_path,
                       args.invert, args.smooth, args.resolution)

    finally:
        if cleanup_workdir and os.path.exists(workdir):
            shutil.rmtree(workdir, ignore_errors=True)


if __name__ == "__main__":
    main()

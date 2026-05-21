#!/usr/bin/env python3
"""Run the top-level HDL test and compare its final grid to a Python model."""

from __future__ import annotations

import shutil
import subprocess
import sys
from pathlib import Path


CELLS = 64
SOURCE_ADDR = 8
NUM_ITERATIONS = 4
PHASE_STEP = 0x4000
AMPLITUDE_Q313 = 8192
C_E = 717
C_B = 2867
FRAC_BITS = 13
INT16_MIN = -(2**15)
INT16_MAX = 2**15 - 1


def signed16(value: int) -> int:
    value &= 0xFFFF
    if value & 0x8000:
        return value - 0x10000
    return value


def saturate16(value: int) -> int:
    return max(INT16_MIN, min(INT16_MAX, int(value)))


def q313_multiply(a: int, b: int) -> int:
    return saturate16((int(a) * int(b)) >> FRAC_BITS)


def source_from_phase(phase: int) -> int:
    phase &= 0xFFFF
    if phase == 0x0000:
        return 0
    if phase == 0x4000:
        return AMPLITUDE_Q313
    if phase == 0x8000:
        return 0
    if phase == 0xC000:
        return -AMPLITUDE_Q313
    raise ValueError(f"test only supports quadrant phase, got 0x{phase:04x}")


def reference_fields() -> tuple[list[int], list[int]]:
    ey = [0 for _ in range(CELLS)]
    bz = [0 for _ in range(CELLS)]
    phase = 0

    for _ in range(NUM_ITERATIONS):
        phase = (phase + PHASE_STEP) & 0xFFFF
        source = source_from_phase(phase)

        ey_new = ey.copy()
        bz_new = bz.copy()

        for cell in range(1, CELLS - 1):
            delta_h = q313_multiply(C_E, bz[cell + 1] - bz[cell])
            ey_new[cell] = saturate16(ey[cell] + delta_h)

        ey_new[SOURCE_ADDR] = source

        for cell in range(1, CELLS - 1):
            delta_e = q313_multiply(C_B, ey_new[cell] - ey_new[cell + 1])
            bz_new[cell] = saturate16(bz[cell] + delta_e)

        ey_new[0] = 0
        ey_new[-1] = 0
        bz_new[0] = 0
        bz_new[-1] = 0

        ey = [signed16(value) for value in ey_new]
        bz = [signed16(value) for value in bz_new]

    return ey, bz


def read_dump(path: Path) -> list[int]:
    values = [int(line.strip()) for line in path.read_text().splitlines() if line.strip()]
    if len(values) != CELLS:
        raise RuntimeError(f"{path} has {len(values)} values, expected {CELLS}")
    return values


def first_mismatches(actual: list[int], expected: list[int]) -> list[str]:
    mismatches: list[str] = []
    for idx, (actual_value, expected_value) in enumerate(zip(actual, expected)):
        if actual_value != expected_value:
            mismatches.append(
                f"cell {idx}: expected {expected_value}, actual {actual_value}, diff {actual_value - expected_value}"
            )
        if len(mismatches) == 12:
            break
    return mismatches


def run_hdl(repo_root: Path) -> None:
    build_dir = repo_root / "build"
    build_dir.mkdir(exist_ok=True)

    iverilog = shutil.which("iverilog") or shutil.which("iverilog-oss")
    if iverilog is None:
        raise RuntimeError("could not find iverilog or iverilog-oss on PATH")

    vvp = shutil.which("vvp") or shutil.which("vvp-oss")
    if vvp is None:
        raise RuntimeError("could not find vvp or vvp-oss on PATH")

    compile_cmd = [
        iverilog,
        "-g2012",
        "-Wall",
        "-o",
        "build/tb_top_fdtd_reference.vvp",
        "tests/tb_top_fdtd_reference.sv",
        "src/hdl/top_fdtd_system.sv",
        "src/hdl/fsm_controller.sv",
        "src/hdl/fdtd_engine.sv",
        "src/hdl/Ey.sv",
        "src/hdl/Bz.sv",
        "src/hdl/bram_module.v",
        "src/hdl/cordic_generator.v",
    ]
    simulate_cmd = [vvp, "build/tb_top_fdtd_reference.vvp"]

    for cmd in (compile_cmd, simulate_cmd):
        completed = subprocess.run(
            cmd,
            cwd=repo_root,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
        )
        if completed.stdout:
            print(completed.stdout, end="")
        if completed.returncode != 0:
            raise RuntimeError(f"command failed: {' '.join(cmd)}")


def main() -> int:
    repo_root = Path(__file__).resolve().parents[1]
    run_hdl(repo_root)

    actual_ey = read_dump(repo_root / "build/top_fdtd_reference_ey.txt")
    actual_bz = read_dump(repo_root / "build/top_fdtd_reference_bz.txt")
    expected_ey, expected_bz = reference_fields()

    ey_mismatches = first_mismatches(actual_ey, expected_ey)
    bz_mismatches = first_mismatches(actual_bz, expected_bz)

    if ey_mismatches or bz_mismatches:
        print("TOP_REFERENCE_FAIL")
        if ey_mismatches:
            print("Ey mismatches:")
            print("\n".join(ey_mismatches))
        if bz_mismatches:
            print("Bz mismatches:")
            print("\n".join(bz_mismatches))
        return 1

    print("TOP_REFERENCE_PASS")
    print(f"Compared {CELLS} Ey cells and {CELLS} Bz cells after {NUM_ITERATIONS} iterations.")
    return 0


if __name__ == "__main__":
    sys.exit(main())

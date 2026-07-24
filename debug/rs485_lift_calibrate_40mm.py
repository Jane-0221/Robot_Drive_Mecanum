#!/usr/bin/env python3
"""
RS485 lift 40 mm calibration helper.

Observed tuning from bench test:
- Direct PV motion has stop lag, especially at 240 RPM.
- A segmented 80 RPM approach plus 20/10 RPM fine correction reached
  39.77 mm for a 40.00 mm command, so the accepted program tolerance is 0.30 mm.

Workflow:
1. Set the current lift position as program 0 mm.
2. Move upward by about 40 mm using segmented PV commands.
3. Ask for the measured physical distance.
4. Submit CALIBRATE_DISTANCE and CALIBRATE_HEIGHT so future mm commands use
   the measured scale and current height.
"""

from __future__ import annotations

import argparse
import os
import re
import socket
import struct
import subprocess
import sys
import time
from pathlib import Path


CMD_FORWARD = 2
CMD_REVERSE = 3
CMD_STOP = 4
CMD_SET_ZERO = 11
CMD_CALIBRATE_HEIGHT = 16
CMD_CALIBRATE_DISTANCE = 18

STATUS_OFF_POSITION_MM = 28
STATUS_OFF_UNITS_PER_MM = 44
STATUS_OFF_DI_FAULT = 52
STATUS_OFF_LAST_ERROR = 88

PARAM_IDX_MOTION = 3
PARAM_IDX_DI = 5
PARAM_IDX_FAULT = 6
PARAM_IDX_RPM = 7

COMMAND_OFF_MOVE_MM = 8
COMMAND_OFF_TARGET_HEIGHT_MM = 12
COMMAND_OFF_COMMAND_DISTANCE_MM = 28
COMMAND_OFF_ACTUAL_DISTANCE_MM = 32


def find_tool(command: str, candidates: list[str]) -> str:
    for folder in os.environ.get("PATH", "").split(os.pathsep):
        path = Path(folder) / command
        if path.exists():
            return str(path)
    for candidate in candidates:
        if Path(candidate).exists():
            return candidate
    raise FileNotFoundError(command)


def resolve_symbols(nm_path: str, elf_path: Path) -> dict[str, int]:
    names = {
        "rs485_lift_pending_command",
        "rs485_lift_pending_valid",
        "rs485_lift_status_debug",
        "rs485_lift_param_debug",
    }
    output = subprocess.check_output([nm_path, "-n", str(elf_path)], text=True, errors="ignore")
    symbols: dict[str, int] = {}
    for line in output.splitlines():
        parts = line.split()
        if len(parts) >= 3 and parts[-1] in names:
            symbols[parts[-1]] = int(parts[0], 16)
    missing = sorted(names - symbols.keys())
    if missing:
        raise RuntimeError("Missing ELF symbols: " + ", ".join(missing))
    return symbols


class OpenOcdTelnet:
    def __init__(self, port: int):
        self.sock = socket.create_connection(("127.0.0.1", port), timeout=5.0)
        self.sock.settimeout(1.0)
        time.sleep(0.1)
        self._drain()

    def close(self) -> None:
        self.sock.close()

    def _drain(self) -> str:
        data = b""
        while True:
            try:
                chunk = self.sock.recv(4096)
                if not chunk:
                    break
                data += chunk
                if len(chunk) < 4096:
                    break
            except OSError:
                break
        return data.decode("ascii", errors="ignore")

    def command(self, text: str) -> str:
        self.sock.sendall((text + "\n").encode("ascii"))
        time.sleep(0.055)
        return self._drain()

    def mdw(self, address: int) -> int:
        last = ""
        for _ in range(6):
            last = self.command(f"mdw 0x{address:08x} 1")
            match = re.search(r":\s*([0-9a-fA-F]{8})", last)
            if match:
                return int(match.group(1), 16)
            time.sleep(0.08)
        raise RuntimeError("Bad mdw output: " + last)

    def mww(self, address: int, value: int) -> None:
        self.command(f"mww 0x{address:08x} 0x{value & 0xFFFFFFFF:08x}")

    def mwb(self, address: int, value: int) -> None:
        self.command(f"mwb 0x{address:08x} 0x{value & 0xFF:02x}")


def float_to_word(value: float) -> int:
    return struct.unpack("<I", struct.pack("<f", float(value)))[0]


def word_to_float(value: int) -> float:
    return struct.unpack("<f", struct.pack("<I", value & 0xFFFFFFFF))[0]


def signed_i16(value: int) -> int:
    value &= 0xFFFF
    return value if value < 0x8000 else value - 0x10000


class LiftDebug:
    def __init__(self, telnet: OpenOcdTelnet, symbols: dict[str, int]):
        self.t = telnet
        self.sym = symbols

    def read_status(self) -> dict[str, float | int]:
        status = self.sym["rs485_lift_status_debug"]
        params = self.sym["rs485_lift_param_debug"]
        di_fault = self.t.mdw(status + STATUS_OFF_DI_FAULT)
        rpm_raw = self.t.mdw(params + PARAM_IDX_RPM * 4)
        return {
            "position_mm": word_to_float(self.t.mdw(status + STATUS_OFF_POSITION_MM)),
            "units_per_mm": word_to_float(self.t.mdw(status + STATUS_OFF_UNITS_PER_MM)),
            "di": self.t.mdw(params + PARAM_IDX_DI * 4) & 0xFFFF,
            "fault": self.t.mdw(params + PARAM_IDX_FAULT * 4) & 0xFFFF,
            "motion": self.t.mdw(params + PARAM_IDX_MOTION * 4),
            "rpm": signed_i16(rpm_raw),
            "last_error": self.t.mdw(status + STATUS_OFF_LAST_ERROR),
            "status_di": di_fault & 0xFFFF,
            "status_fault": (di_fault >> 16) & 0xFFFF,
        }

    def submit(self, command: int, rpm: int, accel: int = 3000, **floats: float) -> None:
        base = self.sym["rs485_lift_pending_command"]
        self.t.mww(base + 0, (command & 0xFF) | ((rpm & 0xFFFF) << 16))
        self.t.mww(base + 4, accel & 0xFFFF)
        self.t.mww(base + COMMAND_OFF_MOVE_MM, float_to_word(floats.get("move_mm", 0.0)))
        if "target_height_mm" in floats:
            self.t.mww(base + COMMAND_OFF_TARGET_HEIGHT_MM, float_to_word(floats["target_height_mm"]))
        if "command_distance_mm" in floats:
            self.t.mww(base + COMMAND_OFF_COMMAND_DISTANCE_MM, float_to_word(floats["command_distance_mm"]))
        if "actual_distance_mm" in floats:
            self.t.mww(base + COMMAND_OFF_ACTUAL_DISTANCE_MM, float_to_word(floats["actual_distance_mm"]))
        self.t.mwb(self.sym["rs485_lift_pending_valid"], 1)

    def stop(self, rpm: int = 80) -> dict[str, float | int]:
        self.submit(CMD_STOP, rpm)
        time.sleep(0.75)
        return self.read_status()

    def set_zero(self) -> dict[str, float | int]:
        self.submit(CMD_SET_ZERO, 80)
        time.sleep(0.9)
        return self.read_status()

    def start_motion(self, command: int, rpm: int, accel: int = 3000) -> bool:
        expected_motion = 16 if command == CMD_FORWARD else 32
        for attempt in range(1, 5):
            self.submit(command, rpm, accel)
            time.sleep(0.55)
            status = self.read_status()
            print(
                f"start attempt {attempt}: pos={status['position_mm']:.2f} "
                f"motion={status['motion']} rpm={status['rpm']} "
                f"di=0x{status['di']:04x} fault=0x{status['fault']:04x}",
                flush=True,
            )
            if status["motion"] == expected_motion or abs(int(status["rpm"])) > 5:
                return True
        return False

    def run_segment(self, command: int, seconds: float, rpm: int, accel: int = 3000) -> bool:
        if not self.start_motion(command, rpm, accel):
            self.stop(rpm)
            return False
        time.sleep(seconds)
        self.stop(rpm)
        return True


def choose_forward_segment(remaining_mm: float) -> tuple[float, int, int]:
    if remaining_mm > 25.0:
        return 4.5, 80, 3000
    if remaining_mm > 12.0:
        return 2.0, 80, 3000
    if remaining_mm > 6.0:
        return 0.75, 80, 3000
    if remaining_mm > 2.0:
        return 0.45, 20, 1000
    return 0.8, 10, 500


def correct_to_target(lift: LiftDebug, target_mm: float, tolerance_mm: float) -> dict[str, float | int]:
    for _ in range(10):
        status = lift.read_status()
        error = target_mm - float(status["position_mm"])
        if abs(error) <= tolerance_mm:
            return status

        if error > 0.0:
            seconds, rpm, accel = choose_forward_segment(error)
            command = CMD_FORWARD
        else:
            overshoot = abs(error)
            command = CMD_REVERSE
            if overshoot > 6.0:
                seconds, rpm, accel = 0.55, 20, 1000
            elif overshoot > 2.0:
                seconds, rpm, accel = 0.30, 20, 1000
            else:
                seconds, rpm, accel = 0.6, 10, 500

        print(
            f"correction: pos={status['position_mm']:.2f} target={target_mm:.2f} "
            f"error={error:.2f} cmd={command} rpm={rpm} seconds={seconds:.2f}",
            flush=True,
        )
        if not lift.run_segment(command, seconds, rpm, accel):
            break
    return lift.stop()


def start_openocd(openocd_path: str, port: int) -> subprocess.Popen[str]:
    scripts_dir = Path(r"C:\msys64\ucrt64\share\openocd\scripts")
    args = [openocd_path]
    if scripts_dir.exists():
        args += ["-s", str(scripts_dir)]
    args += [
        "-c",
        "adapter driver cmsis-dap",
        "-c",
        "cmsis_dap_vid_pid 0xfaed 0x4870",
        "-c",
        "transport select swd",
        "-f",
        "target/stm32h7x.cfg",
        "-c",
        "adapter speed 100",
        "-c",
        "gdb_port disabled",
        "-c",
        "tcl_port disabled",
        "-c",
        f"telnet_port {port}",
        "-c",
        "init",
    ]
    return subprocess.Popen(args, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, text=True)


def port_open(port: int) -> bool:
    try:
        with socket.create_connection(("127.0.0.1", port), timeout=0.3):
            return True
    except OSError:
        return False


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--distance-mm", type=float, default=40.0)
    parser.add_argument("--tolerance-mm", type=float, default=0.30)
    parser.add_argument("--elf", default=r"build\Omni_damiao.elf")
    parser.add_argument("--port", type=int, default=50002)
    parser.add_argument("--no-apply", action="store_true", help="Move only; do not write calibration.")
    args = parser.parse_args()

    elf_path = Path(args.elf)
    nm_path = find_tool(
        "arm-none-eabi-nm.exe",
        [
            r"C:\ST\STM32CubeCLT_1.21.0\GNU-tools-for-STM32\bin\arm-none-eabi-nm.exe",
            r"C:\Users\Godwin\13.2 Rel1\bin\arm-none-eabi-nm.exe",
        ],
    )
    openocd_path = find_tool("openocd.exe", [r"C:\msys64\ucrt64\bin\openocd.exe"])

    openocd_process: subprocess.Popen[str] | None = None
    if not port_open(args.port):
        openocd_process = start_openocd(openocd_path, args.port)
        for _ in range(40):
            if port_open(args.port):
                break
            time.sleep(0.2)

    telnet = OpenOcdTelnet(args.port)
    try:
        lift = LiftDebug(telnet, resolve_symbols(nm_path, elf_path))
        print(f"Stop first. Calibration move target: {args.distance_mm:.2f} mm")
        lift.stop()

        zero_status = lift.set_zero()
        start_pos = float(zero_status["position_mm"])
        target_pos = start_pos + args.distance_mm
        start_units_per_mm = float(zero_status["units_per_mm"])
        print(f"Program zero set. start={start_pos:.2f} mm target={target_pos:.2f} mm")
        print("Start measuring physical travel now.")

        final_status = correct_to_target(lift, target_pos, args.tolerance_mm)
        program_distance = float(final_status["position_mm"]) - start_pos
        print(
            f"Move finished. program_distance={program_distance:.2f} mm "
            f"target={args.distance_mm:.2f} mm fault=0x{final_status['fault']:04x}"
        )

        if args.no_apply:
            return 0

        measured_text = input("Enter measured physical distance in mm, blank to skip calibration: ").strip()
        if not measured_text:
            print("Calibration skipped.")
            return 0
        measured_mm = float(measured_text)
        if measured_mm <= 0.0:
            raise ValueError("measured distance must be > 0")

        lift.submit(
            CMD_CALIBRATE_DISTANCE,
            80,
            command_distance_mm=program_distance,
            actual_distance_mm=measured_mm,
        )
        time.sleep(1.0)
        lift.submit(CMD_CALIBRATE_HEIGHT, 80, target_height_mm=measured_mm)
        time.sleep(1.0)
        status = lift.read_status()
        print(
            f"Calibration applied. old_units_per_mm={start_units_per_mm:.3f} "
            f"new_units_per_mm={status['units_per_mm']:.3f} "
            f"current_program_height={status['position_mm']:.2f} mm"
        )
        return 0
    finally:
        telnet.close()
        if openocd_process is not None:
            openocd_process.terminate()
            try:
                openocd_process.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                openocd_process.kill()


if __name__ == "__main__":
    sys.exit(main())

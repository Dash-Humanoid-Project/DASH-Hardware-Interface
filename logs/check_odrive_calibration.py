#!/usr/bin/env python3
"""
Check one ODrive's calibration/axis-state over USB, one board at a time.

Written to diagnose the left-arm (Teensy 3) issue where nodes 7/8 never
entered AXIS_STATE_CLOSED_LOOP_CONTROL despite healthy CAN feedback, and
node 6 stayed in closed-loop control after an IdleCommand. Run this against
each of the four arm ODrives (via USB, not CAN) to check calibration state
and confirm the CAN node_id actually flashed on each board.

Usage:
    python3 check_odrive_calibration.py

Plug in one ODrive via USB, let it print, then follow the prompt to
disconnect it and plug in the next one. Ctrl+C to stop.

Requires the same `odrive` Python package odrivetool itself uses. Written
defensively against attribute naming (axis0.current_state / motor.is_calibrated
/ config.can.node_id etc.) since this project's exact ODrive firmware/API
version wasn't confirmed when writing this — if an attribute doesn't exist
on your version it prints "<not found>" instead of crashing, so you may need
to adjust attribute names below to match `dir(odrv.axis0)` on your setup.
"""
import sys

try:
    import odrive
except ImportError:
    print("Couldn't import the `odrive` package. Run this with the same")
    print("Python environment/interpreter that `odrivetool` itself uses.")
    sys.exit(1)


def get(obj, attr):
    try:
        return getattr(obj, attr)
    except AttributeError:
        return "<not found on this firmware/API version>"


def describe_axis(axis, label):
    print(f"  {label}:")
    print(f"    current_state              = {get(axis, 'current_state')}  "
          f"(8 = CLOSED_LOOP_CONTROL, 1 = IDLE, on most fw versions)")
    print(f"    error                       = {get(axis, 'error')}  (0 = no error)")

    motor = get(axis, "motor")
    if not isinstance(motor, str):
        print(f"    motor.is_calibrated         = {get(motor, 'is_calibrated')}")
        print(f"    motor.error                 = {get(motor, 'error')}")
    else:
        print(f"    motor.*                     = {motor}")

    encoder = get(axis, "encoder")
    if not isinstance(encoder, str):
        print(f"    encoder.is_ready            = {get(encoder, 'is_ready')}")
        print(f"    encoder.error               = {get(encoder, 'error')}")
    else:
        print(f"    encoder.*                   = {encoder}")

    config = get(axis, "config")
    if not isinstance(config, str):
        print(f"    config.startup_motor_calibration           = {get(config, 'startup_motor_calibration')}")
        print(f"    config.startup_encoder_offset_calibration  = {get(config, 'startup_encoder_offset_calibration')}")
        print(f"    config.startup_closed_loop_control         = {get(config, 'startup_closed_loop_control')}")
        can_cfg = get(config, "can")
        if not isinstance(can_cfg, str):
            print(f"    config.can.node_id                         = {get(can_cfg, 'node_id')}  "
                  f"(confirm this matches what you expect for this physical board)")
        else:
            # Older API: node_id may live directly on the axis, not config.can
            print(f"    can_node_id (axis-level)                   = {get(axis, 'can_node_id')}")
    else:
        print(f"    config.*                    = {config}")


def main():
    while True:
        print("\nWaiting for an ODrive on USB (Ctrl+C to stop)...")
        try:
            odrv = odrive.find_any(timeout=30)
        except KeyboardInterrupt:
            print("Stopped.")
            return
        except Exception as e:
            print(f"find_any() failed or timed out: {e}")
            continue

        print(f"Found ODrive, serial_number={get(odrv, 'serial_number')}, "
              f"fw_version={get(odrv, 'fw_version_major')}.{get(odrv, 'fw_version_minor')}.{get(odrv, 'fw_version_revision')}")

        for axis_name in ("axis0", "axis1"):
            axis = get(odrv, axis_name)
            if isinstance(axis, str):
                continue
            describe_axis(axis, axis_name)

        try:
            input("\nUnplug this ODrive, plug in the next one, then press Enter (Ctrl+C to stop)...")
        except KeyboardInterrupt:
            print("\nStopped.")
            return


if __name__ == "__main__":
    main()

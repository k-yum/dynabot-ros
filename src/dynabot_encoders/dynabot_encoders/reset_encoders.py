#!/usr/bin/env python3
import subprocess
import time

def flash_teensy():
    # Path to your compiled HEX file
    hex_file = "/home/igv/.cache/arduino/sketches/F1B23A3F4F21A06D93A0EF14DE4A65AD/encoder_count.ino.hex"
    # Specify the MCU type for your Teensy board (adjust --mcu value as needed)
    cmd = ["teensy_loader_cli", "--mcu=imxrt1062", "-w", "-v", hex_file]
    try:
        # Run the command and wait for it to complete
        subprocess.run(cmd, check=True)
        print("Teensy flashed successfully.")
    except subprocess.CalledProcessError as e:
        print("Error flashing Teensy:", e)

def main():
    # Optional: wait a bit if needed (e.g., for the micro-ROS agent to initialize)
    time.sleep(3)
    flash_teensy()

if __name__ == '__main__':
    main()

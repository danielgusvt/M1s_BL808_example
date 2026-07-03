#!/usr/bin/env python3
"""
Capture grayscale JPEGs streamed by the STM32 LoRa receiver over serial and
display them live.

The STM32 prints each image as:
    JPEG_BEGIN len=<N> chunks=<C> snr=<S> CONFIG1=0x.. CONFIG2=0x.. CONFIG3=0x..
    <hex line per chunk>
    ...
    JPEG_END received=<r>/<C> gaps=<g> time=<t>ms
This script reassembles the hex between the markers, decodes the JPEG, shows it
in a window, and saves it as rx_<n>.jpg.

Usage:   python3 lora_image_rx.py [serial_port]   (default /dev/ttyACM0)
Deps:    pip install pyserial opencv-python numpy
"""
import binascii
import re
import sys

import cv2
import numpy as np
import serial

PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyACM0"
BAUD = 115200

HEX_RE = re.compile(rb"^[0-9a-fA-F]+$")


def show(data, count):
    """Decode JPEG bytes, save and display. Returns True on success."""
    img = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_GRAYSCALE)
    if img is None:
        print("  ! decode failed (incomplete / corrupt JPEG)")
        return False
    fname = f"rx_{count:03d}.jpg"
    with open(fname, "wb") as f:
        f.write(data)
    print(f"  saved {fname}  ({len(data)} bytes, {img.shape[1]}x{img.shape[0]})")
    cv2.imshow("LoRa image", img)
    cv2.waitKey(1)
    return True


def main():
    ser = serial.Serial(PORT, BAUD, timeout=1)
    print(f"Listening on {PORT} @ {BAUD} baud  (Ctrl-C to quit)")

    collecting = False
    hex_parts = []
    count = 0

    while True:
        line = ser.readline().strip()
        if not line:
            continue

        if line.startswith(b"JPEG_BEGIN"):
            collecting = True
            hex_parts = []
            print(line.decode(errors="replace"))
        elif line.startswith(b"JPEG_END"):
            print(line.decode(errors="replace"))
            if collecting:
                collecting = False
                try:
                    data = binascii.unhexlify(b"".join(hex_parts))
                    if show(data, count + 1):
                        count += 1
                except (binascii.Error, ValueError) as e:
                    print(f"  ! bad hex stream: {e}")
        elif collecting and HEX_RE.match(line):
            hex_parts.append(line)
        else:
            # passthrough for any other receiver log lines
            print(line.decode(errors="replace"))


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()

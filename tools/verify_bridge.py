#!/usr/bin/env python3
"""Check real USB serial through the bridge using a JustFloat TCP reader."""

import argparse
import contextlib
import math
from pathlib import Path
import socket
import struct
import subprocess
import sys
import time

from vofa_bridge import JUST_FLOAT_DELIMITER


def verify(bridge_port, seconds, serial_port):
    build = Path("build")
    build.mkdir(exist_ok=True)
    wave = build / "bridge-verified-wave.csv"
    log = build / "bridge-runtime.log"
    command = [sys.executable, "-u", "tools/vofa_bridge.py", "--tcp-port", str(bridge_port),
               "--wave-file", str(wave)]
    if serial_port:
        command += ["--port", serial_port]
    with log.open("w") as output:
        bridge = subprocess.Popen(command, stdout=output, stderr=subprocess.STDOUT)
        try:
            measure_connection(bridge, bridge_port, seconds, wave, log)
        finally:
            if bridge.poll() is None:
                bridge.send_signal(2)
                with contextlib.suppress(subprocess.TimeoutExpired):
                    bridge.wait(3)
            if bridge.poll() is None:
                bridge.terminate()
                bridge.wait()


def connect(bridge, port, log):
    for _ in range(80):
        try:
            return socket.create_connection(("127.0.0.1", port), 0.2)
        except ConnectionRefusedError:
            assert bridge.poll() is None, log.read_text()
            time.sleep(0.05)
    raise TimeoutError("bridge did not listen")


def measure_connection(bridge, bridge_port, seconds, wave, log):
    with connect(bridge, bridge_port, log) as client:
        client.settimeout(2)
        data = bytearray()
        data.extend(client.recv(65536))
        start = time.monotonic()
        sent_overlong = sent = False
        next_status = 15
        total = 0
        replies = []
        client.sendall(b"TB 500us\n")
        while time.monotonic() - start < seconds:
            data.extend(client.recv(65536))
            if len(data) >= 1024 * 1024:
                points, text, consumed = scan_frames(data)
                total += points
                replies.extend(text)
                del data[:consumed]
            if not sent_overlong and time.monotonic() - start > 1:
                client.sendall(b"x" * 216)
                sent_overlong = True
            if not sent and time.monotonic() - start > 2:
                client.sendall(b"\nSTOP\nWAVE?\nMEAS?\nRUN\nSTAT?\nTRIG -1\nTB 200us\n")
                sent = True
            if time.monotonic() - start >= next_status:
                client.sendall(b"STAT?\n")
                next_status += 15
        elapsed = time.monotonic() - start
    final_points, final_replies, _ = scan_frames(data)
    total += final_points
    replies.extend(final_replies)
    assert any(line == "OK" for line in replies), replies
    assert replies.count("ERR") == 2, replies
    assert any(line.startswith("OK FRAME=") and "Vpp" in line for line in replies), replies
    assert any(line.startswith("WAVE frame=") and "points=240" in line for line in replies), replies
    status = [line for line in replies if line.startswith("OK MS=")]
    assert status and all("ADCERR=0 DMADROP=0 TXDROP=0 RXERR=0" in line for line in status), status
    assert all("LCDSYNC=1 LCDMISS=0" in line for line in status), status
    rate = total / elapsed
    assert 197000 < rate < 204000, (total, elapsed)
    assert wave.is_file() and len(wave.read_text().splitlines()) == 241
    details = log.read_text()
    assert "Stream gap" not in details and "WAVE frame=" in details and "Vpp" in details, details
    assert "ADCERR=0 DMADROP=0 TXDROP=0 RXERR=0" in details, details
    print(f"BRIDGE PASS: CH340 to localhost JustFloat, {total} points at {rate:.0f}/s including interpolation. "
          "TCP OK/ERR/measurement replies, unterminated overlong input, 240-point CSV, and LCD scan sync verified.")


def scan_frames(data):
    """Apply JustFloat's aligned-sample rule to complete frames in one TCP stream."""
    start = total = 0
    replies = []
    read_voltage = struct.Struct("<f").unpack_from
    while (end := data.find(JUST_FLOAT_DELIMITER, start)) >= 0:
        size = end - start
        if size == 4:
            (volts,) = read_voltage(data, start)
            assert math.isfinite(volts) and 0 <= volts <= 3.3001
            total += 1
        else:
            assert size % 4 != 0 and data[end - 2:end] == b"\r\n", data[start:end].hex()
            replies.append(data[start:end].decode("ascii").strip())
        start = end + len(JUST_FLOAT_DELIMITER)
    return total, replies, start


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seconds", type=int, default=15)
    parser.add_argument("--port")
    parser.add_argument("--tcp-port", type=int, default=19000)
    args = parser.parse_args()
    verify(args.tcp_port, args.seconds, args.port)

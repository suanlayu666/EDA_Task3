#!/usr/bin/env python3
"""Translate KF56-10's packed USB serial stream to standard VOFA+ JustFloat."""

import argparse
import binascii
import csv
import glob
import select
import socket
import struct
import sys
import time
from dataclasses import dataclass
from functools import lru_cache
from pathlib import Path

import serial

SYNC = b"\xa5\x5a"
HEADER = struct.Struct("<2sBBHI")
WAVE_HEADER = struct.Struct("<IHHH")
JUST_FLOAT_TAIL = 0x7F800000
JUST_FLOAT_DELIMITER = struct.pack("<I", JUST_FLOAT_TAIL)
MAX_TCP_PENDING = 2_000_000
FEC_PACKETS = 126
FEC_LANES = 6
STREAM_POINTS = 256
STREAM_RANGE = struct.Struct("<HH")
SERIAL_BAUD = 1_200_000


@dataclass(frozen=True)
class Packet:
    kind: int
    channels: int
    sequence: int
    data: bytes
    part: int = 0


class Decoder:
    """Validate CRC and resynchronize across split USB reads."""

    def __init__(self):
        self.buffer = bytearray()
        self.bad_crc = 0
        self.skipped = 0

    def feed(self, incoming):
        self.buffer.extend(incoming)
        while len(self.buffer) >= HEADER.size:
            start = self.buffer.find(SYNC)
            if start < 0:
                self.skipped += len(self.buffer) - 1
                del self.buffer[:-1]
                break
            if start:
                self.skipped += start
                del self.buffer[:start]
            if len(self.buffer) < HEADER.size:
                break
            _, kind, flags, size, sequence = HEADER.unpack_from(self.buffer)
            stream = kind in (1, 4, 5)
            channels = (flags & 1) + 1 if stream else flags & 3
            part = flags >> 1 if stream else 0
            if kind not in (1, 2, 3, 4, 5) or size > 730 or (kind != 2 and channels not in (1, 2)):
                del self.buffer[0]
                self.skipped += 1
                continue
            end = HEADER.size + size
            if len(self.buffer) < end + 2:
                break
            expected = struct.unpack_from("<H", self.buffer, end)[0]
            if binascii.crc_hqx(self.buffer[2:end], 0xFFFF) != expected:
                self.bad_crc += 1
                del self.buffer[0]
                continue
            payload = bytes(self.buffer[HEADER.size:end])
            del self.buffer[:end + 2]
            yield Packet(kind, channels, sequence, payload, part)


def open_serial(path, baud):
    """Use the board's CH340C with no application-controlled RTS/DTR."""
    port = serial.Serial(
        port=None, baudrate=baud, timeout=0, write_timeout=0.2, exclusive=True
    )
    port.port = path
    port.dtr = port.rts = False
    port.open()
    return port


def select_port(path):
    """Pick the only attached USB serial adapter when the user supplies no path."""
    if path:
        return path
    paths = sorted(glob.glob("/dev/cu.usbserial*") + glob.glob("/dev/cu.wchusbserial*"))
    if len(paths) != 1:
        raise SystemExit(f"Specify --port. Detected candidates: {', '.join(paths) or '(none)'}")
    return paths[0]


def unpack_samples(data):
    """Recover exactly two unsigned ADC codes from each three-byte group."""
    return (
        code
        for a, b, c in struct.iter_unpack("BBB", data)
        for code in (a | ((b & 15) << 8), (b >> 4) | (c << 4))
    )


def unpack_stream(packet):
    """Expand block-relative codes back into ADC-code units."""
    channels = packet.channels
    data = packet.data
    codes = data[STREAM_RANGE.size * channels:]
    ranges = [STREAM_RANGE.unpack_from(data, STREAM_RANGE.size * ch) for ch in range(channels)]
    if channels == 1:
        low, span = ranges[0]
        scale = span / 255.0
        return [low + value * scale for value in codes]
    (low_a, span_a), (low_b, span_b) = ranges
    scale_a, scale_b = span_a / 255.0, span_b / 255.0
    out = []
    for a, b in struct.iter_unpack("BB", codes):
        out.extend((low_a + a * scale_a, low_b + b * scale_b))
    return out


def point_count(packet):
    """Count time samples per channel in a packed stream packet."""
    return len(packet.data) // packet.channels - STREAM_RANGE.size


def gf_multiply(a, b):
    """Multiply two bytes in GF(256) with modulus 0x11d."""
    result = 0
    while b:
        if b & 1:
            result ^= a
        a = (a << 1) ^ (0x11D if a & 0x80 else 0)
        b >>= 1
    return result


@lru_cache(maxsize=256)
def multiplication_table(factor):
    """Return a byte translation table for one field multiplier."""
    return bytes(gf_multiply(factor, x) for x in range(256))


@lru_cache(maxsize=1)
def parity_weights():
    """Match the firmware's repeated Q = 2*Q + payload recurrence."""
    weights = [1]
    for _ in range(FEC_PACKETS // FEC_LANES - 1):
        weights.append(gf_multiply(2, weights[-1]))
    return weights[::-1]


class PacketRecovery:
    """Recover up to two missing payloads per interleaved parity lane."""

    def __init__(self):
        self.key = None
        self.group = {}
        self.parity = {}
        self.next_part = 0
        self.recovered = 0
        self.unrecoverable = 0

    def residual(self, kind, lane, check):
        """Remove received terms from a parity value, leaving missing terms."""
        remaining = int.from_bytes(check, "little")
        weights = parity_weights()
        for i in range(lane, FEC_PACKETS, FEC_LANES):
            packet = self.group.get(i)
            if packet is None:
                continue
            term = packet.data
            if kind == 5:
                term = term.translate(multiplication_table(weights[i // FEC_LANES]))
            remaining ^= int.from_bytes(term, "little")
        return remaining.to_bytes(len(check), "little")

    def recover(self, lane):
        """Solve the two parity equations for missing payloads in one lane."""
        missing = [i for i in range(lane, FEC_PACKETS, FEC_LANES) if i not in self.group]
        checks = self.parity[lane]
        if not missing or len(missing) > len(checks):
            return
        weights = parity_weights()
        weight_a = weights[missing[0] // FEC_LANES]
        if len(missing) == 1:
            if 4 in checks:
                payloads = (self.residual(4, lane, checks[4]),)
            else:
                table = multiplication_table(weight_a)
                payloads = (self.residual(5, lane, checks[5]).translate(bytes(table.index(x) for x in range(256))),)
        else:
            weight_b = weights[missing[1] // FEC_LANES]
            p = self.residual(4, lane, checks[4])
            q = self.residual(5, lane, checks[5])
            b_times_p = p.translate(multiplication_table(weight_b))
            difference = multiplication_table(weight_a ^ weight_b)
            inverse = bytes(difference.index(x) for x in range(256))
            lost_a = bytes(x ^ y for x, y in zip(q, b_times_p)).translate(inverse)
            lost_b = bytes(x ^ y for x, y in zip(p, lost_a))
            payloads = lost_a, lost_b

        channels, first = self.key
        mask = 0x7FFFFFFF if channels == 1 else 0x3FFFFFFF
        for i, payload in zip(missing, payloads):
            self.group[i] = Packet(1, channels, (first + i * 256) & mask, payload, i)
        self.recovered += len(missing)

    def feed(self, packet):
        if packet.kind in (2, 3):
            yield packet
            return
        is_sample = packet.kind == 1
        limit = FEC_PACKETS if is_sample else FEC_LANES
        if packet.part >= limit or len(packet.data) != (STREAM_POINTS + STREAM_RANGE.size) * packet.channels:
            return
        mask = 0x7FFFFFFF if packet.channels == 1 else 0x3FFFFFFF
        first = (packet.sequence - STREAM_POINTS * packet.part) & mask if is_sample else packet.sequence
        key = (packet.channels, first)
        if self.key != key:
            if self.key and self.key[0] == key[0] and self.group and self.next_part < FEC_PACKETS:
                self.unrecoverable += FEC_PACKETS - len(self.group)
                yield from (self.group[i] for i in sorted(self.group) if i >= self.next_part)
            self.key = key
            self.group.clear()
            self.parity.clear()
            self.next_part = 0
        if self.next_part == FEC_PACKETS:
            return

        if is_sample:
            self.group[packet.part] = packet
        else:
            lane = packet.part
            self.parity.setdefault(lane, {})[packet.kind] = packet.data
            self.recover(lane)

        # Forward each contiguous packet immediately. Only wait for parity at a gap.
        while self.next_part in self.group:
            yield self.group[self.next_part]
            self.next_part += 1


class JustFloat:
    """Render real samples; optionally add an interpolated midpoint for each CH1 interval."""

    def __init__(self, raw_points):
        self.raw_points = raw_points
        self.tail = []

    def reset(self):
        self.tail.clear()

    def encode(self, packet, reference_mv):
        scale = reference_mv / 4_095_000
        codes = unpack_stream(packet)
        if packet.channels == 2:
            out = bytearray(len(codes) // 2 * 12)
            for offset, (a, b) in enumerate(zip(codes[::2], codes[1::2])):
                struct.pack_into("<ffI", out, offset * 12, a * scale, b * scale, JUST_FLOAT_TAIL)
        elif self.raw_points:
            out = bytearray(len(codes) * 8)
            for offset, code in enumerate(codes):
                struct.pack_into("<fI", out, offset * 8, code * scale, JUST_FLOAT_TAIL)
        else:
            # Two look-behind and three look-ahead points keep interpolation continuous
            # across packets. Original samples remain at every even plotted index.
            values = (self.tail or [codes[0]] * 2) + codes
            out = bytearray((len(values) - 5) * 16)
            for i in range(2, len(values) - 3):
                a, b, c, d, e, f = values[i - 2:i + 4]
                midpoint = (3 * (a + f) - 25 * (b + e) + 150 * (c + d)) / 256
                midpoint = max(0, min(4095, midpoint))
                struct.pack_into("<fIfI", out, (i - 2) * 16, c * scale, JUST_FLOAT_TAIL,
                                 midpoint * scale, JUST_FLOAT_TAIL)
            self.tail = values[-5:]
        return out


def encode_reply(text):
    """Keep byte-console text distinct from 4-byte JustFloat samples."""
    body = text.encode("ascii", "replace")
    if (len(body) + 2) % 4 == 0:
        body += b" "
    return body + b"\r\n" + JUST_FLOAT_DELIMITER


def save_wave(packet, path):
    """Write the queried capture on its original time span, separate from live data."""
    span, points, mv, triggered = WAVE_HEADER.unpack_from(packet.data)
    summary = f"WAVE frame={packet.sequence} points={points} span={span}us triggered={bool(triggered)}"
    if not path:
        return summary
    channels = packet.channels
    scale = mv / 4_095_000
    codes = iter(unpack_samples(packet.data[WAVE_HEADER.size:]))
    with path.open("w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(["time_us", *[f"CH{i + 1}_V" for i in range(channels)]])
        for i in range(points):
            writer.writerow([i * span / (points - 1), *(next(codes) * scale for _ in range(channels))])
    return summary


def send_command(port, line):
    """Forward one complete line to the firmware."""
    line = line.rstrip(b"\r\n")
    if line:
        port.write(line + b"\n")
        if line.upper().startswith(b"VREF "):
            port.write(b"CONF?\n")


def run_bridge(port, args):
    """Relay the real serial stream to VOFA+, and return commands to the same port."""
    decoder = Decoder()
    recovery = PacketRecovery()
    encoder = JustFloat(args.raw_points)
    pending = bytearray()
    command_tail = bytearray()
    refresh_reference = False
    client = None
    next_sequence = None
    reference = args.reference_mv
    last_channels = 0
    retry_at = time.monotonic() + 0.35
    with socket.socket() as server:
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind(("127.0.0.1", args.tcp_port))
        server.listen(1)
        print(f"Serial {port.port} @ {port.baudrate}; VOFA+ TCP Client 127.0.0.1:{args.tcp_port}, JustFloat", flush=True)
        while True:
            inputs = [server, port]
            if client:
                inputs.append(client)
            if sys.stdin.isatty():
                inputs.append(sys.stdin)
            readable, writable, _ = select.select(inputs, [client] if client and pending else [], [], 0.10)
            if server in readable:
                new_client, _ = server.accept()
                new_client.setblocking(False)
                if client:
                    client.close()
                client = new_client
                pending.clear()
                command_tail.clear()
                refresh_reference = False
                encoder.reset()
                print("VOFA+ TCP connected", flush=True)
            if client and client in readable:
                try:
                    message = client.recv(4096)
                except ConnectionResetError:
                    message = b""
                except BlockingIOError:
                    message = None
                if message:
                    # Keep TCP input transparent: the firmware sees overlong or partial lines.
                    port.write(message)
                    command_tail.extend(message.replace(b"\r", b"\n"))
                    lines = command_tail.split(b"\n")
                    command_tail = lines.pop()
                    refresh_reference |= any(line.lstrip().upper().startswith(b"VREF ") for line in lines)
                    if refresh_reference and not command_tail:
                        port.write(b"CONF?\n")
                        refresh_reference = False
                    if len(command_tail) > 4096:
                        command_tail.clear()
                elif message == b"":
                    client.close()
                    client = None
                    pending.clear()
            if sys.stdin in readable:
                send_command(port, sys.stdin.readline().encode("ascii", "replace"))
            if port in readable:
                previous_recovered = recovery.recovered
                for raw_packet in decoder.feed(port.read(32768)):
                    for packet in recovery.feed(raw_packet):
                        if packet.kind == 2:
                            line = packet.data.decode("ascii", "replace")
                            print(line, flush=True)
                            if client:
                                pending.extend(encode_reply(line))
                            for field in line.split():
                                if field.startswith("VREF=") and field.endswith("mV"):
                                    reference = int(field[5:-2])
                        elif packet.kind == 3:
                            summary = save_wave(packet, args.wave_file)
                            print(summary, flush=True)
                            if client:
                                pending.extend(encode_reply(summary))
                        else:
                            gap = (packet.channels == last_channels and next_sequence is not None
                                   and packet.sequence != next_sequence)
                            if gap:
                                print(f"Stream gap: expected {next_sequence}, got {packet.sequence}", file=sys.stderr, flush=True)
                            if gap or packet.channels != last_channels:
                                encoder.reset()
                            mask = 0x7FFFFFFF if packet.channels == 1 else 0x3FFFFFFF
                            next_sequence = (packet.sequence + point_count(packet)) & mask
                            last_channels = packet.channels
                            if client:
                                pending.extend(encoder.encode(packet, reference))
                if recovery.recovered > previous_recovered:
                    print(f"Recovered serial packets: {recovery.recovered}", flush=True)
            if client and client in writable and pending:
                try:
                    del pending[:client.send(pending)]
                except (BrokenPipeError, ConnectionResetError):
                    client.close()
                    client = None
                    pending.clear()
                except BlockingIOError:
                    pass
            if len(pending) > MAX_TCP_PENDING:
                print("VOFA+ stopped reading; reconnect to resume", file=sys.stderr, flush=True)
                client.close()
                client = None
                pending.clear()
            if not last_channels and time.monotonic() >= retry_at:
                port.write(f"CONF?\nSTREAM {args.channels}\n".encode())
                retry_at = time.monotonic() + 0.75


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", help="Board serial port, such as /dev/cu.usbserial-1130")
    parser.add_argument("--baud", type=int, default=SERIAL_BAUD)
    parser.add_argument("--tcp-port", type=int, default=9000)
    parser.add_argument("--channels", type=int, choices=(1, 2), default=1)
    parser.add_argument("--reference-mv", type=int, default=3300)
    parser.add_argument("--raw-points", action="store_true", help="Send only received samples without curve interpolation")
    parser.add_argument("--wave-file", type=Path, help="CSV output from WAVE?")
    args = parser.parse_args()
    with open_serial(select_port(args.port), args.baud) as port:
        try:
            run_bridge(port, args)
        except KeyboardInterrupt:
            port.write(b"STREAM OFF\n")


if __name__ == "__main__":
    main()

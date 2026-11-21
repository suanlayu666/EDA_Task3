#!/usr/bin/env python3
"""Run the command and transport checks that do not require a signal generator."""

import argparse
import collections
import select
import time

from vofa_bridge import Decoder, PacketRecovery, open_serial, point_count, select_port, WAVE_HEADER, SERIAL_BAUD


class Board:
    def __init__(self, port):
        self.port = port
        self.decoder = Decoder()
        self.recovery = PacketRecovery()
        self.text = collections.deque()
        self.last_wave = None
        self.next_point = None
        self.points = self.gaps = 0

    def read(self, timeout=0.05):
        if not select.select([self.port], [], [], timeout)[0]:
            return
        for raw_packet in self.decoder.feed(self.port.read(32768)):
            for packet in self.recovery.feed(raw_packet):
                if packet.kind == 1:
                    self.points += point_count(packet)
                    if self.next_point is not None and self.next_point != packet.sequence:
                        self.gaps += 1
                    self.next_point = (packet.sequence + point_count(packet)) & 0xFFFFFFFF
                elif packet.kind == 2:
                    self.text.append(packet.data.decode("ascii"))
                elif packet.kind == 3:
                    self.last_wave = packet

    def pump(self, seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            self.read(min(0.05, max(0, deadline - time.monotonic())))

    def command(self, line, expected="OK"):
        return self.command_batch(line, (expected,))[0]

    def command_batch(self, data, expected):
        """Send a single burst and verify every reply in order."""
        self.text.clear()
        self.port.write(data)
        deadline = time.monotonic() + 1.5
        replies = []
        while len(replies) < len(expected) and time.monotonic() < deadline:
            if self.text:
                reply = self.text.popleft()
                assert reply.startswith(expected[len(replies)]), (data, reply)
                replies.append(reply)
            else:
                self.read()
        if len(replies) != len(expected):
            raise TimeoutError((data, replies))
        return replies

    def text_command(self, line, expected="OK"):
        return self.command((line + "\n").encode(), expected)


def fields(text):
    return {k: v for item in text.split() if "=" in item for k, v in [item.split("=", 1)]}


def check_commands(board):
    """Exercise text validation, frozen snapshots, and no-trigger mode transitions."""
    board.text_command("MODE AUTO")
    board.text_command("EDGE FALL")
    board.text_command("TB 500us")
    board.text_command("TRIG 0.50")
    board.text_command("VDIV 250mV")
    before = board.text_command("CONF?")
    assert all(s in before for s in ("EDGE=FALL", "TB=500us", "VDIV=250mV", "TRIG=500mV"))
    for line in ("BOGUS", "TRIG -1", "TRIG 4", "TRIG nan", "EDGE BOTH", "TB 77us", "VDIV 0", "MEAS? junk"):
        board.text_command(line, "ERR")
    board.command(b"x" * 300, "ERR")
    board.command(b"\ntrig 0.5", "ERR")
    assert board.text_command("CONF?") == before

    board.pump(0.25)
    board.text_command("STOP")
    before = board.text_command("MEAS?")
    board.text_command("WAVE?")
    assert board.last_wave and board.last_wave.channels == 1
    span, points, _, _ = WAVE_HEADER.unpack_from(board.last_wave.data)
    assert (span, points) == (5000, 240), (span, points)
    board.pump(0.2)
    assert board.text_command("MEAS?") == before

    board.text_command("EDGE RISE")
    board.text_command("TRIG 0")
    board.text_command("MODE NORMAL")
    before = fields(board.text_command("STAT?"))
    board.pump(0.3)
    after = fields(board.text_command("STAT?"))
    assert before["FRAME"] == after["FRAME"] and int(before["ADC"]) < int(after["ADC"])
    board.text_command("WAVE?")
    held = board.last_wave
    assert WAVE_HEADER.unpack_from(held.data)[2] == 3300
    board.text_command("VREF 3.25")
    assert "VREF=3250mV" in board.text_command("CONF?")
    board.last_wave = None
    board.text_command("WAVE?")
    assert board.last_wave == held
    board.text_command("VREF 3.3")
    board.text_command("SINGLE")
    assert "SINGLE ARM" in board.text_command("CONF?")
    board.text_command("RUN")
    assert "SINGLE ARM" in board.text_command("CONF?")
    board.text_command("MODE AUTO")
    board.text_command("TRIG 1.5")
    board.text_command("TB 200us")
    board.text_command("VDIV 0.5")
    board.pump(0.25)
    assert int(fields(board.text_command("STAT?"))["FRAME"]) > int(after["FRAME"])
    print("BOARD PASS: RUN/STOP SINGLE TRIG EDGE TB VDIV MEAS? WAVE?, invalid/300-byte/partial input, normal hold and captured VREF")


def check_transport(board, seconds, channels):
    """Keep drawing and send commands during one uninterrupted sample-rate run."""
    before = fields(board.text_command("STAT?"))
    board.next_point = None
    board.recovery = PacketRecovery()
    board.points = board.gaps = 0
    crc_before = board.decoder.bad_crc
    recovered_before = board.recovery.recovered
    board.text_command(f"STREAM {channels}")
    start = time.monotonic()
    next_query = start + 2
    fft_page = False
    while time.monotonic() - start < seconds:
        if not fft_page and time.monotonic() - start >= seconds / 2:
            board.text_command("PAGE FFT")
            fft_page = True
        if time.monotonic() >= next_query:
            board.command_batch(b"MEAS?\nTRIG -1\n" * 4, ("OK", "ERR") * 4)
            next_query += 2
        board.read()
    board.text_command("STREAM OFF")
    after = fields(board.text_command("STAT?"))
    board.text_command("PAGE SCOPE")
    elapsed = time.monotonic() - start
    sample_hz = int(after["FS"])
    assert before["FS"] == after["FS"]
    target = sample_hz / (2 if channels == 1 else 4)
    adc_count = (int(after["ADC"]) - int(before["ADC"])) & 0xFFFFFFFF
    adc_elapsed = (int(after["MS"]) - int(before["MS"])) & 0xFFFFFFFF
    assert adc_elapsed > 0 and abs(1000 * adc_count / adc_elapsed / sample_hz - 1) < 0.015
    rate = board.points / elapsed
    assert abs(rate - target) < target * 0.015, (rate, board.points, elapsed)
    assert board.gaps == 0, (board.gaps, board.decoder.bad_crc - crc_before)
    for name in ("ADCERR", "DMADROP", "TXDROP", "RXERR"):
        assert before[name] == after[name], (name, before[name], after[name])
    assert int(after["FPS"]) >= 10, after
    assert after["LCD"] == "9341" and after["TOUCH"] == "1", after
    assert before["LCDSYNC"] == after["LCDSYNC"] == "1", (before, after)
    assert before["LCDMISS"] == after["LCDMISS"], (before, after)
    print(f"BOARD PASS: ADC {sample_hz} Hz, CH={channels} {elapsed:.1f}s {board.points} points/channel ({rate:.0f} points/s), zero delivered gaps/DMA/TX drops, LCD {after['FPS']} FPS, scope/FFT scan sync and zero new LCD misses, CRC errors {board.decoder.bad_crc - crc_before}, packets recovered {board.recovery.recovered - recovered_before}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port")
    parser.add_argument("--seconds", type=int, default=30)
    args = parser.parse_args()
    with open_serial(select_port(args.port), SERIAL_BAUD) as port:
        board = Board(port)
        board.pump(1.25)
        board.text_command("STREAM OFF")
        check_commands(board)
        check_transport(board, args.seconds, 1)
        board.text_command("CHAN 2")
        board.text_command("TB 5000us")
        board.pump(0.2)
        board.last_wave = None
        board.text_command("WAVE?")
        assert board.last_wave and board.last_wave.channels == 2
        span, points, _, _ = WAVE_HEADER.unpack_from(board.last_wave.data)
        assert (span, points) == (50000, 240), (span, points)
        check_transport(board, max(6, args.seconds // 3), 2)
        board.text_command("TB 200us")
        board.text_command("CHAN 1")
        print("BOARD PASS: longest 50 ms dual capture and 240 simultaneous snapshot points")


if __name__ == "__main__":
    main()

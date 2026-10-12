"""Frame-decoder tests against synthetic stream_audio byte streams."""
import numpy as np

from hardware.pinger.daisy_stream import (
    MAGIC, _decode, _fmt_info, parse_buffer,
)


def make_frame(seq, fmt, interleaved_i16):
    """One stream_audio frame: [DA 7A][seq][fmt][payload]. 16-bit only."""
    payload = np.asarray(interleaved_i16, "<i2").tobytes()
    return MAGIC + bytes([seq & 0xFF, fmt]) + payload


FMT16_64 = 0x41  # 16-bit, 64 samples/frame, decimation 1 (96 kHz)


def frame16(seq, value=1000):
    return make_frame(seq, FMT16_64, [value] * 128)  # 64 samples x 2 ch


def test_fmt_info():
    assert _fmt_info(0x00) == (16000, 16, 32, 4 + 32 * 2 * 2)   # legacy
    assert _fmt_info(0x41) == (96000, 16, 64, 4 + 64 * 2 * 2)
    assert _fmt_info(0xC1) == (96000, 24, 64, 4 + 64 * 2 * 3)
    assert _fmt_info(0x42) == (48000, 16, 64, 4 + 64 * 2 * 2)
    assert _fmt_info(0x45) is None                              # dec 5 invalid


def test_decode_16bit_roundtrip():
    vals = np.array([0, 1, -1, 32767, -32768, 123, -456, 7] * 8, "<i2")
    pcm = _decode(vals.tobytes(), 16, len(vals) // 2)
    assert pcm.shape == (len(vals) // 2, 2)
    assert pcm.dtype == np.float32
    np.testing.assert_array_equal(
        pcm.reshape(-1), vals.astype(np.float32) / 32768.0)


def test_decode_24bit_sign_extension():
    vals = np.array([0, 1, -1, 8388607, -8388608, -70000], np.int32)
    raw = bytearray()
    for v in vals:
        raw += int(v & 0xFFFFFF).to_bytes(3, "little")
    pcm = _decode(bytes(raw), 24, len(vals) // 2)
    np.testing.assert_array_equal(
        pcm.reshape(-1), vals.astype(np.float32) / 8388608.0)


def test_parse_buffer_needs_lookahead():
    # 3 complete frames: only the first two parse (the third has no
    # trailing magic to confirm its boundary yet).
    buf = frame16(1) + frame16(2) + frame16(3)
    batches, consumed, last_seq = parse_buffer(buf, None, None)
    assert len(batches) == 1                     # one fmt group
    fmt, payloads = batches[0]
    assert fmt == FMT16_64 and len(payloads) == 2
    assert consumed == 2 * len(frame16(0))
    assert last_seq == 2


def test_parse_buffer_magic_inside_payload_ok_when_synced():
    # A payload that CONTAINS the magic bytes parses fine from a synced
    # position (the lookahead confirms the true frame boundary).
    evil = [0x7ADA] * 128       # int16 whose little-endian bytes are DA 7A
    buf = make_frame(1, FMT16_64, evil) + frame16(2) + frame16(3)
    batches, consumed, _ = parse_buffer(buf, None, None)
    payloads = batches[0][1]
    assert len(payloads) == 2
    got = _decode(payloads[0], 16, 64)
    np.testing.assert_array_equal(
        got.reshape(-1),
        np.array(evil, "<i2").astype(np.float32) / 32768.0)


def test_parse_buffer_resyncs_after_byte_loss():
    # Desync: a truncated frame whose remaining payload is full of DA 7A
    # patterns. The parser must walk past every false magic (fmt byte 0x7A
    # has decimation 10 -> invalid) and lock onto the next real frame.
    evil = make_frame(1, FMT16_64, [0x7ADA] * 128)
    buf = evil[50:] + frame16(2) + frame16(3)
    batches, consumed, last_seq = parse_buffer(buf, None, None)
    payloads = batches[0][1]
    assert len(payloads) == 1                    # only frame 2 (3 unconfirmed)
    assert last_seq == 2
    got = _decode(payloads[0], 16, 64)
    assert np.all(got == 1000 / 32768.0)


def test_parse_buffer_counts_seq_gaps():
    stats = {}
    buf = frame16(1) + frame16(2) + frame16(5) + frame16(6)
    parse_buffer(buf, None, stats)
    assert stats["dropped"] == 1
    assert stats["rate"] == 96000 and stats["bits"] == 16


def test_parse_buffer_split_across_calls():
    # Partial tail: f1 is confirmed (f2's magic is visible in the first 10
    # extra bytes); f2 itself is incomplete and parses on the next call.
    f1, f2, f3 = frame16(1), frame16(2), frame16(3)
    whole = f1 + f2 + f3
    cut = len(f1) + 10
    batches, consumed, last_seq = parse_buffer(whole[:cut], None, None)
    assert [len(p) for _, p in batches] == [1]    # f1 accepted
    assert consumed == len(f1)
    assert last_seq == 1
    batches, consumed, last_seq = parse_buffer(whole[consumed:], last_seq,
                                               None)
    assert [len(p) for _, p in batches] == [1]    # f2 accepted (f3 pending)
    assert last_seq == 2

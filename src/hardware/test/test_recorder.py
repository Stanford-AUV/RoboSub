"""Crash-safe per-channel WAV recording."""
import wave

import numpy as np

from hardware.pinger.recorder import SessionRecorder, resolve_root


def read_wav(path):
    with wave.open(path, "rb") as w:
        assert w.getnchannels() == 1 and w.getsampwidth() == 2
        return (np.frombuffer(w.readframes(w.getnframes()), "<i2"),
                w.getframerate())


def stereo(i16_ch_a, i16_ch_b):
    pcm = np.stack([np.asarray(i16_ch_a, np.float32) / 32768.0,
                    np.asarray(i16_ch_b, np.float32) / 32768.0], axis=1)
    return pcm.astype(np.float32)


def test_resolve_root_env_override(monkeypatch, tmp_path):
    monkeypatch.setenv("ROBOSUB_DIR", str(tmp_path))
    assert resolve_root() == str(tmp_path)


def test_roundtrip_bit_exact(tmp_path):
    rec = SessionRecorder(str(tmp_path), 96000)
    a = np.array([0, 1, -1, 32767, -32768, 1234], "<i2")
    b = np.array([5, -5, 100, -100, 0, 42], "<i2")
    rec.write(0, stereo(a, b))
    rec.close()
    got_a, rate = read_wav(str(tmp_path / "raw" / "ch0" / "0.wav"))
    got_b, _ = read_wav(str(tmp_path / "raw" / "ch1" / "0.wav"))
    assert rate == 96000
    np.testing.assert_array_equal(got_a, a)
    np.testing.assert_array_equal(got_b, b)


def test_readable_mid_stream_without_close(tmp_path):
    # Header is patched on flush: a crash loses at most the tail, never
    # the file. Simulate by reading while the writer is still open.
    rec = SessionRecorder(str(tmp_path), 96000)
    a = np.arange(1000, dtype="<i2")
    rec.write(2, stereo(a, a))
    rec._flush()                      # what the periodic patcher calls
    got, _ = read_wav(str(tmp_path / "raw" / "ch2" / "0.wav"))
    np.testing.assert_array_equal(got, a)
    rec.close()


def test_fragments_on_reconnect(tmp_path):
    rec = SessionRecorder(str(tmp_path), 96000)
    a = np.array([1, 2, 3], "<i2")
    rec.write(0, stereo(a, a))
    rec.next_fragment(0)
    rec.write(0, stereo(a * 10, a * 10))
    rec.close()
    np.testing.assert_array_equal(
        read_wav(str(tmp_path / "raw" / "ch0" / "0.wav"))[0], a)
    np.testing.assert_array_equal(
        read_wav(str(tmp_path / "raw" / "ch0" / "1.wav"))[0], a * 10)


def test_write_failure_disables_without_raising(tmp_path):
    logs = []
    rec = SessionRecorder(str(tmp_path), 96000, log=logs.append)
    a = np.array([1, 2], "<i2")
    rec.write(0, stereo(a, a))
    for w in rec._writers.values():   # simulate disk death
        w._f.close()
    rec.write(0, stereo(a, a))        # must not raise
    assert rec.enabled is False
    assert any("recording disabled" in s for s in logs)
    rec.write(0, stereo(a, a))        # still fine, still silent
    rec.close()                       # close after failure must not raise


def test_float_conversion_saturates(tmp_path):
    rec = SessionRecorder(str(tmp_path), 96000)
    pcm = np.array([[1.5, -1.5], [1.0, -1.0]], np.float32)  # out of range
    rec.write(0, pcm)
    rec.close()
    got, _ = read_wav(str(tmp_path / "raw" / "ch0" / "0.wav"))
    np.testing.assert_array_equal(got, [32767, 32767])
    got, _ = read_wav(str(tmp_path / "raw" / "ch1" / "0.wav"))
    np.testing.assert_array_equal(got, [-32768, -32768])

import numpy as np
import pytest

from main import segment_into_frames


def _points(n: int) -> np.ndarray:
    return np.arange(n).reshape(-1, 1).astype(np.float64)


def test_exact_multiple_of_frame_size_has_no_remainder():
    frames = segment_into_frames(_points(300), frame_size=100)

    assert len(frames) == 3
    assert all(len(frame) == 100 for frame in frames)


def test_remainder_is_appended_as_last_frame():
    frames = segment_into_frames(_points(250), frame_size=100)

    assert len(frames) == 3
    assert len(frames[0]) == 100
    assert len(frames[1]) == 100
    assert len(frames[2]) == 50


def test_fewer_points_than_frame_size_yields_single_remainder_frame():
    frames = segment_into_frames(_points(42), frame_size=100)

    assert len(frames) == 1
    assert len(frames[0]) == 42


def test_empty_input_yields_no_frames():
    frames = segment_into_frames(_points(0), frame_size=100)

    assert frames == []


def test_frames_preserve_point_order():
    points = _points(250)
    frames = segment_into_frames(points, frame_size=100)

    reconstructed = np.concatenate(frames, axis=0)
    np.testing.assert_array_equal(reconstructed, points)

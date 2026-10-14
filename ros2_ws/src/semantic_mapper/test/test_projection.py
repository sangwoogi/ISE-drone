import numpy as np
import pytest

from semantic_mapper.association import ObjectAssociator
from semantic_mapper.projection import bbox_to_camera_point, pixel_to_camera, robust_depth


def test_pixel_to_camera_center():
    assert pixel_to_camera(320, 240, 2.0, 500, 500, 320, 240) == pytest.approx((0.0, 0.0, 2.0))


def test_pixel_to_camera_offset():
    x, y, z = pixel_to_camera(420, 240, 2.0, 500, 500, 320, 240)
    assert (x, y, z) == pytest.approx((0.4, 0.0, 2.0))


def test_robust_depth_median_ignores_invalid():
    d = np.full((100, 100), 2.0, dtype=np.float32)
    d[40:60, 40:60] = np.nan
    d[45, 45] = 0.0
    d[50, 50] = 2.2
    assert robust_depth(d, 20, 20, 80, 80) == pytest.approx(2.0, abs=0.05)


def test_robust_depth_uint16_mm():
    d = np.full((50, 50), 1500, dtype=np.uint16)
    assert robust_depth(d, 0, 0, 50, 50) == pytest.approx(1.5)


def test_bbox_to_camera_point_fixture():
    # plan §47: chair bbox [320,210,500,620] at 2.0 m, K = fx=fy=600, cx=320, cy=240 (640x480-ish)
    d = np.full((700, 700), 2.0, dtype=np.float32)
    K = [600, 0, 320, 0, 600, 240, 0, 0, 1]
    x, y, z = bbox_to_camera_point(d, (320, 210, 500, 620), K)
    assert z == pytest.approx(2.0)
    assert x == pytest.approx((410 - 320) * 2.0 / 600)
    assert y == pytest.approx((415 - 240) * 2.0 / 600)


def test_association_merges_same_object():
    a = ObjectAssociator(distance_threshold=0.5, alpha=0.5, min_observations=2)
    o1 = a.update('chair', (2.0, 1.0, 0.5), 0.9, 0.0)
    o2 = a.update('chair', (2.2, 1.1, 0.5), 0.8, 1.0)
    o3 = a.update('chair', (4.0, 1.0, 0.5), 0.8, 2.0)
    assert o1.id == o2.id == 'chair_01'
    assert o3.id == 'chair_02'
    assert o2.observation_count == 2
    assert o2.position == pytest.approx((2.1, 1.05, 0.5))
    assert [o.id for o in a.confirmed()] == ['chair_01']

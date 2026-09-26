"""UTM projection and grid-convergence tests. Cross-checked against pyproj when available."""
import math

import pytest

from hardware_bringup.utm import grid_convergence, latlon_to_utm, utm_zone_from_lon

# WashU east end; the planner's bounding box is roughly lat 38.6473-38.6490, lon -90.3041 - -90.3013
CAMPUS_POINTS = [
    (38.648000, -90.302500),
    (38.64817527, -90.30128152),  # Gazebo world datum
    (38.647310, -90.304114),
    (38.649011, -90.301293),
    (38.600000, -90.200000),
]

# global_planner.py hardcodes this as the UTM coordinates of the map origin
PLANNER_ORIGIN = (734765 + 91.46309417473815, 4281212 - 24.993696585102057)


def test_zone_for_st_louis_is_15():
    assert utm_zone_from_lon(-90.3025) == 15
    assert utm_zone_from_lon(-93.0) == 15  # the central meridian of zone 15
    assert utm_zone_from_lon(-90.0) == 16  # exactly on the 15/16 boundary rounds up
    assert utm_zone_from_lon(-95.9) == 15


def test_projection_lands_near_planner_origin():
    e, n = latlon_to_utm(38.64817527, -90.30128152, zone=15)
    # AUTONOMY_SYSTEM_OUTLINE notes the hand-measured origin differs from the analytic
    # datum by a few metres, so only a coarse check here; the exact check is vs pyproj.
    assert abs(e - PLANNER_ORIGIN[0]) < 10.0
    assert abs(n - PLANNER_ORIGIN[1]) < 10.0


def test_grid_convergence_magnitude_and_sign():
    gamma = grid_convergence(38.648, -90.3025, zone=15)
    # first-order: (lon - lon0) * sin(lat) = 2.6975 deg * sin(38.648 deg) = 1.6846 deg
    assert math.degrees(gamma) == pytest.approx(1.685, abs=0.01)
    # east of the central meridian in the northern hemisphere: grid north lies east of true north
    assert gamma > 0
    # west of the central meridian the sign flips
    assert grid_convergence(38.648, -95.0, zone=15) < 0


pyproj = pytest.importorskip('pyproj')


@pytest.mark.parametrize('lat,lon', CAMPUS_POINTS)
def test_matches_pyproj_epsg32615_to_millimetres(lat, lon):
    tf = pyproj.Transformer.from_crs('EPSG:4326', 'EPSG:32615', always_xy=True)
    e_ref, n_ref = tf.transform(lon, lat)
    e, n = latlon_to_utm(lat, lon, zone=15)
    assert e == pytest.approx(e_ref, abs=2e-3)
    assert n == pytest.approx(n_ref, abs=2e-3)


def test_convergence_matches_pyproj_two_point_method():
    tf = pyproj.Transformer.from_crs('EPSG:4326', 'EPSG:32615', always_xy=True)
    lat, lon = 38.648, -90.3025
    e0, n0 = tf.transform(lon, lat)
    e1, n1 = tf.transform(lon, lat + 1e-5)  # a point due true-north
    dx, dy = e1 - e0, n1 - n0
    assert dx < 0  # true north points slightly west of grid north here
    gamma_ref = math.atan2(-dx, dy)
    assert grid_convergence(lat, lon, zone=15) == pytest.approx(gamma_ref, abs=1e-6)

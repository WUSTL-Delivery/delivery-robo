"""gps_datum_tf's datum maths: UTM on the robot, local ENU about the Gazebo world origin in the sim."""
import math

import pytest

pytest.importorskip('rclpy')  # gps_datum_tf is a node module

from hardware_bringup.gps_datum_tf import (  # noqa: E402
    PLANNER_UTM_ORIGIN_X, PLANNER_UTM_ORIGIN_Y, datum_transform_local_enu, datum_transform_utm,
    geodetic_to_enu)
from hardware_bringup.utm import WGS84_A, WGS84_F, grid_convergence, latlon_to_utm  # noqa: E402

# delivery-autonomy washu.world <spherical_coordinates>: lat, lon, elevation
WORLD_ORIGIN = (38.64817527, -90.30128152, 0.0)

E2 = WGS84_F * (2.0 - WGS84_F)
PHI0 = math.radians(WORLD_ORIGIN[0])
# meridional and prime-vertical radii of curvature at the origin
M0 = WGS84_A * (1.0 - E2) / (1.0 - E2 * math.sin(PHI0) ** 2) ** 1.5
N0 = WGS84_A / math.sqrt(1.0 - E2 * math.sin(PHI0) ** 2)


def test_origin_is_zero():
    assert geodetic_to_enu(*WORLD_ORIGIN, *WORLD_ORIGIN) == pytest.approx((0.0, 0.0, 0.0), abs=1e-6)


def test_straight_up_is_up():
    lat, lon, alt = WORLD_ORIGIN
    assert geodetic_to_enu(lat, lon, alt + 10.0, *WORLD_ORIGIN) == pytest.approx((0.0, 0.0, 10.0), abs=1e-6)


@pytest.mark.parametrize('de,dn', [(-95.0, 25.5), (100.0, 0.0), (0.0, -100.0), (-250.0, -120.0)])
def test_small_offsets_match_radii_of_curvature(de, dn):
    # first order: north = M dphi, east = N cos(phi) dlambda. Over a few hundred metres the
    # tangent plane departs from that by d^2 / 2R, a few millimetres.
    lat = WORLD_ORIGIN[0] + math.degrees(dn / M0)
    lon = WORLD_ORIGIN[1] + math.degrees(de / (N0 * math.cos(PHI0)))
    e, n, u = geodetic_to_enu(lat, lon, 0.0, *WORLD_ORIGIN)
    assert e == pytest.approx(de, abs=0.01)
    assert n == pytest.approx(dn, abs=0.01)
    assert u == pytest.approx(-(de * de + dn * dn) / (2.0 * WGS84_A), abs=0.005)


def test_gazebo_fix_lands_on_ground_truth():
    # Gazebo (gz-sim 10, washu.world) published this /gps/fix with the robot settled after the
    # spawn drop; /world/washu_campus/dynamic_pose/info had the model (base_footprint) at
    # (-94.6296, 25.5040, 2.6706). The sensor sits 0.15 m above base_footprint, at base_link.
    x, y, z, yaw = datum_transform_local_enu(38.6484050126, -90.3023684719, 2.8212, WORLD_ORIGIN)
    assert (x, y) == pytest.approx((-94.6296, 25.5040), abs=0.01)
    assert z == pytest.approx(2.6706 + 0.15, abs=0.01)
    assert abs(yaw) < 1e-4


def test_local_enu_yaw_is_meridian_convergence():
    # True north at a point west of the origin leans east in the origin's ENU frame (meridians
    # meet at the pole): negative yaw, first order dlambda * sin(phi).
    lat = WORLD_ORIGIN[0]
    for de in (-250.0, 250.0):
        dlam = de / (N0 * math.cos(PHI0))
        lon = WORLD_ORIGIN[1] + math.degrees(dlam)
        yaw = datum_transform_local_enu(lat, lon, 0.0, WORLD_ORIGIN)[3]
        assert yaw == pytest.approx(dlam * math.sin(PHI0), rel=0.02)
        assert math.copysign(1.0, yaw) == math.copysign(1.0, de)


def test_utm_transform_is_unchanged():
    lat, lon = 38.648, -90.3025
    origin = (PLANNER_UTM_ORIGIN_X, PLANNER_UTM_ORIGIN_Y)
    e, n = latlon_to_utm(lat, lon, 15, True)
    x, y, z, yaw = datum_transform_utm(lat, lon, 15, True, origin)
    assert (x, y, z) == pytest.approx((e - origin[0], n - origin[1], 0.0))
    assert yaw == pytest.approx(grid_convergence(lat, lon, 15, True))


@pytest.mark.parametrize('lat,lon,alt', [
    (38.6484050126, -90.3023684719, 2.8212),
    (38.647310, -90.304114, 5.0),
    (38.649011, -90.301293, -3.0),
])
def test_matches_proj_topocentric(lat, lon, alt):
    pyproj = pytest.importorskip('pyproj')
    try:
        tf = pyproj.Transformer.from_pipeline(
            '+proj=pipeline +step +proj=cart +ellps=WGS84 +step +proj=topocentric +ellps=WGS84 '
            f'+lat_0={WORLD_ORIGIN[0]} +lon_0={WORLD_ORIGIN[1]} +h_0={WORLD_ORIGIN[2]}')
        ref = tf.transform(lon, lat, alt)
    except pyproj.exceptions.ProjError as ex:  # PROJ < 8 has no topocentric
        pytest.skip(f'no PROJ topocentric: {ex}')
    assert geodetic_to_enu(lat, lon, alt, *WORLD_ORIGIN) == pytest.approx(ref, abs=1e-4)

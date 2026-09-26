"""WGS84 -> UTM (Transverse Mercator) and grid convergence, in pure Python.

Used by ``gps_datum_tf`` to place the ``odom`` frame (true east/north, origin at the first GPS
fix) inside the planner's ``map`` frame (UTM zone 15N grid east/north, origin at
``global_planner.utm_origin``). Formulas are Snyder, *Map Projections: A Working Manual*
(USGS PP 1395), eqs. 3-21 and 8-9..8-13; they agree with PROJ's exact ``utm`` to well under
a millimetre within a few degrees of the central meridian (see ``test_utm.py``, which
cross-checks against pyproj EPSG:32615 when it is installed).

No dependency on pyproj so Stage 2 of the integration plan (frames) can be verified on the
robot before the autonomy Python dependencies are installed.
"""
import math

WGS84_A = 6378137.0
WGS84_F = 1.0 / 298.257223563
_E2 = WGS84_F * (2.0 - WGS84_F)         # first eccentricity squared
_EP2 = _E2 / (1.0 - _E2)                # second eccentricity squared
_K0 = 0.9996
_FALSE_EASTING = 500000.0
_FALSE_NORTHING_SOUTH = 10000000.0


def utm_zone_from_lon(lon_deg):
    return int((lon_deg + 180.0) // 6.0) + 1


def central_meridian_deg(zone):
    return (zone - 1) * 6 - 180 + 3


def latlon_to_utm(lat_deg, lon_deg, zone, northern=True):
    """Return ``(easting, northing)`` metres in UTM ``zone`` (WGS84)."""
    phi = math.radians(lat_deg)
    dlam = math.radians(lon_deg - central_meridian_deg(zone))
    sin_p, cos_p, tan_p = math.sin(phi), math.cos(phi), math.tan(phi)

    n = WGS84_A / math.sqrt(1.0 - _E2 * sin_p * sin_p)
    t = tan_p * tan_p
    c = _EP2 * cos_p * cos_p
    a = dlam * cos_p

    e2, e4, e6 = _E2, _E2 ** 2, _E2 ** 3
    m = WGS84_A * (
        (1.0 - e2 / 4.0 - 3.0 * e4 / 64.0 - 5.0 * e6 / 256.0) * phi
        - (3.0 * e2 / 8.0 + 3.0 * e4 / 32.0 + 45.0 * e6 / 1024.0) * math.sin(2.0 * phi)
        + (15.0 * e4 / 256.0 + 45.0 * e6 / 1024.0) * math.sin(4.0 * phi)
        - (35.0 * e6 / 3072.0) * math.sin(6.0 * phi)
    )

    easting = _K0 * n * (
        a
        + (1.0 - t + c) * a ** 3 / 6.0
        + (5.0 - 18.0 * t + t * t + 72.0 * c - 58.0 * _EP2) * a ** 5 / 120.0
    ) + _FALSE_EASTING
    northing = _K0 * (
        m + n * tan_p * (
            a * a / 2.0
            + (5.0 - t + 9.0 * c + 4.0 * c * c) * a ** 4 / 24.0
            + (61.0 - 58.0 * t + t * t + 600.0 * c - 330.0 * _EP2) * a ** 6 / 720.0
        )
    )
    if not northern:
        northing += _FALSE_NORTHING_SOUTH
    return easting, northing


def grid_convergence(lat_deg, lon_deg, zone, northern=True, step_deg=1e-5):
    """Angle from true north to grid north, radians, positive when grid north lies EAST of true north.

    Derived numerically from the projection itself (project the point and a point slightly
    due-north of it) so no sign convention is hardcoded.

    A frame aligned to true east/north (``odom``) expressed in a frame aligned to grid
    east/north (``map``) is rotated by exactly this angle: ``map->odom`` yaw = +gamma.
    Proof: the odom +y axis (true north) is ``R(yaw) @ (0, 1) = (-sin yaw, cos yaw)``, and the
    true-north direction in grid coordinates is ``(dx, dy)/|d|`` for a small step north, so
    ``yaw = atan2(-dx, dy)``, which is what this function returns.
    """
    e0, n0 = latlon_to_utm(lat_deg, lon_deg, zone, northern)
    e1, n1 = latlon_to_utm(lat_deg + step_deg, lon_deg, zone, northern)
    return math.atan2(-(e1 - e0), n1 - n0)

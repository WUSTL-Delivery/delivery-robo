"""GPS datum: gates the raw fix stream, forwards it to the EKF, and publishes ``map -> odom``.

``state_estimation`` takes the FIRST ``NavSatFix`` it receives as the permanent origin of its
``odom`` frame, checks neither ``status`` nor NaN, and never re-derives it. This node sits
between the GPS driver and the EKF so that:

1. only fixes with ``status.status >= min_status`` and finite coordinates get through (a
   ``STATUS_NO_FIX`` first message would otherwise poison the datum for the whole run).
   ``ublox_dgnss`` reports -2 before its first NAV-STATUS, -1 without a fix, 0 for a plain
   fix and 1 once differential/RTK corrections are applied; it never reports 2;
2. the driver's QoS no longer matters: the input subscription is best-effort (compatible
   with any publisher), the output is reliable, which is what the autonomy nodes expect;
3. the exact fix the EKF receives first is the datum used for ``map -> odom``: forwarding
   starts only once ``gps_out_topic`` has a subscriber (``wait_for_subscriber``), and the
   first forwarded fix is the datum. If every subscriber disappears (the EKF restarted) the
   datum is re-derived for the next one, so the two stay consistent as long as the robot is
   stationary while the stack (re)starts.

``map`` is the planner's frame: UTM zone 15N grid east/north, origin at the planner's
hardcoded ``utm_origin``. ``odom`` is the EKF's frame: true east/north in the WGS84 local
tangent plane at the datum (``latlon2meters.py`` since delivery-autonomy TDM-38), origin at the
datum. Hence

    translation = UTM(datum) - utm_origin
    yaw         = grid convergence at the datum  (about +1.685 deg here)

Leaving the yaw at zero would misplace planner paths by ~2.9 m per 100 m from the datum.
Not modelled (a rigid transform cannot): the UTM point scale (1.00028, i.e. ~2.8 cm per 100 m
from the datum) and the change of the convergence across campus (0.002 deg, ~1 cm over 300 m).
Before TDM-38 the EKF used a spherical (equirectangular) radius, which added 0.1-0.3 % (the
steady ~0.25 m EKF bias at the end of the 190 m sim route; 0.002 m after).

``projection: local_enu`` is for the Gazebo sim (``autonomy_real.launch.py sim:=true``). There
``map`` is the Gazebo world frame, and the NavSat sensor converts world positions to lat/lon as
local east/north/up about the world's ``<spherical_coordinates>`` origin (heading 0). The
datum's position in ``map`` is therefore its exact ENU position about that origin
(``enu_origin_lat/lon/alt``), z included. The sim's GPS altitude is base_link's height, which
puts ``odom`` (EKF z = 0) at base_link. The yaw is the meridian convergence between the origin
and the datum: ~1e-5 rad on campus. This replaces a fixed ``map -> odom`` at the spawn pose.
That constant was off by however far the robot had moved before the EKF's first fix (it settles
0.37 m from the spawn request after the drop). Taking both from the same fix removes that race
for x/y. z is the datum's own: if the EKF starts while the robot is still falling, z stays that
high. Nothing in the stack reads it, but RViz shows the robot floating.
"""
import math

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import NavSatFix
from tf2_msgs.msg import TFMessage

from hardware_bringup.utm import WGS84_A, WGS84_F, grid_convergence, latlon_to_utm

# global_planner.py: true_origin_x = 734765 + 91.46309417473815, true_origin_y = 4281212 - 24.993696585102057
PLANNER_UTM_ORIGIN_X = 734856.4630941747
PLANNER_UTM_ORIGIN_Y = 4281187.006303415

PROJECTIONS = ('utm', 'local_enu')

_E2 = WGS84_F * (2.0 - WGS84_F)


def geodetic_to_ecef(lat_deg, lon_deg, alt):
    """WGS84 latitude/longitude (deg) and ellipsoidal height (m) -> ECEF ``(x, y, z)`` metres."""
    phi, lam = math.radians(lat_deg), math.radians(lon_deg)
    sin_p = math.sin(phi)
    n = WGS84_A / math.sqrt(1.0 - _E2 * sin_p * sin_p)
    return ((n + alt) * math.cos(phi) * math.cos(lam),
            (n + alt) * math.cos(phi) * math.sin(lam),
            (n * (1.0 - _E2) + alt) * sin_p)


def geodetic_to_enu(lat_deg, lon_deg, alt, lat0_deg, lon0_deg, alt0):
    """Local east/north/up metres of a WGS84 point about the origin ``(lat0, lon0, alt0)``.

    The exact tangent-plane frame, through ECEF. Gazebo's spherical coordinates relate world
    positions to the NavSat sensor's lat/lon/altitude the same way.
    """
    x, y, z = geodetic_to_ecef(lat_deg, lon_deg, alt)
    x0, y0, z0 = geodetic_to_ecef(lat0_deg, lon0_deg, alt0)
    dx, dy, dz = x - x0, y - y0, z - z0
    phi, lam = math.radians(lat0_deg), math.radians(lon0_deg)
    sp, cp, sl, cl = math.sin(phi), math.cos(phi), math.sin(lam), math.cos(lam)
    return (-sl * dx + cl * dy,
            -sp * cl * dx - sp * sl * dy + cp * dz,
            cp * cl * dx + cp * sl * dy + sp * dz)


def datum_transform_utm(lat_deg, lon_deg, zone, northern, origin_xy):
    """``map -> odom`` as ``(x, y, z, yaw)`` on the robot: UTM grid minus the planner origin."""
    easting, northing = latlon_to_utm(lat_deg, lon_deg, zone, northern)
    gamma = grid_convergence(lat_deg, lon_deg, zone, northern)
    return easting - origin_xy[0], northing - origin_xy[1], 0.0, gamma


def datum_transform_local_enu(lat_deg, lon_deg, alt, origin_lla, step_deg=1e-5):
    """``map -> odom`` as ``(x, y, z, yaw)`` in the sim: ENU about the Gazebo world origin.

    yaw = direction of true north at the datum in the origin's ENU frame, measured as in
    ``utm.grid_convergence``: ``atan2(-dx, dy)`` of a short step due north.
    """
    e, n, u = geodetic_to_enu(lat_deg, lon_deg, alt, *origin_lla)
    e1, n1, _ = geodetic_to_enu(lat_deg + step_deg, lon_deg, alt, *origin_lla)
    return e, n, u, math.atan2(-(e1 - e), n1 - n)


class GpsDatumTf(Node):
    def __init__(self):
        super().__init__('gps_datum_tf')
        p = self._declare
        self._in_topic = p('gps_in_topic', '/fix')
        self._out_topic = p('gps_out_topic', '/gps/fix')
        datum_topic = p('datum_topic', '/gps/datum')
        self._min_status = int(p('min_status', 0))
        self._zone = int(p('utm_zone', 15))
        self._northern = bool(p('northern_hemisphere', True))
        self._origin = (float(p('planner_origin_x', PLANNER_UTM_ORIGIN_X)),
                        float(p('planner_origin_y', PLANNER_UTM_ORIGIN_Y)))
        # 'utm' on the robot; 'local_enu' in the Gazebo sim, about the world's spherical origin
        self._projection = p('projection', 'utm')
        self._enu_origin = (float(p('enu_origin_lat', float('nan'))),
                            float(p('enu_origin_lon', float('nan'))),
                            float(p('enu_origin_alt', 0.0)))
        self._map_frame = p('map_frame', 'map')
        self._odom_frame = p('odom_frame', 'odom')
        self._wait_for_subscriber = bool(p('wait_for_subscriber', True))
        self._restamp = bool(p('restamp', False))
        check_hz = float(p('subscriber_check_hz', 1.0))

        if self._in_topic == self._out_topic:
            raise ValueError('gps_in_topic and gps_out_topic must differ (the node would relay to itself)')
        if self._projection not in PROJECTIONS:
            raise ValueError(f'projection must be one of {PROJECTIONS}, got {self._projection!r}')
        if self._projection == 'local_enu' and not all(math.isfinite(v) for v in self._enu_origin):
            raise ValueError('projection local_enu needs enu_origin_lat and enu_origin_lon (the Gazebo '
                             "world's <spherical_coordinates>)")

        self._datum = None
        self._armed = True
        self._had_subscriber = False
        self._checked_preexisting = False
        self._received = 0
        self._dropped = 0
        self._forwarded = 0

        # Best-effort subscription matches both reliable and best-effort publishers.
        in_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10,
                            reliability=ReliabilityPolicy.BEST_EFFORT)
        latched = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                             reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._fix_pub = self.create_publisher(NavSatFix, self._out_topic, 10)
        self._datum_pub = self.create_publisher(NavSatFix, datum_topic, latched)
        # Not tf2_ros.StaticTransformBroadcaster: its Jazzy Python version silently drops a
        # second transform for a child frame it has already sent, so the datum could never be
        # re-derived. Same latched QoS on /tf_static; listeners keep the latest per child frame.
        self._tf_static_pub = self.create_publisher(TFMessage, '/tf_static', latched)
        self.create_subscription(NavSatFix, self._in_topic, self._on_fix, in_qos)
        self.create_timer(1.0 / check_hz, self._check_tick)

        if self._projection == 'utm':
            frame = f'UTM zone {self._zone}{"N" if self._northern else "S"}, planner origin {self._origin}'
        else:
            frame = f'local ENU about lat/lon/alt {self._enu_origin}'
        self.get_logger().info(
            f'gps_datum_tf: {self._in_topic} -> {self._out_topic} (min_status {self._min_status}), '
            f'{self._map_frame} -> {self._odom_frame} from the first forwarded fix; {frame}'
        )

    def _declare(self, name, default):
        return self.declare_parameter(name, default).value

    # ------------------------------------------------------------------
    @staticmethod
    def _is_valid(msg, min_status):
        if msg.status.status < min_status:
            return False
        if not (math.isfinite(msg.latitude) and math.isfinite(msg.longitude)):
            return False
        if msg.latitude == 0.0 and msg.longitude == 0.0:
            return False
        return True

    def _check_tick(self):
        n = self.count_subscribers(self._out_topic)
        if not self._checked_preexisting:
            self._checked_preexisting = True
            if n > 0 and self._forwarded == 0:
                self.get_logger().warning(
                    f'{n} node(s) already subscribe to {self._out_topic} (an EKF that outlived a restart '
                    'of this node?). Their datum is unknown here; restart the whole stack with the '
                    'robot stationary if map->base_link looks offset.'
                )
        if n > 0:
            self._had_subscriber = True
        elif self._had_subscriber and not self._armed:
            self._had_subscriber = False
            self._armed = True
            self.get_logger().warning(
                f'all {self._out_topic} subscribers gone (EKF restarted?); the datum will be '
                're-derived from the next forwarded fix. Keep the robot stationary.'
            )
        if self._datum is None:
            self.get_logger().info(
                f'waiting for a fix with status >= {self._min_status} '
                f'(received {self._received}, dropped {self._dropped}, subscribers {n})',
                throttle_duration_sec=5.0,
            )

    def _on_fix(self, msg):
        self._received += 1
        if not self._is_valid(msg, self._min_status):
            self._dropped += 1
            self.get_logger().warning(
                f'dropping fix: status {msg.status.status} < {self._min_status} or non-finite '
                f'({self._dropped} dropped so far)', throttle_duration_sec=5.0,
            )
            return
        if self._wait_for_subscriber and self.count_subscribers(self._out_topic) == 0:
            self.get_logger().info(
                f'have a valid fix but nothing subscribes to {self._out_topic} yet; holding',
                throttle_duration_sec=5.0,
            )
            return
        if self._armed:
            self._set_datum(msg)
            self._armed = False
        self._forward(msg)

    def _forward(self, msg):
        stamp_is_zero = msg.header.stamp.sec == 0 and msg.header.stamp.nanosec == 0
        if self._restamp or stamp_is_zero:
            msg.header.stamp = self.get_clock().now().to_msg()
        self._fix_pub.publish(msg)
        self._forwarded += 1

    def _set_datum(self, msg):
        lat, lon = msg.latitude, msg.longitude
        if self._projection == 'utm':
            tx, ty, tz, gamma = datum_transform_utm(lat, lon, self._zone, self._northern, self._origin)
            how = 'grid convergence'
        else:
            alt = msg.altitude if math.isfinite(msg.altitude) else self._enu_origin[2]
            tx, ty, tz, gamma = datum_transform_local_enu(lat, lon, alt, self._enu_origin)
            how = 'meridian convergence'

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self._map_frame
        t.child_frame_id = self._odom_frame
        t.transform.translation.x = float(tx)
        t.transform.translation.y = float(ty)
        t.transform.translation.z = float(tz)
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = math.sin(gamma / 2.0)
        t.transform.rotation.w = math.cos(gamma / 2.0)
        self._tf_static_pub.publish(TFMessage(transforms=[t]))

        self._datum = msg
        self._datum_pub.publish(msg)
        self.get_logger().info(
            f'datum set: lat {lat:.8f} lon {lon:.8f} alt {msg.altitude:.3f} status {msg.status.status} '
            f'-> {self._map_frame}->{self._odom_frame} = ({tx:.3f}, {ty:.3f}, {tz:.3f}) m, '
            f'yaw {math.degrees(gamma):.4f} deg ({how}, {self._projection})'
        )
        if math.hypot(tx, ty) > 2000.0:
            self.get_logger().error(
                f'datum is {math.hypot(tx, ty):.0f} m from the {self._map_frame} origin: wrong UTM zone, '
                'wrong planner_origin / enu_origin, or the GPS is not on campus'
            )


def main(args=None):
    rclpy.init(args=args)
    node = GpsDatumTf()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

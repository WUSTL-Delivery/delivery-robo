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
hardcoded ``utm_origin``. ``odom`` is the EKF's frame: true east/north
(``latlon2meters.py``), origin at the datum. Hence

    translation = UTM(datum) - utm_origin
    yaw         = grid convergence at the datum  (about +1.685 deg here)

Leaving the yaw at zero would misplace planner paths by ~2.9 m per 100 m from the datum.
Not modelled (a rigid transform cannot): UTM point scale (1.00028) and the equirectangular
radius error in ``latlon2meters`` (0.1-0.3 %), together up to ~0.3 m per 100 m.
"""
import math

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import NavSatFix
from tf2_msgs.msg import TFMessage

from hardware_bringup.utm import grid_convergence, latlon_to_utm

# global_planner.py: true_origin_x = 734765 + 91.46309417473815, true_origin_y = 4281212 - 24.993696585102057
PLANNER_UTM_ORIGIN_X = 734856.4630941747
PLANNER_UTM_ORIGIN_Y = 4281187.006303415


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
        self._map_frame = p('map_frame', 'map')
        self._odom_frame = p('odom_frame', 'odom')
        self._wait_for_subscriber = bool(p('wait_for_subscriber', True))
        self._restamp = bool(p('restamp', False))
        check_hz = float(p('subscriber_check_hz', 1.0))

        if self._in_topic == self._out_topic:
            raise ValueError('gps_in_topic and gps_out_topic must differ (the node would relay to itself)')

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

        self.get_logger().info(
            f'gps_datum_tf: {self._in_topic} -> {self._out_topic} (min_status {self._min_status}), '
            f'{self._map_frame} -> {self._odom_frame} from the first forwarded fix; '
            f'UTM zone {self._zone}{"N" if self._northern else "S"}, planner origin {self._origin}'
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
        easting, northing = latlon_to_utm(lat, lon, self._zone, self._northern)
        gamma = grid_convergence(lat, lon, self._zone, self._northern)
        tx = easting - self._origin[0]
        ty = northing - self._origin[1]

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self._map_frame
        t.child_frame_id = self._odom_frame
        t.transform.translation.x = float(tx)
        t.transform.translation.y = float(ty)
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = math.sin(gamma / 2.0)
        t.transform.rotation.w = math.cos(gamma / 2.0)
        self._tf_static_pub.publish(TFMessage(transforms=[t]))

        self._datum = msg
        self._datum_pub.publish(msg)
        self.get_logger().info(
            f'datum set: lat {lat:.8f} lon {lon:.8f} status {msg.status.status} -> '
            f'UTM{self._zone} E {easting:.3f} N {northing:.3f}; '
            f'{self._map_frame}->{self._odom_frame} = ({tx:.3f}, {ty:.3f}) m, '
            f'yaw {math.degrees(gamma):.4f} deg (grid convergence)'
        )
        if math.hypot(tx, ty) > 2000.0:
            self.get_logger().error(
                f'datum is {math.hypot(tx, ty):.0f} m from the planner origin: wrong UTM zone, '
                'wrong planner_origin, or the GPS is not on campus'
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

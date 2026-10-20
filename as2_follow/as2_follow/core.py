"""ROS/SDK-independent geometry and conservative tracking controller."""
import math
from dataclasses import dataclass


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def finite(values):
    return all(math.isfinite(v) for v in values)


def select_team(poses, side):
    if len(poses) < 7 or not all(finite(p) for p in poses[:7]):
        raise ValueError('Need finite ball and RM1..6 poses')
    negative = tuple(i for i in range(1, 7) if poses[i][0] < -0.05)
    positive = tuple(i for i in range(1, 7) if poses[i][0] > 0.05)
    if len(negative) != 3 or len(positive) != 3:
        raise ValueError('Initialization needs exactly three RMs on each X half')
    return negative if side == 'lower' else positive


@dataclass
class Limits:
    xmin: float = -2.3
    xmax: float = 2.3
    speed: float = 0.12
    acceleration: float = 0.08
    yaw_speed: float = 0.25


class Tracker:
    def __init__(self, initial, limits):
        self.initial = initial
        self.limits = limits
        self.origin = None
        self.previous = None
        self.velocity = (0.0, 0.0, 0.0)

    def world_pose(self, odom):
        if not finite(odom):
            raise ValueError('Non-finite odometry')
        if self.origin is None:
            self.origin = odom
        angle = wrap(self.initial[2] - self.origin[2])
        dx, dy = odom[0] - self.origin[0], odom[1] - self.origin[1]
        p = (self.initial[0] + math.cos(angle)*dx - math.sin(angle)*dy,
             self.initial[1] + math.sin(angle)*dx + math.cos(angle)*dy,
             wrap(odom[2] + angle))
        if self.previous:
            if math.hypot(p[0]-self.previous[0], p[1]-self.previous[1]) > 0.25:
                raise ValueError('Odometry jumped; restart and recalibrate')
            if abs(wrap(p[2]-self.previous[2])) > 0.4:
                raise ValueError('Yaw jumped; restart and recalibrate')
        self.previous = p
        return p

    def stop(self):
        self.velocity = (0.0, 0.0, 0.0)
        return self.velocity

    def command(self, pose, target_x, ball, dt):
        lim = self.limits
        x, y, yaw = pose
        if not finite((*pose, target_x, *ball, dt)) or dt <= 0:
            raise ValueError('Invalid controller input')
        if not lim.xmin-0.15 <= x <= lim.xmax+0.15 or abs(y-self.initial[1]) > 0.3:
            raise ValueError('Outside sideline corridor; manual reposition required')
        ex = clamp(target_x, lim.xmin, lim.xmax) - x
        ey = self.initial[1] - y
        wx = 0.0 if abs(ex) < 0.04 else 0.45*ex
        wy = 0.0 if abs(ey) < 0.025 else 0.45*ey
        length = math.hypot(wx, wy)
        if length > lim.speed:
            wx, wy = wx*lim.speed/length, wy*lim.speed/length
        # Ball bearing is in field frame; commands are in current body frame.
        bearing = math.atan2(ball[1]-y, ball[0]-x)
        error = wrap(bearing-yaw)
        wz = 0.0 if abs(error) < 0.04 else clamp(0.7*error, -lim.yaw_speed, lim.yaw_speed)
        # Ramp in world coordinates so turning does not rotate previous velocity.
        oldx, oldy, oldz = self.velocity
        dx, dy = wx-oldx, wy-oldy
        delta = math.hypot(dx, dy)
        step = lim.acceleration*min(dt, 0.1)
        if delta > step:
            wx, wy = oldx+dx*step/delta, oldy+dy*step/delta
        wz = oldz + clamp(wz-oldz, -0.3*min(dt, 0.1), 0.3*min(dt, 0.1))
        self.velocity = wx, wy, wz
        return (math.cos(yaw)*wx + math.sin(yaw)*wy,
                -math.sin(yaw)*wx + math.cos(yaw)*wy, wz)

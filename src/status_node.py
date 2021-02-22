#!/usr/bin/env python3
from __future__ import absolute_import, division, print_function

import curses
import sys
from collections import deque
from datetime import datetime

import numpy as np
import rospy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import ExtendedState, PositionTarget, State  # StatusText
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import BatteryState, Image, NavSatFix

GPS_FIX_DICT = {
    0: ('No GPS', curses.COLOR_RED),
    1: ('No fix', curses.COLOR_RED),
    2: ('2D lock', curses.COLOR_BLUE),
    3: ('3D lock', curses.COLOR_BLUE),
    4: ('DGPS', curses.COLOR_MAGENTA),
    5: ('RTK float', curses.COLOR_YELLOW),
    6: ('RTK fix', curses.COLOR_GREEN)
}


def get_color(color):
    return curses.color_pair(color)


def frequency_from_messages(messages):
    durations = []
    for i in range(len(messages) - 1):
        duration = messages[i + 1].header.stamp - messages[i].header.stamp
        durations.append(duration.to_sec())
    frequency = 1 / np.mean(durations)
    if np.isnan(frequency):
        return 0
    return frequency


class StatusNode:

    def __init__(self, screen):

        rospy.init_node('status_node', argv=sys.argv)

        self.rate = rospy.get_param('~rate', default=1.0)

        # Curses setup
        self.screen = curses.initscr()
        self.rows, self.cols = self.screen.getmaxyx()

        height_status = 15

        self.status = curses.newwin(height_status, self.cols, 1, 2)
        # self.console = curses.newwin(self.rows - height_status, self.cols, 12, 2)
        self.lines = 0
        self.text = ''

        self.screen.keypad(True)
        curses.curs_set(False)  # Hide cursor

        colors = [curses.COLOR_BLACK, curses.COLOR_BLUE, curses.COLOR_CYAN,
                  curses.COLOR_GREEN, curses.COLOR_MAGENTA, curses.COLOR_RED,
                  curses.COLOR_WHITE, curses.COLOR_YELLOW]

        # Curses color setup
        curses.use_default_colors()
        for color in colors:
            curses.init_pair(color, color, -1)

        # Default variables
        self.status_battery_perc = None

        self.state = State()
        self.state_sub = rospy.Subscriber('mavros/state', State,
                                          callback=self.state_callback,
                                          queue_size=1)

        self.battery = BatteryState()
        self.battery_sub = rospy.Subscriber('mavros/battery', BatteryState,
                                            callback=self.battery_callback,
                                            queue_size=1)

        self.extended = ExtendedState()
        self.extended_sub = rospy.Subscriber('mavros/extended_state', ExtendedState,
                                             callback=self.extended_callback,
                                             queue_size=1)

        # self.statustext = StatusText()
        # self.statustext_sub = rospy.Subscriber('mavros/statustext/recv', StatusText,
        #                                        callback=self.statustext_callback,
        #                                        queue_size=1)

        self.gps = NavSatFix()
        self.gps_sub = rospy.Subscriber('mavros/global_position/raw/fix', NavSatFix,
                                        callback=self.gps_callback,
                                        queue_size=1)

        self.local_pose = PoseStamped()
        self.local_pose_sub = rospy.Subscriber('mavros/local_position/pose', PoseStamped,
                                               callback=self.local_pose_callback,
                                               queue_size=1)

        self.global_pose = PoseStamped()
        self.global_pose_sub = rospy.Subscriber('global_position/pose', PoseStamped,
                                                callback=self.global_pose_callback,
                                                queue_size=1)

        self.diagnostics = DiagnosticArray()
        self.diagnostic_gps = DiagnosticStatus()
        self.diagnostics_sub = rospy.Subscriber('/diagnostics', DiagnosticArray,
                                                callback=self.diagnostics_callback,
                                                queue_size=1)

        self.setpoint = PositionTarget()
        self.setpoint_sub = rospy.Subscriber('mavros/setpoint_raw/local', PositionTarget,
                                             callback=self.setpoint_callback,
                                             queue_size=1)

        self.cameras = ['front', 'right', 'back', 'left']
        self.image_subscribers = []
        self.images = {c: deque(maxlen=10) for c in self.cameras}
        for camera in self.cameras:
            topic = f'camera_{camera}/image_raw'
            subscriber = rospy.Subscriber(topic, Image, callback=self.image_callback,
                                          callback_args=camera, queue_size=1,
                                          buff_size=2 ** 24)
            self.image_subscribers.append(subscriber)

    def battery_callback(self, battery_msg):
        if battery_msg.location == 'id0':
            self.battery = battery_msg

    def state_callback(self, state_msg):
        self.state = state_msg

    def extended_callback(self, extended_msg):
        self.extended = extended_msg

    def diagnostics_callback(self, diagnostics_msg):
        for status in diagnostics_msg.status:
            if 'GPS' in status.name:
                self.diagnostic_gps = status

    def gps_callback(self, gps_msg):
        self.gps = gps_msg

    def local_pose_callback(self, pose_msg):
        self.local_pose = pose_msg

    def global_pose_callback(self, pose_msg):
        self.global_pose = pose_msg

    def setpoint_callback(self, setpoint_msg):
        self.setpoint = setpoint_msg

    def image_callback(self, image_msg, camera):
        self.images[camera].append(image_msg)

    def statustext_callback(self, statustext_msg):

        screen = self.console
        time_str = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        # time_str = datetime.datetime.fromtimestamp(unix_time)

        text = statustext_msg.text
        severity = statustext_msg.severity
        msg = statustext_msg

        severity_red = [msg.EMERGENCY, msg.ALERT, msg.CRITICAL, msg.ERROR]
        severity_yellow = [msg.WARNING, msg.NOTICE]
        severity_neutral = [msg.INFO, msg.DEBUG]

        color = curses.COLOR_CYAN
        if severity in severity_red:
            color = curses.COLOR_RED
        elif severity in severity_yellow:
            color = curses.COLOR_YELLOW
        elif severity in severity_neutral:
            color = curses.COLOR_WHITE

        self.text = f'{time_str}: {text} ({color})'
        # screen.addstr(self.lines, 0, log, get_color(color))

        self.lines += 1
        screen.refresh()

    def print_status(self):

        screen = self.status

        screen.clear()

        # rospy.loginfo(status)

        # print(status)
        x_tab = 0
        x_indent = 14
        row = 0

        # Battery
        battery_percentage = int(self.battery.percentage * 100)
        color = curses.COLOR_CYAN
        if battery_percentage > 50:
            color = curses.COLOR_GREEN
        elif battery_percentage > 25:
            color = curses.COLOR_YELLOW
        elif battery_percentage > 0:
            color = curses.COLOR_RED
        status_battery = str(battery_percentage) + '%'
        screen.addstr(row, x_tab, 'Battery: ')
        screen.addstr(row, x_indent, status_battery, get_color(color))
        row += 1

        # Armed
        if self.state.armed:
            color = curses.COLOR_RED
            status_armed = 'Yes'
        else:
            color = curses.COLOR_GREEN
            status_armed = 'No'
        screen.addstr(row, x_tab, 'Armed: ')
        screen.addstr(row, x_indent, status_armed, get_color(color))
        row += 1

        # Mode
        color = curses.COLOR_CYAN
        mode = self.state.mode
        if mode.startswith('AUTO'):
            mode = mode.split('.')[-1]
        mode = mode.capitalize()

        if mode == 'Offboard':
            color = curses.COLOR_RED
        else:
            color = curses.COLOR_BLUE

        if mode == '':
            mode = 'None'
        elif mode == 'Posctl':
            mode = 'Position'
        elif mode == 'Rtl':
            mode = 'Return'

        status_mode = '{}'.format(mode)
        screen.addstr(row, x_tab, 'Mode: ')
        screen.addstr(row, x_indent, status_mode, get_color(color))
        row += 1

        # Extended status
        if self.extended.landed_state == self.extended.LANDED_STATE_IN_AIR:
            status_extended = 'Air'
            color = curses.COLOR_RED
        elif self.extended.landed_state == self.extended.LANDED_STATE_LANDING:
            status_extended = 'Landed'
            color = curses.COLOR_GREEN
        elif self.extended.landed_state == self.extended.LANDED_STATE_ON_GROUND:
            status_extended = 'Ground'
            color = curses.COLOR_GREEN
        elif self.extended.landed_state == self.extended.LANDED_STATE_TAKEOFF:
            status_extended = 'Takeoff'
            color = curses.COLOR_RED
        elif self.extended.landed_state == self.extended.LANDED_STATE_UNDEFINED:
            status_extended = 'Undefined'
            color = curses.COLOR_CYAN
        screen.addstr(row, x_tab, 'State: ')
        screen.addstr(row, x_indent, status_extended, get_color(color))
        row += 1

        # GPS info
        satellites = 0
        fix_type, color = GPS_FIX_DICT[0]
        for value in self.diagnostic_gps.values:
            if value.key == 'Satellites visible':
                satellites = value.value
            elif value.key == 'Fix type':
                fix_type, color = GPS_FIX_DICT[int(value.value)]
        screen.addstr(row, x_tab, 'GPS info: ')
        screen.addstr(row, x_indent, f'{fix_type} ({satellites} sat)', get_color(color))
        row += 2

        # GPS pos
        latitude = self.gps.latitude
        longitude = self.gps.longitude
        altitude = round(self.gps.altitude, 2)
        status_gps = f'{latitude:.7f} {longitude:.7f} {altitude:.2f}  (LLA)'
        screen.addstr(row, x_tab, 'GPS pos: ')
        screen.addstr(row, x_indent, status_gps)
        row += 1

        # Local pose
        p = self.local_pose.pose.position
        q = self.local_pose.pose.orientation
        quaternion = [q.x, q.y, q.z, q.w]
        try:
            rot = R.from_quat(quaternion)
        except ValueError:
            rot = R.from_euler('zyx', [0.0, 0.0, 0.0])
        yaw, pitch, roll = rot.as_euler('zyx', degrees=True)
        x, y, z = round(p.x, 2), round(p.y, 2), round(p.z, 2)
        yaw, pitch, roll = int(yaw), int(pitch), int(roll)
        screen.addstr(row, x_tab, 'Local pos: ')
        screen.addstr(row, x_indent, f'{x:.2f} {y:.2f} {z:.2f} (XYZ)    {roll} {pitch} {yaw} (RPY)')
        row += 1

        # Global pose
        p = self.global_pose.pose.position
        q = self.global_pose.pose.orientation
        quaternion = [q.x, q.y, q.z, q.w]
        try:
            rot = R.from_quat(quaternion)
        except ValueError:
            rot = R.from_euler('zyx', [0.0, 0.0, 0.0])
        yaw, pitch, roll = rot.as_euler('zyx', degrees=True)
        x, y, z = round(p.x, 2), round(p.y, 2), round(p.z, 2)
        yaw, pitch, roll = int(yaw), int(pitch), int(roll)
        screen.addstr(row, x_tab, 'Global pos: ')
        screen.addstr(row, x_indent, f'{x:.2f} {y:.2f} {z:.2f} (XYZ)    {roll} {pitch} {yaw} (RPY)')
        row += 1

        # Setpoint
        v = self.setpoint.velocity
        vx, vy, vz = round(v.x, 2), round(v.y, 2), round(v.z, 2)
        yaw = int(np.rad2deg(self.setpoint.yaw))
        screen.addstr(row, x_tab, 'Setpoint: ')
        screen.addstr(row, x_indent, f'{vx:.2f} {vy:.2f} {vz:.2f} (XYZ)    {yaw} (Y)')
        row += 1

        # Cameras
        freqs = {c: 0 for c in self.cameras}
        for cam, messages in self.images.items():
            freqs[cam] = frequency_from_messages(messages)
        ff, fr, fb, fl = [int(round(v)) for k, v in freqs.items()]
        screen.addstr(row, x_tab, 'Cameras: ')
        screen.addstr(row, x_indent, f'{ff} {fr} {fb} {fl}  (front right back left [Hz])')
        row += 1

        screen.refresh()
        self.screen.refresh()

    def run(self):
        rate = rospy.Rate(self.rate)
        try:
            while not rospy.is_shutdown():
                self.print_status()
                rate.sleep()
        except rospy.ROSInterruptException:
            curses.nocbreak()
            self.screen.keypad(False)
            curses.echo()


def curses_main(screen):
    StatusNode(screen).run()


def main():
    try:
        curses.wrapper(curses_main)
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()

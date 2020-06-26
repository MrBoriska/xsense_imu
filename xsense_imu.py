from engine.pyalice import *

import select
import sys

import mtdevice
import mtdef

#from std_msgs.msg import Header, String, UInt16
#from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus, MagneticField,\
#    FluidPressure, Temperature, TimeReference
#from geometry_msgs.msg import TwistStamped, PointStamped
#from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus

import time
import datetime
import calendar
import serial

# transform Euler angles or matrix into quaternions
from math import radians, sqrt, atan2

#from tf.transformations import quaternion_from_matrix, quaternion_from_euler,\
#    identity_matrix

class XSensDriver(Codelet):

    def get_param(self, name, default):
        try:
            v = getattr(self.config, name)
            if v is None: raise KeyError
            print("Info: Found parameter: %s, value: %s" % (name, str(v)))
        except KeyError:
            v = default
            print("Warn: Cannot find value for parameter: %s, assigning "
                        "default: %s" % (name, str(v)))
        return v

    def start(self):
        device = self.get_param('device', 'auto')
        baudrate = self.get_param('baudrate', 0)
        timeout = self.get_param('timeout', 0.002)
        initial_wait = self.get_param('initial_wait', 0.1)
        if device == 'auto':
            devs = mtdevice.find_devices(timeout=timeout,
                                         initial_wait=initial_wait)
            if devs:
                device, baudrate = devs[0]
                print("Info: Detected MT device on port %s @ %d bps"
                              % (device, baudrate))
            else:
                print("Fatal: could not find proper MT device.")
                print("Could not find proper MT device.")
                return
        if not baudrate:
            baudrate = mtdevice.find_baudrate(device, timeout=timeout,
                                              initial_wait=initial_wait)
        if not baudrate:
            print("Fatal: could not find proper baudrate.")
            print("Could not find proper baudrate.")
            return

        print("Info: MT node interface: %s at %d bd." % (device, baudrate))
        self.mt = mtdevice.MTDevice(device, baudrate, timeout,
                                    initial_wait=initial_wait)

        # optional no rotation procedure for internal calibration of biases
        # (only mark iv devices)
        no_rotation_duration = self.get_param('~no_rotation_duration', 0)
        if no_rotation_duration:
            print("Info: Starting the no-rotation procedure to estimate the "
                          "gyroscope biases for %d s. Please don't move the IMU"
                          " during this time." % no_rotation_duration)
            self.mt.SetNoRotation(no_rotation_duration)

        self.frame_id = self.get_param('frame_id', '/base_imu')

        self.frame_local = self.get_param('frame_local', 'ENU')

        #self.angular_velocity_covariance = matrix_from_diagonal(
        #    get_param_list('~angular_velocity_covariance_diagonal', [radians(0.025)] * 3)
        #)
        #self.linear_acceleration_covariance = matrix_from_diagonal(
        #    get_param_list('~linear_acceleration_covariance_diagonal', [0.0004] * 3)
        #)
        #self.orientation_covariance = matrix_from_diagonal(
        #    get_param_list("~orientation_covariance_diagonal", [radians(1.), radians(1.), radians(9.)])
        #)

        #self.diag_msg = DiagnosticArray()
        #self.stest_stat = DiagnosticStatus(name='mtnode: Self Test', level=1,
        #                                   message='No status information')
        #self.xkf_stat = DiagnosticStatus(name='mtnode: XKF Valid', level=1,
        #                                 message='No status information')
        #self.gps_stat = DiagnosticStatus(name='mtnode: GPS Fix', level=1,
        #                                 message='No status information')
        #self.diag_msg.status = [self.stest_stat, self.xkf_stat, self.gps_stat]

        # publishers created at first use to reduce topic clutter
        self.diag_pub = None
        self.imu_pub = None
        self.raw_gps_pub = None
        self.vel_pub = None
        self.mag_pub = None
        self.pos_gps_pub = None
        self.temp_pub = None
        self.press_pub = None
        self.analog_in1_pub = None  # decide type+header
        self.analog_in2_pub = None  # decide type+header
        self.ecef_pub = None
        self.time_ref_pub = None
        # TODO pressure, ITOW from raw GPS?
        self.old_bGPS = 256  # publish GPS only if new

        # publish a string version of all data; to be parsed by clients
        #self.str_pub = rospy.Publisher('imu_data_str', String, queue_size=10)
        self.last_delta_q_time = None
        self.delta_q_rate = None







        # This part will be run once in the beginning of the program
        # We can tick periodically, on every message, or blocking. See documentation for details.
        self.tick_periodically(1.0)

    def tick(self):
        # This part will be run at every tick. We are ticking periodically in this example.

        # Print out message to console. The message is set in ping_python.app.json file.
        print(self.config.message)


def main():
    app = Application(app_filename="packages/xsense_imu/xsense_imu.app.json")
    app.nodes["xsense_imu_node"].add(XSensDriver)
    app.start_wait_stop()


if __name__ == '__main__':
    main()

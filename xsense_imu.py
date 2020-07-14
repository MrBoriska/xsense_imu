from engine.pyalice import *

import select
import sys

import mtdevice
import mtdef

import time
import datetime
import calendar
import serial
import traceback

import numpy
import quaternion

import json5

# transform Euler angles or matrix into quaternions
from math import radians, sqrt, atan2

from transformations import quaternion_from_matrix, quaternion_from_euler,\
    identity_matrix, euler_from_quaternion

class XSensDriver(Codelet):

    def get_param(self, name, default):
        try:
            v = getattr(self.config, name)
            if v is None: raise KeyError
            self.log_info("Found parameter: %s, value: %s" % (name, str(v)))
        except KeyError:
            v = default
            self.log_warning("Cannot find value for parameter: %s, assigning "
                        "default: %s" % (name, str(v)))
        return v

    def matrix_from_diagonal(self, diagonal):
        n = len(diagonal)
        matrix = [0] * n * n
        for i in range(0, n):
            matrix[i*n + i] = diagonal[i]
        return tuple(matrix)

    def start(self):
        self.log_info("XSenseDriver starting by time %f" % self.app.clock.time)

        self.freq_div = self.get_param("freq_div", 1)

        base_freq = self.get_param("base_freq", 500)

        device = self.get_param('device', 'auto')

        baudrate = self.get_param('baudrate', 0)
        timeout = self.get_param('timeout', 0.002)
        initial_wait = self.get_param('initial_wait', 0.1)
        if device == 'auto':
            devs = mtdevice.find_devices(timeout=timeout,
                                         initial_wait=initial_wait)
            if devs:
                device, baudrate = devs[0]
                self.log_info("Detected MT device on port %s @ %d bps"
                              % (device, baudrate))
            else:
                self.log_critical("Could not find proper MT device.")
                return
        if not baudrate:
            baudrate = mtdevice.find_baudrate(device, timeout=timeout,
                                                      initial_wait=initial_wait)
        if not baudrate:
            self.log_critical("Could not find proper baudrate.")
            return

        self.log_info("MT node interface: %s at %d bd." % (device, baudrate))
        self.mt = mtdevice.MTDevice(device, baudrate, timeout,
                                    initial_wait=initial_wait)

        # optional no rotation procedure for internal calibration of biases
        # (only mark iv devices)
        no_rotation_duration = self.get_param('no_rotation_duration', 0)
        if no_rotation_duration:
            self.log_info("Starting the no-rotation procedure to estimate the "
                          "gyroscope biases for %d s. Please don't move the IMU"
                          " during this time." % no_rotation_duration)
            self.mt.SetNoRotation(no_rotation_duration)

        self.frame_id = self.get_param('frame_id', '/base_imu')
        self.base_frame_id = self.get_param('base_frame_id', '/base_link')
        self.frame_local = self.get_param('frame_local', 'ENU')
        
        # Provides XSense pose to Isaac app pose tree.
        q = self.get_param("rotate_from_base_frame", [0,0,0])
        t = self.get_param("trans_from_base_frame", [0,0,0])
        self.app.atlas.set_pose(
            self.frame_id, # lhs (str): left-hand-side frame name
            self.base_frame_id, # rhs (str): right-hand-side frame name
            self.app.clock.time, # app_time (float): Isaac app time in seconds
            # pose (List): tuple of (numpy.quaternion, numpy.array(3)) for rotation and translation accordingly.
            (quaternion.from_euler_angles(q), numpy.array(t)) 
        )
        

        # Covariances... (not used in Isaac SDK)
        self.angular_velocity_covariance = self.matrix_from_diagonal(
            self.get_param('angular_velocity_covariance_diagonal', [radians(0.025)] * 3)
        )
        self.linear_acceleration_covariance = self.matrix_from_diagonal(
            self.get_param('linear_acceleration_covariance_diagonal', [0.0004] * 3)
        )
        self.orientation_covariance = self.matrix_from_diagonal(
            self.get_param("orientation_covariance_diagonal", [radians(1.), radians(1.), radians(9.)])
        )

        """
        self.diag_msg = DiagnosticArray()
        self.stest_stat = DiagnosticStatus(name='mtnode: Self Test', level=1,
                                           message='No status information')
        self.xkf_stat = DiagnosticStatus(name='mtnode: XKF Valid', level=1,
                                         message='No status information')
        self.gps_stat = DiagnosticStatus(name='mtnode: GPS Fix', level=1,
                                         message='No status information')
        self.diag_msg.status = [self.stest_stat, self.xkf_stat, self.gps_stat]
        """


        # publishers created at first use to reduce topic clutter
        self.diag_pub = self.isaac_proto_tx("JsonProto", "diagnostics")
        self.imu_pub = self.isaac_proto_tx("ImuProto", "imu_raw")
        #self.raw_gps_pub = None
        #self.vel_pub = None
        #self.mag_pub = None
        #self.pos_gps_pub = None
        #self.temp_pub = self.isaac_proto_tx("Float64", "temperature")
        #self.press_pub = None
        #self.analog_in1_pub = None  # decide type+header
        #self.analog_in2_pub = None  # decide type+header
        #self.ecef_pub = None
        #self.time_ref_pub = None
        # TODO pressure, ITOW from raw GPS?
        #self.old_bGPS = 256  # publish GPS only if new

        # publish a string version of all data; to be parsed by clients
        #self.str_pub = rospy.Publisher('imu_data_str', String, queue_size=10)
        
        self.last_delta_q_time = None
        self.delta_q_rate = None



        # This part will be run once in the beginning of the program
        # We can tick periodically, on every message, or blocking. See documentation for details.
        #self.tick_blocking()
        self.tick_periodically(1/base_freq)

    def tick(self):
        # This part will be run at every tick. We are ticking periodically in this example.
        try:
            self.spin_once()
        # Ctrl-C signal interferes with select with the ROS signal handler
        # should be OSError in python 3.?
        except (select.error, OSError, serial.serialutil.SerialException):
            traceback.print_exc()
        except Exception as e:
            traceback.print_exc()
            raise e


    def reset_vars(self):
        self.imu_msg = self.imu_pub.init().proto
        #self.imu_msg.orientation_covariance = (-1., )*9
        #self.imu_msg.angular_velocity_covariance = (-1., )*9
        #self.imu_msg.linear_acceleration_covariance = (-1., )*9
        self.pub_imu = False
        #self.raw_gps_msg = NavSatFix()
        #self.pub_raw_gps = False
        #self.pos_gps_msg = NavSatFix()
        #self.pub_pos_gps = False
        #self.vel_msg = TwistStamped()
        #self.pub_vel = False
        #self.mag_msg = MagneticField()
        #self.mag_msg.magnetic_field_covariance = (0, )*9
        #self.pub_mag = False
        #self.temp_msg = Temperature()
        #self.temp_msg.variance = 0.
        #self.pub_temp = False
        #self.press_msg = FluidPressure()
        #self.press_msg.variance = 0.
        #self.pub_press = False
        #self.anin1_msg = UInt16()
        #self.pub_anin1 = False
        #self.anin2_msg = UInt16()
        #self.pub_anin2 = False
        #self.ecef_msg = PointStamped()
        #self.pub_ecef = False
        #self.pub_diag = False

    def spin_once(self):
        '''Read data from device and publishes ROS messages.'''
        def convert_coords(x, y, z, source, dest=self.frame_local):
            """Convert the coordinates between ENU, NED, and NWU."""
            if source == dest:
                return x, y, z
            # convert to ENU
            if source == 'NED':
                x, y, z = y, x, -z
            elif source == 'NWU':
                x, y, z = -y, x, z
            # convert to desired
            if dest == 'NED':
                x, y, z = y, x, -z
            elif dest == 'NWU':
                x, y, z = y, -x, z
            return x, y, z

        def convert_quat(q, source, dest=self.frame_local):
            """Convert a quaternion between ENU, NED, and NWU."""
            def q_mult(xxx_todo_changeme, xxx_todo_changeme1):
                """Quaternion multiplication."""
                (w0, x0, y0, z0) = xxx_todo_changeme
                (w1, x1, y1, z1) = xxx_todo_changeme1
                w = w0*w1 - x0*x1 - y0*y1 - z0*z1
                x = w0*x1 + x0*w1 + y0*z1 - z0*y1
                y = w0*y1 - x0*z1 + y0*w1 + z0*x1
                z = w0*z1 + x0*y1 - y0*x1 + z0*w1
                return (w, x, y, z)
            q_enu_ned = (0, 1./sqrt(2), 1./sqrt(2), 0)
            q_enu_nwu = (1./sqrt(2), 0, 0, -1./sqrt(2))
            q_ned_nwu = (0, -1, 0, 0)
            q_ned_enu = (0, -1./sqrt(2), -1./sqrt(2), 0)
            q_nwu_enu = (1./sqrt(2), 0, 0, 1./sqrt(2))
            q_nwu_ned = (0, 1, 0, 0)
            if source == 'ENU':
                if dest == 'ENU':
                    return q
                elif dest == 'NED':
                    return q_mult(q_enu_ned, q)
                elif dest == 'NWU':
                    return q_mult(q_enu_nwu, q)
            elif source == 'NED':
                if dest == 'ENU':
                    return q_mult(q_ned_enu, q)
                elif dest == 'NED':
                    return q
                elif dest == 'NWU':
                    return q_mult(q_ned_nwu, q)
            elif source == 'NWU':
                if dest == 'ENU':
                    return q_mult(q_nwu_enu, q)
                elif dest == 'NED':
                    return q_mult(q_nwu_ned, q)
                elif dest == 'NWU':
                    return q

        #def quaternion_from_matrix(m: 'matrix 3x3'):
        #    pass
        #def quaternion_from_euler(roll, pitch, yaw):
        #    pass
        #def identity_matrix():
        #    pass
        #def quat_to_rpy(w, x, y, z):
        #
        #    return roll, pitch, yaw

        """
        def publish_time_ref(secs, nsecs, source):
            # Publish a time reference
            # Doesn't follow the standard publishing pattern since several time
            # refs could be published simultaneously
            if self.time_ref_pub is None:
                self.time_ref_pub = rospy.Publisher(
                    'time_reference', TimeReference, queue_size=10)
            time_ref_msg = TimeReference()
            time_ref_msg.header = self.h
            time_ref_msg.time_ref.secs = secs
            time_ref_msg.time_ref.nsecs = nsecs
            time_ref_msg.source = source
            self.time_ref_pub.publish(time_ref_msg)

        def stamp_from_itow(itow, y=None, m=None, d=None, ns=0, week=None):
            # Return (secs, nsecs) from GPS time of week ms information
            if y is not None:
                stamp_day = datetime.datetime(y, m, d)
            elif week is not None:
                epoch = datetime.datetime(1980, 1, 6)  # GPS epoch
                stamp_day = epoch + datetime.timedelta(weeks=week)
            else:
                today = datetime.date.today()  # using today by default
                stamp_day = datetime.datetime(today.year, today.month,
                                              today.day)
            iso_day = stamp_day.isoweekday()  # 1 for Monday, 7 for Sunday
            # stamp for the GPS start of the week (Sunday morning)
            start_of_week = stamp_day - datetime.timedelta(days=iso_day)
            # stamp at the millisecond precision
            stamp_ms = start_of_week + datetime.timedelta(milliseconds=itow)
            secs = calendar.timegm((stamp_ms.year, stamp_ms.month, stamp_ms.day,
                                    stamp_ms.hour, stamp_ms.minute,
                                    stamp_ms.second, 0, 0, -1))
            nsecs = stamp_ms.microsecond * 1000 + ns
            if nsecs < 0:  # ns can be negative
                secs -= 1
                nsecs += 1e9
            return (secs, nsecs)

        # MTData
        def fill_from_RAW(raw_data):
            # Fill messages with information from 'raw' MTData block.
            # don't publish raw imu data anymore
            # TODO find what to do with that
            rospy.loginfo("Got MTi data packet: 'RAW', ignored!")

        def fill_from_RAWGPS(rawgps_data):
            # Fill messages with information from 'rawgps' MTData block
            if rawgps_data['bGPS'] < self.old_bGPS:
                self.pub_raw_gps = True
                # LLA
                self.raw_gps_msg.latitude = rawgps_data['LAT']*1e-7
                self.raw_gps_msg.longitude = rawgps_data['LON']*1e-7
                self.raw_gps_msg.altitude = rawgps_data['ALT']*1e-3
                # NED vel # TODO?
            self.old_bGPS = rawgps_data['bGPS']

        def fill_from_Temp(temp):
            # Fill messages with information from 'temperature' MTData block
            self.pub_temp = True
            self.temp_msg.temperature = temp

        def fill_from_Calib(imu_data):
            # Fill messages with information from 'calibrated' MTData block
            try:
                self.pub_imu = True
                x, y, z = imu_data['gyrX'], imu_data['gyrY'], imu_data['gyrZ']
                self.imu_msg.angular_velocity.x = x
                self.imu_msg.angular_velocity.y = y
                self.imu_msg.angular_velocity.z = z
                self.imu_msg.angular_velocity_covariance = self.angular_velocity_covariance
                self.pub_vel = True
                self.vel_msg.twist.angular.x = x
                self.vel_msg.twist.angular.y = y
                self.vel_msg.twist.angular.z = z
            except KeyError:
                pass
            try:
                self.pub_imu = True
                x, y, z = imu_data['accX'], imu_data['accY'], imu_data['accZ']
                self.imu_msg.linear_acceleration.x = x
                self.imu_msg.linear_acceleration.y = y
                self.imu_msg.linear_acceleration.z = z
                self.imu_msg.linear_acceleration_covariance = self.linear_acceleration_covariance
            except KeyError:
                pass
            try:
                self.pub_mag = True
                x, y, z = imu_data['magX'], imu_data['magY'], imu_data['magZ']
                self.mag_msg.magnetic_field.x = x
                self.mag_msg.magnetic_field.y = y
                self.mag_msg.magnetic_field.z = z
            except KeyError:
                pass
        """
        def fill_from_Orient(orient_data):
            '''Fill messages with information from 'orientation' MTData block.
            '''
            self.pub_imu = True
            #self.imu_msg = self.imu_pub.initProto()
            if 'quaternion' in orient_data:
                w, x, y, z = orient_data['quaternion']
            elif 'roll' in orient_data:
                x, y, z, w = quaternion_from_euler(
                    radians(orient_data['roll']), radians(orient_data['pitch']),
                    radians(orient_data['yaw']))
            elif 'matrix' in orient_data:
                m = identity_matrix()
                m[:3, :3] = orient_data['matrix']
                x, y, z, w = quaternion_from_matrix(m)


            w, x, y, z = convert_quat((w, x, y, z), o['frame'])


            roll, pitch, yaw = euler_from_quaternion((w, x, y, z))

            self.imu_msg.angleRoll = roll
            self.imu_msg.anglePitch = pitch
            self.imu_msg.angleYaw = yaw

            #imu_msg.orientation_covariance = self.orientation_covariance

        """
        def fill_from_Auxiliary(aux_data):
            # Fill messages with information from 'Auxiliary' MTData block
            try:
                self.anin1_msg.data = o['Ain_1']
                self.pub_anin1 = True
            except KeyError:
                pass
            try:
                self.anin2_msg.data = o['Ain_2']
                self.pub_anin2 = True
            except KeyError:
                pass

        def fill_from_Pos(position_data):
            '''Fill messages with information from 'position' MTData block.'''
            self.pub_pos_gps = True
            self.pos_gps_msg.latitude = position_data['Lat']
            self.pos_gps_msg.longitude = position_data['Lon']
            self.pos_gps_msg.altitude = position_data['Alt']

        def fill_from_Vel(velocity_data):
            '''Fill messages with information from 'velocity' MTData block.'''
            self.pub_vel = True
            x, y, z = convert_coords(
                velocity_data['Vel_X'], velocity_data['Vel_Y'],
                velocity_data['Vel_Z'], o['frame'])
            self.vel_msg.twist.linear.x = x
            self.vel_msg.twist.linear.y = y
            self.vel_msg.twist.linear.z = z

        def fill_from_Stat(status):
            # Fill messages with information from 'status' MTData block
            self.pub_diag = True
            if status & 0b0001:
                self.stest_stat.level = DiagnosticStatus.OK
                self.stest_stat.message = "Ok"
            else:
                self.stest_stat.level = DiagnosticStatus.ERROR
                self.stest_stat.message = "Failed"
            if status & 0b0010:
                self.xkf_stat.level = DiagnosticStatus.OK
                self.xkf_stat.message = "Valid"
            else:
                self.xkf_stat.level = DiagnosticStatus.WARN
                self.xkf_stat.message = "Invalid"
            if status & 0b0100:
                self.gps_stat.level = DiagnosticStatus.OK
                self.gps_stat.message = "Ok"
                self.raw_gps_msg.status.status = NavSatStatus.STATUS_FIX
                self.raw_gps_msg.status.service = NavSatStatus.SERVICE_GPS
                # we borrow the status from the raw gps for pos_gps_msg
                self.pos_gps_msg.status.status = NavSatStatus.STATUS_FIX
                self.pos_gps_msg.status.service = NavSatStatus.SERVICE_GPS
            else:
                self.gps_stat.level = DiagnosticStatus.WARN
                self.gps_stat.message = "No fix"
                self.raw_gps_msg.status.status = NavSatStatus.STATUS_NO_FIX
                self.raw_gps_msg.status.service = 0
                # we borrow the status from the raw gps for pos_gps_msg
                self.pos_gps_msg.status.status = NavSatStatus.STATUS_NO_FIX
                self.pos_gps_msg.status.service = 0

        def fill_from_Sample(ts):
            # Catch 'Sample' MTData blocks
            self.h.seq = ts

        # MTData2
        def fill_from_Temperature(o):
            # Fill messages with information from 'Temperature' MTData2 block
            self.pub_temp = True
            self.temp_msg.temperature = o['Temp']

        def fill_from_Timestamp(o):
            # Fill messages with information from 'Timestamp' MTData2 block
            try:
                # put timestamp from gps UTC time if available
                y, m, d, hr, mi, s, ns, f = o['Year'], o['Month'], o['Day'],\
                    o['Hour'], o['Minute'], o['Second'], o['ns'], o['Flags']
                if f & 0x4:
                    secs = calendar.timegm((y, m, d, hr, mi, s, 0, 0, 0))
                    publish_time_ref(secs, ns, 'UTC time')
            except KeyError:
                pass
            try:
                itow = o['TimeOfWeek']
                secs, nsecs = stamp_from_itow(itow)
                publish_time_ref(secs, nsecs, 'integer time of week')
            except KeyError:
                pass
            try:
                sample_time_fine = o['SampleTimeFine']
                # int in 10kHz ticks
                secs = int(sample_time_fine / 10000)
                nsecs = 1e5 * (sample_time_fine % 10000)
                publish_time_ref(secs, nsecs, 'sample time fine')
            except KeyError:
                pass
            try:
                sample_time_coarse = o['SampleTimeCoarse']
                publish_time_ref(sample_time_coarse, 0, 'sample time coarse')
            except KeyError:
                pass
            # TODO find what to do with other kind of information
            pass
        """

        def fill_from_Orientation_Data(o):
            '''Fill messages with information from 'Orientation Data' MTData2
            block.'''
            self.pub_imu = True
            #self.imu_msg = self.imu_pub.initProto()
            try:
                x, y, z, w = o['Q1'], o['Q2'], o['Q3'], o['Q0']
            except KeyError:
                pass
            try:
                x, y, z, w = quaternion_from_euler(radians(o['Roll']),
                                                   radians(o['Pitch']),
                                                   radians(o['Yaw']))
            except KeyError:
                pass
            try:
                a, b, c, d, e, f, g, h, i = o['a'], o['b'], o['c'], o['d'],\
                    o['e'], o['f'], o['g'], o['h'], o['i']
                m = identity_matrix()
                m[:3, :3] = ((a, b, c), (d, e, f), (g, h, i))
                x, y, z, w = quaternion_from_matrix(m)
            except KeyError:
                pass
            w, x, y, z = convert_quat((w, x, y, z), o['frame'])

            roll, pitch, yaw = euler_from_quaternion((w, x, y, z))

            self.imu_msg.angleRoll = roll
            self.imu_msg.anglePitch = pitch
            self.imu_msg.angleYaw = yaw

            #self.imu_msg.orientation_covariance = self.orientation_covariance
        """
        def fill_from_Pressure(o):
            # Fill messages with information from 'Pressure' MTData2 block
            self.press_msg.fluid_pressure = o['Pressure']
            self.pub_press = True
        """
        def fill_from_Acceleration(o):
            # Fill messages with information from 'Acceleration' MTData2 block
            
            self.pub_imu = True
            #self.imu_msg = self.imu_pub.initProto()

            # FIXME not sure we should treat all in that same way
            try:
                x, y, z = o['Delta v.x'], o['Delta v.y'], o['Delta v.z']
            except KeyError:
                pass
            try:
                x, y, z = o['freeAccX'], o['freeAccY'], o['freeAccZ']
            except KeyError:
                pass
            try:
                x, y, z = o['accX'], o['accY'], o['accZ']
            except KeyError:
                pass

            self.imu_msg.linearAccelerationX = x
            self.imu_msg.linearAccelerationY = y
            self.imu_msg.linearAccelerationZ = z

            #self.imu_msg.linear_acceleration_covariance = self.linear_acceleration_covariance
        """
        def fill_from_Position(o):
            '''Fill messages with information from 'Position' MTData2 block.'''
            try:
                self.pos_gps_msg.latitude = o['lat']
                self.pos_gps_msg.longitude = o['lon']
                # altMsl is deprecated
                alt = o.get('altEllipsoid', o.get('altMsl', 0))
                self.pos_gps_msg.altitude = alt
                self.pub_pos_gps = True
            except KeyError:
                pass
            try:
                x, y, z = o['ecefX'], o['ecefY'], o['ecefZ']
                # TODO: ecef units not specified: might not be in meters!
                self.ecef_msg.point.x = x
                self.ecef_msg.point.y = y
                self.ecef_msg.point.z = z
                self.pub_ecef = True
            except KeyError:
                pass

        def fill_from_GNSS(o):
            '''Fill messages with information from 'GNSS' MTData2 block.'''
            try:  # PVT
                # time block
                itow, y, m, d, ns, f = o['itow'], o['year'], o['month'],\
                    o['day'], o['nano'], o['valid']
                if f & 0x4:
                    secs, nsecs = stamp_from_itow(itow, y, m, d, ns)
                    publish_time_ref(secs, nsecs, 'GNSS time UTC')
                # flags
                fixtype = o['fixtype']
                if fixtype == 0x00:
                    self.raw_gps_msg.status.status = NavSatStatus.STATUS_NO_FIX  # no fix
                    self.raw_gps_msg.status.service = 0
                else:
                    self.raw_gps_msg.status.status = NavSatStatus.STATUS_FIX  # unaugmented
                    self.raw_gps_msg.status.service = NavSatStatus.SERVICE_GPS
                # lat lon alt
                self.raw_gps_msg.latitude = o['lat']
                self.raw_gps_msg.longitude = o['lon']
                self.raw_gps_msg.altitude = o['height']/1e3
                # TODO should we separate raw_gps and GNSS?
                self.pub_raw_gps = True
                # TODO velocity?
                # TODO 2D heading?
                # TODO DOP?
            except KeyError:
                pass
            # TODO publish Sat Info
        """
        def fill_from_Angular_Velocity(o):
            '''Fill messages with information from 'Angular Velocity' MTData2
            block.'''
            try:
                dqw, dqx, dqy, dqz = (o['Delta q0'], o['Delta q1'],
                    o['Delta q2'], o['Delta q3'])

                now = self.tick_time

                if self.last_delta_q_time is None:
                    self.last_delta_q_time = now
                else:
                    # update rate (filtering needed to account for lag variance)
                    delta_t = now - self.last_delta_q_time
                    if self.delta_q_rate is None:
                        self.delta_q_rate = 1./delta_t
                    delta_t_filtered = .95/self.delta_q_rate + .05*delta_t
                    # rate in necessarily integer
                    self.delta_q_rate = round(1./delta_t_filtered)
                    self.last_delta_q_time = now
                    # relationship between \Delta q and velocity \bm{\omega}:
                    # \bm{w} = \Delta t . \bm{\omega}
                    # \theta = |\bm{w}|
                    # \Delta q = [cos{\theta/2}, sin{\theta/2)/\theta . \bm{\omega}
                    # extract rotation angle over delta_t
                    ca_2, sa_2 = dqw, sqrt(dqx**2 + dqy**2 + dqz**2)
                    ca = ca_2**2 - sa_2**2
                    sa = 2*ca_2*sa_2
                    rotation_angle = atan2(sa, ca)
                    # compute rotation velocity
                    rotation_speed = rotation_angle * self.delta_q_rate
                    f = rotation_speed / sa_2
                    x, y, z = f*dqx, f*dqy, f*dqz


                    self.pub_imu = True
                    #self.imu_msg = self.imu_pub.initProto()
                    self.imu_msg.angularVelocityX = x
                    self.imu_msg.angularVelocityY = y
                    self.imu_msg.angularVelocityZ = z

                    #self.imu_msg.angular_velocity_covariance = self.angular_velocity_covariance
                    #self.vel_msg.twist.angular.x = x
                    #self.vel_msg.twist.angular.y = y
                    #self.vel_msg.twist.angular.z = z
                    #self.pub_vel = True
            except KeyError:
                pass
            try:
                x, y, z = o['gyrX'], o['gyrY'], o['gyrZ']
                
                self.pub_imu = True
                #self.imu_msg = self.imu_pub.initProto()
                self.imu_msg.angularVelocityX = x
                self.imu_msg.angularVelocityY = y
                self.imu_msg.angularVelocityZ = z

                #self.imu_msg.angular_velocity_covariance = self.angular_velocity_covariance
                #self.vel_msg.twist.angular.x = x
                #self.vel_msg.twist.angular.y = y
                #self.vel_msg.twist.angular.z = z
                #self.pub_vel = True
            except KeyError:
                pass
        """
        def fill_from_GPS(o):
            '''Fill messages with information from 'GPS' MTData2 block.'''
            # TODO DOP
            try:    # SOL
                x, y, z = o['ecefX'], o['ecefY'], o['ecefZ']
                self.ecef_msg.point.x = x * 0.01  # data is in cm
                self.ecef_msg.point.y = y * 0.01
                self.ecef_msg.point.z = z * 0.01
                self.pub_ecef = True
                vx, vy, vz = o['ecefVX'], o['ecefVY'], o['ecefVZ']
                self.vel_msg.twist.linear.x = vx * 0.01  # data is in cm
                self.vel_msg.twist.linear.y = vy * 0.01
                self.vel_msg.twist.linear.z = vz * 0.01
                self.pub_vel = True
                itow, ns, week, f = o['iTOW'], o['fTOW'], o['Week'], o['Flags']
                if (f & 0x0C) == 0xC:
                    secs, nsecs = stamp_from_itow(itow, ns=ns, week=week)
                    publish_time_ref(secs, nsecs, 'GPS Time')
                # TODO there are other pieces of information that we could
                # publish
            except KeyError:
                pass
            try:    # Time UTC
                itow, y, m, d, ns, f = o['iTOW'], o['year'], o['month'],\
                    o['day'], o['nano'], o['valid']
                if f & 0x4:
                    secs, nsecs = stamp_from_itow(itow, y, m, d, ns)
                    publish_time_ref(secs, nsecs, 'GPS Time UTC')
            except KeyError:
                pass
            # TODO publish SV Info

        def fill_from_SCR(o):
            '''Fill messages with information from 'SCR' MTData2 block.'''
            # TODO that's raw information
            pass

        def fill_from_Analog_In(o):
            '''Fill messages with information from 'Analog In' MTData2 block.'''
            try:
                self.anin1_msg.data = o['analogIn1']
                self.pub_anin1 = True
            except KeyError:
                pass
            try:
                self.anin2_msg.data = o['analogIn2']
                self.pub_anin2 = True
            except KeyError:
                pass

        def fill_from_Magnetic(o):
            '''Fill messages with information from 'Magnetic' MTData2 block.'''
            x, y, z = o['magX'], o['magY'], o['magZ']
            self.mag_msg.magnetic_field.x = x
            self.mag_msg.magnetic_field.y = y
            self.mag_msg.magnetic_field.z = z
            self.pub_mag = True

        def fill_from_Velocity(o):
            '''Fill messages with information from 'Velocity' MTData2 block.'''
            x, y, z = convert_coords(o['velX'], o['velY'], o['velZ'],
                                     o['frame'])
            self.vel_msg.twist.linear.x = x
            self.vel_msg.twist.linear.y = y
            self.vel_msg.twist.linear.z = z
            self.pub_vel = True

        def fill_from_Status(o):
            '''Fill messages with information from 'Status' MTData2 block.'''
            try:
                status = o['StatusByte']
                fill_from_Stat(status)
            except KeyError:
                pass
            try:
                status = o['StatusWord']
                fill_from_Stat(status)
            except KeyError:
                pass
        
        """
        def find_handler_name(name):
            return "fill_from_%s" % (name.replace(" ", "_"))



        # get data
        try:
            data = self.mt.read_measurement()
        except mtdef.MTTimeoutException:
            time.sleep(0.1)
            return
        
        # common header
        #self.h = Header()
        #self.h.stamp = rospy.Time.now()
        #self.h.frame_id = self.frame_id

        self.reset_vars()

        # Divide frequency
        if self.tick_count % self.freq_div == 1:
            return

        # fill messages based on available data fields
        for n, o in list(data.items()):
            try:
                locals()[find_handler_name(n)](o)
            except KeyError:
                #self.log_warning("Unknown MTi data packet: '%s', ignoring." % n)
                pass

        # publish available information
        if self.pub_imu:
            #self.log_debug(self.imu_pub.msg.proto)

            self.show("imu.lin_acc_x", self.imu_msg.linearAccelerationX, self.tick_time)
            self.show("imu.lin_acc_y", self.imu_msg.linearAccelerationY, self.tick_time)
            self.show("imu.lin_acc_z", self.imu_msg.linearAccelerationZ, self.tick_time)
            self.show("imu.ang_vel_x", self.imu_msg.angularVelocityX, self.tick_time)
            self.show("imu.ang_vel_y", self.imu_msg.angularVelocityY, self.tick_time)
            self.show("imu.ang_vel_z", self.imu_msg.angularVelocityZ, self.tick_time)
            self.show("imu.roll", self.imu_msg.angleRoll, self.tick_time)
            self.show("imu.pitch", self.imu_msg.anglePitch, self.tick_time)
            self.show("imu.yaw", self.imu_msg.angleYaw, self.tick_time)

            self.imu_pub.publish()


        """
        if self.pub_raw_gps:
            self.raw_gps_msg.header = self.h
            if self.raw_gps_pub is None:
                self.raw_gps_pub = rospy.Publisher('raw_fix', NavSatFix, queue_size=10)
            self.raw_gps_pub.publish(self.raw_gps_msg)
        if self.pub_pos_gps:
            self.pos_gps_msg.header = self.h
            if self.pos_gps_pub is None:
                self.pos_gps_pub = rospy.Publisher('fix', NavSatFix, queue_size=10)
            self.pos_gps_pub.publish(self.pos_gps_msg)
        if self.pub_vel:
            self.vel_msg.header = self.h
            if self.vel_pub is None:
                self.vel_pub = rospy.Publisher('velocity', TwistStamped,
                                               queue_size=10)
            self.vel_pub.publish(self.vel_msg)
        if self.pub_mag:
            self.mag_msg.header = self.h
            if self.mag_pub is None:
                self.mag_pub = rospy.Publisher('imu/mag', MagneticField,
                                               queue_size=10)
            self.mag_pub.publish(self.mag_msg)
        if self.pub_temp:
            self.temp_msg.header = self.h
            if self.temp_pub is None:
                self.temp_pub = rospy.Publisher('temperature', Temperature,
                                                queue_size=10)
            self.temp_pub.publish(self.temp_msg)
        if self.pub_press:
            self.press_msg.header = self.h
            if self.press_pub is None:
                self.press_pub = rospy.Publisher('pressure', FluidPressure,
                                                 queue_size=10)
            self.press_pub.publish(self.press_msg)
        if self.pub_anin1:
            if self.analog_in1_pub is None:
                self.analog_in1_pub = rospy.Publisher('analog_in1',
                                                      UInt16, queue_size=10)
            self.analog_in1_pub.publish(self.anin1_msg)
        if self.pub_anin2:
            if self.analog_in2_pub is None:
                self.analog_in2_pub = rospy.Publisher('analog_in2', UInt16,
                                                      queue_size=10)
            self.analog_in2_pub.publish(self.anin2_msg)
        if self.pub_ecef:
            self.ecef_msg.header = self.h
            if self.ecef_pub is None:
                self.ecef_pub = rospy.Publisher('ecef', PointStamped,
                                                queue_size=10)
            self.ecef_pub.publish(self.ecef_msg)
        if self.pub_diag:
            #self.diag_msg.header = self.h
            if self.diag_pub is None:
                self.diag_pub = self.isaac_proto_tx("JsonProto", "diagnostics")
            self.diag_pub.publish(self.diag_msg)
        # publish string representation
        self.str_pub.publish(str(data))
        """





def main():
    app = Application(app_filename="packages/xsense_imu/xsense_imu.app.json")
    app.nodes["xsense_imu_node"].add(XSensDriver)
    app.start_wait_stop()


if __name__ == '__main__':
    main()

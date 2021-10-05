from yaw_controller import *
from pid import *
from lowpass import *
import rospy

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, vehicle_mass, fuel_capacity, brake_deadband, decel_limit,
                accel_limit, wheel_radius, wheel_base, steer_ratio, max_lat_accel, max_steer_angle):
        # TODO: Implement

        self.vehicle_mass = vehicle_mass
        self.fuel_capacity = fuel_capacity
        self.brake_deadband = brake_deadband
        self.decel_limit = decel_limit
        self.accel_limit = accel_limit
        self.wheel_radius = wheel_radius

        self.last_time = rospy.get_time()
        self.last_vel = None

        # yaw_controller
        # convert target linear and angular velocity to steering angle
        self.yaw_controller = YawController(wheel_base, steer_ratio, 0.1, max_lat_accel, max_steer_angle)

        # throttle_controller
        kp = 0.3
        ki = .1
        kd = 0.0
        mn = 0.0 # min throttle
        mx = 0.3 # max throttle
        self.throttle_controller = PID(kp, ki, kd, mn, mx)

        tau = 0.5 # 1/(2*pi*tau) = cutoff_frequency
        ts = 0.02 # sample time
        self.vel_lpf = LowPassFilter(tau, ts)

    def control(self, current_vel, dbw_enabled, linear_vel, angular_vel):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        
        if not dbw_enabled:
            self.throttle_controller.reset()
            return 0.0, 0.0, 0.0

        # velocity
        current_vel = self.vel_lpf.filt(current_vel)

        # steer angle
        steering = self.yaw_controller.get_steering(linear_vel, angular_vel, current_vel)

        # throttle
        vel_error = linear_vel - current_vel
        self.last_vel = current_vel
        current_time = rospy.get_time()
        sample_time = current_time - self.last_time
        self.last_time = current_time

        throttle = self.throttle_controller.step(vel_error, sample_time)

        # brake
        brake = 0.

        if linear_vel < 3 and current_vel < .5:
            throttle = 0
            brake = 700 # nm to hold in place
        elif throttle < .1 and vel_error < 0:
            throttle = 0
            decel =  max(vel_error, self.decel_limit)
            brake  = abs(decel) * self.vehicle_mass * self.wheel_radius # torque nm

        return throttle, brake, steering



























        return 1., 0., 0.

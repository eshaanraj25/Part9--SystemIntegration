from pid import PID
from lowpass import LowPassFilter
from yaw_controller import YawController
import rospy

# Define constants
GAS_DENSITY = 2.858
ONE_MPH = 0.44704
MAX_BRAKE_FORCE = 700.0


class Controller(object):
    def __init__(self, vehicle_mass, fuel_capacity, brake_deadband, decel_limit,
                 accel_limit, wheel_radius, wheel_base, steer_ratio, max_lat_accel,
                 max_steer_angle):
        
        # Initialize Yaw Controller
        self.yaw_controller = YawController(wheel_base, steer_ratio, 0.1, max_lat_accel, max_steer_angle)

        # Initialize PID Controller
        kp = 0.3
        ki = 0.1
        kd = 0.0
        mn = 0.0
        mx = 0.4
        self.throttle_controller = PID(kp, ki, kd, mn, mx)

        # Initialize Low Pass Filter
        tau = 0.5
        ts = 0.02
        self.vel_lpf = LowPassFilter(tau, ts)

        # Initialize class variables
        self.vehicle_mass = vehicle_mass
        self.fuel_capacity = fuel_capacity
        self.brake_deadband = brake_deadband
        self.decel_limit = decel_limit
        self.accel_limit = accel_limit
        self.wheel_radius = wheel_radius

        self.last_time = rospy.get_time()

    # Function to send control commands
    def control(self, current_vel, dbw_enabled, linear_vel, angular_vel):
        # Default commands when drive by wire not enabled
        if not dbw_enabled:
            self.throttle_controller.reset()
            return 0.0, 0.0, 0.0

        # Calculate difference in velocity
        current_vel = self.vel_lpf.filt(current_vel)
        steering = self.yaw_controller.get_steering(linear_vel, angular_vel, current_vel)
        vel_error = linear_vel - current_vel
        self.last_vel = current_vel

        # Calculate difference in time
        current_time = rospy.get_time()
        sample_time = current_time - self.last_time
        self.last_time = current_time

        # Calculate throttle
        throttle = self.throttle_controller.step(vel_error, sample_time)
        brake = 0

        # Conditions on lower magnitude variables
        if linear_vel == 0.0 and current_vel < 0.1:
            throttle = 0
            brake = MAX_BRAKE_FORCE

        elif throttle < 0.1 and vel_error < 0:
            throttle = 0
            decel = max(vel_error, self.decel_limit)
            brake = abs(decel) * self.vehicle_mass * self.wheel_radius

        return throttle, brake, steering


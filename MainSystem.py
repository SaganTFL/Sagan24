from pymavlink import mavutil
import math
import cv2


def map_value_to_range(value, in_min=-1, in_max=1, out_min=-2000, out_max=2000):
    return int((value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)


class AUVController:
    def __init__(self, connection_string):
        # connecting to the vehicle
        self.master = mavutil.mavlink_connection(connection_string)
        print("heartbeat waiting...")
        self.master.wait_heartbeat()
        print("heartbeat found")

    @staticmethod
    def get_image():
        cap = cv2.VideoCapture("udp:192.168.2.1:5650")
        ret, frame = cap.read()
        return frame

    def arm_vehicle(self):
        # arming vehicle
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1, 0, 0, 0, 0, 0, 0
        )
        print("Waiting for the vehicle to arm")
        self.master.motors_armed_wait()  # Waiting for arming the vehicle
        print('Armed!')

    def disarm_vehicle(self):
    # Disarming the vehicle
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            0, 0, 0, 0, 0, 0, 0
        )
        print("Waiting for the vehicle to disarm")
        self.master.motors_disarmed_wait()  # Waiting for disarming the vehicle
        print('Disarmed!')

    def print_motor_outputs(self):
        print(self.master.recv_match(type='SERVO_OUTPUT_RAW', blocking=True))

    def go_to_vehicle_raw(self, x=0, y=0, z=500, r=0, buttons=0, s=0, t=0, see_motor_output=0):
        # Manual controls
        enable_extensions = (1<<0) + (1<<1)
        self.master.mav.manual_control_send(
            self.master.target_system,
            x,#x is straight 2000 is full front -2000 full back
            y,#y is sideways 2000 is full right -2000 full left
            z,#z is up 1500 is full up -500 full down (pwm is 1100 up 1900 down all z axis motors)
            r,
            buttons,
            0,
            enable_extensions,
            s,
            t
        )
        if see_motor_output:
            self.print_motor_outputs()


    def to_straight_pose(self): #todo: not working well (maybe pid and deadbands)
        # straightening the vehicle all axis
        # msg = self.master.recv_match(type='ATTITUDE', blocking=True)
        # if msg:
        #     pitch = msg.pitch  # Pitch angle (radian)
        #     yaw = msg.yaw
        #     roll = msg.roll
        #
        #     # radians to degrees
        #     yaw_degrees = yaw * (180.0 / math.pi)
        #     roll_degrees = roll * (180.0 / math.pi)
        #     pitch_degrees = pitch * (180.0 / math.pi)
        #
        #     # Command for straighten the vehicle
        #     self.go_to_vehicle_raw(
        #         0,
        #         0,
        #         0,
        #         self.up_or_down_r(yaw_degrees),
        #         0,
        #         self.up_or_down_x_and_y(pitch_degrees),
        #         self.up_or_down_x_and_y(roll_degrees)
        #     )
        msg = self.master.recv_match(type='ATTITUDE', blocking=True)
        if msg:
            pitch = msg.pitch  # Pitch angle (radian)
            yaw = msg.yaw
            roll = msg.roll
            # radians to degrees
            yaw_degrees = yaw * (180.0 / math.pi)
            roll_degrees = roll * (180.0 / math.pi)
            pitch_degrees = pitch * (180.0 / math.pi)


            # if(yaw_degrees<10 and yaw_degrees > -10):
            #     yaw_degrees *= 2
            print(yaw_degrees)
            # Command for straighten the vehicle
            if(yaw_degrees<10 and yaw_degrees> -10):
                self.errorSum -= yaw_degrees
            set_yaw = (0-yaw_degrees + self.errorSum * 0.05)/180
            set_pitch = (0-pitch_degrees)/180
            set_roll = (0-roll_degrees)/180
            if(yaw_degrees<3 and yaw_degrees>-3):
                return True

            self.go_to_vehicle(
                0,
                0,
                0,
                set_yaw,
                0,
                set_pitch,
                set_roll,
                0
                # self.up_or_down_x_and_y(pitch_degrees),
                # self.up_or_down_x_and_y(roll_degrees)
            )


    def set_servo(self, servo_pin, pwm_value, see_motor_output=0):
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
            0,
            servo_pin,
            pwm_value,
            0, 0, 0, 0, 0
        )
        if see_motor_output:
            self.print_motor_outputs()
    def aux_servo_set(self, pin, state):
        # self.master.mav.servo_output_raw_send(
        #     0,          # time_usec (zaman damgası)
        #     1,          # port (1 = AUX pinleri)
        #     value,   # servo1_raw (kanal 9)
        #     0,  # servo2_raw (kanal 10)
        #     0,          # servo3_raw
        #     0,          # servo4_raw
        #     0,          # servo5_raw
        #     0,          # servo6_raw
        #     0,          # servo7_raw
        #     0           # servo8_raw
        # )
        # self.master.mav.manual_control_send(
        #     self.master.target_system,
        #     0,#x is straight 2000 is full front -2000 full back
        #     0,#y is sideways 2000 is full right -2000 full left
        #     500,#z is up 1500 is full up -500 full down (pwm is 1100 up 1900 down all z axis motors)
        #     0,
        #     0,
        #     4,
        #     0,
        #     0,
        #     value
        # )
        self.master.mav.command_long_send(
            self.master.target_system,     # target system
            self.master.target_component,  # target component
            mavutil.mavlink.MAV_CMD_DO_SET_RELAY,  # MAVLink command for digital output
            0,                        # confirmation
            pin,                      # Pin number (relay number)
            state,                    # 1: High, 0: Low
            0, 0, 0, 0, 0            # Unused parameters
        )

    def set_mode(self, mode_name):
        #(example: 'STABILIZE', 'MANUAL', 'DEPTH_HOLD')

        mode_id = self.master.mode_mapping()[mode_name]

        self.master.mav.set_mode_send(
            self.master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id
        )
        print("Aracın modu " + mode_name + " olarak değiştirildi.")
    def configure_motors(self, motor=0, func=0):
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_CONFIGURE_ACTUATOR,
            motor, #do nothing config
            0,
            0,
            0,
            func,
            0, 0
        )
    @staticmethod
    def up_or_down_x_and_y(a):
        #control values for pitch and roll
        return -500 if a < 0 else 500

    @staticmethod
    def up_or_down_r(a):
        # control values for yaw
        return -500 if a < 0 else 500



    def go_full_straight(self, see_motor_output=0):
        self.go_to_vehicle_raw(2000, 0, 0,0,0,0,0,see_motor_output)
    def go_full_back(self, see_motor_output=0):
        self.go_to_vehicle_raw(-2000,0, 0,0,0,0,0,see_motor_output)
    def go_full_right(self, see_motor_output=0):
        self.go_to_vehicle_raw(0,2000, 0,0,0,0,0,see_motor_output)
    def go_full_left(self, see_motor_output=0):
        self.go_to_vehicle_raw(0,-2000, 0,0,0,0,0,see_motor_output)
    def go_full_up(self, see_motor_output=0):
        self.go_to_vehicle_raw(0,0,1500,0,0,0,0,see_motor_output)
    def go_full_down(self, see_motor_output=0):
        self.go_to_vehicle_raw(0,0,-1500,0,0,0,0,see_motor_output)

    def go_to_vehicle(self, x=0, y=0, z=0, r=0, buttons=0, s=0, t=0, see_motor_output=0):
        x_mapped = map_value_to_range(x)
        y_mapped = map_value_to_range(y)
        z_mapped = map_value_to_range(z,-1, 1, -500, 1500)
        r_mapped = map_value_to_range(r)
        s_mapped = map_value_to_range(s,-1, 1, -1000, 1000)
        t_mapped = map_value_to_range(t,-1, 1, -1000, 1000)
        self.go_to_vehicle_raw(x_mapped, y_mapped, z_mapped, r_mapped, buttons, s_mapped, t_mapped, see_motor_output)

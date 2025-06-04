# -*- coding: utf-8 -*-
"""
unitree_sdk2py_bridge.py

Modified for Windows compatibility by replacing RecurrentThread
with standard threading.Thread and manual sleep logic.
"""
import mujoco
import numpy as np
import pygame
import sys
import struct
import threading # Added for standard threads
import time      # Added for sleep and perf_counter

from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelPublisher
from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import WirelessController_
from unitree_sdk2py.idl.default import unitree_go_msg_dds__SportModeState_
from unitree_sdk2py.idl.default import unitree_go_msg_dds__WirelessController_
# from unitree_sdk2py.utils.thread import RecurrentThread # Removed dependency
import config

if config.ROBOT=="g1":
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
    from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowState_ as LowState_default
else: # Assuming Go2 or similar if not G1
    from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowCmd_
    from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowState_
    from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowState_ as LowState_default

TOPIC_LOWCMD = "rt/lowcmd"
TOPIC_LOWSTATE = "rt/lowstate"
TOPIC_HIGHSTATE = "rt/sportmodestate"
TOPIC_WIRELESS_CONTROLLER = "rt/wirelesscontroller"

MOTOR_SENSOR_NUM = 3
NUM_MOTOR_IDL_GO = 20 # Used to determine IDL type, maybe adjust based on actual robot model?
# NUM_MOTOR_IDL_HG = 35 # Not directly used here after idl_type logic removed for simplicity

class UnitreeSdk2Bridge:

    def __init__(self, mj_model, mj_data):
        self.mj_model = mj_model
        self.mj_data = mj_data
        self.num_motor = self.mj_model.nu
        self.dim_motor_sensor = MOTOR_SENSOR_NUM * self.num_motor

        self.have_imu = False
        self.have_frame_sensor = False
        self.dt = self.mj_model.opt.timestep # Simulation timestep used for some intervals
        # self.idl_type = (self.num_motor > NUM_MOTOR_IDL_GO) # Simplified: Logic depends on imports now

        self.joystick = None
        self._running = True # Flag to control background threads

        # Check sensors
        for i in range(self.mj_model.nsensor): # Check all sensors
            name = mujoco.mj_id2name(
                self.mj_model, mujoco._enums.mjtObj.mjOBJ_SENSOR, i
            )
            # Correct sensor check based on provided example
            # Assuming sensor layout: motor_pos (nu), motor_vel (nu), motor_tau (nu), imu (10), frame (6)
            if name == "imu_quat": # Check if an IMU sensor exists
                 # Check dimensions to be safe
                 if self.mj_model.sensor_dim[i] >= 4: # Quaternion needs 4
                    self.have_imu = True # Corrected flag name
            elif name == "frame_pos": # Check if frame sensors exist
                 if self.mj_model.sensor_dim[i] >= 3: # Position needs 3
                    self.have_frame_sensor = True # Corrected flag name


        # --- Unitree sdk2 message publishers ---
        self.low_state = LowState_default()
        self.low_state_puber = ChannelPublisher(TOPIC_LOWSTATE, LowState_)
        self.low_state_puber.Init()
        # Replace RecurrentThread with standard Thread
        self.lowStateThread = threading.Thread(
            target=self._loop_publish_low_state, name="sim_lowstate", daemon=True
        )
        self.lowStateThread.start()


        self.high_state = unitree_go_msg_dds__SportModeState_()
        self.high_state_puber = ChannelPublisher(TOPIC_HIGHSTATE, SportModeState_)
        self.high_state_puber.Init()
        # Replace RecurrentThread with standard Thread
        self.HighStateThread = threading.Thread(
             target=self._loop_publish_high_state, name="sim_highstate", daemon=True
        )
        self.HighStateThread.start()


        self.wireless_controller = unitree_go_msg_dds__WirelessController_()
        self.wireless_controller_puber = ChannelPublisher(
            TOPIC_WIRELESS_CONTROLLER, WirelessController_
        )
        self.wireless_controller_puber.Init()
        # Replace RecurrentThread with standard Thread (Note: different interval 0.01)
        self.WirelessControllerThread = threading.Thread(
            target=self._loop_publish_wireless_controller,
            name="sim_wireless_controller",
            daemon=True,
        )
        self.WirelessControllerThread.start()

        # --- Unitree sdk2 message subscriber ---
        self.low_cmd_suber = ChannelSubscriber(TOPIC_LOWCMD, LowCmd_)
        self.low_cmd_suber.Init(self.LowCmdHandler, 10) # Use the handler method

        # --- Joystick setup --- (Keep the key map)
        self.key_map = {
            "R1": 0, "L1": 1, "start": 2, "select": 3, "R2": 4, "L2": 5,
            "F1": 6, "F2": 7, "A": 8, "B": 9, "X": 10, "Y": 11,
            "up": 12, "right": 13, "down": 14, "left": 15,
        }
        # Axis and button IDs will be set in SetupJoystick


    # --- Thread loop functions (replacing RecurrentThread logic) ---
    def _loop_publish_low_state(self):
        """Runs PublishLowState periodically based on self.dt."""
        while self._running:
            start_time = time.perf_counter()
            # --- Critical section (optional lock if needed, but mj_data access seems okay from separate threads in this setup) ---
            # locker.acquire() # If you need strict synchronisation with the main sim thread for mj_data
            try:
                self.PublishLowState()
            finally:
                # locker.release() # Release lock if used
                pass
            # --- End Critical section ---

            elapsed_time = time.perf_counter() - start_time
            sleep_time = self.dt - elapsed_time
            if sleep_time > 0:
                time.sleep(sleep_time)
            # else: print(f"Warning: LowState loop took longer ({elapsed_time:.5f}s) than interval ({self.dt}s)")


    def _loop_publish_high_state(self):
        """Runs PublishHighState periodically based on self.dt."""
        while self._running:
            start_time = time.perf_counter()
            # --- Critical section (optional lock) ---
            try:
                self.PublishHighState()
            finally:
                pass
            # --- End Critical section ---
            elapsed_time = time.perf_counter() - start_time
            sleep_time = self.dt - elapsed_time
            if sleep_time > 0:
                time.sleep(sleep_time)
            # else: print(f"Warning: HighState loop took longer ({elapsed_time:.5f}s) than interval ({self.dt}s)")

    def _loop_publish_wireless_controller(self):
        """Runs PublishWirelessController periodically (interval 0.01s)."""
        interval = 0.01 # Specific interval for controller
        while self._running:
            start_time = time.perf_counter()
            # No mj_data access here, so lock is likely unnecessary
            self.PublishWirelessController()
            elapsed_time = time.perf_counter() - start_time
            sleep_time = interval - elapsed_time
            if sleep_time > 0:
                time.sleep(sleep_time)
            # else: print(f"Warning: WirelessController loop took longer ({elapsed_time:.5f}s) than interval ({interval}s)")


    def stop_threads(self):
        """Signals the background threads to stop."""
        self._running = False
        # Optionally join threads here if needed for clean shutdown
        # self.lowStateThread.join()
        # self.HighStateThread.join()
        # self.WirelessControllerThread.join()

    # --- Original methods ---

    def LowCmdHandler(self, msg: LowCmd_):
        """Handles incoming LowCmd messages to control the robot."""
        if self.mj_data is not None:
            num_motors_in_cmd = len(msg.motor_cmd)
            for i in range(min(self.num_motor, num_motors_in_cmd)): # Avoid index error if cmd has fewer/more motors
                # Simple PD control based on LowCmd
                # Sensor data layout assumed: pos (0..nu-1), vel (nu..2*nu-1)
                pos_error = msg.motor_cmd[i].q - self.mj_data.sensordata[i]
                vel_error = msg.motor_cmd[i].dq - self.mj_data.sensordata[i + self.num_motor]

                # Apply control torque = feedforward + Kp*pos_err + Kd*vel_err
                self.mj_data.ctrl[i] = (
                    msg.motor_cmd[i].tau +
                    msg.motor_cmd[i].kp * pos_error +
                    msg.motor_cmd[i].kd * vel_error
                 )
            # Zero out control for motors not in the command? Or leave as is?
            # for i in range(num_motors_in_cmd, self.num_motor):
            #     self.mj_data.ctrl[i] = 0.0


    def PublishLowState(self):
        """Publishes the current low-level state (motors, IMU, remote)."""
        if self.mj_data is None:
            return

        # Motor States
        # Assumed sensor layout: pos (0..nu-1), vel (nu..2*nu-1), torque (2*nu..3*nu-1)
        num_motors_to_publish = min(self.num_motor, len(self.low_state.motor_state))
        for i in range(num_motors_to_publish):
            self.low_state.motor_state[i].q = self.mj_data.sensordata[i]
            self.low_state.motor_state[i].dq = self.mj_data.sensordata[i + self.num_motor]
            # Check if torque sensors exist (total sensors > 3*nu)
            if self.mj_model.nsensor > i + 2 * self.num_motor:
                 self.low_state.motor_state[i].tau_est = self.mj_data.sensordata[i + 2 * self.num_motor]
            else:
                 self.low_state.motor_state[i].tau_est = 0.0 # Or some default

        # IMU State
        # Assumed sensor layout after motors: quat (4), gyro (3), accel (3) = 10 sensors
        imu_sensor_start_index = self.dim_motor_sensor
        if self.have_imu and self.mj_model.nsensor >= imu_sensor_start_index + 10:
             # Quaternion (w, x, y, z) - MuJoCo order might be w,x,y,z or x,y,z,w - CHECK DOCUMENTATION
             # Assuming MuJoCo sensor order is w, x, y, z matching DDS
             self.low_state.imu_state.quaternion[0] = self.mj_data.sensordata[imu_sensor_start_index + 0] # w
             self.low_state.imu_state.quaternion[1] = self.mj_data.sensordata[imu_sensor_start_index + 1] # x
             self.low_state.imu_state.quaternion[2] = self.mj_data.sensordata[imu_sensor_start_index + 2] # y
             self.low_state.imu_state.quaternion[3] = self.mj_data.sensordata[imu_sensor_start_index + 3] # z
             # Gyroscope
             self.low_state.imu_state.gyroscope[0] = self.mj_data.sensordata[imu_sensor_start_index + 4]
             self.low_state.imu_state.gyroscope[1] = self.mj_data.sensordata[imu_sensor_start_index + 5]
             self.low_state.imu_state.gyroscope[2] = self.mj_data.sensordata[imu_sensor_start_index + 6]
             # Accelerometer
             self.low_state.imu_state.accelerometer[0] = self.mj_data.sensordata[imu_sensor_start_index + 7]
             self.low_state.imu_state.accelerometer[1] = self.mj_data.sensordata[imu_sensor_start_index + 8]
             self.low_state.imu_state.accelerometer[2] = self.mj_data.sensordata[imu_sensor_start_index + 9]

        # Wireless Remote (Joystick) - Packing logic seems complex, verify it matches expectations
        if self.joystick is not None and hasattr(self, 'axis_id'): # Check if joystick initialized
            pygame.event.get() # Keep pygame responsive

            # Buttons (packed into bytes) - Be careful with bit order and mapping
            # Example assumes specific bit layout - VERIFY THIS
            byte2 = 0
            byte2 |= (int(self.joystick.get_axis(self.axis_id["LT"]) > 0) << 2) # L2/LT
            byte2 |= (int(self.joystick.get_axis(self.axis_id["RT"]) > 0) << 3) # R2/RT
            byte2 |= (int(self.joystick.get_button(self.button_id["SELECT"])) << 4)
            byte2 |= (int(self.joystick.get_button(self.button_id["START"])) << 5)
            byte2 |= (int(self.joystick.get_button(self.button_id["LB"])) << 6) # L1/LB
            byte2 |= (int(self.joystick.get_button(self.button_id["RB"])) << 7) # R1/RB
            self.low_state.wireless_remote[2] = byte2

            byte3 = 0
            hat_x, hat_y = self.joystick.get_hat(0)
            byte3 |= (int(hat_x < 0) << 0) # left
            byte3 |= (int(hat_y < 0) << 1) # down (pygame y-axis is inverted)
            byte3 |= (int(hat_x > 0) << 2) # right
            byte3 |= (int(hat_y > 0) << 3) # up   (pygame y-axis is inverted)
            byte3 |= (int(self.joystick.get_button(self.button_id["Y"])) << 4)
            byte3 |= (int(self.joystick.get_button(self.button_id["X"])) << 5)
            byte3 |= (int(self.joystick.get_button(self.button_id["B"])) << 6)
            byte3 |= (int(self.joystick.get_button(self.button_id["A"])) << 7)
            self.low_state.wireless_remote[3] = byte3

            # Axes (packed as floats) - Check byte order (struct uses native by default)
            # Mapping LX, RX, RY, LY to specific byte offsets - VERIFY THIS
            sticks = [
                self.joystick.get_axis(self.axis_id["LX"]),  # Left X
                self.joystick.get_axis(self.axis_id["RX"]),  # Right X
               -self.joystick.get_axis(self.axis_id["RY"]), # Right Y (inverted?)
               -self.joystick.get_axis(self.axis_id["LY"]), # Left Y (inverted?)
            ]
            # Pack floats into byte array segments
            # Offsets: 4-7=LX, 8-11=RX, 12-15=RY, 20-23=LY (Indices seem specific, verify)
            try:
                 self.low_state.wireless_remote[4:8] = struct.pack("f", sticks[0])
                 self.low_state.wireless_remote[8:12] = struct.pack("f", sticks[1])
                 self.low_state.wireless_remote[12:16] = struct.pack("f", sticks[2])
                 # Indices 16-19 are skipped in the original code?
                 self.low_state.wireless_remote[20:24] = struct.pack("f", sticks[3])
            except struct.error as e:
                 print(f"Error packing joystick axes: {e}")
                 # Handle error, maybe zero out axes
                 for i in range(4, 24): self.low_state.wireless_remote[i] = 0

        # Publish the state
        self.low_state_puber.Write(self.low_state)


    def PublishHighState(self):
        """Publishes the current high-level state (position, velocity)."""
        if self.mj_data is None:
             return

        # Assumed sensor layout after IMU: frame_pos (3), frame_vel (3) = 6 sensors
        frame_sensor_start_index = self.dim_motor_sensor + 10 # After motors (3*nu) and IMU (10)

        if self.have_frame_sensor and self.mj_model.nsensor >= frame_sensor_start_index + 6:
            # Body Position (World Frame)
            self.high_state.position[0] = self.mj_data.sensordata[frame_sensor_start_index + 0]
            self.high_state.position[1] = self.mj_data.sensordata[frame_sensor_start_index + 1]
            self.high_state.position[2] = self.mj_data.sensordata[frame_sensor_start_index + 2]
            # Body Velocity (World Frame)
            self.high_state.velocity[0] = self.mj_data.sensordata[frame_sensor_start_index + 3]
            self.high_state.velocity[1] = self.mj_data.sensordata[frame_sensor_start_index + 4]
            self.high_state.velocity[2] = self.mj_data.sensordata[frame_sensor_start_index + 5]

            # Publish the state
            self.high_state_puber.Write(self.high_state)
        # else:
            # Optionally clear or log if sensors are missing


    def PublishWirelessController(self):
        """Publishes the controller state in WirelessController_ format."""
        if self.joystick is not None and hasattr(self, 'axis_id'): # Check if joystick initialized
            pygame.event.get() # Keep pygame responsive

            key_state = [0] * 16 # Map keys according to self.key_map
            # --- Map buttons ---
            key_state[self.key_map["R1"]] = int(self.joystick.get_button(self.button_id["RB"]))
            key_state[self.key_map["L1"]] = int(self.joystick.get_button(self.button_id["LB"]))
            key_state[self.key_map["start"]] = int(self.joystick.get_button(self.button_id["START"]))
            key_state[self.key_map["select"]] = int(self.joystick.get_button(self.button_id["SELECT"]))
            key_state[self.key_map["R2"]] = int(self.joystick.get_axis(self.axis_id["RT"]) > 0) # Treat trigger axis as button
            key_state[self.key_map["L2"]] = int(self.joystick.get_axis(self.axis_id["LT"]) > 0) # Treat trigger axis as button
            # F1, F2 seem unmapped in the provided joystick layouts
            key_state[self.key_map["F1"]] = 0
            key_state[self.key_map["F2"]] = 0
            key_state[self.key_map["A"]] = int(self.joystick.get_button(self.button_id["A"]))
            key_state[self.key_map["B"]] = int(self.joystick.get_button(self.button_id["B"]))
            key_state[self.key_map["X"]] = int(self.joystick.get_button(self.button_id["X"]))
            key_state[self.key_map["Y"]] = int(self.joystick.get_button(self.button_id["Y"]))
            # --- Map D-Pad/Hat ---
            hat_x, hat_y = self.joystick.get_hat(0)
            key_state[self.key_map["up"]] = int(hat_y > 0)  # Pygame hat Y is inverted
            key_state[self.key_map["right"]] = int(hat_x > 0)
            key_state[self.key_map["down"]] = int(hat_y < 0) # Pygame hat Y is inverted
            key_state[self.key_map["left"]] = int(hat_x < 0)

            # Combine into a single integer
            key_value = 0
            for i in range(16):
                key_value |= (key_state[i] << i)
            self.wireless_controller.keys = key_value

            # --- Map Axes --- (Note the inversions match PublishLowState)
            self.wireless_controller.lx = self.joystick.get_axis(self.axis_id["LX"])
            self.wireless_controller.ly = -self.joystick.get_axis(self.axis_id["LY"]) # Inverted
            self.wireless_controller.rx = self.joystick.get_axis(self.axis_id["RX"])
            self.wireless_controller.ry = -self.joystick.get_axis(self.axis_id["RY"]) # Inverted

            # Publish the controller state
            self.wireless_controller_puber.Write(self.wireless_controller)


    def SetupJoystick(self, device_id=0, js_type="xbox"):
        """Initializes pygame and the specified joystick."""
        try:
            pygame.init()
            pygame.joystick.init()
            joystick_count = pygame.joystick.get_count()

            if joystick_count > device_id:
                self.joystick = pygame.joystick.Joystick(device_id)
                self.joystick.init()
                print(f"Initialized Joystick {device_id}: {self.joystick.get_name()}")
            else:
                print(f"Error: Joystick device ID {device_id} not found. Found {joystick_count} joysticks.")
                print("Please ensure the joystick is connected and the device_id is correct.")
                # Decide how to handle this - exit or continue without joystick?
                # sys.exit("Joystick not found.") # Or set self.joystick = None
                self.joystick = None
                return # Exit setup if joystick not found

            # Define axis and button mappings based on type
            if js_type.lower() == "xbox":
                # Adjust based on how your specific Xbox controller is detected by pygame on Windows
                # These indices might vary! Use a joystick testing tool if unsure.
                self.axis_id = {
                    "LX": 0, "LY": 1, "RX": 3, "RY": 4, "LT": 2, "RT": 5,
                    # DPad might be axes or hat depending on driver/controller
                    # "DX": 6, "DY": 7 # Often Hat is preferred: get_hat(0)
                }
                self.button_id = {
                    "A": 0, "B": 1, "X": 2, "Y": 3, "LB": 4, "RB": 5,
                    "SELECT": 6, "START": 7, # Adjust SELECT/START (Back/Menu) if needed
                    # Add mappings for stick presses if needed: LSB: 8, RSB: 9 etc.
                }
            elif js_type.lower() == "switch":
                 # Indices for Switch Pro Controller might also vary
                 self.axis_id = {
                    "LX": 0, "LY": 1, "RX": 2, "RY": 3, "LT": 5, "RT": 4, # Note RT/LT indices
                    # "DX": ?, "DY": ? # Usually Hat: get_hat(0)
                 }
                 self.button_id = {
                    "B": 0, "A": 1, "Y": 2, "X": 3, # Note A/B/X/Y order
                    "LB": 4, "RB": 5, "LT": 6, "RT": 7, # Triggers as buttons?
                    "SELECT": 8, "START": 9, # Minus/Plus buttons
                    # Add stick presses, Home, Capture if needed
                 }
            else:
                print(f"Warning: Unsupported joystick type '{js_type}'. Using default XBOX mapping (may be incorrect).")
                # Default to XBOX or provide a generic mapping
                self.axis_id = { "LX": 0, "LY": 1, "RX": 3, "RY": 4, "LT": 2, "RT": 5 }
                self.button_id = { "A": 0, "B": 1, "X": 2, "Y": 3, "LB": 4, "RB": 5, "SELECT": 6, "START": 7 }

            print(f"Using '{js_type}' mapping.")
            print("Axis mapping:", self.axis_id)
            print("Button mapping:", self.button_id)

        except pygame.error as e:
            print(f"Pygame error during joystick setup: {e}")
            self.joystick = None
            # sys.exit("Failed to initialize joystick.")

    def PrintSceneInformation(self):
        """Prints information about the MuJoCo model's bodies, joints, actuators, and sensors."""
        print("\n<<------------- MuJoCo Scene Information ------------->>")

        print("\n<<------------- Bodies ------------->>")
        for i in range(self.mj_model.nbody):
            name = mujoco.mj_id2name(self.mj_model, mujoco._enums.mjtObj.mjOBJ_BODY, i)
            if name: # Skip unnamed bodies (like 'world') if desired
                print(f"Body Index: {i}, Name: {name}")

        print("\n<<------------- Joints ------------->>")
        for i in range(self.mj_model.njnt):
            name = mujoco.mj_id2name(self.mj_model, mujoco._enums.mjtObj.mjOBJ_JOINT, i)
            jnt_type = mujoco.mjtJoint(self.mj_model.jnt_type[i])
            body_id = self.mj_model.jnt_bodyid[i]
            body_name = mujoco.mj_id2name(self.mj_model, mujoco._enums.mjtObj.mjOBJ_BODY, body_id)
            if name:
                print(f"Joint Index: {i}, Name: {name}, Type: {jnt_type.name}, Body: {body_name} ({body_id})")

        print("\n<<------------- Actuators (Controls) ------------->>")
        for i in range(self.mj_model.nu):
            name = mujoco.mj_id2name(self.mj_model, mujoco._enums.mjtObj.mjOBJ_ACTUATOR, i)
            trntype = mujoco.mjtTrn(self.mj_model.actuator_trntype[i])
            trnid = self.mj_model.actuator_trnid[i, 0]
            target_name = ""
            if trntype == mujoco.mjtTrn.mjTRN_JOINT:
                 target_name = mujoco.mj_id2name(self.mj_model, mujoco._enums.mjtObj.mjOBJ_JOINT, trnid)
            elif trntype == mujoco.mjtTrn.mjTRN_TENDON:
                 target_name = mujoco.mj_id2name(self.mj_model, mujoco._enums.mjtObj.mjOBJ_TENDON, trnid)
            # Add other types if needed
            if name:
                print(f"Actuator Index: {i}, Name: {name}, Target: {target_name} ({trntype.name})")

        print("\n<<------------- Sensors ------------->>")
        adr_counter = 0 # Keep track of sensor data address
        for i in range(self.mj_model.nsensor):
            name = mujoco.mj_id2name(self.mj_model, mujoco._enums.mjtObj.mjOBJ_SENSOR, i)
            sensor_type = mujoco.mjtSensor(self.mj_model.sensor_type[i])
            dim = self.mj_model.sensor_dim[i]
            adr = self.mj_model.sensor_adr[i]
            # Verify address calculation if needed: assert adr == adr_counter
            if name:
                print(f"Sensor Index: {i}, Name: {name}, Type: {sensor_type.name}, Dim: {dim}, Start Address: {adr}")
            adr_counter += dim
        print(f"Total Sensor Dimensions: {self.mj_model.nsensordata}")
        print("-----------------------------------------------------\n")


# --- ElasticBand class (appears unchanged from original) ---
class ElasticBand:
    def __init__(self):
        self.stiffness = 200.0 # Use floats
        self.damping = 100.0   # Use floats
        self.point = np.array([0.0, 0.0, 3.0]) # Anchor point in world
        self.length = 0.0      # Resting length
        self.enable = True

    def Advance(self, x, dx):
        """
        Calculates the elastic band force.
        Args:
            x: current position of the attached body part (e.g., torso base link position)
            dx: current velocity of the attached body part
        Returns:
            3D force vector to apply
        """
        if not self.enable:
            return np.zeros(3)

        delta_x = self.point - np.asarray(x[:3]) # Ensure x is array-like and take first 3 elements
        distance = np.linalg.norm(delta_x)

        if distance < 1e-6: # Avoid division by zero if distance is very small
            return np.zeros(3)

        direction = delta_x / distance
        stretch = distance - self.length

        # Velocity component along the band direction
        velocity_along_band = np.dot(np.asarray(dx[:3]), direction)

        # Force = spring force (Hooke's law) + damping force
        # Only apply spring force if stretched beyond resting length
        spring_force_magnitude = self.stiffness * max(0, stretch)
        damping_force_magnitude = -self.damping * velocity_along_band # Damping opposes velocity

        total_force_magnitude = spring_force_magnitude + damping_force_magnitude
        force_vector = total_force_magnitude * direction

        return force_vector

    def MujuocoKeyCallback(self, key):
        """Handles key presses for modifying the elastic band in the viewer."""
        # Check if glfw is available (it should be if viewer is running)
        try:
             import glfw
        except ImportError:
             print("GLFW not found, key callbacks for ElasticBand disabled.")
             return

        # Use key constants directly from glfw module
        if key == glfw.KEY_7:
            self.length = max(0.0, self.length - 0.1) # Prevent negative length
            print(f"ElasticBand: Length decreased to {self.length:.2f}")
        elif key == glfw.KEY_8:
            self.length += 0.1
            print(f"ElasticBand: Length increased to {self.length:.2f}")
        elif key == glfw.KEY_9:
            self.enable = not self.enable
            status = "enabled" if self.enable else "disabled"
            print(f"ElasticBand: {status}")

# --- Example Usage (Illustrative - usually called from main script) ---
if __name__ == '__main__':
    # This block likely won't run directly when imported,
    # but shows how the class might be used.
    print("Unitree SDK2 Bridge module loaded.")
    print("This script should be imported, not run directly for simulation.")
    # Example: Create dummy model/data for basic testing
    # model = mujoco.MjModel.from_xml_string("<mujoco><worldbody/></mujoco>")
    # data = mujoco.MjData(model)
    # bridge = UnitreeSdk2Bridge(model, data)
    # bridge.PrintSceneInformation() # Test printing
    # print("Bridge initialized (dummy).")
    # time.sleep(2) # Keep alive briefly
    # bridge.stop_threads()
    # print("Bridge threads stopped.")
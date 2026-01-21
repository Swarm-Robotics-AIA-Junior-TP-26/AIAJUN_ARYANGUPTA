import time
import math
from pymavlink import mavutil

# --- Configuration ---
CONNECTION_STRING = 'udpin:localhost:14540' # Standard PX4 SITL offboard port
TARGET_ALTITUDE = 5.0                       # Target altitude for takeoff (meters)
CRUISE_SPEED = 3.0                          # Navigation speed (m/s)
WAYPOINT_THRESHOLD = 1.0                    # Distance threshold to switch waypoints (meters)

# Define a list of Waypoints [North (x), East (y), Down (z)]
# Note: Down is negative for altitude (e.g., -5.0 is 5m up)
WAYPOINTS = [
    [10, 0, -5],
    [10, 10, -10],
    [-10, 10, -5],
    [0, 0, 0]
]

class PX4Controller:
    def __init__(self):
        print(f"Connecting to {CONNECTION_STRING}...")
        self.master = mavutil.mavlink_connection(CONNECTION_STRING)
        self.master.wait_heartbeat()
        print(f"Connected to System {self.master.target_system}, Component {self.master.target_component}")
        
        # State variables
        self.current_pos = [0, 0, 0] # [x, y, z]
        self.boot_time = time.time()

    def get_position(self):
        """ Listens for LOCAL_POSITION_NED messages to update current state """
        msg = self.master.recv_match(type='LOCAL_POSITION_NED', blocking=False)
        if msg:
            self.current_pos = [msg.x, msg.y, msg.z]
        return self.current_pos

    def send_velocity_setpoint(self, vx, vy, vz):
        """
        Sends a velocity setpoint (SET_POSITION_TARGET_LOCAL_NED).
        
        Mask logic (1 means IGNORE):
        - Bit 0,1,2: Ignore Position (x, y, z)
        - Bit 3,4,5: Enable Velocity (vx, vy, vz) -> set to 0
        - Bit 6,7,8: Ignore Acceleration
        - Bit 9: Ignore Yaw
        - Bit 10: Ignore Yaw Rate
        
        Mask = 0b0000110111000111 = 3527 (decimal)
        """
        type_mask = 0b0000110111000111
        
        self.master.mav.set_position_target_local_ned_send(
            0,                          # time_boot_ms (ignored)
            self.master.target_system,  # target system
            self.master.target_component, # target component
            mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
            type_mask,                  # type_mask
            0, 0, 0,                    # x, y, z positions (ignored)
            vx, vy, vz,                 # vx, vy, vz velocity (m/s)
            0, 0, 0,                    # afx, afy, afz acceleration (ignored)
            0, 0                        # yaw, yaw_rate (ignored)
        )

    def arm(self):
        print("Arming drone...")
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1, 0, 0, 0, 0, 0, 0 # Param1=1 (Arm)
        )
        # Wait for acknowledgment
        self.master.motors_armed_wait()
        print("Armed!")

    def set_offboard_mode(self):
        print("Switching to OFFBOARD mode...")
        # PX4 requires a stream of setpoints BEFORE switching mode
        # We send a few seconds of zero-velocity commands
        for _ in range(10):
            self.send_velocity_setpoint(0, 0, 0)
            time.sleep(0.1)

        # Send mode switch command
        # MAV_MODE_FLAG_CUSTOM_MODE_ENABLED = 1
        # PX4_CUSTOM_MAIN_MODE_OFFBOARD = 6
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            0,
            1, 6, 0, 0, 0, 0, 0 # Param1=BaseMode, Param2=CustomMainMode
        )
        print("OFFBOARD mode command sent.")

    def run(self):
        try:
            # 1. Arm and Set Mode
            self.set_offboard_mode()
            self.arm()

            # 2. Takeoff (Velocity Control)
            print(f"Taking off to {TARGET_ALTITUDE}m...")
            while self.current_pos[2] > -TARGET_ALTITUDE + 0.5:
                self.get_position()
                # Send negative Z velocity to go UP (NED system)
                self.send_velocity_setpoint(0, 0, -1.0) 
                time.sleep(0.1)
            
            # Hover briefly
            print("Takeoff Complete. Hovering...")
            for _ in range(20):
                self.send_velocity_setpoint(0, 0, 0)
                time.sleep(0.1)

            # 3. Waypoint Navigation
            for i, wp in enumerate(WAYPOINTS):
                print(f"Navigating to Waypoint {i+1}: {wp}")
                
                while True:
                    curr = self.get_position()
                    
                    # Calculate vector to target
                    dx = wp[0] - curr[0]
                    dy = wp[1] - curr[1]
                    dz = wp[2] - curr[2]
                    
                    distance = math.sqrt(dx**2 + dy**2 + dz**2)
                    
                    if distance < WAYPOINT_THRESHOLD:
                        print(f"Reached Waypoint {i+1}")
                        break
                    
                    # Normalize vector and scale by speed
                    vx = (dx / distance) * CRUISE_SPEED
                    vy = (dy / distance) * CRUISE_SPEED
                    vz = (dz / distance) * CRUISE_SPEED
                    
                    self.send_velocity_setpoint(vx, vy, vz)
                    
                    # Offboard requires >2Hz updates. We sleep 0.1s (10Hz)
                    time.sleep(0.1)

            # 4. Landing
            print("Waypoints complete. Landing...")
            while self.current_pos[2] < -0.5: # While above ground
                self.get_position()
                # Positive Z velocity to go DOWN
                self.send_velocity_setpoint(0, 0, 1.0) 
                time.sleep(0.1)

            print("Landed. Disarming...")
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0, 0, 0, 0, 0, 0, 0, 0 # Param1=0 (Disarm)
            )

        except KeyboardInterrupt:
            print("\nEmergency Stop! Sending zero velocity and landing...")
            for _ in range(20):
                self.send_velocity_setpoint(0, 0, 0)
                time.sleep(0.1)
            # Failsafe land mode
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_NAV_LAND,
                0, 0, 0, 0, 0, 0, 0, 0
            )

if __name__ == "__main__":
    controller = PX4Controller()
    controller.run()
import time
import sys
import threading
import queue
import tkinter as tk
from tkinter import simpledialog, scrolledtext
from pymavlink import mavutil
from MissionPlannerScriptV2.src.interfaces.mavlink_commands import MAVLinkCommandInterface  # Importa la clase separada

# ------------------------------------------------------------------------------
# QueueLogger: Redirects stdout messages to a queue for GUI display.
# ------------------------------------------------------------------------------
class QueueLogger:
    def __init__(self, log_queue):
        self.log_queue = log_queue

    def write(self, msg):
        if msg.strip():
            self.log_queue.put(msg)

    def flush(self):
        pass


# ------------------------------------------------------------------------------
# DroneController:
#   Composite methods for basic operations (e.g., monitoring messages).
# ------------------------------------------------------------------------------
class DroneController:
    def __init__(self, connection_string='udp:127.0.0.1:14551'):
        print("Connecting to drone...")
        self.drone = mavutil.mavlink_connection(connection_string)
        self.drone.wait_heartbeat()
        print(f"Connected to drone! System ID: {self.drone.target_system}, Component ID: {self.drone.target_component}")
        self.target_system = self.drone.target_system
        self.target_component = self.drone.target_component
        self.home_lat = None
        self.home_lon = None
        self.home_alt = None
        self.running = True
        self.message_thread = threading.Thread(target=self.monitor_messages)
        self.message_thread.daemon = True
        self.message_thread.start()
        print(f"Current flight mode: {self.drone.flightmode}")

    def monitor_messages(self):
        print_details = False
        print_all = False
        while self.running:
            msg = self.drone.recv_match(blocking=True)
            if msg:
                if print_all:
                    print(f"RECEIVED MAVLink Message: {msg.get_type()}")
                if msg.get_type() == 'COMMAND_ACK':
                    result = msg.result
                    command = msg.command
                    cmd_desc = self.get_command_description(command)
                    res_desc = self.get_result_description(result)
                    print(f"Command ACK received: {cmd_desc} - {res_desc}")
                if msg.get_type() == 'STATUSTEXT':
                    print(f"STATUS TEXT: {msg.text}")

    def get_command_description(self, command):
        mapping = {
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM: "Arm/Disarm",
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF: "Takeoff",
            mavutil.mavlink.MAV_CMD_NAV_LAND: "Land",
            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT: "Waypoint (Go to Position)",
            mavutil.mavlink.MAV_CMD_DO_SET_MODE: "Set Flight Mode",
            mavutil.mavlink.MAV_CMD_RUN_PREARM_CHECKS: "Run Prearm Checks",
        }
        return mapping.get(command, f"Unknown Command ({command})")

    def get_result_description(self, result):
        mapping = {
            mavutil.mavlink.MAV_RESULT_ACCEPTED: "ACCEPTED",
            mavutil.mavlink.MAV_RESULT_TEMPORARILY_REJECTED: "TEMPORARILY REJECTED",
            mavutil.mavlink.MAV_RESULT_DENIED: "DENIED",
            mavutil.mavlink.MAV_RESULT_UNSUPPORTED: "UNSUPPORTED",
            mavutil.mavlink.MAV_RESULT_FAILED: "FAILED",
            mavutil.mavlink.MAV_RESULT_IN_PROGRESS: "IN PROGRESS",
        }
        return mapping.get(result, f"Unknown Result ({result})")

    def composite_set_mode(self, mode):
        logs = []
        success, log1 = MAVLinkCommandInterface.set_mode_via_set_mode_msg(self.drone, mode)
        logs.append(log1)
        if success:
            return True, "\n".join(logs)
        success, log2 = MAVLinkCommandInterface.set_mode_via_cmd_long(self.drone, mode)
        logs.append(log2)
        if success:
            return True, "\n".join(logs)
        return False, "\n".join(logs)

    def goto_position(self, lat, lon, alt=10, timeout=600):
        logs = []
        if self.drone.flightmode != "GUIDED":
            logs.append("Current mode is not GUIDED; setting mode to GUIDED.")
            success, mode_log = self.composite_set_mode("GUIDED")
            logs.append(mode_log)
            if not success:
                logs.append("Failed to set GUIDED mode. Aborting goto operation.")
                return False, "\n".join(logs)
        success, wp_log = MAVLinkCommandInterface.send_waypoint(self.drone, lat, lon, alt)
        logs.append(wp_log)
        if not success:
            logs.append("Failed to send waypoint command.")
            return False, "\n".join(logs)
        logs.append("Polling for arrival at target position...")
        start_time = time.time()
        while time.time() - start_time < timeout:
            msg = self.drone.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1)
            if msg:
                current_lat = msg.lat / 1e7
                current_lon = msg.lon / 1e7
                current_alt = msg.relative_alt / 1000.0
                distance = MAVLinkCommandInterface.get_distance(current_lat, current_lon, float(lat), float(lon))
                alt_diff = abs(current_alt - float(alt))
                logs.append(f"Current position: Lat={current_lat}, Lon={current_lon}, Alt={current_alt}")
                logs.append(f"Distance: {distance:.2f} m, Alt diff: {alt_diff:.2f} m")
                if distance < 5 and alt_diff < 1:
                    logs.append("Target position reached!")
                    return True, "\n".join(logs)
            time.sleep(1)
        logs.append("Timeout waiting to reach position")
        return False, "\n".join(logs)

    def store_home_position(self):
        logs = []
        pos = MAVLinkCommandInterface.get_current_position(self.drone)
        if pos != (0, 0, 0):
            self.home_lat, self.home_lon, self.home_alt = pos
            logs.append(f"Home position stored: Lat: {self.home_lat}, Lon: {self.home_lon}, Alt: {self.home_alt}")
            return True, "\n".join(logs)
        else:
            logs.append("Failed to obtain current position.")
            return False, "\n".join(logs)

    def return_to_home(self):
        if self.home_lat is None or self.home_lon is None:
            print("Home position not set!")
            return False, "Home position not set!"
        print("Returning to home...")
        return self.goto_position(self.home_lat, self.home_lon, self.home_alt)

# ------------------------------------------------------------------------------
# DroneControllerApp: Tkinter GUI application for drone control.
# ------------------------------------------------------------------------------
class DroneControllerApp(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Drone Control")
        self.geometry("800x600")
        self.log_queue = queue.Queue()
        sys.stdout = QueueLogger(self.log_queue)
        self.drone_controller = DroneController()
        self.create_widgets()
        self.after(100, self.poll_log_queue)

    def create_widgets(self):
        # Pre-Flight / Mode Buttons
        preflight_frame = tk.Frame(self)
        preflight_frame.pack(side=tk.TOP, fill=tk.X)
        preflight_buttons = {
            "Set Guided Mode": lambda: self.run_drone_command(lambda: self.drone_controller.composite_set_mode("GUIDED")),
            "Set Stabilize Mode": lambda: self.run_drone_command(lambda: self.drone_controller.composite_set_mode("STABILIZE")),
            "Set Auto Mode": lambda: self.run_drone_command(lambda: self.drone_controller.composite_set_mode("AUTO")),
            "Check Prearm": lambda: self.run_drone_command(lambda: MAVLinkCommandInterface.send_check_prearm_status(self.drone_controller.drone)),
            "Disable Checks": lambda: self.run_drone_command(lambda: MAVLinkCommandInterface.send_disable_arming_checks(self.drone_controller.drone)),

        }
        for label, command in preflight_buttons.items():
            btn = tk.Button(preflight_frame, text=label, command=command)
            btn.pack(side=tk.LEFT, padx=5, pady=5)

        # Mission Execution Buttons
        mission_frame = tk.Frame(self)
        mission_frame.pack(side=tk.TOP, fill=tk.X)
        mission_buttons = {
            "Execute Point Mission": self.execute_point_mission_command,
            "Execute Circle Mission": self.execute_circle_mission_command
        }
        for label, command in mission_buttons.items():
            btn = tk.Button(mission_frame, text=label, command=command)
            btn.pack(side=tk.LEFT, padx=5, pady=5)

        # Arming / Flight Control Buttons
        control_frame = tk.Frame(self)
        control_frame.pack(side=tk.TOP, fill=tk.X)
        control_buttons = {
            "Arm": lambda: self.run_drone_command(lambda: MAVLinkCommandInterface.send_arm(self.drone_controller.drone)),
            "Force Arm": lambda: self.run_drone_command(lambda: MAVLinkCommandInterface.send_force_arm(self.drone_controller.drone)),
            "Disarm": lambda: self.run_drone_command(lambda: MAVLinkCommandInterface.send_disarm(self.drone_controller.drone)),
            "Store Home": lambda: self.run_drone_command(self.drone_controller.store_home_position),
            "Takeoff (10m)": lambda: self.run_drone_command(lambda: MAVLinkCommandInterface.send_takeoff(self.drone_controller.drone, 10)),
            "Go to Position": self.goto_command,
            "Return Home": lambda: self.run_drone_command(self.drone_controller.return_to_home),
            "Land": lambda: self.run_drone_command(lambda: MAVLinkCommandInterface.send_land(self.drone_controller.drone))
        }
        for label, command in control_buttons.items():
            btn = tk.Button(control_frame, text=label, command=command)
            btn.pack(side=tk.LEFT, padx=5, pady=5)

        # Log Display
        self.log_text = scrolledtext.ScrolledText(self, state='disabled', height=20)
        self.log_text.pack(side=tk.BOTTOM, fill=tk.BOTH, expand=True)

    def poll_log_queue(self):
        while not self.log_queue.empty():
            msg = self.log_queue.get_nowait()
            self.log_text.configure(state='normal')
            self.log_text.insert(tk.END, msg + "\n")
            self.log_text.configure(state='disabled')
            self.log_text.see(tk.END)
        self.after(100, self.poll_log_queue)

    def run_drone_command(self, command_func):
        def wrapper():
            success, log_text = command_func()
            print(log_text)
        threading.Thread(target=wrapper, daemon=True).start()

    def goto_command(self):
        lat = simpledialog.askfloat("Go to Position", "Enter latitude:", initialvalue=40.5418)
        lon = simpledialog.askfloat("Go to Position", "Enter longitude:", initialvalue=-4.0333)
        alt = simpledialog.askfloat("Go to Position", "Enter altitude (m):", initialvalue=10)
        if lat is not None and lon is not None and alt is not None:
            self.run_drone_command(lambda: self.drone_controller.goto_position(lat, lon, alt))
        else:
            print("Operation cancelled.")



    def execute_point_mission_command(self):
        destination_lat = simpledialog.askfloat("Execute Point Mission", "Enter destination latitude:", initialvalue=40.5418)
        destination_lon = simpledialog.askfloat("Execute Point Mission", "Enter destination longitude:", initialvalue=-4.0333)
        altitude = simpledialog.askfloat("Execute Point Mission", "Enter altitude (m):", initialvalue=10)
        wait_time = simpledialog.askinteger("Execute Point Mission", "Enter wait time (s):", initialvalue=30)
        if None not in (destination_lat, destination_lon, altitude, wait_time):
            destination = (destination_lat, destination_lon)
            self.run_drone_command(lambda: MAVLinkCommandInterface.execute_point_mission(self.drone_controller.drone, destination, altitude, wait_time))
        else:
            print("Operation cancelled.")

    def execute_circle_mission_command(self):
        origin_lat = simpledialog.askfloat("Load Circle Mission", "Enter origin latitude:", initialvalue=40.5490)
        origin_lon = simpledialog.askfloat("Load Circle Mission", "Enter origin longitude:", initialvalue=-4.0323)
        destination_lat = simpledialog.askfloat("Execute Circle Mission", "Enter destination latitude (center):", initialvalue=40.5418)
        destination_lon = simpledialog.askfloat("Execute Circle Mission", "Enter destination longitude (center):", initialvalue=-4.0333)
        altitude = simpledialog.askfloat("Execute Circle Mission", "Enter altitude (m):", initialvalue=10)
        radius = simpledialog.askfloat("Execute Circle Mission", "Enter circle radius (m):", initialvalue=30)
        num_points = simpledialog.askinteger("Execute Circle Mission", "Enter number of waypoints:", initialvalue=30)
        if None not in (destination_lat, destination_lon, altitude, radius, num_points):
            self.run_drone_command(lambda: MAVLinkCommandInterface.execute_circle_mission(self.drone_controller.drone, origin_lat, origin_lon,destination_lat,destination_lon, altitude, radius, num_points))
        else:
            print("Operation cancelled.")

if __name__ == "__main__":
    app = DroneControllerApp()
    app.mainloop()

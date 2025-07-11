import time
import math
import traceback
from pymavlink import mavutil

class MAVLinkCommandInterface:
    @staticmethod
    def calculate_bearing(from_lat, from_lon, to_lat, to_lon):
        """
        Calcula el bearing (en grados) desde (from_lat, from_lon) hasta (to_lat, to_lon)
        usando la fórmula del rumbo inicial. Se normaliza a [0,360).
        """
        phi1 = math.radians(from_lat)
        phi2 = math.radians(to_lat)
        delta_lon = math.radians(to_lon - from_lon)
        x = math.sin(delta_lon) * math.cos(phi2)
        y = math.cos(phi1)*math.sin(phi2) - math.sin(phi1)*math.cos(phi2)*math.cos(delta_lon)
        bearing = math.degrees(math.atan2(x, y))
        return (bearing + 360) % 360

    @staticmethod
    def wait_for_mode(drone_connection, mode, timeout=10):
        logs = []
        def log_message(msg):
            logs.append(msg)
            print(msg)
        log_message(f"Waiting for {mode} mode...")
        start_time = time.time()
        while time.time() - start_time < timeout:
            if drone_connection.flightmode == mode:
                log_message(f"Mode: {mode}")
                return True, "\n".join(logs)
            else:
                log_message(f"Current mode: {drone_connection.flightmode}")
            time.sleep(1)
        log_message(f"Timeout waiting for {mode} mode!")
        return False, "\n".join(logs)

    @staticmethod
    def set_mode_via_set_mode_msg(drone_connection, mode, timeout=10):
        logs = []
        def log_message(msg):
            logs.append(msg)
            print(msg)
        start_time = time.time()
        while time.time() - start_time < timeout:
            if mode not in drone_connection.mode_mapping():
                log_message(f"Error: {mode} is not a valid mode.")
                log_message(f"Valid modes: {list(drone_connection.mode_mapping().keys())}")
                return False, "\n".join(logs)
            mode_id = drone_connection.mode_mapping()[mode]
            log_message("SENDING COMMAND: SET_MODE (set_mode_send)")
            log_message(f"  mode: {mode}, mode_id: {mode_id}")
            log_message("-" * 50)
            drone_connection.mav.set_mode_send(
                drone_connection.target_system,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                mode_id
            )
            time.sleep(2)
            if drone_connection.flightmode == mode:
                log_message(f"Mode changed to {mode} successfully!")
                return True, "\n".join(logs)
            log_message("Retrying set_mode using set_mode_send...")
        log_message("Failed to set mode via set_mode_send after 30 seconds")
        return False, "\n".join(logs)

    @staticmethod
    def set_mode_via_cmd_long(drone_connection, mode, timeout=10):
        logs = []
        def log_message(msg):
            logs.append(msg)
            print(msg)
        start_time = time.time()
        while time.time() - start_time < timeout:
            if mode not in drone_connection.mode_mapping():
                log_message(f"Error: {mode} is not a valid mode.")
                log_message(f"Valid modes: {list(drone_connection.mode_mapping().keys())}")
                return False, "\n".join(logs)
            mode_id = drone_connection.mode_mapping()[mode]
            log_message("SENDING COMMAND: SET_MODE (command_long_send)")
            log_message(f"  mode: {mode}, mode_id: {mode_id}")
            log_message("-" * 50)
            drone_connection.mav.command_long_send(
                drone_connection.target_system, drone_connection.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                mode_id, 0, 0, 0, 0, 0
            )
            time.sleep(2)
            if drone_connection.flightmode == mode:
                log_message(f"Mode changed to {mode} successfully!")
                return True, "\n".join(logs)
            log_message("Retrying set_mode using command_long_send...")
        log_message("Failed to set mode via command_long_send after 30 seconds")
        return False, "\n".join(logs)

    @staticmethod
    def send_arm(drone_connection, timeout=10):
        logs = []
        def log_message(msg):
            logs.append(msg)
            print(msg)
        log_message("SENDING COMMAND: ARM")
        log_message("  Action: Arm motors")
        log_message("-" * 50)
        drone_connection.mav.command_long_send(
            drone_connection.target_system, drone_connection.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
            1, 0, 0, 0, 0, 0, 0
        )
        start_time = time.time()
        while time.time() - start_time < timeout:
            if drone_connection.motors_armed():
                log_message("Armed!")
                return True, "\n".join(logs)
            time.sleep(1)
            log_message("Waiting for arming...")
        log_message("Failed to arm after 30 seconds")
        return False, "\n".join(logs)

    @staticmethod
    def send_disarm(drone_connection, timeout=10):
        logs = []
        def log_message(msg):
            logs.append(msg)
            print(msg)
        log_message("SENDING COMMAND: DISARM")
        log_message("  Action: Disarm motors")
        log_message("-" * 50)
        drone_connection.mav.command_long_send(
            drone_connection.target_system, drone_connection.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
            0, 0, 0, 0, 0, 0, 0
        )
        start_time = time.time()
        while time.time() - start_time < timeout:
            if not drone_connection.motors_armed():
                log_message("Disarmed!")
                return True, "\n".join(logs)
            time.sleep(1)
        log_message("Failed to disarm after 30 seconds")
        return False, "\n".join(logs)

    @staticmethod
    def send_takeoff(drone_connection, altitude=10, timeout=10):
        logs = []
        def log_message(msg):
            logs.append(msg)
            print(msg)
        start_time = time.time()
        while time.time() - start_time < timeout:
            log_message(f"SENDING COMMAND: TAKEOFF, altitude: {altitude}")
            log_message("-" * 50)
            drone_connection.mav.command_long_send(
                drone_connection.target_system, drone_connection.target_component,
                mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0,
                0, 0, 0, 0, 0, 0, altitude
            )
            wait_start = time.time()
            while time.time() - wait_start < timeout:
                current_alt = MAVLinkCommandInterface.get_current_altitude(drone_connection)
                if abs(current_alt - altitude) < 1.0:
                    log_message("Reached target altitude.")
                    return True, "\n".join(logs)
                time.sleep(1)
            log_message("Retrying takeoff command...")
        log_message("Failed to reach target altitude after 30 seconds")
        return False, "\n".join(logs)

    @staticmethod
    def send_waypoint(drone_connection, lat, lon, alt=10, timeout=10):
        logs = []
        def log_message(msg):
            logs.append(msg)
            print(msg)
        start_time = time.time()
        while time.time() - start_time < timeout:
            try:
                log_message(f"SENDING COMMAND: WAYPOINT, lat: {lat}, lon: {lon}, alt: {alt}")
                log_message("-" * 50)
                lat_float = float(lat)
                lon_float = float(lon)
                alt_float = float(alt)
                lat_int = int(lat_float * 1e7)
                lon_int = int(lon_float * 1e7)
                drone_connection.mav.mission_item_int_send(
                    drone_connection.target_system,
                    drone_connection.target_component,
                    0,  # sequence (to be managed during mission upload)
                    mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                    mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                    2,  # current
                    0,  # autocontinue
                    0,  # param1: hold time
                    2,  # param2: acceptance radius
                    0,  # param3: pass radius
                    0,  # param4: yaw (se asignará externamente)
                    lat_int,
                    lon_int,
                    alt_float,
                    0   # mission_type
                )
                log_message("Waypoint command sent.")
                return True, "\n".join(logs)
            except Exception as e:
                log_message(f"Error sending waypoint: {e} - Retrying...")
                time.sleep(2)
        log_message("Failed to send waypoint command after 30 seconds")
        return False, "\n".join(logs)

    @staticmethod
    def send_land(drone_connection, timeout=10):
        logs = []
        def log_message(msg):
            logs.append(msg)
            print(msg)
        start_time = time.time()
        while time.time() - start_time < timeout:
            log_message("SENDING COMMAND: LAND")
            log_message("  Action: Land at current position")
            log_message("-" * 50)
            drone_connection.mav.command_long_send(
                drone_connection.target_system, drone_connection.target_component,
                mavutil.mavlink.MAV_CMD_NAV_LAND, 0,
                0, 0, 0, 0, 0, 0, 0
            )
            wait_start = time.time()
            while time.time() - wait_start < timeout:
                current_alt = MAVLinkCommandInterface.get_current_altitude(drone_connection)
                if current_alt < 0.5:
                    log_message("Landed!")
                    return True, "\n".join(logs)
                time.sleep(1)
            log_message("Retrying land command...")
        log_message("Failed to land after 30 seconds")
        return False, "\n".join(logs)

    @staticmethod
    def send_check_prearm_status(drone_connection, timeout=10):
        logs = []
        def log_message(msg):
            logs.append(msg)
            print(msg)
        start_time = time.time()
        while time.time() - start_time < timeout:
            log_message("SENDING COMMAND: CHECK PREARM STATUS")
            log_message("-" * 50)
            drone_connection.mav.command_long_send(
                drone_connection.target_system, drone_connection.target_component,
                mavutil.mavlink.MAV_CMD_RUN_PREARM_CHECKS, 0,
                0, 0, 0, 0, 0, 0, 0
            )
            time.sleep(5)
            log_message("Prearm status check command sent.")
            return True, "\n".join(logs)
        log_message("Failed to send prearm status check command after 30 seconds")
        return False, "\n".join(logs)

    @staticmethod
    def send_disable_arming_checks(drone_connection, timeout=10):
        logs = []
        def log_message(msg):
            logs.append(msg)
            print(msg)
        start_time = time.time()
        while time.time() - start_time < timeout:
            log_message("SENDING COMMAND: DISABLE ARMING CHECKS")
            log_message("-" * 50)
            drone_connection.mav.param_set_send(
                drone_connection.target_system, drone_connection.target_component,
                b'ARMING_CHECK',
                0,
                mavutil.mavlink.MAV_PARAM_TYPE_INT32
            )
            time.sleep(2)
            msg = drone_connection.recv_match(type='PARAM_VALUE', blocking=True, timeout=2)
            if msg and msg.param_id.decode('utf-8') == 'ARMING_CHECK':
                log_message(f"ARMING_CHECK set to {msg.param_value}")
                return True, "\n".join(logs)
            log_message("Retrying disable arming checks...")
        log_message("Failed to disable arming checks after 30 seconds")
        return False, "\n".join(logs)

    @staticmethod
    def send_force_arm(drone_connection, timeout=10):
        logs = []
        def log_message(msg):
            logs.append(msg)
            print(msg)
        start_time = time.time()
        while time.time() - start_time < timeout:
            log_message("SENDING COMMAND: FORCE ARM")
            log_message("  Action: Force arm motors (using magic number 21196)")
            log_message("-" * 50)
            drone_connection.mav.command_long_send(
                drone_connection.target_system, drone_connection.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
                1, 21196, 0, 0, 0, 0, 0
            )
            time.sleep(1)
            if drone_connection.motors_armed():
                log_message("Force armed successfully!")
                return True, "\n".join(logs)
            log_message("Waiting for force arming...")
        log_message("Failed to force arm after 30 seconds")
        return False, "\n".join(logs)

    @staticmethod
    def get_current_altitude(drone_connection):
        msg = drone_connection.messages.get('GLOBAL_POSITION_INT')
        if msg:
            return msg.relative_alt / 1000.0  # Convert mm to m
        return 0

    @staticmethod
    def get_current_position(drone_connection):
        msg = drone_connection.messages.get('GLOBAL_POSITION_INT')
        if msg:
            lat = msg.lat / 1e7
            lon = msg.lon / 1e7
            alt = msg.relative_alt / 1000.0
            return lat, lon, alt
        return 0, 0, 0

    @staticmethod
    def get_distance(lat1, lon1, lat2, lon2):
        lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])
        dlon = lon2 - lon1
        dlat = lat2 - lat1
        a = math.sin(dlat / 2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2)**2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        r = 6371000  # Earth's radius in meters
        return c * r

    @staticmethod
    def calculate_point_at_distance(drone_connection, lat, lon, distance, bearing):
        R = 6371000
        lat_rad = math.radians(lat)
        lon_rad = math.radians(lon)
        lat2_rad = math.asin(
            math.sin(lat_rad) * math.cos(distance / R) +
            math.cos(lat_rad) * math.sin(distance / R) * math.cos(bearing)
        )
        lon2_rad = lon_rad + math.atan2(
            math.sin(bearing) * math.sin(distance / R) * math.cos(lat_rad),
            math.cos(distance / R) - math.sin(lat_rad) * math.sin(lat2_rad)
        )
        return math.degrees(lat2_rad), math.degrees(lon2_rad)

    # --------------------------------------------------------------------------
    # Mission Upload Helper Methods
    # --------------------------------------------------------------------------
    @staticmethod
    def clear_mission(drone_connection, timeout=10):
        logs = []
        def log_message(msg):
            logs.append(msg)
            print(msg)
        log_message("Clearing previous mission...")
        drone_connection.mav.mission_clear_all_send(drone_connection.target_system, drone_connection.target_component)
        ack = drone_connection.recv_match(type='MISSION_ACK', blocking=True, timeout=timeout)
        if not ack:
            log_message("No ACK received for mission clear. Assuming mission cleared.")
            return True, "\n".join(logs)
        if ack.type != mavutil.mavlink.MAV_MISSION_ACCEPTED:
            log_message(f"Mission clear ACK not accepted ({ack.type}). Assuming mission cleared.")
            return True, "\n".join(logs)
        log_message("Previous mission cleared.")
        return True, "\n".join(logs)

    @staticmethod
    def send_mission_count(drone_connection, count, timeout=10):
        logs = []
        def log_message(msg):
            logs.append(msg)
            print(msg)
        log_message(f"Sending mission count: {count}")
        drone_connection.mav.mission_count_send(drone_connection.target_system, drone_connection.target_component, count)
        time.sleep(0.2)
        log_message("Mission count sent.")
        return True, "\n".join(logs)

    @staticmethod
    def send_mission_item(drone_connection, wp, timeout=10):
        logs = []
        def log_message(msg):
            logs.append(msg)
            print(msg)
        log_message(f"Sending mission item {wp['seq']}")
        drone_connection.mav.mission_item_send(
            drone_connection.target_system,
            drone_connection.target_component,
            wp['seq'],
            wp['frame'],
            wp['command'],
            wp['current'],
            wp['autocontinue'],
            wp['param1'],
            wp['param2'],
            wp['param3'],
            wp['param4'],
            wp['param5'],
            wp['param6'],
            wp['param7']
        )
        time.sleep(0.2)
        log_message(f"Mission item {wp['seq']} sent.")
        return True, "\n".join(logs)

    @staticmethod
    def wait_for_mission_ack(drone_connection, timeout=10):
        logs = []
        def log_message(msg):
            logs.append(msg)
            print(msg)
        log_message("Waiting for mission ACK...")
        ack = drone_connection.recv_match(type='MISSION_ACK', blocking=True, timeout=timeout)
        if not ack:
            log_message("No mission ACK received. Assuming mission accepted.")
            return True, "\n".join(logs)
        if ack.type != mavutil.mavlink.MAV_MISSION_ACCEPTED:
            log_message(f"Mission ACK not accepted: {ack.type}. Assuming mission accepted.")
            return True, "\n".join(logs)
        log_message("Mission ACK received: Mission accepted.")
        return True, "\n".join(logs)

    # --------------------------------------------------------------------------
    # Mission Execution Methods
    # --------------------------------------------------------------------------
    @staticmethod
    def execute_point_mission(drone_connection, dest_lat, dest_lon, altitude=10, wait_time=30, timeout=30):
        logs = []
        def log_message(msg):
            logs.append(msg)
            print(msg)
        home = MAVLinkCommandInterface.get_current_position(drone_connection)
        log_message(f"Home position: {home}")

        success, msg_log = MAVLinkCommandInterface.set_mode_via_set_mode_msg(drone_connection, "GUIDED", timeout)
        log_message(msg_log)
        if not success:
            return False, "\n".join(logs)

        success, msg_log = MAVLinkCommandInterface.send_arm(drone_connection, timeout)
        log_message(msg_log)
        if not success:
            return False, "\n".join(logs)

        success, msg_log = MAVLinkCommandInterface.send_takeoff(drone_connection, altitude, timeout)
        log_message(msg_log)
        if not success:
            return False, "\n".join(logs)

        wait_start = time.time()
        while time.time() - wait_start < timeout:
            current_alt = MAVLinkCommandInterface.get_current_altitude(drone_connection)
            if abs(current_alt - altitude) < 1.0:
                log_message("Reached target altitude.")
                break
            time.sleep(1)
        else:
            log_message("Timeout waiting for target altitude.")
            return False, "\n".join(logs)

        success, msg_log = MAVLinkCommandInterface.send_waypoint(drone_connection, dest_lat, dest_lon, altitude, timeout)

        log_message(msg_log)
        if not success:
            return False, "\n".join(logs)
        wait_start = time.time()
        while time.time() - wait_start < timeout:
            pos = MAVLinkCommandInterface.get_current_position(drone_connection)
            d = MAVLinkCommandInterface.get_distance(pos[0], pos[1], dest_lat, dest_lon)
            alt_diff = abs(pos[2] - altitude)
            if d < 5 and alt_diff < 1:
                log_message("Reached destination.")
                break
            time.sleep(1)
        else:
            log_message("Timeout waiting to reach destination.")
            return False, "\n".join(logs)
        log_message(f"Waiting for {wait_time} seconds at destination")
        time.sleep(wait_time)
        success, msg_log = MAVLinkCommandInterface.send_waypoint(drone_connection, home[0], home[1], altitude, timeout)
        log_message(msg_log)
        if not success:
            return False, "\n".join(logs)
        wait_start = time.time()
        while time.time() - wait_start < timeout:
            pos = MAVLinkCommandInterface.get_current_position(drone_connection)
            d = MAVLinkCommandInterface.get_distance(pos[0], pos[1], home[0], home[1])
            alt_diff = abs(pos[2] - altitude)
            if d < 5 and alt_diff < 1:
                log_message("Returned home.")
                break
            time.sleep(1)
        else:
            log_message("Timeout waiting to return home.")
            return False, "\n".join(logs)
        success, msg_log = MAVLinkCommandInterface.send_land(drone_connection, timeout)
        log_message(msg_log)
        success, msg_log = MAVLinkCommandInterface.send_disarm(drone_connection, timeout)
        log_message(msg_log)
        return True, "\n".join(logs)

    @staticmethod
    def set_roi(drone_connection, lat, lon, alt, timeout=10):
        logs = []
        def log_message(msg):
            logs.append(msg)
            print(msg)
        log_message("Setting ROI to center of circle...")
        drone_connection.mav.command_long_send(
            drone_connection.target_system,
            drone_connection.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_ROI_LOCATION, 0,
            lat,
            lon,
            alt,
            0, 0, 0, 0
        )
        time.sleep(1)
        log_message("ROI set.")
        return True, "\n".join(logs)

    @staticmethod
    def clear_roi(drone_connection, timeout=10):
        logs = []
        def log_message(msg):
            logs.append(msg)
            print(msg)
        log_message("Clearing ROI...")
        drone_connection.mav.command_long_send(
            drone_connection.target_system,
            drone_connection.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_ROI, 0,
            0, 0, 0, 0, 0, 0, 0
        )
        time.sleep(1)
        log_message("ROI cleared.")
        return True, "\n".join(logs)

    @staticmethod
    def execute_circle_mission(drone_connection,origin_lat, origin_lon, center_lat, center_lon, altitude=10, radius=30, num_points=30, timeout=50):
        logs = []
        def log_message(msg):
            logs.append(msg)
            print(msg)

        origin = MAVLinkCommandInterface.get_current_position(drone_connection)
        log_message(f"Home (origin): {origin}")

        # Establecer modo GUIDED, armar y despegar.
        success, msg_log = MAVLinkCommandInterface.set_mode_via_set_mode_msg(drone_connection, "GUIDED", timeout)
        log_message(msg_log)
        if not success:
            return False, "\n".join(logs)
        success, msg_log = MAVLinkCommandInterface.send_arm(drone_connection, timeout)
        log_message(msg_log)
        if not success:
            return False, "\n".join(logs)
        success, msg_log = MAVLinkCommandInterface.send_takeoff(drone_connection, altitude, timeout)
        log_message(msg_log)
        if not success:
            return False, "\n".join(logs)
        wait_start = time.time()
        while time.time() - wait_start < timeout:
            current_alt = MAVLinkCommandInterface.get_current_altitude(drone_connection)
            if abs(current_alt - altitude) < 1.0:
                log_message("Reached target altitude.")
                break
            time.sleep(1)
        else:
            log_message("Timeout waiting for target altitude.")
            return False, "\n".join(logs)



        # Calcular los waypoints del círculo alrededor del centro (destination)

        circle_waypoints = []
        for i in range(num_points):
            angle = 2 * math.pi * i / num_points
            wp_lat, wp_lon = MAVLinkCommandInterface.calculate_point_at_distance(drone_connection, center_lat, center_lon, radius, angle)
            circle_waypoints.append((wp_lat, wp_lon))


        # Encontrar el waypoint más cercano al origen

        min_index = None
        min_distance = float('inf')
        for i, (wp_lat, wp_lon) in enumerate(circle_waypoints):
            d = MAVLinkCommandInterface.get_distance(origin_lat, origin_lon, wp_lat, wp_lon)
            log_message(f"The distance to the waypoint {i} is {d}")
            if d < min_distance:
                min_distance = d
                min_index = i
        log_message(f"Closest waypoint index to origin: {min_index} (distance: {min_distance:.2f} m)")

        # Reordenar la lista para que el primer waypoint sea el más cercano al origen
        ordered_waypoints = circle_waypoints[min_index:] + circle_waypoints[:min_index]

        # Agregar el origen al final para retornar
        ordered_waypoints.append(origin)
        mission_waypoints = []

        current_wp=0
        wp0 = {
            'seq': current_wp,
            'frame': mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            'command': mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            'current': 1,
            'autocontinue': 1,
            'param1': 0,
            'param2': 2,
            'param3': 0,
            'param4': 0,
            'param5': ordered_waypoints[0][0],
            'param6': ordered_waypoints[0][1],
            'param7': altitude
        }
        mission_waypoints.append(wp0)
        current_wp += 1


        # UNSUPPORTED !!!!!

        # roi_waypoint_center = {
        #     'seq': current_wp,  # Número de secuencia apropiado en la misión
        #     'frame': mavutil.mavlink.MAV_FRAME_GLOBAL,
        #     'command': mavutil.mavlink.MAV_CMD_DO_SET_ROI_LOCATION,
        #     'current': 0,
        #     'autocontinue': 1,
        #     'param1': 0,
        #     'param2': 0,
        #     'param3': 0,
        #     'param4': 0,
        #     'param5': center_lat,
        #     'param6': center_lon,
        #     'param7': 0
        # }
        # mission_waypoints.append(roi_waypoint_center)
        # current_wp += 1


        for i in range(1, len(ordered_waypoints)-1):
            bearing=MAVLinkCommandInterface.calculate_bearing(wp_lat, wp_lon, center_lat, center_lon)
            wp_lat, wp_lon = ordered_waypoints[i]
            wp = {
                'seq':current_wp,
                'frame': mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                'command': mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                'current': 0,
                'autocontinue': 1,
                'param1': 0,
                'param2': 2,
                'param3': 0,
                'param4': bearing,
                'param5': wp_lat,
                'param6': wp_lon,
                'param7': altitude
            }
            mission_waypoints.append(wp)
            current_wp += 1


        # UNSUPPORTED !!!
        # Insertar un ítem de misión para limpiar el ROI (desactivar la orientación forzada)
        # roi_set_item = {
        #     'seq': current_wp,
        #     'frame': mavutil.mavlink.MAV_FRAME_GLOBAL,
        #     'command': mavutil.mavlink.MAV_CMD_DO_SET_ROI,
        #     'current': 0,
        #     'autocontinue': 1,
        #     'param1': 0,
        #     'param2': 0,
        #     'param3': 0,
        #     'param4': 0,
        #     'param5': 0,
        #     'param6': 0,
        #     'param7': 0
        # }
        # mission_waypoints.append(roi_set_item)
        # current_wp += 1


        wp_return = {
            'seq':current_wp,
            'frame': mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            'command': mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            'current': 0,
            'autocontinue': 1,
            'param1': 0,
            'param2': 2,
            'param3': 0,
            'param4': 0,
            'param5': origin[0],
            'param6': origin[1],
            'param7': altitude
        }
        mission_waypoints.append(wp_return)
        current_wp += 1
        # Agregar finalmente el waypoint de aterrizaje en el origen.
        wp_land = {
            'seq': current_wp,
            'frame': mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            'command': mavutil.mavlink.MAV_CMD_NAV_LAND,
            'current': 0,
            'autocontinue': 1,
            'param1': 0,
            'param2': 0,
            'param3': 0,
            'param4': 0,
            'param5': origin[0],
            'param6': origin[1],
            'param7': altitude
        }
        mission_waypoints.append(wp_land)
        current_wp += 1
        # Subir la misión
        success, log_clear = MAVLinkCommandInterface.clear_mission(drone_connection, timeout)
        log_message(log_clear)
        if not success:
            log_message("Failed to clear previous mission.")
            return False, "\n".join(logs)
        total_waypoints = len(mission_waypoints)
        success, log_count = MAVLinkCommandInterface.send_mission_count(drone_connection, total_waypoints, timeout)
        log_message(log_count)
        if not success:
            log_message("Failed to send mission count.")
            return False, "\n".join(logs)
        for i, wp in enumerate(mission_waypoints):
            success, log_item = MAVLinkCommandInterface.send_mission_item(drone_connection, wp, timeout)
            log_message(log_item)
            if not success:
                log_message(f"Failed to send mission item {i}.")
                return False, "\n".join(logs)
        success, log_ack = MAVLinkCommandInterface.wait_for_mission_ack(drone_connection, timeout)
        log_message(log_ack)
        if not success:
            log_message("Mission upload failed.")
            return False, "\n".join(logs)
        log_message("Mission uploaded successfully!")
        # Cambiar a modo AUTO para ejecutar la misión
        success, msg_log = MAVLinkCommandInterface.set_mode_via_set_mode_msg(drone_connection, "AUTO", timeout)
        log_message(msg_log)
        if not success:
            return False, "\n".join(logs)
        log_message("Circle mission execution initiated.")
        return True, "\n".join(logs)
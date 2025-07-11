import logging
import time
import math
from pymavlink import mavutil
from collections import deque
import numpy as np
import threading

class MavlinkMasterDroneReaderInterface:
   
    def __init__(self, state, mavlinkConnection):
        self.state_ = state
        
        self.logger = logging.getLogger("MavLink master Reader Interface")
        self.mavlinkConnection_ = mavlinkConnection
        self.lidar_readings = deque(maxlen=20)
        self.last_evasion_time = 0
        self.last_yaw_rate = 0
        self.ultima_evasion = 0
        self._evasion_timer = None
        self.is_executing_evasion = False
        self.evasion_cooldown = 3.0  # Cooldown period in seconds
        

        dt = 0.1  # time step
        self.status = np.zeros(9)  ## UAV vector de estado incial: [x, y, z, vx, vy, vz, ax, ay, az]
        self.status[:3] = [0.0, 0.0, 0.0]  # initial position
        self.status[3:6] = [2.0, 2.0, 0.0]  # initial velocity: 2 m/s in x and y

        self.A = np.eye(9)
        for i in range(3):
            self.A[i, i+3] = dt
            self.A[i, i+6] = 0.5 * dt**2
            self.A[i+3, i+6] = dt


        # Definimos matriz B - (Matriz de control)
        self.B = np.zeros((9, 3))
        # Position rows (x, y, z) ← from acceleration
        self.B[0, 0] = 0.5 * dt**2
        self.B[1, 1] = 0.5 * dt**2
        self.B[2, 2] = 0.5 * dt**2
        # Velocity rows (vx, vy, vz) ← from acceleration
        self.B[3, 0] = dt
        self.B[4, 1] = dt
        self.B[5, 2] = dt
        # Acceleration rows (ax, ay, az) ← set directly to control
        self.B[6, 0] = 1.0
        self.B[7, 1] = 1.0
        self.B[8, 2] = 1.0
        self.constant_speed = np.linalg.norm(self.status[3:6])


    def process_model(self, state):
        dt = 0.1
        A = np.eye(9)
        for i in range(3):
            A[i, i+3] = dt
            A[i, i+6] = 0.5 * dt**2
            A[i+3, i+6] = dt

        a_noise_std = 0.2
        v_max = 2.0
        a_max = 3.0

        noise = np.zeros(9)
        noise[6:] = np.random.normal(0, a_noise_std, 3)

        new_state = self.A @ state + noise
        new_state[3:6] = np.clip(new_state[3:6], -v_max, v_max)
        new_state[6:9] = np.clip(new_state[6:9], -a_max, a_max)
        return new_state
    def set_mode(self, mode):
            # Mapear modo a número
            mode_mapping = self.mavlinkConnection_.mode_mapping()
            if mode not in mode_mapping:
                self.logger.error(f"Unknown mode: {mode}")
                return

            mode_id = mode_mapping[mode]

            # Enviar set_mode
            self.mavlinkConnection_.set_mode(mode_id)
            self.logger.info(f"[INFO] {mode}")

    def update_state(self, ax_r, ay_r, yaw_nuevo_r):
        if self.is_executing_evasion: #Significa que ya se está ejecutando
            print(f"[Flag (Is_Executing_evasion)] {self.is_executing_evasion} -- En curso UPDATE STATE. ")
            return
        self.is_executing_evasion = True
        print("\n [INFO] Modificando ESTADO Dron --> OBSTACULO Detectado ")
        dt = 0.1  # time step
        estado = self.get_current_state()
        # Imprimir todo el estado del dron
        #self.logger.info(f"Estado INICIAL del dron: {estado}")
       
        #CALCULAMOS NUEVO ESTADO
        
        yaw_rate = yaw_nuevo_r
        yaw = np.radians(estado["heading"]) 
        # Apply control accelerations via B matrix
        u = np.array([ax_r, ay_r, 0.0])
        new_state = self.A @ self.status + self.B @ u

        # Add noise
        new_state = self.process_model(new_state)
        # CORREGIR ALTITUD
        new_state[2] = self.status[2]  # Mantén altitud

        #Actualizamos las velocidades
        new_state[3] += ax_r * dt  # vx
        new_state[4] += ay_r * dt  # vy

        # Limitar velocidades máximas
        v_max = 2.0
        new_state[3:5] = np.clip(new_state[3:5], -v_max, v_max)

        new_state[5] = 0.0             # vz = 0
        
        new_yaw = yaw + yaw_rate * dt
        self.expected_yaw   = np.degrees(new_yaw) % 360
        # Align velocity with yaw and maintain constant speed
        #new_state[3:5] = self.constant_speed * np.array([np.cos(new_yaw), np.sin(new_yaw)])
        
        new_state[5] = 0.0  # Ensure vz = 0 (fixed altitude)

        a_max = 3.0
     
        #self.logger.info("\n [Command] EVASION Command set_position_target.")
        print("[Command] EVASION Command set_position_target.")
        print(f"[INFO] YAW Actual: {np.degrees(yaw)}, YAW Objetivo : {self.expected_yaw}")

        vx = new_state[3] 
        vy= new_state[4]
        ax = self.status[6]
        ay = self.status[7]
        az = 0.0
        try:
            # Primero, aseguramos conexión
            #self.mavlinkConnection_.wait_heartbeat()
            #self.logger.info("Heartbeat received. Changing mode to GUIDED.")

            msg = self.mavlinkConnection_.recv_match(type="SYSTEM_TIME", blocking=False)
            if msg:
                timestamp_ms = msg.time_boot_ms
            else:
                timestamp_ms = int((time.time() % 86400) * 1000)
            # Ahora timestamp_ms lo usas en tu set_position_target

            # Cambiar a GUIDED
            self.set_mode('GUIDED')
            print("Logger.info [Command] Modo cambiado a GUIDED.")
        
            #self.logger.info("[COMMAND] Mode set to GUIDED.")
            

            # Sincronizar tiempo
            system_time = self.mavlinkConnection_.recv_match(type="SYSTEM_TIME", blocking=True)
            time_pair = (time.time(), system_time.time_boot_ms)

            # Definir la posición objetivo
            #ax = 0
            ay = estado["accel_y"]
            az = -9.7             

            # Enviar el comando
            # Setup the bitfields to tell the vehicle to ignore velocity and accelerations
            ignore_all_except_accel = (
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_X_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_Y_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
            )

            self.mavlinkConnection_.mav.set_position_target_global_int_send(
                timestamp_ms,  # Timestamp en milisegundos
                self.mavlinkConnection_.target_system,
                self.mavlinkConnection_.target_component,
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                ignore_all_except_accel,
                0,  # Latitud en grados * 1e7
                0,  # Longitud en grados * 1e7
                100,              # Altitud en metros
                vx, vy, 0,         # Velocidades (no utilizadas)
                0, 0, 0,      # Aceleraciones en m/s²
                new_yaw,               # Yaw 
                0                # Yaw rate (no utilizado)
            )
            #self.logger.info("Comando de EVASIÓN enviado.")
            estado_actualizado = self.get_current_state()

###################

            # Imprimir todo el estado del dron
            #self.logger.info(f"Estado ACTUALIZADO del dron: {estado_actualizado}")
        
            #self.logger.info("\n ############ [End] UPDATE STATE and EVASION COMMAND ... ###############")
            print("[Command] END -- EVASION Command set_position_target.")


           # Programamos el retorno a AUTO + liberado de flag tras N segundos
            delay = 1.0  # segundos a esperar antes de volver a AUTO
            print(f"[DEBUG] Programando finish_evasion en {delay}s")
            # Guarda el Timer para que no sea recolectado por GC
            self._evasion_timer = threading.Timer(delay, self.finish_evasion)
            self._evasion_timer.daemon = True
            self._evasion_timer.start()

            return
        
        except Exception as e:
            self.logger.info(f"Error en evasión: {e}")
            self.reset_evasion_flag()  # Reset flag on error
            return
        
    def reset_evasion_flag(self):
        """Reset the evasion flag after execution completes"""
        self.is_executing_evasion = False
        print("[Flag (Executing_evasion) ] False")
    
    def finish_evasion(self):
        self.set_mode('AUTO')
        self.is_executing_evasion = False
        print("Logger.info [Command] Modo cambiado a AUTO tras evasión.")
            

    
    def ReadTuplaArray(self, msg):
         # Los canales suelen estar en chan1_raw, chan2_raw, etc. hasta chan8_raw
        lidar_range = 10.0
        rc7_value = msg.chan7_raw
        # Vamos a imprimirlo para comprobar
        #print(f"RC7 value: {rc7_value}")
        # Supongamos que queremos detectar cuando RC7 sube a más de 1800
        if rc7_value is not None:
            # Extraer los valores originales
            distancia_recuperada = (rc7_value >> 9) & 0x7F  # 0x7F = 0b1111111
            angulo_recuperado = rc7_value & 0x1FF
            
            if  angulo_recuperado> 360:
                angulo_recuperado = 360
            
            print(f"Distancia: {distancia_recuperada}, Ángulo: {angulo_recuperado}")
            ##Cualquier Distancia mayor a nuestro lidar_range, le vamos a poner ese tope
            min_dist = lidar_range

            if distancia_recuperada < min_dist:
                min_dist = distancia_recuperada

            #Estos valores los podríamos guardar en una lista
            self.lidar_readings.append((min_dist, angulo_recuperado))
            #print(self.lidar_readings)
            if len(self.lidar_readings) >15: # Como se lanza según inicializamos.
                print("[INFO] Evaluando Distancias. ")
                self.evasion_strategy(self.lidar_readings)
                return


    def normalize_angle_pm_pi(self, angle):
        return (angle + np.pi) % (2 * np.pi) - np.pi

    def normalize_angle_2pi(self, angle):
        return angle % (2 * np.pi)
    
    def normalize_angle(self, angle):
        return angle % (2 * np.pi)
    def evasion_strategy(self, data_array):
        danger_angles = []
        #print(f"[Flag (Is_Executing_evasion)] Valor: {self.is_executing_evasion}")
        # Lo que nos va a dar a que aceleración cambiar
        found_obstacle= False
        lidar_range = 15
        lidar_data = data_array
        estado_dron = self.get_current_state() 
        yaw = np.radians(estado_dron["heading"])  # Convertir de grados a radianes
        yaw_degrees = estado_dron["heading"]
    
        yaw_rate = estado_dron["yaw_rate"]
        #print(f"Yaw Rate: {yaw_rate}")
        
        warning_distance = 4.0
        danger_distance = 2.0
        max_acceleration = 3.0  # m/s^2

        max_yaw_rate = np.radians(45)  # ±45°/s
        YAW_GAIN = 2.0  # Proportional gain for yaw rate
        YAW_SMOOTHING_FACTOR = 0.7  # Low-pass filter for yaw_rate
         # RUIDO
        a_noise_std = 0.2
        distances = np.array([d for d, _ in lidar_data])
        angles_body = np.array([a for _, a in lidar_data])
        angles_body_rad = np.radians([a for _, a in lidar_data])
        #print(angles_body)
        valid_indices = distances < lidar_range
        if not np.any(valid_indices):
        # No obstáculos
            yaw_rate = np.random.normal(0, a_noise_std)
            yaw_rate = np.clip(yaw_rate, -max_yaw_rate, max_yaw_rate)
            yaw_rate = YAW_SMOOTHING_FACTOR * self.last_yaw_rate + (1 - YAW_SMOOTHING_FACTOR) * yaw_rate
            self.last_yaw_rate = yaw_rate
            print(f"[Flag (Is_Executing_evasion)] (2) Change to {self.is_executing_evasion}")
            return np.array([0.0, 0.0, 0.0, yaw_rate])
        
        min_dist_index = np.argmin(distances[valid_indices])
        min_dist = distances[valid_indices][min_dist_index]
        min_angle_body = angles_body[valid_indices][min_dist_index] #En grados de 0 a 360º
        min_angle_body_rad = np.radians(min_angle_body)
        
        
        ax = 0.0
        ay = 0.0
        az = 0.0
        if min_dist <= danger_distance:
            print(f"[INFO] Detectado mínima distancia < Danger Distancia (200 m): {min_dist:.2f}m, ") 

            found_obstacle= True
             # --- Estrategia basada en abanicos (huecos libres) ---
            danger_mask = distances <= danger_distance
            print(f"Distances: {distances}")
            print(f"Mask: {danger_mask}")
            if np.sum(danger_mask) < 2:
                print("[INFO] No hay suficientes obstáculos para definir zonas de evasión.")
                return np.array([0.0, 0.0])  # o cualquier otro comportamiento por defecto
            danger_angles = np.sort(self.normalize_angle_2pi(angles_body_rad[danger_mask]))
            print("Abanico Ángulo detectado < Distancia Danger ( 2m ):", np.degrees(danger_angles))
            
            # Paso 2: Agregar extremos para cierre de círculo
            extended_angles = np.concatenate(([danger_angles[-1] - 2 * np.pi], danger_angles, [danger_angles[0] + 2 * np.pi]))

            # Paso 3: Calcular huecos entre ángulos consecutivos
            angle_diffs = np.diff(extended_angles)
            largest_gap_index = np.argmax(angle_diffs)
            gap_start = extended_angles[largest_gap_index]
        
            gap_end = extended_angles[largest_gap_index + 1]
            print(f"Gap_Inicial: {np.degrees(self.normalize_angle(gap_start)):.2f}°")
            print(f"Gap_Final: {np.degrees(self.normalize_angle(gap_end)):.2f}°")
             # Paso 4: Dirección de escape = centro del hueco
            avoidance_angle_body = (gap_start + gap_end) / 2.0 #Esto está en radianes
            print(f"YAW:" , np.degrees(yaw))
            avoidance_angle_world = self.normalize_angle(avoidance_angle_body + yaw) #Se normaliza el ángulo resultante a [0, 2π].
            print(f"Ángulo al que cambiamos: {np.degrees(avoidance_angle_body):.2f}°, [EVASIÓN] Hueco más amplio: {np.degrees(angle_diffs[largest_gap_index]):.2f}°")
            print(f"Ángulo de evasión (mundo): {np.degrees(avoidance_angle_world):.2f}°")
            # Paso 5: Convertimos a mundo y generamos comandos
            ax = max_acceleration * np.cos(avoidance_angle_world ) #Cos y Sen se calcula con radianes
            ay = max_acceleration * np.sin(avoidance_angle_world )

            #  El yaw_error está bien calculado como diferencia entre el yaw actual y el objetivo.
            yaw_error = avoidance_angle_world - yaw
            # Normaliza el error a [-pi, pi]
            yaw_error = (yaw_error + np.pi) % (2 * np.pi) - np.pi


            yaw_rate = YAW_GAIN * yaw_error
            yaw_rate = np.clip(yaw_rate, -max_yaw_rate, max_yaw_rate)
            yaw_rate = YAW_SMOOTHING_FACTOR * self.last_yaw_rate + (1 - YAW_SMOOTHING_FACTOR) * yaw_rate
            self.last_yaw_rate = yaw_rate
            

        elif min_dist <= warning_distance:
            print(f"[INFO] Detectado mínima distancia < Warning Distancia (400 m): {min_dist} dm , ")
            found_obstacle= True
            ## Weaker avoidance: Scale strength with distance
            #avoidance_angle_body = min_angle_body + np.pi
            avoidance_angle_body = self.normalize_angle(min_angle_body_rad + np.pi)

            strength = max_acceleration * (warning_distance - min_dist) / (warning_distance - danger_distance)
            
            #avoidance_angle_world = avoidance_angle_body + yaw
            avoidance_angle_world = self.normalize_angle(avoidance_angle_body + yaw)
            ax = strength * np.cos(avoidance_angle_world)
            ay = strength * np.sin(avoidance_angle_world)
            
            yaw_error = avoidance_angle_world - yaw
            # Normaliza el error a [-pi, pi]
            yaw_error = (yaw_error + np.pi) % (2 * np.pi) - np.pi
            
            yaw_rate = YAW_GAIN * yaw_error
            yaw_rate = np.clip(yaw_rate, -max_yaw_rate, max_yaw_rate)
            yaw_rate = YAW_SMOOTHING_FACTOR * self.last_yaw_rate + (1 - YAW_SMOOTHING_FACTOR) * yaw_rate
            self.last_yaw_rate = yaw_rate
            # Print the detected minimum distance and angles
            print(f"Ángulo detectado: {min_angle_body}° , ángulo al que cambiamos: {np.degrees(avoidance_angle_body):.2f}°")
            print(f"Ángulo de evasión (mundo): {np.degrees(avoidance_angle_world):.2f}°")
            
        if (found_obstacle == True) and not self.is_executing_evasion:
            print("[INFO] Lanzamos evasión")
            self.update_state(ax, ay, yaw_rate)
        return

    def ReadCameraChannel(self, msg):
        # newCameraValue = None 
        # if self.channelCam_ == 1:
        #     newCameraValue = int(msg.chan1_raw)
        # elif self.channelCam_ == 2:
        #     newCameraValue = int(msg.chan2_raw)
        # elif self.channelCam_ == 3:
        #     newCameraValue = int(msg.chan3_raw)
        # elif self.channelCam_ == 4:
        #     newCameraValue = int(msg.chan4_raw)
        # elif self.channelCam_ == 5:
        #     newCameraValue = int(msg.chan5_raw)
        # elif self.channelCam_ == 6:
        #     newCameraValue = int(msg.chan6_raw)
        # elif self.channelCam_ == 7:
        #     newCameraValue = int(msg.chan7_raw)
        # elif self.channelCam_ == 8:
        #     newCameraValue = int(msg.chan8_raw)
        # elif self.channelCam_ == 9:
        #     newCameraValue = int(msg.chan9_raw)
        # elif self.channelCam_ == 10:
        #     newCameraValue = int(msg.chan10_raw)
        # elif self.channelCam_ == 11:
        #     newCameraValue = int(msg.chan11_raw)
        # elif self.channelCam_ == 12:
        #     newCameraValue = int(msg.chan12_raw)
        # elif self.channelCam_ == 13:
        #     newCameraValue = int(msg.chan13_raw)
        # elif self.channelCam_ == 14:
        #     newCameraValue = int(msg.chan14_raw)
        # elif self.channelCam_ == 15:
        #     newCameraValue = int(msg.chan15_raw)
        # elif self.channelCam_ == 16:
        #     newCameraValue = int(msg.chan16_raw)
        # else:
        #     self.logger.error("UNKNOWN RC Channnel configured for Camera Value (must be between 1 and 16")
        
        #self.logger.info(f"Received New RC Channel Camera value: {int(msg.chan5_raw)}")
        return
    

    def get_current_state(self):
        telemetry = self.state_.masterDroneTelemetry_
        return {
            "latitude": telemetry.latitude_,
            "longitude": telemetry.longitude_,
            "altitude": telemetry.altitude_,
            "groundspeed": telemetry.groundspeed_,
            "heading": telemetry.heading_,
            "pitch": telemetry.pitch_,
            "roll": telemetry.roll_,
            "accel_x": telemetry.accel_x_,  # Aceleración en el eje X
            "accel_y": telemetry.accel_y_,  # Aceleración en el eje Y
            "accel_z": telemetry.accel_z_, # Aceleración en el eje Z
            "yaw_rate":   telemetry.yaw_rate_   
        }
    
    def HandleHighresImu(self, msg):
        self.state_.masterDroneTelemetry_.accel_x_ = msg.xacc / 1000.0 * 9.80665  # m/s²
        self.state_.masterDroneTelemetry_.accel_y_ = msg.yacc / 1000.0 * 9.80665
        self.state_.masterDroneTelemetry_.accel_z_ = msg.zacc / 1000.0 * 9.80665
        #self.logger.info( f"Received MAVLink HighresImu: A_x={self.state_.masterDroneTelemetry_.accel_x_ }, A_y={self.state_.masterDroneTelemetry_.accel_y_ },A_z={self.state_.masterDroneTelemetry_.accel_z_ }")

    def HandleVfrHud(self, msg):
        self.state_.masterDroneTelemetry_.groundspeed_ = msg.groundspeed #Velocidad sobre el suelo = Velocidad
        self.state_.masterDroneTelemetry_.altitude_ = msg.alt #Altitud respecto el mar
        self.state_.masterDroneTelemetry_.heading_ = msg.heading #Rumbo, dirección en grados respecto al norte.
        """ #self.logger.info( f"Received MAVLink VFR_HUD: Altitude={msg.alt}, Heading={msg.heading}")
        yaw_tolerance = 5.0
        #COMPROBACIÓN ASÍNCRONA DE NUESTRO GIRO
        heading = msg.heading % 360
        if hasattr(self, "expected_yaw"):
            diff = abs((heading - self.expected_yaw + 180) % 360 - 180)
            if diff < self.yaw_tolerance:
                print("[INFO] Giro completado (asincrónico).")
                # Cambiar a GUIDED
                self.set_mode('AUTO')
                self.logger.info(" Mode set to AUTO.")
                del self.expected_yaw
            elif time.time() - self.yaw_start_time > self.yaw_timeout:
                print("[WARN] No completó giro a tiempo.")
                del self.expected_yaw """


    def HandleAttitude(self, msg):
        self.state_.masterDroneTelemetry_.pitch_ = msg.pitch
        self.state_.masterDroneTelemetry_.roll_ = msg.roll
        self.state_.masterDroneTelemetry_.yaw_rate_ = msg.yawspeed  # rad/s
        #self.logger.info(f"Received MAVLink ATTITUDE: Pitch={msg.pitch}, Roll={msg.roll}")

    def HandleGlobalPosition(self, msg):
        self.state_.masterDroneTelemetry_.latitude_ = msg.lat / 1e7
        self.state_.masterDroneTelemetry_.longitude_ = msg.lon / 1e7
        self.state_.masterDroneTelemetry_.altitude_ = msg.alt
        #self.logger.info( f"Received MAVLink GLOBAL_POSITION_INT: Lat={self.state_.masterDroneTelemetry_.latitude_}, Lon={self.state_.masterDroneTelemetry_.longitude_}")

    def HandleSysStatus(self, msg):
        self.state_.battery_.voltageValue_ = msg.voltage_battery / 1000.0
        self.state_.battery_.currentValue_ = msg.current_battery / 100.0

    # self.logger.info( f"Received MAVLink SYS_STATUS: Voltage={self.state_.battery_.voltageValue_}V")

    def MavlinkReader(self):
        self.mavlinkConnection_.wait_heartbeat()
        self.state_.control_.masterHearthBeatReceived_ = True
        #self.logger.info("Initiating MAVLink Read loop...")

        while True:
            msg = self.mavlinkConnection_.recv_match(blocking=False)

            #self.logger.debug("Received master msg")
            if msg is not None:
                try:
                    msgType = msg.get_type()
                    #print("Tipo de mensaje: ", msgType)
                    if msgType == 'ATTITUDE':
                        #self.logger.debug("Received ATTITUDE message.")
                        self.HandleAttitude(msg)
                        #self.HandleAttitude(msg)

                    if msgType == 'RC_CHANNELS':
                        self.ReadTuplaArray(msg)
                        #print(f"MSG value: {msg}")
                    #Posicion
                    if msgType == 'GLOBAL_POSITION_INT':
                        #self.logger.debug("Received GLOBAL_POSITION_INT message.")
                        self.HandleGlobalPosition(msg)
                        
                    #Velocidad, heading en grados. 
                    if msgType == 'VFR_HUD':
                        #self.logger.debug("Received VFR_HUD message.")
                        self.HandleVfrHud(msg)
            
                    #Aceleración
                    if msgType == 'RAW_IMU':
                        self.HandleHighresImu(msg)
                        #self.logger.debug("Received RAW_IMU message (aceleración en metros cuadrados, m/s^2).")
                except Exception as e:
                    self.logger.error(f"Error in PymavlinkRead: {e}")

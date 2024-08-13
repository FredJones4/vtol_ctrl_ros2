make px4_sitl gz_standard_vtol
[0/1] cd /home/cmhales/PX4-Autopilot/build/px4_sitl_default/src/modules/simulation/gz_bridg...v PX4_SIM_MODEL=gz_standard_vtol /home/cmhales/PX4-Autopilot/build/px4_sitl_default/bin/px4

______  __   __    ___ 
| ___ \ \ \ / /   /   |
| |_/ /  \ V /   / /| |
|  __/   /   \  / /_| |
| |     / /^\ \ \___  |
\_|     \/   \/     |_/

px4 starting.

INFO  [px4] startup script: /bin/sh etc/init.d-posix/rcS 0
INFO  [init] found model autostart file as SYS_AUTOSTART=4004
INFO  [param] selected parameter default file parameters.bson
INFO  [param] importing from 'parameters.bson'
INFO  [parameters] BSON document size 404 bytes, decoded 404 bytes (INT32:14, FLOAT:6)
INFO  [param] selected parameter backup file parameters_backup.bson
ERROR [param] Parameter FW_L1_PERIOD not found.
INFO  [dataman] data manager file './dataman' size is 7866640 bytes
INFO  [init] starting gazebo with world: /home/cmhales/PX4-Autopilot/Tools/simulation/gz/worlds/default.sdf
WARN  [init] PX4_GZ_MODEL_NAME or PX4_GZ_MODEL not set using PX4_SIM_MODEL.
INFO  [gz_bridge] world: default, model name: standard_vtol_0, simulation model: standard_vtol
Warning: Ignoring XDG_SESSION_TYPE=wayland on Gnome. Use QT_QPA_PLATFORM=wayland to run on Wayland anyway.
INFO  [lockstep_scheduler] setting initial absolute time to 8000 us
INFO  [commander] LED: open /dev/led0 failed (22)
INFO  [tone_alarm] home set
INFO  [mavlink] mode: Normal, data rate: 4000000 B/s on udp port 18570 remote port 14550
INFO  [mavlink] partner IP: 127.0.0.1
INFO  [tone_alarm] notify negative
INFO  [mavlink] mode: Onboard, data rate: 4000000 B/s on udp port 14580 remote port 14540
INFO  [mavlink] mode: Onboard, data rate: 4000 B/s on udp port 14280 remote port 14030
INFO  [mavlink] mode: Gimbal, data rate: 400000 B/s on udp port 13030 remote port 13280
INFO  [logger] logger started (mode=all)
INFO  [logger] Start file log (type: full)
INFO  [logger] [logger] ./log/2024-08-13/22_58_42.ulg	
INFO  [logger] Opened full log file: ./log/2024-08-13/22_58_42.ulg
INFO  [mavlink] MAVLink only on localhost (set param MAV_{i}_BROADCAST = 1 to enable network)
INFO  [mavlink] MAVLink only on localhost (set param MAV_{i}_BROADCAST = 1 to enable network)
INFO  [px4] Startup script returned successfully
pxh> INFO  [uxrce_dds_client] synchronized with time offset 1723589921582647us
INFO  [uxrce_dds_client] successfully created rt/fmu/out/failsafe_flags data writer, topic id: 80
INFO  [uxrce_dds_client] successfully created rt/fmu/out/position_setpoint_triplet data writer, topic id: 149
INFO  [uxrce_dds_client] successfully created rt/fmu/out/sensor_combined data writer, topic id: 168
INFO  [uxrce_dds_client] successfully created rt/fmu/out/timesync_status data writer, topic id: 191
INFO  [uxrce_dds_client] successfully created rt/fmu/out/vehicle_control_mode data writer, topic id: 212
INFO  [uxrce_dds_client] successfully created rt/fmu/out/vehicle_gps_position data writer, topic id: 215
INFO  [uxrce_dds_client] successfully created rt/fmu/out/vehicle_local_position data writer, topic id: 219
INFO  [uxrce_dds_client] successfully created rt/fmu/out/vehicle_odometry data writer, topic id: 224
INFO  [uxrce_dds_client] successfully created rt/fmu/out/vehicle_status data writer, topic id: 229
INFO  [uxrce_dds_client] successfully created rt/fmu/out/vehicle_attitude data writer, topic id: 206
INFO  [uxrce_dds_client] successfully created rt/fmu/out/vehicle_global_position data writer, topic id: 213
INFO  [commander] Ready for takeoff!
WARN  [health_and_arming_checks] Preflight Fail: Yaw estimate error
INFO  [commander] Armed by internal command	
INFO  [tone_alarm] arming warning
INFO  [commander] Takeoff detected	
INFO  [commander] Landing detected	
INFO  [commander] Disarmed by landing	
INFO  [tone_alarm] notify neutral
INFO  [logger] closed logfile, bytes written: 108044716
ERROR [timesync] Time jump detected. Resetting time synchroniser.


#include <stdio.h>
#include <stdint.h>  
#include "osdconfig.h"
#include "telemetry.h"
#include "mavlink_parse.h"
#include "mavlink.h"
#include <math.h>

#include <unistd.h>
#include <stdlib.h>

int mavlink_parse_buffer(telemetry_data_t *td, uint8_t *buf, int buflen) {
  int new_data = 0;
  int i;

  mavlink_message_t msg;
  mavlink_status_t status; 
  //printf("mavlink parse buffer\n");

  for(i=0; i<buflen; i++) {
    uint8_t c = buf[i];
    if(mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)){
    new_data=1;
	
    switch (msg.msgid) {
        case MAVLINK_MSG_ID_HEARTBEAT:
			{
			//uint32_t custom_mode; /*< A bitfield for use for autopilot-specific flags.*/
			//uint8_t type; /*< Type of the MAV (quadrotor, helicopter, etc., up to 15 types, defined in MAV_TYPE ENUM)*/
			//uint8_t autopilot; /*< Autopilot type / class. defined in MAV_AUTOPILOT ENUM*/
			//uint8_t base_mode; /*< System mode bitfield, see MAV_MODE_FLAG ENUM in mavlink/include/mavlink_types.h*/
			//uint8_t system_status; /*< System status flag, see MAV_STATE ENUM*/
			//uint8_t mavlink_version; /*< MAVLink version, not writable by user, gets added by protocol because of magic data type: uint8_t_mavlink_version*/

			uint8_t base_mode = mavlink_msg_heartbeat_get_base_mode(&msg);
			uint8_t custom_mode = (uint8_t)mavlink_msg_heartbeat_get_custom_mode(&msg);
			
		
			#ifdef DEBUG
			printf("mavlink_parse.c: MAVLINK_MSG_ID_HEARTBEAT\ncustom mode=%d\n",custom_mode);
			printf("base mode=%d\n",base_mode); 
			printf("td->armed=%d\n",td->armed);
			#endif
			
			uint8_t motor_armed = (base_mode >> 7) & 0x01; //get bit 7
			
			if (motor_armed == 1 && td->armed == 0) {
				td->message_pending = true;
				td->message = "--ARMED--";
				td->home_set = true;
				
			}else if (motor_armed == 0 && td->armed == 1) {
				td->message_pending = true;
				td->message = "--DISARMED--";
				td->home_set = false;
			}
			td->armed = motor_armed;
			
			//MAVLINK Flight Modes
			#ifdef ARDUCOPTER
			switch((int)custom_mode)
				{
				case MODE_STABILIZE:	
					td->flight_mode =  "STABILIZE";
					break;
				case MODE_ACRO:
					td->flight_mode =  "ACRO";
					break;
				case MODE_ALTHOLD:
					td->flight_mode =  "ALT HOLD";
					break;
				case MODE_AUTO:
					td->flight_mode =  "AUTO";
					break;
				case MODE_GUIDED:
					td->flight_mode =  "GUIDED";
					break;
				case MODE_LOITER:
					td->flight_mode =  "LOITER";
					break;
				case MODE_RTL:
					td->flight_mode =  "RTL";
					break;
				case MODE_CIRCLE:
					td->flight_mode =  "CIRCLE";
					break;
				case MODE_LAND:
					td->flight_mode =  "LAND";
					break;					
				case MODE_DRIFT:
					td->flight_mode =  "DRIFT";
					break;	
				case MODE_SPORT:
					td->flight_mode =  "SPORT";
					break;	
				case MODE_FLIP:
					td->flight_mode =  "FLIP";
					break;
				case MODE_AUTOTUNE:
					td->flight_mode =  "AUTOTUNE";
					break;
				case MODE_POSHOLD:
					td->flight_mode =  "POS HOLD";
					break;
				case MODE_BRAKE:
					td->flight_mode =  "BRAKE";
					break;
				case MODE_THROW:
					td->flight_mode =  "THROW";
					break;

				default:
					td->flight_mode =  "FLIGHT MODE";
					break;
				}
			#endif
			#ifdef DEBUG
			printf("motor_armed=%d\n",motor_armed);
			printf("flight mode=%s\n\n",td->flight_mode);
			#endif
			
			}
			break;		

	   case MAVLINK_MSG_ID_SYS_STATUS:
			{
			//uint32_t onboard_control_sensors_present; /*< Bitmask showing which onboard controllers and sensors are present. Value of 0: not present. Value of 1: present. Indices defined by ENUM MAV_SYS_STATUS_SENSOR*/
			//uint32_t onboard_control_sensors_enabled; /*< Bitmask showing which onboard controllers and sensors are enabled:  Value of 0: not enabled. Value of 1: enabled. Indices defined by ENUM MAV_SYS_STATUS_SENSOR*/
			//uint32_t onboard_control_sensors_health; 	/*< Bitmask showing which onboard controllers and sensors are operational or have an error:  Value of 0: not enabled. Value of 1: enabled. Indices defined by ENUM MAV_SYS_STATUS_SENSOR*/
			//uint16_t load; 				/*< Maximum usage in percent of the mainloop time, (0%: 0, 100%: 1000) should be always below 1000*/
			//uint16_t voltage_battery; 	/*< Battery voltage, in millivolts (1 = 1 millivolt)*/
			//int16_t current_battery; 		/*< Battery current, in 10*milliamperes (1 = 10 milliampere), -1: autopilot does not measure the current*/
			//uint16_t drop_rate_comm; 		/*< Communication drops in percent, (0%: 0, 100%: 10'000), (UART, I2C, SPI, CAN), dropped packets on all links (packets that were corrupted on reception on the MAV)*/
			//int8_t battery_remaining; 	/*< Remaining battery energy: (0%: 0, 100%: 100), -1: autopilot estimate the remaining battery*/

			td->voltage = (float)(mavlink_msg_sys_status_get_voltage_battery(&msg) / 1000.0f);
			td->ampere = (float)(mavlink_msg_sys_status_get_current_battery(&msg) * 10.0f / 1000.0f);
			td->battery_remaining = (float)(mavlink_msg_sys_status_get_battery_remaining(&msg));
			
			#ifdef DEBUG
			printf("Voltage:%f\n", battery_v);
			printf("bat rem:%f\n", battery_remaining); 
			printf("Current:%f\n", battery_a);
			#endif
			}
			break;

		case MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT:
			{
			//float nav_roll; 		/*< Current desired roll in degrees*/
			//float nav_pitch; 		/*< Current desired pitch in degrees*/
			//float alt_error; 		/*< Current altitude error in meters*/
			//float aspd_error; 	/*< Current airspeed error in meters/second*/
			//float xtrack_error; 	/*< Current crosstrack error on x-y plane in meters*/
			//int16_t nav_bearing; 	/*< Current desired heading in degrees*/
			//int16_t target_bearing; /*< Bearing to current MISSION/target in degrees*/
			//uint16_t wp_dist; 	/*< Distance to active MISSION in meters*/

			td->wp_target_bearing = mavlink_msg_nav_controller_output_get_target_bearing(&msg);
			td->wp_dist = mavlink_msg_nav_controller_output_get_wp_dist(&msg);
			}
			break;
			
		case MAVLINK_MSG_ID_MISSION_CURRENT:
			{
			//uint16_t seq; /*< Sequence*/
			td->wp_number = (uint8_t)mavlink_msg_mission_current_get_seq(&msg);
			}
			break; 
			
		case MAVLINK_MSG_ID_GPS_RAW_INT:
			{         
			//uint64_t time_usec; /*< Timestamp (microseconds since UNIX epoch or microseconds since system boot)*/
			//int32_t lat; /*< Latitude (WGS84), in degrees * 1E7*/
			//int32_t lon; /*< Longitude (WGS84), in degrees * 1E7*/
			//int32_t alt; /*< Altitude (AMSL, NOT WGS84), in meters * 1000 (positive for up). Note that virtually all GPS modules provide the AMSL altitude in addition to the WGS84 altitude.*/
			//uint16_t eph; /*< GPS HDOP horizontal dilution of position (unitless). If unknown, set to: UINT16_MAX*/
			//uint16_t epv; /*< GPS VDOP vertical dilution of position (unitless). If unknown, set to: UINT16_MAX*/
			//uint16_t vel; /*< GPS ground speed (m/s * 100). If unknown, set to: UINT16_MAX*/
			//uint16_t cog; /*< Course over ground (NOT heading, but direction of movement) in degrees * 100, 0.0..359.99 degrees. If unknown, set to: UINT16_MAX*/
			//uint8_t fix_type; /*< 0-1: no fix, 2: 2D fix, 3: 3D fix, 4: DGPS, 5: RTK. Some applications will not use the value of this field unless it is at least two, so always correctly fill in the fix.*/
			//uint8_t satellites_visible; /*< Number of satellites visible. If unknown, set to 255*/

			td->latitude = (float)mavlink_msg_gps_raw_int_get_lat(&msg) / 10000000.0f;
			td->longitude = (float)mavlink_msg_gps_raw_int_get_lon(&msg) / 10000000.0f;
			td->gps_alt = (float)mavlink_msg_gps_raw_int_get_alt(&msg) / 10000000.0f;
			//td->gps_speed = (float)mavlink_msg_gps_raw_int_get_vel(&msg);
			td->hdop = (float)mavlink_msg_gps_raw_int_get_eph(&msg);
			td->fix_type = mavlink_msg_gps_raw_int_get_fix_type(&msg);
			td->sats = mavlink_msg_gps_raw_int_get_satellites_visible(&msg);
		    }
			break;
			
		case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
			{
			//uint32_t time_boot_ms; 	/*< Timestamp (milliseconds since system boot)*/
			//int32_t lat; 			/*< Latitude, expressed as degrees * 1E7*/
			//int32_t lon; 			/*< Longitude, expressed as degrees * 1E7*/
			//int32_t alt; 			/*< Altitude in meters, expressed as * 1000 (millimeters)*/
			//int32_t relative_alt; 	/*< Altitude above ground in meters, expressed as * 1000 (millimeters)*/
			//int16_t vx; 			/*< Ground X Speed (Latitude, positive north), expressed as m/s * 100*/
			//int16_t vy; 			/*< Ground Y Speed (Longitude, positive east), expressed as m/s * 100*/
			//int16_t vz; 			/*< Ground Z Speed (Altitude, positive down), expressed as m/s * 100*/
			//uint16_t hdg; 			/*< Vehicle heading (yaw angle) in degrees * 100, 0.0..359.99 degrees. If unknown, set to: UINT16_MAX*/
			
			//time_boot_ms = mavlink_msg_global_position_int_get_time_boot_ms(&msg);
			//lat = mavlink_msg_global_position_int_get_lat(&msg);
			//lon = mavlink_msg_global_position_int_get_lon(&msg);
			//alt = mavlink_msg_global_position_int_get_alt(&msg);
			//relative_alt = mavlink_msg_global_position_int_get_relative_alt(&msg);
			//vx = mavlink_msg_global_position_int_get_vx(&msg);
			//vy = mavlink_msg_global_position_int_get_vy(&msg);
			//vz = mavlink_msg_global_position_int_get_vz(&msg);
			//hdg = mavlink_msg_global_position_int_get_hdg(&msg);
			
			}
			break;
			
 		case MAVLINK_MSG_ID_RC_CHANNELS:
			{
			//uint32_t time_boot_ms; /*< Timestamp (milliseconds since system boot)*/
			//uint16_t chan1_raw; /*< RC channel 1 value, in microseconds. A value of UINT16_MAX implies the channel is unused.*/
			//...
			//uint16_t chan18_raw; /*< RC channel 18 value, in microseconds. A value of UINT16_MAX implies the channel is unused.*/
			//uint8_t chancount; /*< Total number of RC channels being received. This can be larger than 18, 
			//	indicating that more channels are available but not given in this message. This value should be 0 when no RC channels are available.*/
			//uint8_t rssi; /*< Receive signal strength indicator, 0: 0%, 100: 100%, 255: invalid/unknown.*/
			float rc_channel_rssi = mavlink_msg_rc_channels_get_rssi(&msg);
			if(rc_channel_rssi != 255)
				td->rc_rssi = rc_channel_rssi;
			}
			break; 
			
		case MAVLINK_MSG_ID_VFR_HUD:
			{
			//float airspeed; /*< Current airspeed in m/s*/
			//float groundspeed; /*< Current ground speed in m/s*/
			//float alt; /*< Current altitude (MSL), in meters*/
			//float climb; /*< Current climb rate in meters/second*/
			//int16_t heading; /*< Current heading in degrees, in compass units (0..360, 0=north)*/
			//uint16_t throttle; /*< Current throttle setting in integer percent, 0 to 100*/

			td->airspeed = mavlink_msg_vfr_hud_get_airspeed(&msg);
			td->speed = mavlink_msg_vfr_hud_get_groundspeed(&msg);
			td->altitude = mavlink_msg_vfr_hud_get_alt(&msg);
			td->throttle = (uint8_t)mavlink_msg_vfr_hud_get_throttle(&msg);
			td->climb = mavlink_msg_vfr_hud_get_climb(&msg); 
			td->heading = mavlink_msg_vfr_hud_get_heading(&msg); // 0..360 deg, 0=north
			}
			break;
			
		case MAVLINK_MSG_ID_HOME_POSITION:
			{
			//int32_t latitude; 	/*< Latitude (WGS84), in degrees * 1E7*/
			//int32_t longitude; 	/*< Longitude (WGS84, in degrees * 1E7*/
			//int32_t altitude; 	/*< Altitude (AMSL), in meters * 1000 (positive for up)*/
			//float x; 				/*< Local X position of this position in the local coordinate frame*/
			//float y; 				/*< Local Y position of this position in the local coordinate frame*/
			//float z; 				/*< Local Z position of this position in the local coordinate frame*/
			//float q[4]; 			/*< World to surface normal and heading transformation of the takeoff position. 
									/*Used to indicate the heading and slope of the ground*/
			//float approach_x; 	/*< Local X position of the end of the approach vector. */
			//float approach_y; 	/*< Local Y position of the end of the approach vector. */
			//float approach_z; 	/*< Local Z position of the end of the approach vector. */
									/* Multicopters should set this position based on their takeoff path. 
									Grass-landing fixed wing aircraft should set it the same way as multicopters. 
									Runway-landing fixed wing aircraft should set it to the opposite direction of the takeoff, 
									assuming the takeoff happened from the threshold / touchdown zone.*/

			td->home_lat = (float)mavlink_msg_home_position_get_latitude(&msg) / 10000000.0f;
			td->home_lon = (float)mavlink_msg_home_position_get_longitude(&msg) / 10000000.0f;
			td->home_alt = (float)mavlink_msg_home_position_get_altitude(&msg)/ 10000000.0f;
			td->home_set = true;
			}
			break;
			
		case MAVLINK_MSG_ID_ALTITUDE:
			{      
			//uint64_t time_usec; /*< Timestamp (milliseconds since system boot)*/
			//float altitude_monotonic; /*< This altitude measure is initialized on system boot and monotonic (it is never reset, but represents the local altitude change). The only guarantee on this field is that it will never be reset and is consistent within a flight. The recommended value for this field is the uncorrected barometric altitude at boot time. This altitude will also drift and vary between flights.*/
			//float altitude_amsl; /*< This altitude measure is strictly above mean sea level and might be non-monotonic (it might reset on events like GPS lock or when a new QNH value is set). It should be the altitude to which global altitude waypoints are compared to. Note that it is *not* the GPS altitude, however, most GPS modules already output AMSL by default and not the WGS84 altitude.*/
			//float altitude_local; /*< This is the local altitude in the local coordinate frame. It is not the altitude above home, but in reference to the coordinate origin (0, 0, 0). It is up-positive.*/
			//float altitude_relative; /*< This is the altitude above the home position. It resets on each change of the current home position.*/
			//float altitude_terrain; /*< This is the altitude above terrain. It might be fed by a terrain database or an altimeter. Values smaller than -1000 should be interpreted as unknown.*/
			//float bottom_clearance; /*< This is not the altitude, but the clear space below the system according to the fused clearance estimate. It generally should max out at the maximum range of e.g. the laser altimeter. It is generally a moving target. A negative value indicates no measurement available.*/
			
			float terrain_altitude = mavlink_msg_altitude_get_altitude_terrain(&msg);
			td->vehicle_clearance = mavlink_msg_altitude_get_bottom_clearance(&msg);
			}
			break;
			
		case MAVLINK_MSG_ID_ATTITUDE:
			{      
			//uint32_t time_boot_ms; /*< Timestamp (milliseconds since system boot)*/
			//float roll; /*< Roll angle (rad, -pi..+pi)*/
			//float pitch; /*< Pitch angle (rad, -pi..+pi)*/
			//float yaw; /*< Yaw angle (rad, -pi..+pi)*/
			//float rollspeed; /*< Roll angular speed (rad/s)*/
			//float pitchspeed; /*< Pitch angular speed (rad/s)*/
			//float yawspeed; /*< Yaw angular speed (rad/s)*/
			
			float pitch = ToDeg(mavlink_msg_attitude_get_pitch(&msg));
			if(pitch!=0)
				td->pitch = pitch;
			float roll = ToDeg(mavlink_msg_attitude_get_roll(&msg));
			if(roll!=0)
				td->roll=roll;
			}
			break;
			
		case MAVLINK_MSG_ID_ADSB_VEHICLE:
			{            
			//uint32_t ICAO_address; /*< ICAO address*/
			//int32_t lat; /*< Latitude, expressed as degrees * 1E7*/
			//int32_t lon; /*< Longitude, expressed as degrees * 1E7*/
			//int32_t altitude; /*< Altitude(ASL) in millimeters*/
			//uint16_t heading; /*< Course over ground in centidegrees*/
			//uint16_t hor_velocity; /*< The horizontal velocity in centimeters/second*/
			//int16_t ver_velocity; /*< The vertical velocity in centimeters/second, positive is up*/
			//uint16_t flags; /*< Flags to indicate various statuses including valid data fields*/
			//uint16_t squawk; /*< Squawk code*/
			//uint8_t altitude_type; /*< Type from ADSB_ALTITUDE_TYPE enum*/
			//char callsign[9]; /*< The callsign, 8+null*/
			//uint8_t emitter_type; /*< Type from ADSB_EMITTER_TYPE enum*/
			//uint8_t tslc; /*< Time since last communication in seconds*/

			mavlink_msg_adsb_vehicle_get_callsign(&msg, td->callsign);
			}
			break;
			
		default:
			{
			//printf("unknown mavlink msgid:%d\n",msg.msgid);
			break;
			}
		}
   }
  }	 
return new_data;
}

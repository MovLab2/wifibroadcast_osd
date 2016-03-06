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
			/*< System mode bitfield, see MAV_MODE_FLAG ENUM in mavlink/include/mavlink_types.h*/
			uint8_t base_mode = (uint8_t)mavlink_msg_heartbeat_get_base_mode(&msg);
			/*< A bitfield for use for autopilot-specific flags.*/
			uint8_t custom_mode = (uint8_t)mavlink_msg_heartbeat_get_custom_mode(&msg);
			uint8_t motor_armed = (base_mode >> 7) & 0x01; //get bit 7
			td->flight_mode = custom_mode;
			
			#ifdef DEBUG
			printf("mavlink_parse.c: MAVLINK_MSG_ID_HEARTBEAT\ncustom mode=%d\n",custom_mode);
			printf("base mode=%d\n",base_mode); 
			printf("flight mode=%d\n",td->flight_mode);
			printf("td->armed=%d\n",td->armed);
			printf("motor_armed=%d\n\n",motor_armed);
			#endif
			
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
			break;		
        }
          
	   case MAVLINK_MSG_ID_SYS_STATUS:
			{
			float battery_v = (mavlink_msg_sys_status_get_voltage_battery(&msg) / 1000.0f);
			float battery_a = (mavlink_msg_sys_status_get_current_battery(&msg) * 10.0f / 1000.0f);
			td->voltage = battery_v;
			td->ampere = battery_a;
			
			float battery_remaining = (mavlink_msg_sys_status_get_battery_remaining(&msg));
			td->battery_remaining = battery_remaining;
			
			//printf("Voltage:%f\n", battery_v);
			//printf("bat rem:%f\n", battery_remaining); 
			//printf("Current:%f\n", battery_a);
			break;
			}
		case MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT:
			{
			td->wp_target_bearing = mavlink_msg_nav_controller_output_get_target_bearing(&msg);
			td->wp_dist = mavlink_msg_nav_controller_output_get_wp_dist(&msg);
			}
			break;

		case MAVLINK_MSG_ID_MISSION_CURRENT:
			{
			td->wp_number = (uint8_t)mavlink_msg_mission_current_get_seq(&msg);
			}
			break; 

		case MAVLINK_MSG_ID_GPS_RAW_INT:
			{         
			float lat = mavlink_msg_gps_raw_int_get_lat(&msg) / 10000000.0f;
			float lng = mavlink_msg_gps_raw_int_get_lon(&msg) / 10000000.0f;
			float alt = mavlink_msg_gps_raw_int_get_alt(&msg) / 10000000.0f;
			td->longitude = lng;
			td->latitude = lat;
			td->gps_alt = alt;
			td->hdop = mavlink_msg_gps_raw_int_get_eph(&msg);
			td->sats = mavlink_msg_gps_raw_int_get_satellites_visible(&msg);
			td->fix_type = mavlink_msg_gps_raw_int_get_fix_type(&msg);
			//printf("fix:%s\n", td->fix_type);
		    }
			  break;
		case MAVLINK_MSG_ID_VFR_HUD:
			{
			//printf("vfr hud\n"); 
			float alt = mavlink_msg_vfr_hud_get_alt(&msg);
			float groundspeed = mavlink_msg_vfr_hud_get_groundspeed(&msg);
			td->altitude = alt;
			td->speed = groundspeed;
			td->throttle = (uint8_t)mavlink_msg_vfr_hud_get_throttle(&msg);
			td->climb = mavlink_msg_vfr_hud_get_climb(&msg); 
			td->heading = mavlink_msg_vfr_hud_get_heading(&msg); // 0..360 deg, 0=north
			}
			  break;

		//case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
		//	{
		//	td->home_alt = td->alt - (mavlink_msg_global_position_int_get_relative_alt(&msg)*0.001);
		//	} 

		case MAVLINK_MSG_ID_HOME_POSITION:
			{
			float lat = mavlink_msg_home_position_get_latitude(&msg) / 10000000.0f;
			float lng = mavlink_msg_home_position_get_longitude(&msg) / 10000000.0f;
			float alt = mavlink_msg_home_position_get_altitude(&msg) / 10000000.0f;
			td->home_lat = lat;
			td->home_lon = lng;
			td->home_alt = alt;
			td->home_set = true;
			}
			break;
		case MAVLINK_MSG_ID_ATTITUDE:
			{            
			float pitch = ToDeg(mavlink_msg_attitude_get_pitch(&msg));
			if(pitch!=0)
			td->pitch = pitch;
			float roll = ToDeg(mavlink_msg_attitude_get_roll(&msg));
			if(roll!=0)
			td->roll=roll;
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

uint8_t get_bit(uint8_t bits, uint8_t pos)
{
   return (bits >> pos) & 0x01;
}
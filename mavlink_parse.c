#include <stdio.h>
#include <stdint.h>  
#include "osdconfig.h"
#include "telemetry.h"
#include "mavlink_parse.h"
#include "mavlink.h"
#include <math.h>

#include <unistd.h>
#include <stdlib.h>


void setHomeVars(telemetry_data_t *td) {
  float dstlon, dstlat; 
  long bearing;
 
  if(td->home_set == false && td->fix_type > 1){
    td->home_lat = td->latitude;
    td->home_lon = td->longitude;
    td->home_set = true;
	//printf("set home\n");
	//printf("home_lat:%f\n",td->home_lat);
	//printf("home_lon:%f\n",td->home_lon);
  }
  else if(td->home_set){
  // JRChange: osd_home_alt: check for stable osd_alt (must be stable for 25*120ms = 3s)
    if(td->osd_alt_cnt < 25){
      if(fabs(td->osd_alt_prev - td->osd_alt) > 0.5){
        td->osd_alt_cnt = 0;
        td->osd_alt_prev = td->osd_alt;
      }
      else
      {
        if(++td->osd_alt_cnt >= 25){
          td->osd_home_alt = td->osd_alt;  // take this stable osd_alt as osd_home_alt
          td->haltset = 1;
        }
      }
    }
    // shrinking factor for longitude going to poles direction
    float rads = fabs(td->home_lat) * 0.0174532925;
    double scaleLongDown = cos(rads);
    double scaleLongUp   = 1.0f/cos(rads);

    //DST to Home
    dstlat = fabs(td->home_lat - td->latitude) * 111319.5;
    dstlon = fabs(td->home_lon - td->longitude) * 111319.5 * scaleLongDown;
    td->osd_home_distance = sqrt(pow(dstlat,2) + pow(dstlon,2));
	
    //DIR to Home
    dstlon = (td->home_lon - td->longitude); //OffSet_X
    dstlat = (td->home_lat - td->latitude) * scaleLongUp; //OffSet Y
    bearing = 90 + (atan2(dstlat, -dstlon) * 57.295775); //absolut home direction
    if(bearing < 0) bearing += 360;		//normalization
    bearing = bearing - 180;			//absolut return direction
    if(bearing < 0) bearing += 360;		//normalization
    bearing = bearing - td->heading;	//relative home direction
    if(bearing < 0) bearing += 360; 	//normalization
 
    td->osd_home_direction = round((float)(bearing/360.0f) * 16.0f) + 1;//array of arrows =)
 
    if(td->osd_home_direction > 16) td->osd_home_direction = 0;
  }
} 

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
			//printf("heartbeat\n");           
			uint8_t base_mode = (uint8_t)mavlink_msg_heartbeat_get_base_mode(&msg);
			uint8_t custom_mode = mavlink_msg_heartbeat_get_custom_mode(&msg);
			//printf("custom mode=%d\n",custom_mode);
			//printf("base mode=%d\n",base_mode); 
	
			switch(custom_mode)
			{
			case 0:	
					{
					td->mode="MANUAL";
					break;
					}
			case 1:
							{
							td->mode="CIRCLE";
							break;
							}
			case 2:
							{
							td->mode="STABILIZE";
							break;
							}
			case 3:
							{
							td->mode="TRAINING";
							break;
							}
			case 4:
							{
							td->mode="ACRO";
							break;
							}
			case 5:
							{
							td->mode="FBWA";
							break;
							}
			case 6:
							{
							td->mode="FBWB";
							break;
							}
			case 7:
							{
							td->mode="CRUISE";
							break;
							}
			case 8:
							{
							td->mode="AUTOTUNE";
							break;
							}
			case 10:
							{
							td->mode="AUTO";
							break;
							}
			case 11:
							{
							td->mode="RTL";
							break;
							}
			case 12:
							{
							td->mode="LOITER";
							break;
							}
			case 14:
							{
							td->mode="LAND";
							break;
							}
			case 15:
							{
							td->mode="GUIDED";
							break;
							}
			case 16:
							{
							td->mode="INITIALISING";
							break;
							}

			}

        uint8_t motor_armed = 0;//getBit(base_mode,7);
		
		if(td->armed == 0 && motor_armed) {
			td->start_lat = td->latitude;
			td->start_lng = td->longitude;
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
		float battery_remaining =(mavlink_msg_sys_status_get_battery_remaining(&msg));
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
		//printf("gps raw init\n");           
		float lat = mavlink_msg_gps_raw_int_get_lat(&msg) / 10000000.0f;
		float lng = mavlink_msg_gps_raw_int_get_lon(&msg) / 10000000.0f;
		td->fix_type = mavlink_msg_gps_raw_int_get_fix_type(&msg);
				
		switch (td->fix_type)
			{
			case 0:
			td->osd_fix_type="no fix";
			break;
			case 1:
			td->osd_fix_type="poor fix";
			break;
			case 2:
			td->osd_fix_type="2D fix";
			break;
			case 3:
			td->osd_fix_type="3D fix";
			break;
			default:
			td->osd_fix_type="";
		

			}
		//printf("fix:%s\n", td->osd_fix_type);
        td->sats = mavlink_msg_gps_raw_int_get_satellites_visible(&msg);
        td->longitude = lng;
		td->latitude = lat;
  	  }
          break;
    case MAVLINK_MSG_ID_VFR_HUD:
          {
		//printf("vfr hud\n");           
		float groundspeed = mavlink_msg_vfr_hud_get_groundspeed(&msg);
        float alt = mavlink_msg_vfr_hud_get_alt(&msg);
        td->altitude = alt;
		td->speed = groundspeed;
        td->throttle = (uint8_t)mavlink_msg_vfr_hud_get_throttle(&msg);
        td->climb = mavlink_msg_vfr_hud_get_climb(&msg); 
 		td->heading = mavlink_msg_vfr_hud_get_heading(&msg); // 0..360 deg, 0=north
	  }
          break;

	case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
        {
        td->home_alt = td->alt - (mavlink_msg_global_position_int_get_relative_alt(&msg)*0.001);
        } 

    case MAVLINK_MSG_ID_ATTITUDE:
        {
		//printf("attitude\n");            
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
setHomeVars(td); 
return new_data;
}



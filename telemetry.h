#pragma once
#include <stdbool.h>
#include <stdint.h>
#include <time.h>
#include "osdconfig.h"

typedef struct {
        uint32_t received_packet_cnt;
        uint32_t wrong_crc_cnt;
        int8_t current_signal_dbm;
} wifi_adapter_rx_status_t;

typedef struct {
        time_t last_update;
        uint32_t received_block_cnt;
        uint32_t damaged_block_cnt;
        uint32_t tx_restart_cnt;
        uint32_t wifi_adapter_cnt;
        wifi_adapter_rx_status_t adapter[8];
} wifibroadcast_rx_status_t;

typedef struct {
	float voltage;
	float ampere;
	float baro_altitude;
	float altitude;
	double longitude;
	double latitude;
	float heading;
	float speed;
	int16_t x, y, z;
	int16_t ew, ns;
	bool setting_home;
	bool home_set;
	float home_lat;
	float home_lon;
	float home_alt;
#ifdef LTM
	int16_t roll, pitch;
	uint8_t rssi;
	uint8_t airspeed;
	uint8_t sats;
	uint8_t fix;
#endif
#ifdef MAVLINK
	float osd_alt; 
	uint8_t fix_type;
	uint8_t osd_alt_cnt;		
	char* osd_fix_type;  	
	float osd_home_direction;  
	float osd_home_distance;
	float osd_home_alt;
	float osd_alt_prev;
	char* mode;
	double start_lat;
	double start_lng;
	float pitch;
	float roll;
	float battery_remaining;
	float throttle;
	float climb;
	float lat,lng,alt;
	uint8_t armed;
	uint8_t manual;
    uint8_t hil_enable;
	uint8_t stabilized;
	uint8_t guided;
	uint8_t aut;
	uint16_t wp_target_bearing;
	uint16_t wp_dist;
	uint16_t wp_number;
	uint8_t sats;
	bool haltset;
	char* callsign;
#endif
	wifibroadcast_rx_status_t *rx_status;	
} telemetry_data_t;

wifibroadcast_rx_status_t *telemetry_wbc_status_memory_open(void);

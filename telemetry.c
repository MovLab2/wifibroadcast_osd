#include <unistd.h>
#include <sys/mman.h>
#include <sys/stat.h>        /* For mode constants */
#include <fcntl.h>           /* For O_* constants */
#include <stdio.h>
#include <stdlib.h>
#include "telemetry.h"
#include "osdconfig.h"

void telemetry_init(telemetry_data_t *td, int rx_port) {
	td->voltage = 0;
	td->ampere = 0;
	td->cells = CELLS;
	td->altitude = 0;
	td->longitude = 0;
	td->latitude = 0;
	td->heading = 0;
	td->speed = 0;
	td->x = 0;
	td->y = 0;
	td->z = 0;
	td->ew = 0;
	td->ns = 0;
#ifdef LTM
	td->roll = 0;
	td->pitch = 0;
	td->rssi = 0;
	td->airspeed = 0;
	td->sats = 0;
	td->fix = 0;
#endif
#ifdef MAVLINK
	td->pitch = 0;
	td->roll = 0;
	td->sats = 0;
	td->gps_alt = 0;
	td->vehicle_clearance = 0;
	td->airspeed = 0;
	td->throttle = 0;
	td->climb = 0;
	td->flight_mode = "Flight Mode";
	td->armed = 0;
	td->battery_remaining = 0;
	td->rc_rssi = 0;
	td->wp_lat = 0;
	td->wp_lng = 0;
	td->wp_alt = 0;
	td->home_lat = 0;
	td->home_lon = 0;
	td->home_set = false;
	td->message_pending = false;
	td->message = "Loading...";
	td->callsign = "HAM CALLSIGN";
#endif
	//td->rx_port = 0;
	td->rx_status = telemetry_wbc_status_memory_open(rx_port);
}

wifibroadcast_rx_status_t *telemetry_wbc_status_memory_open(int rx_port) {
	//sprintf(wifibroadcast_rx_status,"/wifibroadcast_rx_status_%d", rx_port);
	//printf("WiFiBroadcast shm_open=%s\n", wifibroadcast_rx_status);
	#ifdef DEBUG
	printf("wifibroadcast_rx_status_t() RX port=%d\n", rx_port);
	#endif
	int fd = shm_open("/wifibroadcast_rx_status_0", O_RDWR, S_IRUSR | S_IWUSR);
	switch(rx_port)
	{
		case 1:
			fd = shm_open("/wifibroadcast_rx_status_1", O_RDWR, S_IRUSR | S_IWUSR);
			break;
		case 2:
			fd = shm_open("/wifibroadcast_rx_status_2", O_RDWR, S_IRUSR | S_IWUSR);
			break;	
		case 3:
			fd = shm_open("/wifibroadcast_rx_status_3", O_RDWR, S_IRUSR | S_IWUSR);
			break;	
		case 4:
			fd = shm_open("/wifibroadcast_rx_status_4", O_RDWR, S_IRUSR | S_IWUSR);
			break;	
		case 5:
			fd = shm_open("/wifibroadcast_rx_status_5", O_RDWR, S_IRUSR | S_IWUSR);
			break;
		case 6:
			fd = shm_open("/wifibroadcast_rx_status_6", O_RDWR, S_IRUSR | S_IWUSR);
			break;	
		case 7:
			fd = shm_open("/wifibroadcast_rx_status_7", O_RDWR, S_IRUSR | S_IWUSR);
			break;	
		case 8:
			fd = shm_open("/wifibroadcast_rx_status_8", O_RDWR, S_IRUSR | S_IWUSR);
			break;	
		case 9:
			fd = shm_open("/wifibroadcast_rx_status_9", O_RDWR, S_IRUSR | S_IWUSR);
			break;	
		default:
			fd = shm_open("/wifibroadcast_rx_status_0", O_RDWR, S_IRUSR | S_IWUSR);
			break;
	}
	
	if(fd < 0) {
			fprintf(stderr, "ERROR: Could not open wifibroadcast rx port %d!\n", rx_port);
		return NULL;
	}

	if (ftruncate(fd, sizeof(wifibroadcast_rx_status_t)) == -1) {
		perror("ftruncate");
		exit(1);
	}

	void *retval = mmap(NULL, sizeof(wifibroadcast_rx_status_t), PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
	if (retval == MAP_FAILED) {
		perror("mmap");
		exit(1);
	}
	#ifdef DEBUG
	printf("wifibroadcast_rx_status_t() RX port=%d is open!\n", rx_port);
	#endif
	return (wifibroadcast_rx_status_t*)retval;
}


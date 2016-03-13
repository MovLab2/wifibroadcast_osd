#pragma once
#include "bcm_host.h"
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include "VG/openvg.h"
#include "VG/vgu.h"
#include "fontinfo.h"
#include "shapes.h"
#include <math.h>
#include <EGL/egl.h>
#include <EGL/eglext.h>
#include <stdio.h>
#include "telemetry.h"
#include <errno.h>
#include <resolv.h>
#include <string.h>
#include <utime.h>
#include <unistd.h>
#include <getopt.h>
#include <pcap.h>
#include <endian.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <stdbool.h>

#define TO_DEG 180.0f / M_PI

void render_init();
void render(telemetry_data_t *td, int cells, bool verbose);

//rotate a polyline/polygon with this
void rotatePoints(float *x, float *y, int angle, int points, int center_x, int center_y);

void paintArrow(int heading, int pos_x, int pos_y);
void paintAHI(int hor_angle, int ver_angle);
void draw_signal(int8_t signal, int package_rssi, int pos_x, int pos_y, float scale);
float distance_between(float lat1, float long1, float lat2, float long2);
float course_to (float lat1, float long1, float lat2, float long2);

//new stuff from fritz walter https://www.youtube.com/watch?v=EQ01b3aJ-rk
//this will only indicate how much % are left. Mavlink specific, but could be used with others as well.
void draw_bat_remaining(int remaining, int pos_x, int pos_y, float scale);
void draw_compass(int heading, int pos_x, int pos_y, bool ladder_enabled, float scale);
void draw_bat_status(float voltage, float current, int pos_x, int pos_y, float scale);
void draw_sat(int sats, int fixtype, int pos_x, int pos_y, float scale);
void draw_position(float lat, float lon, int fix_type, int sats, int pos_x, int pos_y, float scale);
void draw_home_distance(int distance, int pos_x, int pos_y, float scale);
//autopilot mode, mavlink specific, could be used if mode is in telemetry data of other protocols as well
void draw_flight_mode(char* flight_mode, int pos_x, int pos_y, float scale);
void draw_message(char* msg, int pos_x, int pos_y, float scale);
void draw_home_indicator(int home_angle, int pos_x, int pos_y, float scale);
void draw_altitude(int alt, int pos_x, int pos_y, bool ladder_enabled, float scale);
void draw_speed(int speed, int pos_x, int pos_y, bool ladder_enabled, float scale);
//ladder here means the additional lines of the AHI, if true all lines will be drawn, if false only the main line
void draw_horizon(float roll, float pitch, int pos_x, int pos_y, float scale);

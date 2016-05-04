/*
Copyright (c) 2015, befinitiv
Copyright (c) 2012, Broadcom Europe Ltd
modified by Samuel Brucksch https://github.com/SamuelBrucksch/wifibroadcast_osd

All rights reserved.
Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
* Redistributions of source code must retain the above copyright
notice, this list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright
notice, this list of conditions and the following disclaimer in the
documentation and/or other materials provided with the distribution.
* Neither the name of the copyright holder nor the
names of its contributors may be used to endorse or promote products
derived from this software without specific prior written permission.
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CA

#include <stdio.h>ND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

//#define DEBUG
#include "osdconfig.h"
#include <ctype.h>
#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <sys/time.h>
#include "telemetry.h"
#ifdef FRSKY
#include "frsky.h"
#elif defined(LTM)
#include "ltm.h"
#elif defined(MAVLINK)
#include "mavlink_parse.h"
#endif
#include "render.h"
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/select.h>

fd_set set;
struct timeval timeout;

long long current_timestamp() {
    struct timeval te; 
    gettimeofday(&te, NULL); // get current time
    long long milliseconds = te.tv_sec*1000LL + te.tv_usec/1000; // caculate milliseconds
    return milliseconds;
}

int main (int argc, char **argv) {
	uint8_t buf[1024];
	size_t n;
	int c;
	int digit_optind = 0;
	char* exec = argv[0];
	int rx_port = 0;
	int cells = 3;
	bool verbose = false;
	
   while (1) {
        int this_option_optind = optind ? optind : 1;
        int option_index = 0;
        static struct option long_options[] = {
            {"port",    required_argument, NULL,  'p' },
            {"cells", 	required_argument, NULL,  'c' },
			{"verbose", no_argument, 0,  'v' },
			{"help",    no_argument, 0,  'h' },
            {NULL, 0, NULL,  0 }
        };

		c = getopt_long(argc, argv, "hvp:c:", long_options, &option_index);
        if (c == -1)
            break;

		switch (c) {

			case 'v':
				verbose = true;
				break;

		   case 'p':
				rx_port = atoi(optarg);
				if (rx_port < 0 || rx_port > 9) {
					fprintf(stderr, "ERROR: -p invalid RX port value of %d!\n\n", rx_port);
					print_usage(exec);
					exit(-1);
				}
				break;
				
			case 'c':
				cells = atoi(optarg);
				if (cells < 0 || cells > 9) {
					fprintf(stderr, "ERROR: -c invalid battery cells value of %d!\n\n", cells);
					print_usage(exec);
					exit(-1);
				}
				break;

		   case 'h':
				print_usage(exec);
				exit(-1);
				break;

		   default:
				fprintf(stderr, "Usage: %s [-pchv]\n", exec);
				exit(-1);
		}
    }

	//=======================================================
	if(verbose) {
		printf("%s Starting....\n\n",exec);
		printf("Battery Cells:%d\n", cells);
		printf("WiFiBroadcast RX port:%d\n", rx_port);
		printf("\n");
	}
	
	telemetry_data_t td;
	telemetry_init(&td,rx_port);
		
#ifdef FRSKY
	frsky_state_t fs;
#endif
	render_init();
	#ifdef DEBUG
	long long prev_time = current_timestamp();
	#endif
#ifdef MAVLINK	
	if(verbose) {
		printf("Render_ini() started...\nWaiting for Mavlink packets.\n");
	}	
#endif	
	while(1) {
		FD_ZERO(&set);
		FD_SET(STDIN_FILENO, &set);
		timeout.tv_sec = 0;
		timeout.tv_usec = 100*1000;
		n = select(STDIN_FILENO + 1, &set, NULL, NULL, &timeout);
		if(n > 0) {
			n = read(STDIN_FILENO, buf, sizeof(buf));
			if(n == 0) {
			}
			if(n<0) {
				perror("read");
				exit(-1);
			}
#ifdef FRSKY
			frsky_parse_buffer(&fs, &td, buf, n);
#elif defined(LTM)
			ltm_read(&td, buf, n);
#elif defined(MAVLINK)
			mavlink_parse_buffer(&td, buf, n);
#endif
		}
	#ifdef DEBUG
		prev_time = current_timestamp();
		printf("Sending %d packets to render()\n",n);
		render(&td,cells,verbose);
		long long took = current_timestamp() - prev_time;
		printf("Decode and render took %lldms to execute.\n", took);
	#else
		if(verbose && n > 0)
			printf("Sending %d packets to render()\n",n);
		render(&td,cells,verbose);
	#endif
	}
	return 0;
}

void print_usage(char* cmd){
	printf("Usage: cat /dev/mavlink0 | %s [-pchv]\n",cmd);
	printf("  --help -h		This help information.\n");
	printf("  --verbose -v		Print information to stdout.\n");
	printf("  --port -p [0-9]	WiFiBroadcast Port for monitoring RX signal.\n");
	printf("  --cells -c [2-6]	Number of battery cells for battery meter.\n\n");
}



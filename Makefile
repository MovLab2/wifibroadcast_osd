CPPFLAGS+= -I/opt/vc/include/ -I/opt/vc/include/interface/vcos/pthreads -I/opt/vc/include/interface/vmcs_host/linux -I/usr/src/mavlink/common
LDFLAGS+= -lfreetype -lz
LDFLAGS+=-L/opt/vc/lib/ -lGLESv2 -lEGL -lopenmaxil -lbcm_host -lvcos -lvchiq_arm -lpthread -lrt -lm -lshapes

all: mavlink_osd
	
%.o: %.c
	gcc -c -o $@ $< $(CPPFLAGS)
 

frysky_osd: main.o frsky.o render.o telemetry.o
	gcc -o $@ $^ $(LDFLAGS)
	chmod 755 $@
	
mavlink_osd: main.o render.o telemetry.o mavlink_parse.o
	gcc -o $@ $^ $(LDFLAGS) 
	chmod 755 $@
	
ltm_osd:main.o render.o telemetry.o ltm.o 
	gcc -o $@ $^ $(LDFLAGS)
	chmod 755 $@

install:
	cp *_osd /usr/bin

clean:
	rm -f frysky_osd mavlink_osd ltm_osd *.o *~


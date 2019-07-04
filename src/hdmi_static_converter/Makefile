CXX = g++
CFLAGS = -g -O0 -Wall -std=c++14 -I. -I$(shell pg_config --includedir)
CXXFLAGS = -g -O0 -Wall -std=c++14 -I. -I$(shell pg_config --includedir)
LDFLAGS = -L/usr/local/lib -L$(shell pg_config --libdir)
LIBS = -lm -lpq -lis
OBJS = commonlib.o dmp_file.o adas_file.o dmp_to_adas.o hdmi_static_converter.o

hdmi_static_converter: $(OBJS)
	$(CXX) $(OBJS) $(LDFLAGS) $(LIBS) -o $@

clean:
	rm -f *.o *~ hdmi_static_converter *.log

commonlib.o: commonlib.h
dmp_file.o: dmp_file.h dmp_data.h commonlib.h
adas_file.o: adas_file.h adas_data.h commonlib.h
dmp_to_adas.o: dmp_to_adas.h dmp_file.h dmp_data.h adas_file.h adas_data.h commonlib.h
hdmi_static_converter.o: dmp_to_adas.h dmp_file.h dmp_data.h adas_file.h adas_data.h commonlib.h

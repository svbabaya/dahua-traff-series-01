CXX = arm-linux-gnueabi-g++

PROGS = TraffiXtreamS

COMM = common/
COMM_SERV = common/server/
COMM_TRAF = common/traffix/
COMM_OPENCV_I = common/opencv/include/
COMM_FEATUREDET = common/featureTracker/
COMM_SEGMENTDET = common/segmentTracker/

COMM_OPENCV_L = common/opencv/arm/lib/
COMM_OPENCV_LE = common/opencv/arm/3rdparty/lib/

CFLAGS += -Wall -O2 -Isource -I$(COMM) -I$(COMM_SERV) -I$(COMM_TRAF) -I$(COMM_OPENCV_I) -I$(COMM_FEATUREDET) -I$(COMM_SEGMENTDET) 

LDFLAGS += -lcapture -lpthread -lrt
LDFLAGS += -Wl,-Bstatic -llicensekey_stat -Wl,-Bdynamic
LDFLAGS += -llicensekey

LOPENCV = -L$(COMM_OPENCV_L) -lopencv_features2d -lopencv_flann -lopencv_video -lopencv_imgproc -lopencv_highgui -lopencv_core -L$(COMM_OPENCV_LE) -llibjpeg -lzlib -lrt -lpthread -lm -ldl -lstdc++

SRCS = source/main.cpp $(COMM)signalhandler.cpp $(COMM)capturehandler.cpp $(COMM)globals_cam.cpp $(COMM)datastructs.cpp $(COMM)utils_cam.cpp $(COMM_SERV)tcpServerSource.cpp $(COMM_SERV)tcpServerClientHandle.cpp $(COMM_SERV)tcpServerResponse.cpp $(COMM_SERV)tcpServerResponse_traff.cpp $(COMM_TRAF)utils_traff.cpp $(COMM_TRAF)traffzone.cpp $(COMM_TRAF)traffsensor.cpp $(COMM_TRAF)traffcounter.cpp $(COMM_FEATUREDET)featureTracker.cpp $(COMM_FEATUREDET)featureReg.cpp $(COMM_SEGMENTDET)segmentTracker.cpp

OBJS = $(SRCS:.cpp=.o)

define del_obj
	rm -f source/*.o
	rm -f $(COMM)*.o
	rm -f $(COMM_SERV)*.o
	rm -f $(COMM_TRAF)*.o
 	rm -f $(COMM_FEATUREDET)*.o
 	rm -f $(COMM_SEGMENTDET)*.o
endef

all: $(PROGS)
	$(call del_obj)

$(PROGS): $(OBJS)
	$(CXX) $(LDFLAGS) $^ $(LIBS) $(LDLIBS) $(LOPENCV) -o $@

clean:
	rm -f $(PROGS) *.o core
	rm -f *.tar
	$(call del_obj)

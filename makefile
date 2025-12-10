CXX = arm-himix200-linux-v2-g++

PROGS = TraffiXtreamS

COMM = common/
COMM_SERV = common/server/
COMM_TRAF = common/traffix/
COMM_OPENCV_I = common/opencv/include/
COMM_FEATUREDET = common/featureTracker/
COMM_SEGMENTDET = common/segmentTracker/

COMM_OPENCV_L = common/opencv/arm/lib/
COMM_OPENCV_LE = common/opencv/arm/3rdparty/lib/

ARCH_FLAGS = -mcpu=cortex-a7 -mfloat-abi=hard -mfpu=neon-vfpv4

OPTIM_FLAGS = -O2 -fomit-frame-pointer -ffast-math -ftree-vectorize

CFLAGS += $(ARCH_FLAGS) $(OPTIM_FLAGS)
CFLAGS += -Wall -Wextra -Wno-unknown-pragmas
CFLAGS += -Isource -I$(COMM) -I$(COMM_SERV) -I$(COMM_TRAF) -I$(COMM_OPENCV_I) -I$(COMM_FEATUREDET) -I$(COMM_SEGMENTDET)

CXXFLAGS = $(CFLAGS)
CXXFLAGS += -std=c++11 -fno-rtti -fexceptions

LDFLAGS += $(ARCH_FLAGS) -lpthread -lrt # Put away -lcapture, -llicensekey_stat, -llicensekey

LOPENCV = -L$(COMM_OPENCV_L) -lopencv_features2d -lopencv_flann -lopencv_video -lopencv_imgproc -lopencv_highgui -lopencv_core -L$(COMM_OPENCV_LE) -llibjpeg -lzlib -lrt -lpthread -lm -ldl -lstdc++

SRCS = source/main.cpp $(COMM)signalhandler.cpp $(COMM)capturehandler.cpp $(COMM)globals_cam.cpp $(COMM)datastructs.cpp $(COMM)utils_cam.cpp $(COMM_SERV)tcpServerSource.cpp $(COMM_SERV)tcpServerClientHandle.cpp $(COMM_SERV)tcpServerResponse.cpp $(COMM_SERV)tcpServerResponse_traff.cpp $(COMM_TRAF)utils_traff.cpp $(COMM_TRAF)traffzone.cpp $(COMM_TRAF)traffsensor.cpp $(COMM_TRAF)traffcounter.cpp $(COMM_FEATUREDET)featureTracker.cpp $(COMM_FEATUREDET)featureReg.cpp $(COMM_SEGMENTDET)segmentTracker.cpp

OBJS = $(SRCS:.cpp=.o)

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

define del_obj
	rm -f source/*.o
	rm -f $(COMM)*.o
	rm -f $(COMM_SERV)*.o
	rm -f $(COMM_TRAF)*.o
 	rm -f $(COMM_FEATUREDET)*.o
 	rm -f $(COMM_SEGMENTDET)*.o
endef
    
# Debug mode
debug: $(PROGS)
	$(call del_obj)

# Release mode
release: $(PROGS)
	$(call del_obj)
	arm-himix200-linux-v2-strip --strip-unneeded $(PROGS)

# Make release by default
all: release    

$(PROGS): $(OBJS)
	$(CXX) $(LDFLAGS) $^ $(LIBS) $(LDLIBS) $(LOPENCV) -o $@
    
clean:
	rm -f $(PROGS) *.o core
	rm -f *.tar
	$(call del_obj)

.PHONY: all clean debug release

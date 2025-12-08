#ifndef _CAMERA_GLOBALS_H_
#define _CAMERA_GLOBALS_H_

#include "_common.h"
#include "utils_cam.h"
#include "utils_traff.h"

#include <string>
#include <vector>

#ifdef _ProjCam_
namespace camApp {
	extern pthread_mutex_t global_mutex;
	extern std::string global_gvStr;
	extern CamFileWorker global_files;
	extern bool global_initFlag;
	extern Frame global_frame;
}
#endif

namespace traffixApp {
	extern bool global_clrSensorsFlag;
	extern std::vector<int> global_clrSensIDs;
	extern std::vector<TraffStat> global_sensors;
}

#endif //_CAMERA_GLOBALS_H_

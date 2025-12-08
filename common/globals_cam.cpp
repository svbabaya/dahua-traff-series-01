#include "globals_cam.h"

#ifdef _ProjCam_
namespace camApp {
	pthread_mutex_t global_mutex;
	std::string global_gvStr;
	CamFileWorker global_files;
	bool global_initFlag;
	Frame global_frame;
}
#endif

namespace traffixApp {
	bool global_clrSensorsFlag;
	std::vector<int> global_clrSensIDs;
	std::vector<TraffStat> global_sensors;
}

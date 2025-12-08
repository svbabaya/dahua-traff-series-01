#include "utils_traff.h"
#include "utils_cam.h"

#include <fstream>

// Statistics
TraffStat::TraffStat() {
	clear();
}

void TraffStat::resetStat() {
	time = "";
	counter = speed = 0;
	k = 0;
	occup = 0;
	Cnt0 = Cnt1 = Cnt2 = Cnt3 = Cnt4 = 0;
	Spd0 = Spd1 = Spd2 = Spd3 = Spd4 = 0;
	gap = 0;
	headway = 0;
	negcnt = 0;
	totalCounter = 0;
	carDetected = carDetected_sensor = false;
	curCounter = curSpeed = 0;
	memset(curCnt, 0, sizeof(curCnt));
	curLength = -1.0f;
	curClass = "-";
	lastAvgCounter = 0;
}

void TraffStat::clear() {
	resetStat();
	id = 0;
	period = 0;
	videoOk = 1;
	reserv = " ";
	sensName = "Sensor";
	distanceLengthKm = 0.002f;
}

// Params
std::string TraffAlgParams::algParams2text(const TraffAlgParams &pars) {
	std::string config, stmp;

	config += "BINARIZATION_THRESHOLD=" + i2str(pars.BINARIZATION_THRESHOLD, stmp) + "\n";
	config += "SHADOW_MAX_UPPER_THRESHOLD=" + i2str(pars.SHADOW_MAX_UPPER_THRESHOLD, stmp) + "\n";
	config += "SHADOW_DARK_THRESHOLD=" + i2str(pars.SHADOW_DARK_THRESHOLD, stmp) + "\n";
	config += "SHADOW_RANGE=" + i2str(pars.SHADOW_RANGE, stmp) + "\n";
	config += "LIGHT_BEAM_MAX_THRESHOLD=" + i2str(pars.LIGHT_BEAM_MAX_THRESHOLD, stmp) + "\n";
	config += "SHADOW_MAX_WRONG_POINTS=" + i2str(pars.SHADOW_MAX_WRONG_POINTS, stmp) + "\n";
	config += "LIGHT_BEAM_MAX_WRONG_POINTS=" + i2str(pars.LIGHT_BEAM_MAX_WRONG_POINTS, stmp) + "\n";
	config += "OBJECT_CONFIRMATION_POINTS=" + i2str(pars.OBJECT_CONFIRMATION_POINTS, stmp) + "\n";
	config += "OBJECT_ENTER_POINTS_COUNT=" + i2str(pars.OBJECT_ENTER_POINTS_COUNT, stmp) + "\n";
	config += "OBJECT_EXIT_POINTS_COUNT=" + i2str(pars.OBJECT_EXIT_POINTS_COUNT, stmp) + "\n";
	config += "JAM_CAR_IN_ZONE_TIME_SEC=" + i2str(pars.JAM_CAR_IN_ZONE_TIME_SEC, stmp) + "\n";
	config += "INCREASED_OBJECT_ENTER_POINTS_COUNT=" + i2str(pars.INCREASED_OBJECT_ENTER_POINTS_COUNT, stmp) + "\n";
	config += "INCREASED_OBJECT_EXIT_POINTS_COUNT=" + i2str(pars.INCREASED_OBJECT_EXIT_POINTS_COUNT, stmp) + "\n";
	config += "CAR_EXIT_ENTER_INTERVAL_MSEC=" + i2str(pars.CAR_EXIT_ENTER_INTERVAL_MSEC, stmp) + "\n";
	config += "CATCH_SLOW_MOTION_POINTS_COUNT=" + i2str(pars.CATCH_SLOW_MOTION_POINTS_COUNT, stmp) + "\n";
	config += "CATCH_MOTION_POINTS_COUNT=" + i2str(pars.CATCH_MOTION_POINTS_COUNT, stmp) + "\n";
	config += "CATCH_FRAME_MOTION_REPEAT_COUNT=" + i2str(pars.CATCH_FRAME_MOTION_REPEAT_COUNT, stmp) + "\n";
	config += "CATCH_INTER_ZONE_FRAME_DIFFERENCE_JUMP_TIME_MSEC=" + i2str(pars.CATCH_INTER_ZONE_FRAME_DIFFERENCE_JUMP_TIME_MSEC, stmp) + "\n";
	config += "CATCH_NO_MOTION_POINTS_COUNT=" + i2str(pars.CATCH_NO_MOTION_POINTS_COUNT, stmp) + "\n";
	config += "CATCH_CHECK_JAM_MAX_TIME_INTERVAL_MSEC=" + i2str(pars.CATCH_CHECK_JAM_MAX_TIME_INTERVAL_MSEC, stmp) + "\n";
	config += "CATCH_INTER_ZONE_NO_MOTION_WAS_FOUND_TIME_MSEC=" + i2str(pars.CATCH_INTER_ZONE_NO_MOTION_WAS_FOUND_TIME_MSEC, stmp) + "\n";
	config += "CATCH_BACKGROUND_DIFFERENCE_POINTS_COUNT=" + i2str(pars.CATCH_BACKGROUND_DIFFERENCE_POINTS_COUNT, stmp) + "\n";
	config += "CATCH_FRAME_STABLE_REPEAT_COUNT=" + i2str(pars.CATCH_FRAME_STABLE_REPEAT_COUNT, stmp) + "\n";
	config += "CATCH_TIME_TO_FIND_STABLE_INTERVAL_SEC=" + i2str(pars.CATCH_TIME_TO_FIND_STABLE_INTERVAL_SEC, stmp) + "\n";
	config += "SENSOR_PARALLEL_WORK=" + i2str(pars.SENSOR_PARALLEL_WORK, stmp) + "\n";
	config += "RGB_FRAME=" + i2str(pars.RGB_FRAME, stmp) + "\n";
	config += "TRACKER=" + i2str(pars.TRACKER, stmp) + "\n";
	config += "SM_DATA_WAKEUP=" + i2str(pars.SM_DATA_WAKEUP, stmp) + "\n";
	config += "SENSOR_MAXLOCK_TIME_SEC=" + i2str(pars.SENSOR_MAXLOCK_TIME_SEC, stmp) + "\n";
	return config;
}

TraffAlgParams TraffAlgParams::readAlgParamsFromFile(bool &isOpened, std::string fname) {
	TraffAlgParams out;
	isOpened = false;
	try {
		std::ifstream parametersFile;
		parametersFile.open(fname.c_str());
		if (parametersFile.is_open()) {
			isOpened = true;
			std::string row;
			while (getline(parametersFile, row)) {
				size_t equalIndex = row.find("=");
				if (equalIndex != std::string::npos) {
					std::string key = row.substr(0, equalIndex),
						   value = row.substr(equalIndex + 1);
					if (key == "BINARIZATION_THRESHOLD")
						out.BINARIZATION_THRESHOLD = atoi(value.c_str());
					else if (key == "SHADOW_DARK_THRESHOLD")
						out.SHADOW_DARK_THRESHOLD = atoi(value.c_str());
					else if (key == "SHADOW_MAX_UPPER_THRESHOLD")
						out.SHADOW_MAX_UPPER_THRESHOLD = atoi(value.c_str());
					else if (key == "SHADOW_RANGE")
						out.SHADOW_RANGE = atoi(value.c_str());
					else if (key == "OBJECT_ENTER_POINTS_COUNT")
						out.OBJECT_ENTER_POINTS_COUNT = atoi(value.c_str());
					else if (key == "OBJECT_EXIT_POINTS_COUNT")
						out.OBJECT_EXIT_POINTS_COUNT = atoi(value.c_str());
					else if (key == "INCREASED_OBJECT_ENTER_POINTS_COUNT")
						out.INCREASED_OBJECT_ENTER_POINTS_COUNT = atoi(value.c_str());
					else if (key == "INCREASED_OBJECT_EXIT_POINTS_COUNT")
						out.INCREASED_OBJECT_EXIT_POINTS_COUNT = atoi(value.c_str());
					else if (key == "CATCH_MOTION_POINTS_COUNT")
						out.CATCH_MOTION_POINTS_COUNT = atoi(value.c_str());
					else if (key == "CATCH_SLOW_MOTION_POINTS_COUNT")
						out.CATCH_SLOW_MOTION_POINTS_COUNT = atoi(value.c_str());
					else if (key == "CATCH_NO_MOTION_POINTS_COUNT")
						out.CATCH_NO_MOTION_POINTS_COUNT = atoi(value.c_str());
					else if (key == "CATCH_BACKGROUND_DIFFERENCE_POINTS_COUNT")
						out.CATCH_BACKGROUND_DIFFERENCE_POINTS_COUNT = atoi(value.c_str());
					else if (key == "CATCH_FRAME_MOTION_REPEAT_COUNT")
						out.CATCH_FRAME_MOTION_REPEAT_COUNT = atoi(value.c_str());
					else if (key == "CATCH_FRAME_STABLE_REPEAT_COUNT")
						out.CATCH_FRAME_STABLE_REPEAT_COUNT = atoi(value.c_str());
					else if (key == "CATCH_INTER_ZONE_FRAME_DIFFERENCE_JUMP_TIME_MSEC")
						out.CATCH_INTER_ZONE_FRAME_DIFFERENCE_JUMP_TIME_MSEC = atoi(value.c_str());
					else if (key == "CATCH_INTER_ZONE_NO_MOTION_WAS_FOUND_TIME_MSEC")
						out.CATCH_INTER_ZONE_NO_MOTION_WAS_FOUND_TIME_MSEC = atoi(value.c_str());
					else if (key == "CATCH_TIME_TO_FIND_STABLE_INTERVAL_SEC")
						out.CATCH_TIME_TO_FIND_STABLE_INTERVAL_SEC = atoi(value.c_str());
					else if (key == "CATCH_CHECK_JAM_MAX_TIME_INTERVAL_MSEC")
						out.CATCH_CHECK_JAM_MAX_TIME_INTERVAL_MSEC = atoi(value.c_str());
					else if (key == "CAR_EXIT_ENTER_INTERVAL_MSEC")
						out.CAR_EXIT_ENTER_INTERVAL_MSEC = atoi(value.c_str());
					else if (key == "SHADOW_MAX_WRONG_POINTS")
						out.SHADOW_MAX_WRONG_POINTS = atoi(value.c_str());
					else if (key == "LIGHT_BEAM_MAX_WRONG_POINTS")
						out.LIGHT_BEAM_MAX_WRONG_POINTS = atoi(value.c_str());
					else if (key == "OBJECT_CONFIRMATION_POINTS")
						out.OBJECT_CONFIRMATION_POINTS = atoi(value.c_str());
					else if (key == "LIGHT_BEAM_MAX_THRESHOLD")
						out.LIGHT_BEAM_MAX_THRESHOLD = atoi(value.c_str());
					else if (key == "JAM_CAR_IN_ZONE_TIME_SEC")
						out.JAM_CAR_IN_ZONE_TIME_SEC = atoi(value.c_str());
					else if (key == "SENSOR_PARALLEL_WORK")
						out.SENSOR_PARALLEL_WORK = atoi(value.c_str());
					else if (key == "RGB_FRAME")
						out.RGB_FRAME = atoi(value.c_str());
					else if (key == "TRACKER")
						out.TRACKER = atoi(value.c_str());
					else if (key == "SM_DATA_WAKEUP")
						out.SM_DATA_WAKEUP = atoi(value.c_str());
					else if (key == "SENSOR_MAXLOCK_TIME_SEC")
						out.SENSOR_MAXLOCK_TIME_SEC = atoi(value.c_str());
				}
			}
			parametersFile.close();
		}
	}
	catch (const std::exception &e) {
		//LOGERR("Traffix parameters file parsing error!\n");
	}
	return out;
}

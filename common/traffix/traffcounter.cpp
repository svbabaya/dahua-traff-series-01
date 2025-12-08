#include "traffcounter.h"

#if defined(TraffiXtream) && defined(_ProjCam_)

std::string getCurrentDateTime() {
	time_t t;
	time(&t);
	struct tm tloc = *localtime(&t);
	char buf[64];
	const size_t res = strftime(buf, 64, "%Y-%m-%d %H:%M:%S", &tloc);
	if (res > 0) {
		if (res < 64) {
			buf[res] = '\0';
		}
		return buf;
	}
	else {
		return "";
	}
}

TraffCounter::TraffCounter() {
	clear();
}

TraffCounter::~TraffCounter() {
	clear();
}

bool TraffCounter::isInit() const {
	if (sensorList.empty()) {
		return false;
	}
	else {
		return true;
	}
}

void TraffCounter::clear() {
	for (size_t ii = 0; ii < sensorList.size(); ++ii) {
		sensorList[ii].clear();
	}
	sensorList.clear();
	preTraffStats.clear();
	avg = TraffAvgParams();
	timerclear(&averagingStartTime);
	sm_data_wakeup_mode = 0;
	lastSmallIntervalTime = "";
}

void TraffCounter::initTraffSensors(const std::vector<TraffSensor> &sensors, const TraffAvgParams &avgPar,
									bool parallel, int sm_data_wakeup) {
	clear();
	avg = avgPar;
	sensorList = sensors;
	sm_data_wakeup_mode = sm_data_wakeup;
	if (parallel) {
		for (size_t ii = 0; ii < sensorList.size(); ++ii) {
			sensorList[ii].par_init();
		}
	}
}

bool TraffCounter::processImage(const Frame &frame) {
	if (isInit() == false) {
		return false;
	}
	toTraffStat(preTraffStats, false);

	timeval frameTime = frame.t;
	if (frameTime.tv_sec < 1 && frameTime.tv_usec < 1) {
		gettimeofday(&frameTime, NULL);
	}

	for (size_t ii = 0; ii < sensorList.size(); ++ii) {
		sensorList[ii].processImage(frame);
	}
	while (1) {
		timespec tw = { 0, 1000000 };
		size_t n_fin(0);
		for (size_t ii = 0; ii < sensorList.size(); ++ii) {
			if (sensorList[ii].processFinished()) {
				++n_fin;
			}
		}
		if (n_fin != sensorList.size()) {
			nanosleep(&tw, NULL);
		}
		else {
			break;
		}
	}

	bool needDoAveraging(false);
	if (!timerisset(&averagingStartTime)) {
		averagingStartTime = frameTime;
	}
	else {
		timeval resultTime;
		timersub(&frameTime, &averagingStartTime, &resultTime);
		if (resultTime.tv_sec >= avg.smallAvgInterval) {
			needDoAveraging = true;
			averagingStartTime = frameTime;
			for (size_t ii = 0; ii < sensorList.size(); ++ii) {
				sensorList[ii].doAveraging(frameTime);
			}
		}
	}

	if (needDoAveraging) {
		for (size_t ii = 0; ii < sensorList.size(); ++ii) {
			if (sensorList[ii].isBigAvgComplete()) {
				return true;
			}
		}
		return false;
	}
	else {
		return false;
	}
}

void TraffCounter::toTraffStat(std::vector<TraffStat> &stats, bool updateTime) {
	stats.resize(sensorList.size());
	if (updateTime) {
		lastSmallIntervalTime = getCurrentDateTime();
	}
	for (size_t ii = 0; ii < sensorList.size(); ++ii) {
		sensorList[ii].toTraffStat(stats[ii], lastSmallIntervalTime);
	}
}

int TraffCounter::carInChanged(std::vector<TraffStat> &curTraffStats, bool updateTime) {
	toTraffStat(curTraffStats, updateTime);
	if (sm_data_wakeup_mode < 2 || preTraffStats.empty() || curTraffStats.empty()
		|| curTraffStats.size() != preTraffStats.size()) {
			return -1;
		}
	int res(0);
	for (size_t ii = 0; ii < curTraffStats.size(); ++ii) {
		if (curTraffStats[ii].carDetected_sensor == true && preTraffStats[ii].carDetected_sensor == false) {
			++res;
		}
	}
	return (res < 1) ? -1 : res;
}

int TraffCounter::counterChanged(std::vector<TraffStat> &curTraffStats, bool updateTime) {
	toTraffStat(curTraffStats, updateTime);
	if (sm_data_wakeup_mode < 1 || preTraffStats.empty() || curTraffStats.empty()
		|| curTraffStats.size() != preTraffStats.size()) {
			return -1;
		}
	int res(0);
	for (size_t ii = 0; ii < curTraffStats.size(); ++ii) {
		if (curTraffStats[ii].totalCounter != preTraffStats[ii].totalCounter) {
			++res;
		}
	}
	return (res < 1) ? -1 : res;
}

void TraffCounter::resetSensorStat(int sens_id) {
	for (size_t ii = 0; ii < sensorList.size(); ++ii) {
		if (sensorList[ii].getId() == sens_id) {
			sensorList[ii].resetStat();
			sensorList[ii].toTraffStat(preTraffStats[ii], "");
			break;
		}
	}
}

#endif //#if defined(TraffiXtream) && defined(_ProjCam_)

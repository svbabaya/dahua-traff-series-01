#include "traffsensor.h"

#if defined(TraffiXtream) && defined(_ProjCam_)

#include <fstream>
#include <math.h>
#include <numeric>
#include <climits>
#include <stdlib.h>

#if defined(_ProjCam_)
	#include <errno.h>
	#include <sys/time.h>
#endif

const float ONE_MILLIONTH = 1.0f / 1e6f;

inline bool timeGreaterOrEqual(const timeval &t1, const timeval &t2) {
	return (t1.tv_sec > t2.tv_sec) || (t1.tv_sec == t2.tv_sec && t1.tv_usec >= t2.tv_usec);
}

std::vector<TraffPoint> parseZone(std::string &str, int rot, int frameH, int frameW);
int parseValue(const std::string &str, const char *delim);
int extractRot(const std::string &str);
std::vector<TraffPoint> parseFeatureZone(std::string &str, float &featureDist, int rot, int frameH, int frameW);

std::vector<TraffSensor> TraffSensor::readSensorsFromFile(TraffAvgParams &avg, bool &isOpened, std::string fname, 
	const CamParams &cparams, const TraffAlgParams &aparams) {
	std::vector<TraffSensor> sensors;
	avg = TraffAvgParams();
	isOpened = false;

	// Masker mask(cparams.FrameH, cparams.FrameW);
	try {
		std::ifstream configFile;
		configFile.open(fname.c_str());
		if (configFile.is_open()) {
			isOpened = true;
			std::string row;
			while (getline(configFile, row)) {
				if (row.find("INTERVAL") != std::string::npos) {
					size_t equalIndex = row.find("=");
					std::string key = row.substr(0, equalIndex),
						   value = row.substr(equalIndex + 1);
					if (key == "INTERVAL") {
						avg.bigAvgInterval = parseValue(row, "=");
						//LOGINFO("INTERVAL=%d\n", bigAvgInterval);
					}
					else if (key == "SMALL_INTERVAL") {
						avg.smallAvgInterval = parseValue(row, "=");
						//LOGINFO("SMALL_INTERVAL=%d\n", smallAvgInterval);
					}
				}
				else if (row.find("SENSOR") != std::string::npos) {
					sensors.push_back(TraffSensor());
					int sensorId = parseValue(row, " ");
					int rotation = extractRot(row);
					getline(configFile, row);
#ifdef TraffiX_TRACKER
					std::vector<TraffPoint> sensorPoints = parseZone(row, rotation, cparams.FrameH, cparams.FrameW);
#endif
					getline(configFile, row);
					std::vector<TraffPoint> firstZPoints = parseZone(row, rotation, cparams.FrameH, cparams.FrameW);
					getline(configFile, row);
					std::vector<TraffPoint> secondZPoints = parseZone(row, rotation, cparams.FrameH, cparams.FrameW);
					float dist(-1.0f);
					getline(configFile, row);
					if (!row.empty()) {
						dist = atof(row.c_str());
					}
					sensors.back().init(aparams, firstZPoints, secondZPoints, sensorId, dist, rotation);
#ifdef TraffiX_TRACKER
					if (aparams.TRACKER == 1) {
						float 	y1((sensorPoints[0].y + sensorPoints[3].y) / 2.0f),
								x1((sensorPoints[0].x + sensorPoints[3].x) / 2.0f),
								y2((sensorPoints[1].y + sensorPoints[2].y) / 2.0f),
								x2((sensorPoints[1].x + sensorPoints[2].x) / 2.0f);
						y2 = (y1 + y2) / 2.0f;
						x2 = (x1 + x2) / 2.0f;
						const float zoneLengthPix(sqrt((y2 - y1)*(y2 - y1) + (x2 - x1)*(x2 - x1)));
						TraffPoint p1((sensorPoints[0].x + sensorPoints[1].x) / 2.0f, (sensorPoints[0].y + sensorPoints[1].y) / 2.0f),
								   p2((sensorPoints[3].x + sensorPoints[2].x) / 2.0f, (sensorPoints[3].y + sensorPoints[2].y) / 2.0f);
						sensors.back().segmTracker.initSensorInfo(zoneLengthPix, sensors.back().distanceLengthKm,
																  sensorPoints[0], sensorPoints[3],
																  sensorPoints[1], sensorPoints[2],
																  p1, p2,
																  sensors.back().firstZone.rr, sensors.back().secondZone.rr);
					}
					else if (aparams.TRACKER == 2) {
						float   y1((sensorPoints[0].y + sensorPoints[3].y) / 2.0f),
								x1((sensorPoints[0].x + sensorPoints[3].x) / 2.0f),
								y2((sensorPoints[1].y + sensorPoints[2].y) / 2.0f),
								x2((sensorPoints[1].x + sensorPoints[2].x) / 2.0f);
						const float directionAngle(atan2(y1 - y2, x2 - x1));
						y2 = (y1 + y2) / 2.0f;
						x2 = (x1 + x2) / 2.0f;
						const float zoneLengthPix(sqrt((y2 - y1)*(y2 - y1) + (x2 - x1)*(x2 - x1)));
						y1 = (sensorPoints[0].y + sensorPoints[1].y) / 2.0f;
						x1 = (sensorPoints[0].x + sensorPoints[1].x) / 2.0f;
						y2 = (sensorPoints[2].y + sensorPoints[3].y) / 2.0f;
						x2 = (sensorPoints[2].x + sensorPoints[3].x) / 2.0f;
						sensors.back().featureTracker.initSensorInfo(directionAngle, zoneLengthPix, sensors.back().distanceLengthKm, y1, x1, y2, x2);
					}
#endif
				}
#ifdef TraffiX_TRACKER
				else if (row.find("featurePolygon") != std::string::npos) {
					if (aparams.TRACKER == 2) {
						float dist(-1.0f);
						std::vector<TraffPoint> featureZonePoints = parseFeatureZone(row, dist, sensors.back().getRotation(), cparams.FrameH, cparams.FrameW);
						sensors.back().featureTracker.initFeatureDet(featureZonePoints, dist);
					}
				}
#endif
			}
			configFile.close();

			for (size_t ii = 0; ii < sensors.size(); ++ii) {
				sensors[ii].initAvg(avg);
#ifdef TraffiX_TRACKER
				sensors[ii].featureTracker.initExitLine();
#endif
			}
		}
	}
	catch (const std::exception &e) {
		isOpened = false;
		//LOGERR("Config file parsing error!\n");
	}
	return sensors;
}

TraffSensor::TraffSensor() {
	clear();
}

bool TraffSensor::isInit() const {
	if (avgValuesBufferSize > 0 && firstZone.isInit() && secondZone.isInit()) {
		return true;
	}
	else {
		return false;
	}
}

void TraffSensor::resetStat() {
	TraffStat::resetStat();
	prevCurrentLength = prevCurrentSpeed = sumCurSpeed = 0;
	memset(sumCurSpd, 0, sizeof(sumCurSpd));
	sumCarInSensorTime = 0;
	avgValuesBuffer.clear();
	timerclear(&lastAvgTime);
}

void TraffSensor::clear() {
	par_stop();
	resetStat();
	TraffStat::clear();
	firstZone.clear();
	secondZone.clear();
	avgValuesBufferSize = 0;
	avg = TraffAvgParams();
	timerclear(&carDetTime);
	rotation = 0;

#ifdef TraffiX_TRACKER
	featureTracker.clear();
	preTrRes.st = NONE;
	segmTracker.clear();
	timerclear(&stopDetTime);
#endif

}

void TraffSensor::init(const TraffAlgParams &aparams,
					   const std::vector<TraffPoint> &pointList1, const std::vector<TraffPoint> &pointList2, int sensid, float dist, int rot) {
	par_stop();
	id = sensid;
	firstZone.init (aparams, false, pointList1, sensid);
	secondZone.init(aparams, true,  pointList2, sensid);
	distanceLengthKm = (dist > 0) ? dist : 0;
	algPar = aparams;
	rotation = rot;
}

int TraffSensor::getRotation() const {
	return rotation;
}

void TraffSensor::initAvg(const TraffAvgParams &avgParams) {
	avg = avgParams;
	avgValuesBuffer.clear();
	avgValuesBufferSize = 1;
	period = avg.bigAvgInterval;
	if (avg.smallAvgInterval > 0 && (avg.bigAvgInterval % avg.smallAvgInterval == 0)) {
		avgValuesBufferSize = avg.bigAvgInterval / avg.smallAvgInterval;
	}
	avgValuesBuffer.reserve(avgValuesBufferSize);
}

void TraffSensor::processImage(const Frame &frame) {
	if (isInit() == false) {
		return;
	}
	if (thrSt.id >= 0) {
		pthread_mutex_lock(&thrSt.mutex_W);
		thrSt.pFrame = &frame;
		thrSt.condf_W = true;
		pthread_cond_signal(&thrSt.cond_W);
		pthread_mutex_unlock(&thrSt.mutex_W);
	}
	else {
		process(frame);
	}
}

bool TraffSensor::processFinished() {
	if (isInit() == false) {
		return false;
	}
	if (thrSt.id >= 0) {
		pthread_mutex_lock(&thrSt.mutex_W);
		bool res = (thrSt.pFrame == NULL);
		pthread_mutex_unlock(&thrSt.mutex_W);
		return res;
	}
	else {
		return true;
	}
}

void TraffSensor::process(const Frame &frame) {
	bool stopIsDetected(false);

#ifdef TraffiX_TRACKER
	if (algPar.TRACKER == 1) {
		stopIsDetected = segmTracker.isStopDetected();
	}
	else if (algPar.TRACKER == 2) {
		stopIsDetected = featureTracker.isStopDetected();
	}
	if (false == stopIsDetected) {
		timerclear(&stopDetTime);
	}
	else {
		if (false == timerisset(&stopDetTime)) {
			stopDetTime = frame.t;
		}
		else {
			timeval resultTime;
			timersub(&frame.t, &stopDetTime, &resultTime);
			if (algPar.SENSOR_MAXLOCK_TIME_SEC < resultTime.tv_sec) {
				#ifdef LOGINFO
					LOGINFO("- StopLock: S-%d, %d s, stopDet: %d, carDet: %d\n", id, resultTime.tv_sec, int(stopIsDetected), int(carDetected));
				#endif
				firstZone.resetZone();
				secondZone.resetZone();
				stopIsDetected = false;
				timerclear(&stopDetTime);
				if (algPar.TRACKER == 1) {
					segmTracker.resetTrack();
				}
			}
		}
	}
#endif

	int z1res = firstZone.processImage( frame, &secondZone, stopIsDetected);
	int iDbgImgID = 3;
	int z2res = secondZone.processImage(frame, &firstZone,  stopIsDetected);

#ifdef PRINT_TIME
	gettimeofday(&tv_end, NULL);
	msecBase  = (tv_end.tv_sec * 1000 + tv_end.tv_usec / 1000.0) - (tv_start.tv_sec * 1000 + tv_start.tv_usec / 1000.0);
	LOGINFO("- BaseAlg: %d ms\n", msecBase);
#endif

#ifdef TraffiX_TRACKER

	TrackResult t_r;
	const SensorState inSt = (z1res == 1) ? z1Enter : ((z1res == 2) ? z1Exit : newFrame);
	if (algPar.TRACKER == 1) {
		t_r = segmTracker.procState(inSt, frame.t, firstZone.motionMask[0], firstZone.rr, secondZone.motionMask[0], secondZone.rr);
		if (t_r.st == RESULT) {
			preTrRes = t_r;
		}
	}
	else if (algPar.TRACKER == 2) {
		t_r = featureTracker.procState(inSt, frame.t, frame[0],
									   firstZone.motionMask, firstZone.rr, secondZone.motionMask, secondZone.rr);
	}

#ifdef PRINT_TIME
	gettimeofday(&tv_end, NULL);
	msecTrck = (tv_end.tv_sec * 1000 + tv_end.tv_usec / 1000.0) - (tv_start.tv_sec * 1000 + tv_start.tv_usec / 1000.0);
	LOGINFO("- TrckAlg: %d ms\n", msecTrck);
#endif

#endif

	carDetected = secondZone.carDetected;
	if (false == carDetected) {
		timerclear(&carDetTime);
	}
	else {
		if (false == timerisset(&carDetTime)) {
			carDetTime = frame.t;
		}
		else {
			timeval resultTime;
			timersub(&frame.t, &carDetTime, &resultTime);
			if (algPar.SENSOR_MAXLOCK_TIME_SEC < resultTime.tv_sec) {
				#ifdef LOGINFO
				LOGINFO("- CarDetLock: S-%d, %d s, stopDet: %d, carDet: %d\n", id, resultTime.tv_sec, int(stopIsDetected), int(carDetected));
				#endif
				carDetected = false;
				firstZone.resetZone();
				secondZone.resetZone();
				timerclear(&carDetTime);
#ifdef TraffiX_TRACKER
				stopIsDetected = false;
				timerclear(&stopDetTime);
				if (algPar.TRACKER == 1) {
					segmTracker.resetTrack();
				}
#endif
			}
		}
	}

	if (firstZone.carDetected && 
		 ((secondZone.carDetected && timercmp(&secondZone.carEnterTime, &firstZone.carEnterTime, >)) 
		  || (timerisset(&secondZone.lastFDJumpTime)))) {
			carDetected_sensor = true;
		  }
	else {
		carDetected_sensor = false;
	}

	// inSt == SensorState::z1Exit
	if (firstZone.carPassedZone) {
		firstZone.carPassedZone = false;
	}
#ifdef TraffiX_TRACKER
	if (algPar.TRACKER == 1) {
		t_r = preTrRes;
		preTrRes.st = NONE;
	}
#endif
	if ((timerisset(&firstZone.lastFDJumpTime) && timerisset(&secondZone.lastFDJumpTime) &&
		timercmp(&secondZone.lastFDJumpTime, &firstZone.lastFDJumpTime, >))
#ifdef TraffiX_TRACKER
		|| (t_r.st == RESULT && t_r.lengthM > 0)
#endif
			) {
			calcOccup(frame.t); // Occup calculation
#ifdef TraffiX_TRACKER
			calcCounter_Speed_Length(t_r.st == RESULT && t_r.lengthM > 0, t_r.speedKmH, t_r.lengthM);
#else
			calcCounter_Speed_Length(0, 0, 0); // Counter, speed and length calculation
#endif
			timerclear(&firstZone.carEnterTime);
			timerclear(&firstZone.lastFDJumpTime);
			timerclear(&secondZone.carEnterTime);
			timerclear(&secondZone.lastFDJumpTime);
		}
}

bool TraffSensor::isBigAvgComplete() const {
	return avgValuesBuffer.size() == avgValuesBufferSize;
}

void TraffSensor::toTraffStat(TraffStat &out, const std::string &stattime) const {
	out = *this;
	out.time = stattime;
	out.lastAvgCounter = avgValuesBuffer.empty() ? 0 : avgValuesBuffer.back().counter;
}

int TraffSensor::getId() const {
	return id;
}

void TraffSensor::calcCounter_Speed_Length(bool fVal, float valSpeed, float valLength) {
	++curCounter;
	if (totalCounter == INT_MAX) {
		totalCounter = curCounter;
	}
	else {
		++totalCounter;
	}

	curLength = curSpeed = 0;
	if (fVal) {
		curSpeed = valSpeed;
		curLength = valLength;
	}
	else {
		timeval resultTime;
		if (timerisset(&firstZone.carEnterTime) && timerisset(&secondZone.carEnterTime)
			&& timercmp(&secondZone.carEnterTime, &firstZone.carEnterTime, >)) {
			timersub(&secondZone.carEnterTime, &firstZone.carEnterTime, &resultTime);
			float tHour = (resultTime.tv_sec + resultTime.tv_usec * ONE_MILLIONTH) / 3600.0f;
			curSpeed = distanceLengthKm / tHour;
		}
		else {
			curSpeed = prevCurrentSpeed;
		}

		if (timerisset(&firstZone.carEnterTime) && timerisset(&firstZone.carExitTime)
			&& timercmp(&firstZone.carExitTime, &firstZone.carEnterTime, >)) {
			timersub(&firstZone.carExitTime, &firstZone.carEnterTime, &resultTime);
			float tHour = (resultTime.tv_sec + resultTime.tv_usec * ONE_MILLIONTH) / 3600.0f;
			curLength = (curSpeed * tHour) * 1000.0f;
		}
		else {
			curLength = prevCurrentLength;
		}
	}

	if (curSpeed > 200) {
		curSpeed = prevCurrentSpeed;
	}
	if (curLength > 25.0f) {
		curLength = 4.4f;
	}
	else if (curLength > 15.0f) {
		curLength = 15.0f;
	}
	else if (curLength < 2.0f) {
		curLength = 2.0f;
	}

	prevCurrentSpeed = curSpeed;
	sumCurSpeed += curSpeed;
	prevCurrentLength = curLength;

	if (curLength < 2.0f) {
		++curCnt[0];
		sumCurSpd[0] += curSpeed;
		curClass = "<2m";
	}
	else if (curLength < 5.0f) {
		++curCnt[1];
		sumCurSpd[1] += curSpeed;
		curClass = "<5m";
	}
	else if (curLength < 7.5f) {
		++curCnt[2];
		sumCurSpd[2] += curSpeed;
		curClass = "5-7.5m";
	}
	else {
		curClass = ">7.5m";
		++curCnt[3];
		sumCurSpd[3] += curSpeed;
	//}
	//else
	//{
		++curCnt[4];
		sumCurSpd[4] += curSpeed;
	}
}

void TraffSensor::calcOccup(const timeval &curtime) {
	timeval resultTime;
	if (timerisset(&lastAvgTime)) {
		bool carEnterZoneInPreviousAvgInterval = timercmp(&lastAvgTime, &firstZone.carEnterTime, >);
		if (carEnterZoneInPreviousAvgInterval) {
			timersub(&curtime, &lastAvgTime, &resultTime);
			sumCarInSensorTime += resultTime.tv_sec + resultTime.tv_usec * ONE_MILLIONTH;
		}
		else {
			timersub(&curtime, &firstZone.carEnterTime, &resultTime);
			sumCarInSensorTime += resultTime.tv_sec + resultTime.tv_usec * ONE_MILLIONTH;
		}
	}
	else {
		timersub(&curtime, &firstZone.carEnterTime, &resultTime);
		sumCarInSensorTime += resultTime.tv_sec + resultTime.tv_usec * ONE_MILLIONTH;
	}
}

void TraffSensor::doAveraging(const timeval &curtime) {
	if (isInit() == false) {
		return;
	}
	if (carDetected) {
		calcOccup(curtime);
	}
	AveragingValues avgValues;
	avgValues.carInSensorTime = sumCarInSensorTime;
	avgValues.counter = curCounter;
	avgValues.sumSpeed = sumCurSpeed;
	memcpy(avgValues.Cnt, curCnt, sizeof(curCnt));
	memcpy(avgValues.sumSpd, sumCurSpd, sizeof(sumCurSpd));
	if (avgValuesBuffer.size() < avgValuesBufferSize) {
		avgValuesBuffer.push_back(avgValues);
		if (avgValuesBuffer.size() == avgValuesBufferSize) {
			doBigAveraging();
		}
	}
	else {
		for (size_t ii = 1; ii < avgValuesBuffer.size(); ++ii) {
			avgValuesBuffer[ii - 1] = avgValuesBuffer[ii];
		}
		avgValuesBuffer[avgValuesBuffer.size() - 1] = avgValues;
		doBigAveraging();
	}
	sumCarInSensorTime = 0;
	curCounter = 0;
	sumCurSpeed = 0;
	memset(curCnt, 0, sizeof(curCnt));
	memset(sumCurSpd, 0, sizeof(sumCurSpd));
	lastAvgTime = curtime;
}

void TraffSensor::calcAverage(int &cnt, int &spd, float &totalCarInSensorTime, const std::vector<AveragingValues> &avgBuf, int idx) {
	cnt = spd = 0;
	totalCarInSensorTime = 0;
	if (idx < 0) {
		for (size_t ii = 0; ii < avgBuf.size(); ++ii) {
			cnt += avgBuf[ii].counter;
			spd += avgBuf[ii].sumSpeed;
			totalCarInSensorTime += avgBuf[ii].carInSensorTime;
		}
	}
	else {
		for (size_t ii = 0; ii < avgBuf.size(); ++ii) {
			cnt += avgBuf[ii].Cnt[idx];
			spd += avgBuf[ii].sumSpd[idx];
		}
	}
	spd = (cnt > 0) ? spd / cnt : 0;
}

void TraffSensor::doBigAveraging() {
	float totalCarInSensorTime(0);
	calcAverage(counter, speed, totalCarInSensorTime, avgValuesBuffer, -1);
	float totalOccup = totalCarInSensorTime / avg.bigAvgInterval * 100.0f;
	if (totalOccup < 1 && counter > 0) {
		occup = 1;
	}
	else {
		occup = totalOccup;
	}
	if (occup > 100) {
		occup = 100;
	}
	headway = 0;
	if (counter > 0) {
		headway = (float)avg.bigAvgInterval / counter * 10.0f;
	}
	calcAverage(Cnt0, Spd0, totalCarInSensorTime, avgValuesBuffer, 0);
	calcAverage(Cnt1, Spd1, totalCarInSensorTime, avgValuesBuffer, 1);
	calcAverage(Cnt2, Spd2, totalCarInSensorTime, avgValuesBuffer, 2);
	calcAverage(Cnt3, Spd3, totalCarInSensorTime, avgValuesBuffer, 3);
	calcAverage(Cnt4, Spd4, totalCarInSensorTime, avgValuesBuffer, 4);
}

std::vector<TraffPoint> parseZone(std::string &str, int rot, int frameH, int frameW) {
	std::vector<TraffPoint> pointList;
	std::string value;
	TraffPoint p;
	size_t pos1(0);
	int k(0);

	while ((pos1 = str.find(";")) != std::string::npos) {
		value = str.substr(0, pos1);
		if (k % 2 == 0)
			p.x = atoi(value.c_str());
		else {
			p.y = atoi(value.c_str());
			pointList.push_back(p);
		}
		str.erase(0, pos1 + 1);
		++k;
	}

	if (rot == 180) {
		--frameH;
		--frameW;
		for (size_t ii = 0; ii < pointList.size(); ++ii) {
			pointList[ii].x = frameW - pointList[ii].x;
			pointList[ii].y = frameH - pointList[ii].y;
		}
	}

	return pointList;
}

int parseValue(const std::string &str, const char *delim) {
	int result(0);
	size_t pos1 = str.find(delim);
	if (pos1 != std::string::npos) {
		std::string value = str.substr(pos1 + 1);
		result = atoi(value.c_str());
	}
	return result;
}

int extractRot(const std::string &str) {
	int result(0);
	size_t pos1 = str.find("_rot=");
	size_t pos2 = str.find(" ");
	if (pos1 != std::string::npos && pos2 != std::string::npos) {
		std::string value = str.substr(pos1 + 5, pos2 - (pos1 + 5));
		result = atoi(value.c_str());
	}
	return result;
}

std::vector<TraffPoint> parseFeatureZone(std::string &str, float &featureDist, int rot, int frameH, int frameW) {
	std::vector<TraffPoint> points;
	featureDist = -1.0f;
	size_t pos1 = str.find("=");
	if (pos1 != std::string::npos) {
		str.erase(0, pos1 + 1);
		pos1 = str.find("=");
		if (pos1 != std::string::npos)
		{
			featureDist = atof(str.substr(0, pos1).c_str());
			str.erase(0, pos1 + 1);
			points = parseZone(str, rot, frameH, frameW);
		}
	}
	return points;
}

void* TraffSensor::work_thread(void *arg) {
	ThreadSt *pth = (ThreadSt*)arg;
	while (pth->id >= 0) {
		pthread_mutex_lock(&pth->mutex_W);
		while (!pth->condf_W) {
			pthread_cond_wait(&pth->cond_W, &pth->mutex_W);
		}
		pth->condf_W = false;
		if (pth->id < 0) {
			break;
		}
		pth->parent->process(*(pth->pFrame));
		pth->pFrame = NULL;
		pthread_mutex_unlock(&pth->mutex_W);
	}
	pth->id = THR_ST_NONE;
	pthread_mutex_unlock(&pth->mutex_W);
	return 0;
}

void TraffSensor::par_init() {
	par_stop();
	if (thrSt.init_sync()) {
		thrSt.condf_W = false;
		thrSt.id = 1;
		thrSt.parent = this;
		if (pthread_create(&thrSt.tid, NULL, TraffSensor::work_thread, &thrSt) == 0) {
			sched_yield();
		}
		else {
			thrSt.id = THR_ST_NONE;
			thrSt.condf_W = false;
			thrSt.clear_sync();
		}
	}
}

void TraffSensor::par_stop() {
	if (thrSt.id != THR_ST_NONE) {
		pthread_mutex_lock(&thrSt.mutex_W);
		thrSt.id = THR_ST_STOP;
		pthread_mutex_unlock(&thrSt.mutex_W);
		timespec tw = { 0, 3000000 };
		while (thrSt.id != THR_ST_NONE) {
			pthread_mutex_lock(&thrSt.mutex_W);
			thrSt.condf_W = true;
			pthread_cond_signal(&thrSt.cond_W);
			pthread_mutex_unlock(&thrSt.mutex_W);
			nanosleep(&tw, NULL);
		}
		thrSt.clear_sync();
	}
	thrSt.id = THR_ST_NONE;
	thrSt.condf_W = false;
	thrSt.pFrame = NULL;
}

bool TraffSensor::ThreadSt::init_sync() {
	int res = pthread_cond_init(&cond_W, NULL);
	if (res != 0 && res != EBUSY) {
		return false;
	}
	res = pthread_mutex_init(&mutex_W, NULL);
	if (res != 0 && res != EBUSY) {
		pthread_cond_destroy(&cond_W);
		return false;
	}
	return true;
}

void TraffSensor::ThreadSt::clear_sync() {
	pthread_cond_destroy(&cond_W);
	pthread_mutex_destroy(&mutex_W);
}

#endif //#ifdef TraffiXtream && _ProjCam_
 
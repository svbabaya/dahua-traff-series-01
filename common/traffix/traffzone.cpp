#include "traffsensor.h"
#include <iomanip>

#if defined(TraffiXtream) && defined(_ProjCam_)

#include "utils_cam.h"
#if defined(_ProjCam_)
#include <sys/time.h>
#endif

TraffSensor::TraffZone::TraffZone() {
	clear();
}

bool TraffSensor::TraffZone::isInit() const {
	if (imageMask.empty()) {
		return false;
	}
	else {
		return true;
	}
}

void TraffSensor::TraffZone::clear() {
	algPar = TraffAlgParams();

#ifdef TraffiX_TRACKER
	motionMask.release();
#endif

	imageMask.release();
	bgFrame.release();
	refFrame.release();
	midFrame.release();
	preFrame.release();
	carDetected = carPassedZone = false;
	timerclear(&lastFDJumpTime);
	timerclear(&carEnterTime);
	workMode = CATCH;
	catchModeState = FIND_MOTION;
	zoneState = ROAD;
	isSecond = thresholdsWereIncreased = fdJumpWasDetected = noMotionWasDetected = false;
	stableCounter = motionCounter = 0;
	iNoMotionCounter = 0;
	timerclear(&carExitTime);
	timerclear(&zoneLockedBeginTime);
	timerclear(&fdJumpTime);
	timerclear(&motionWasFoundTime);
	timerclear(&noMotionTime);
	sensorId = 0;
}

void TraffSensor::TraffZone::init(const TraffAlgParams &aparams, bool bSecond, const std::vector<TraffPoint> &pointList, int sensid) {
	clear();
	if (pointList.empty()) {
		return;
	}
	algPar = aparams;
	isSecond = bSecond;
	sensorId = sensid;
	thresholdsWereIncreased = false;
	TraffPolygon polygon;
	polygon.setPointList(pointList);
	rr = polygon.getBoundingRect();

#ifdef TraffiX_TRACKER
	motionMask.create(rr.height, rr.width);
#endif

	imageMask.create(rr.height, rr.width);
	for (int yy = 0; yy < rr.height; ++yy) {
		for (int xx = 0; xx < rr.width; ++xx) {
			TraffPoint p(rr.x1 + xx, rr.y1 + yy);
			imageMask[yy][xx] = polygon.containsPoint(p) ? 255 : 0;
		}
	}
}

int TraffSensor::TraffZone::processImage(const Frame &frame, TraffZone *pOtherZone, bool stopDetected) {
	int res(0);
	if (isInit() == false) {
		return res;
	}
	if (preFrame.empty()) {
		if (frame.rgb == false && frame.yuv == false) {
			algPar.RGB_FRAME = false;
		}
		for (int cc = 0; cc < (algPar.RGB_FRAME ? 3 : 1); ++cc) {
			bgFrame.push(RowMat<uchar>(rr.height, rr.width));
			refFrame.push(RowMat<uchar>(rr.height, rr.width));
			midFrame.push(RowMat<uchar>(rr.height, rr.width));
			preFrame.push(RowMat<uchar>(rr.height, rr.width));
		}
		bgFrame.rgb = refFrame.rgb = midFrame.rgb = preFrame.rgb = frame.rgb;
		bgFrame.yuv = refFrame.yuv = midFrame.yuv = preFrame.yuv = frame.yuv;
	}
	else {
		_CurrentTime_ = frame.t;
		switch (workMode) {
		case CATCH:
			catchMode(frame, pOtherZone);
			break;
		case NORMAL:
			res = detectCarInNormalMode(frame);
			catchMode(frame, pOtherZone, stopDetected, res == 2);
			break;
#if USE_GLARE
		case GLARE_NORMAL:
			res = detectCarInGlareNormalMode(frame);
			catchMode(frame, pOtherZone, stopDetected, res == 2);
			break;
#endif
		}
	}

	copyZoneImageFromFullFrame(preFrame, rr, frame);
	return res;
}

void TraffSensor::TraffZone::resetCatchMode() {
	catchModeState = FIND_MOTION;
	stableCounter = motionCounter = 0;
	iNoMotionCounter = 0;
	fdJumpWasDetected = noMotionWasDetected = false;
	bGlareDetect = false;
	timerclear(&fdJumpTime);
	timerclear(&motionWasFoundTime);
	timerclear(&noMotionTime);
}

void TraffSensor::TraffZone::resetZone() {
	resetCatchMode();
	workMode = CATCH;
	zoneState = ROAD;
	carDetected = carPassedZone = thresholdsWereIncreased = false;
	timerclear(&carEnterTime);
	timerclear(&carExitTime);
	timerclear(&zoneLockedBeginTime);
	timerclear(&lastFDJumpTime);
}

int TraffSensor::TraffZone::absDifferenceAndBinarization(const RowMat<uchar> &currentFullFrame, const RowMat<uchar> &frame,
	int* piNumGlare, double *pdMean) const {
	int count(0);
	int iNumGlare(0);
	double dSum=0;
	int iNumPoints = 0;
	const uchar *pImageMask, *pFrame, *pCurrentFullFrame;
	for (int yy = 0; yy < rr.height; ++yy) {
		pImageMask = imageMask[yy];
		pFrame = frame[yy];
		pCurrentFullFrame = currentFullFrame[yy + rr.y1] + rr.x1;
		for (int xx = 0; xx < rr.width; ++xx)
		{
			if (*pImageMask) {
				uchar sub = (*pCurrentFullFrame >= *pFrame) ? (*pCurrentFullFrame - *pFrame)
															: (*pFrame - *pCurrentFullFrame);
				cv::Vec3b oVal = (*pCurrentFullFrame) * cv::Vec3b(1, 1, 1);
				unsigned char iThresh = false;
				unsigned char iGlare = false;
				if (sub > algPar.BINARIZATION_THRESHOLD) {
					++count;
					oVal = cv::Vec3b(0, 255, 0);
				}
				if ((*pCurrentFullFrame) >= 230) {
					++iNumGlare;
					oVal = cv::Vec3b(0, 0, 255);
				};
				dSum += *pCurrentFullFrame;
				iNumPoints++;
			}
			++pImageMask;
			++pFrame;
			++pCurrentFullFrame;
		}
	}
	if (piNumGlare) {
		*piNumGlare = iNumGlare;
	}
	if (pdMean) {
		*pdMean = dSum / iNumPoints;
	}
	return count;
}

void TraffSensor::TraffZone::filterImage(const RowMat<uchar>& currentFullFrame,  RowMat<uchar>& frame,double k, double *pdMean) {
	const uchar* pImageMask;
	uchar	*pFrame;
	const uchar *pCurrentFullFrame;
	double dSum = 0;
	int iNum = 0;
	for (int yy = 0; yy < rr.height; ++yy) {
		pImageMask = imageMask[yy];
		pFrame = frame[yy];
		pCurrentFullFrame = currentFullFrame[yy + rr.y1] + rr.x1;
		for (int xx = 0; xx < rr.width; ++xx) {
			if (*pImageMask) {
//				dSumFrame += *pFrame;
//				dSumCur += *pCurrentFullFrame;
				double dV = k*double(*pFrame)  + (1 - k)*double(*pCurrentFullFrame);
				*pFrame = round(dV);
				dSum += *pFrame;
				iNum++;
			}
			else {
				*pFrame = 255;
			}
			++pImageMask;
			++pFrame;
			++pCurrentFullFrame;
		}
	}

	if (pdMean)
	{
		*pdMean = dSum / iNum;

	};
	//double dMeanCur = dSumCur / iNum;
	//double dMeanFrame = dSumFrame / iNum;
}

#define USE_GLARE 0

void TraffSensor::TraffZone::catchMode(const Frame &curFullFrame, TraffZone *pOtherZone, bool stopDetected, bool carOut) {
	if (preFrame.empty()) {
		return;
	}
	int iNumGlare=0;
	int count = absDifferenceAndBinarization(curFullFrame[0], preFrame[0], &iNumGlare,&dImgMean);
	if ((isSecond || (!isSecond && !carOut)) && count > algPar.CATCH_SLOW_MOTION_POINTS_COUNT) {
		lastFDJumpTime = _CurrentTime_;
	}

	if (stopDetected) {
		pOtherZone->resetCatchMode();
		resetCatchMode();
		return;
	}

	switch (catchModeState) {
	case FIND_MOTION:
	{
		if (count > algPar.CATCH_MOTION_POINTS_COUNT) {
			++motionCounter;
			if (motionCounter >= algPar.CATCH_FRAME_MOTION_REPEAT_COUNT) {
				catchModeState = WAIT_OTHER_ZONE;
				fdJumpWasDetected = true;
				fdJumpTime = _CurrentTime_;
				stableCounter = 0;
			}
		}
		else {
			motionCounter = 0;
			if (workMode==NORMAL) {
				filterImage(curFullFrame[0], bgFrame[0], 0.9,&dBgMean);
			}
		}

#if USE_GLARE
		if (iNumGlare > 30) {
			catchModeState = WAIT_OTHER_ZONE;
			fdJumpWasDetected = true;
			fdJumpTime = _CurrentTime_;
			stableCounter = 0;
			bGlareDetect = true;
		}
#endif
			
	}

	break;

	case WAIT_OTHER_ZONE:
	{

#if USE_GLARE
		if (!bGlareDetect && (iNumGlare > 30)) {
			fdJumpTime = _CurrentTime_;
			bGlareDetect = true;
		}
#endif

		timeval timeSinceFDJump;
		timersub(&_CurrentTime_, &fdJumpTime, &timeSinceFDJump);
		int msec = timeSinceFDJump.tv_sec * 1000 + timeSinceFDJump.tv_usec / 1000;
		if (msec > algPar.CATCH_INTER_ZONE_FRAME_DIFFERENCE_JUMP_TIME_MSEC) {
			pOtherZone->resetCatchMode();
			resetCatchMode();
		}
		else if (pOtherZone->fdJumpWasDetected) {
			bool timeIsCorrect(false);
			if (!isSecond) {
				timeIsCorrect = timercmp(&pOtherZone->fdJumpTime, &fdJumpTime, >);
			}
			else {
				timeIsCorrect = timercmp(&fdJumpTime, &pOtherZone->fdJumpTime, >);
			}
			if (timeIsCorrect) {
#if USE_GLARE
			if (bGlareDetect && pOtherZone->bGlareDetect) {
				workMode = GLARE_NORMAL;
			}
			else if (!bGlareDetect && !pOtherZone->bGlareDetect) {
#endif
				catchModeState = FIND_NO_MOTION;
				motionWasFoundTime = _CurrentTime_;
#if USE_GLARE
				};
#endif
			}
			else {
				pOtherZone->resetCatchMode();
				resetCatchMode();
			}
		}
	}
	break;

	case FIND_NO_MOTION: {
		if (count < algPar.CATCH_NO_MOTION_POINTS_COUNT) {
			if (iNoMotionCounter > 0) {
				noMotionWasDetected = true;
				catchModeState = CHECK_JAM;
				noMotionTime = _CurrentTime_;
				copyZoneImageFromFullFrame(refFrame, rr, curFullFrame);
				dRefMean = dImgMean;
			}
			else {
				++iNoMotionCounter;
			}
		}
		else {
			iNoMotionCounter = 0;
		}
	}
	break;

	case CHECK_JAM:
	{
		timeval timeSinceNoMotion;
		timersub(&_CurrentTime_, &noMotionTime, &timeSinceNoMotion);
		int msec = timeSinceNoMotion.tv_sec * 1000 + timeSinceNoMotion.tv_usec / 1000;
		if (msec >= algPar.CATCH_CHECK_JAM_MAX_TIME_INTERVAL_MSEC) {
			pOtherZone->resetCatchMode();
			resetCatchMode();
		} else if (pOtherZone->noMotionWasDetected) {
			if (msec < algPar.CATCH_INTER_ZONE_NO_MOTION_WAS_FOUND_TIME_MSEC) {
				pOtherZone->resetCatchMode();
				resetCatchMode();
			}
			else {
				pOtherZone->catchModeState = FIND_STABLE_INTERVAL;
				catchModeState = FIND_STABLE_INTERVAL;
			}
		}
		else {
			filterImage(curFullFrame[0],refFrame[0],0.5,&dRefMean);
		}
	}
	break;

	case FIND_STABLE_INTERVAL:
	{
		timeval timeSinceMotionWasFound;
		timersub(&_CurrentTime_, &motionWasFoundTime, &timeSinceMotionWasFound);
		if (timeSinceMotionWasFound.tv_sec >= algPar.CATCH_TIME_TO_FIND_STABLE_INTERVAL_SEC) {
			pOtherZone->resetCatchMode();
			resetCatchMode();
		}
		else {
			count = absDifferenceAndBinarization(curFullFrame[0], refFrame[0], NULL,NULL);
			if (count < algPar.CATCH_BACKGROUND_DIFFERENCE_POINTS_COUNT) {
				++stableCounter;
				if (stableCounter == algPar.CATCH_FRAME_STABLE_REPEAT_COUNT / 2) {
					copyZoneImageFromFullFrame(midFrame, rr, curFullFrame);
					dMidMean = dImgMean;
				}
				if (stableCounter >= algPar.CATCH_FRAME_STABLE_REPEAT_COUNT) {
					copyZoneImageFromFullFrame(bgFrame, TraffRect(0, 0, rr.width-1, rr.height-1), midFrame);
					dBgMean = dMidMean;
					workMode = NORMAL;
					thresholdsWereIncreased = false;
					resetCatchMode();
				}
			}
			else {
				stableCounter = 0;
			}
		}
	}
	break;
	}
}

inline int rgb_diffSq(int dR, int dG, int dB) {
	return dR*dR + dG*dG + dB*dB;
}

inline int yuv_diffSq(int dY, int dU, int dV) {
	const int tdY(298 * dY), tdU(dU);
	dY = tdY + 409 * dV;
	dU = tdY - 100 * dU - 208 * dV;
	dV = tdY + 516 * tdU;
	return dY*dY / 65536 + dU*dU / 65536 + dV*dV / 65536;
}

int TraffSensor::TraffZone::detectCarInNormalMode(const Frame &curFullFrame) {
#ifdef TraffiX_TRACKER
	motionMask.zeros();
	uchar *pMM;
#endif

	int higherPointsCount(0), lightBeamPointsCount(0), lowerPointsCount(0), shadowPointsCount(0);
	const uchar *pImageMask;

#if COMPENSATE_LIGHTCHANGE
	int iCenteredCount=0;
#endif

	Frame::FramePt bgP, frP;
	for (int yy = 0; yy < rr.height; ++yy) {
#ifdef TraffiX_TRACKER
		pMM = motionMask[yy];
#endif

		pImageMask = imageMask[yy];
		bgFrame.getRowPts(bgP, yy);
		curFullFrame.getRowPts(frP, yy+rr.y1, rr.x1);
		for (int xx = 0; xx < rr.width; ++xx) {
			if (*pImageMask) {
				const int fY(*frP[0]), bY(*bgP[0]);
				double dCentfY = fY - dImgMean; 
				double dCentbY = bY - dBgMean;

#if COMPENSATE_LIGHTCHANGE
				if (abs(dCentfY - dCentbY) > algPar.BINARIZATION_THRESHOLD) {
					iCenteredCount++;
				}
#endif

				if (((fY >= bY) ? (fY - bY) : (bY - fY)) > algPar.BINARIZATION_THRESHOLD) {
#ifdef TraffiX_TRACKER
					*pMM = 255;
#endif
					if (fY >= bY) {
						++higherPointsCount;
						if (fY >= algPar.LIGHT_BEAM_MAX_THRESHOLD) {
							++lightBeamPointsCount;
						}
					}
					else {
						++lowerPointsCount;
						if (bY > algPar.BINARIZATION_THRESHOLD)
						{
							int shadowUpperThreshold = bY - algPar.BINARIZATION_THRESHOLD;
							if (shadowUpperThreshold > algPar.SHADOW_MAX_UPPER_THRESHOLD)
								shadowUpperThreshold = algPar.SHADOW_MAX_UPPER_THRESHOLD;

							int shadowLowerThreshold = (shadowUpperThreshold > algPar.SHADOW_DARK_THRESHOLD + algPar.SHADOW_RANGE) ?
															(shadowUpperThreshold - algPar.SHADOW_RANGE) :
															algPar.SHADOW_DARK_THRESHOLD;
							if (fY > shadowLowerThreshold && fY < shadowUpperThreshold)
								++shadowPointsCount;
						}
					}
				}
#ifdef TraffiX_TRACKER
				else if (algPar.RGB_FRAME) {
					const int rgb_sub = curFullFrame.yuv ? yuv_diffSq(bY-fY, int(*bgP[1])-int(*frP[1]), int(*bgP[2])-int(*frP[2]))
														 : rgb_diffSq(bY-fY, int(*bgP[1])-int(*frP[1]), int(*bgP[2])-int(*frP[2]));
					if (rgb_sub > algPar.BINARIZATION_THRESHOLD*algPar.BINARIZATION_THRESHOLD) {
						*pMM = 255;
					}
				}
#endif
			}
			++bgP;
			++frP;
			++pImageMask;
#ifdef TraffiX_TRACKER
			++pMM;
#endif
		}
	}

#if COMPENSATE_LIGHTCHANGE
	iTotalCenteredCount = iCenteredCount;
#endif

	int totalPointsCount = higherPointsCount + lowerPointsCount;
	int specialShadowPointsCount = totalPointsCount - shadowPointsCount;
	int specialLightBeamPointsCount = totalPointsCount - lightBeamPointsCount;

#if COMPENSATE_LIGHTCHANGE
	bool shadowDetected = (shadowPointsCount > 0) && ((specialShadowPointsCount <= algPar.SHADOW_MAX_WRONG_POINTS) || iCenteredCount==0);
#else
	bool shadowDetected = (shadowPointsCount > 0) && (specialShadowPointsCount <= algPar.SHADOW_MAX_WRONG_POINTS);
#endif

	bool lightBeamDetected = (lightBeamPointsCount > 0) && (specialLightBeamPointsCount <= algPar.LIGHT_BEAM_MAX_WRONG_POINTS);

	bool carEnter(false), carOut(false);
	switch (zoneState) {
	case CAR:
		carOut = processCarState(totalPointsCount, shadowDetected, lightBeamDetected);
		break;

	case ROAD:
		carEnter = processRoadState(totalPointsCount, shadowDetected, lightBeamDetected);
		break;

	case SHADOW:
		carEnter = processShadowState(totalPointsCount, specialShadowPointsCount);
		break;

	case LIGHT_BEAM:
		carEnter = processLightBeamState(totalPointsCount, specialLightBeamPointsCount);
		break;
	}

	if (carOut) {
		return 2;
	}
	else if (carEnter) {
		return 1;
	}
	else {
		return 0;
	}
}

#if USE_GLARE

int TraffSensor::TraffZone::detectCarInGlareNormalMode(const Frame& curFullFrame) {
	int iNumGlare = 0;
	int count = absDifferenceAndBinarization(curFullFrame[0], preFrame[0], &iNumGlare);
	iDbgGlareCounter = iNumGlare;

#if 0
	int totalPointsCount = higherPointsCount + lowerPointsCount;
	int specialShadowPointsCount = totalPointsCount - shadowPointsCount;
	int specialLightBeamPointsCount = totalPointsCount - lightBeamPointsCount;
	bool shadowDetected = (shadowPointsCount > 0) && (specialShadowPointsCount <= algPar.SHADOW_MAX_WRONG_POINTS);
	bool lightBeamDetected = (lightBeamPointsCount > 0) && (specialLightBeamPointsCount <= algPar.LIGHT_BEAM_MAX_WRONG_POINTS);
#endif
	bool carEnter(false), carOut(false);
	switch (zoneState) {
	case CAR:
		carOut = (iNumGlare < 30); //processCarState(totalPointsCount, shadowDetected, lightBeamDetected);
		if (carOut) {
			zoneState = ROAD;
			carDetected = false;
			carPassedZone = true;
			carExitTime = _CurrentTime_;
			timerclear(&zoneLockedBeginTime);
		}
		break;
	case ROAD:
#if 0
		carEnter = processRoadState(totalPointsCount, shadowDetected, lightBeamDetected);
#endif
		if (iNumGlare >= 30) {
			carEnter = processCarEnter();
		}
		break;
#if 0
	case SHADOW:
		carEnter = processShadowState(totalPointsCount, specialShadowPointsCount);
		break;

	case LIGHT_BEAM:
		carEnter = processLightBeamState(totalPointsCount, specialLightBeamPointsCount);
		break;
#endif
	}

	if (carOut) {
		return 2;
	}
	else if (carEnter) {
		return 1;
	}
	else {
		return 0;
	}
}
#endif

bool TraffSensor::TraffZone::processCarEnter() {
	timeval resultTime;
	timersub(&_CurrentTime_, &carExitTime, &resultTime);
	if (resultTime.tv_sec > 0 || resultTime.tv_usec / 1000 > algPar.CAR_EXIT_ENTER_INTERVAL_MSEC) {
		zoneState = CAR;
		carDetected = true;
		carEnterTime = _CurrentTime_;
		zoneLockedBeginTime = _CurrentTime_;
		return true;
	}
	else {
		return false;
	}
}

bool TraffSensor::TraffZone::processRoadState(int totalPointsCount, bool shadowDetected, bool lightBeamDetected) {
	bool carEnter(false);
#if COMPENSATE_LIGHTCHANGE
	if ((totalPointsCount > getObjectEnterPointsCount()) && (iTotalCenteredCount > totalPointsCount / 100))
#else
	if (totalPointsCount > getObjectEnterPointsCount())
#endif 
	{
		if (shadowDetected) {
			zoneState = SHADOW;
		}
		else if (lightBeamDetected) {
			zoneState = LIGHT_BEAM;
		}
		else {
			carEnter = processCarEnter();
		}
	}
	return carEnter;
}

bool TraffSensor::TraffZone::processCarState(int totalPointsCount, bool shadowDetected, bool lightBeamDetected) {
	bool carOut(false);
	if (shadowDetected) {
		zoneState = SHADOW;
		carOut = true;
	}
	else if (lightBeamDetected) {
		zoneState = LIGHT_BEAM;
		carOut = true;
	}
	else if (totalPointsCount < getObjectExitPointsCount()) {
		zoneState = ROAD;
		carOut = true;
	}
	if (carOut) {
		carDetected = false;
		carPassedZone = true;
		carExitTime = _CurrentTime_;
		timerclear(&zoneLockedBeginTime);
	}
	else {
		timeval carInZoneTime;
		timersub(&_CurrentTime_, &zoneLockedBeginTime, &carInZoneTime);
		if (zoneLockedBeginTime.tv_sec > 0 && carInZoneTime.tv_sec > algPar.JAM_CAR_IN_ZONE_TIME_SEC && !thresholdsWereIncreased)
		{
			/*objectEnterPointsCount = algPar.INCREASED_OBJECT_ENTER_POINTS_COUNT;
			objectExitPointsCount = algPar.INCREASED_OBJECT_EXIT_POINTS_COUNT;*/
			thresholdsWereIncreased = true;
		}
	}
	return carOut;
}

bool TraffSensor::TraffZone::processShadowState(int totalPointsCount, int specialShadowPointsCount) {
	bool carEnter(false);
	
#if COMPENSATE_LIGHTCHANGE
	if ((specialShadowPointsCount > algPar.OBJECT_CONFIRMATION_POINTS) && iTotalCenteredCount >10)
#else
	if (specialShadowPointsCount > algPar.OBJECT_CONFIRMATION_POINTS)
#endif
	carEnter = processCarEnter();
	else if (totalPointsCount < getObjectExitPointsCount() && specialShadowPointsCount < algPar.SHADOW_MAX_WRONG_POINTS) {
		zoneState = ROAD;
	}
	return carEnter;
}

bool TraffSensor::TraffZone::processLightBeamState(int totalPointsCount, int specialLightBeamPointsCount) {
	bool carEnter(false);
	if (specialLightBeamPointsCount > algPar.OBJECT_CONFIRMATION_POINTS) {
		carEnter = processCarEnter();
	}
	else if (totalPointsCount < getObjectExitPointsCount() && specialLightBeamPointsCount < algPar.LIGHT_BEAM_MAX_WRONG_POINTS) {
		zoneState = ROAD;
	}
	return carEnter;
}

#endif //#if defined(TraffiXtream) && defined(_ProjCam_)

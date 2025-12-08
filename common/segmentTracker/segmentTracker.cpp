#include "segmentTracker.h"

#if defined(TraffiXtream) && defined(TraffiX_TRACKER)

#include <opencv2/imgproc/imgproc.hpp>

#ifdef FeatureDetExtCV
	#include <opencv2/highgui/highgui.hpp>
#endif

const float SCALE_COEF(0.5f);
const int MORPH_SIZE_D(3);
const int MORPH_SIZE_E(5);
const int MIN_SEGM_PT_NUM(100);
const float EPS_TOLINE_DIST(2.0f);
const float Z1_EXIT_COEF(0.35f);
const float MIN_COORD_SHIFT(1.0f);
const int FLOAT_INT_PRECISION = 10000;
const float ONE_MILLIONTH = 1.0f / 1e6f;

inline bool isValid(const float &dist, const float &orig = 0) {
	return !(dist < (orig + EPS_TOLINE_DIST));
}

inline bool notValid(const float &dist, const float &orig = 0) {
	return dist < (orig + EPS_TOLINE_DIST);
}

SegmentBasedDetector::SegmentBasedDetector() {
	clear();
	trckVec.reserve(350);
}

void SegmentBasedDetector::clear() {
	init = false;
	zoneLengthPix = zoneLengthKm = 0;
	resetTrack();
}

void SegmentBasedDetector::resetTrack() {
	trckVec.clear();
	stopDetected = z2enter = false;
	preState = newFrame;
	z2enterIdx = -1;
}

SegmentBasedDetector::TrackData::TrackData(const timeval &t)
	: distEnter(-1.0f), distExit(-1.0f), deltaEnter(-1.0f), deltaExit(-1.0f), stopped(false), tt(t) {}

void SegmentBasedDetector::TrackData::init(const float &distEnt, const float &distExi, const TrackData *pPreData) {
	distEnter = distEnt;
	distExit = distExi;
	if (notValid(distEnter)) {
		distEnter = -1.0f;
	}

	if (pPreData) {
		deltaEnter = (notValid(distEnter) || notValid(pPreData->distEnter)) ? -1.0f : distEnter - pPreData->distEnter;
		if (notValid(distExit) || notValid(pPreData->distExit)) {
			distExit = deltaExit = -1.0f;
		}
		else {
			deltaExit = pPreData->distExit - distExit;
		}
	}

	if (deltaEnter < 0) {
		if (deltaExit < 0) {
			stopped = false;
		}
		else {
			stopped = (deltaExit < MIN_COORD_SHIFT);
		}
	}
	else {
		if (deltaExit < 0) {
			stopped = (deltaEnter < MIN_COORD_SHIFT);
		}
		else {
			stopped = (deltaExit < MIN_COORD_SHIFT && deltaEnter < MIN_COORD_SHIFT);
		}
	}
}

void SegmentBasedDetector::calcNormLineCoefs(NormLineCoefs &line, const cv::Point2f &inp1, const cv::Point2f &inp2, const TraffRect &roi) {
	cv::Point2f p1t(inp1), p2t(inp2);
	p1t.x -= roi.x1;
	p1t.y -= roi.y1;
	p2t.x -= roi.x1;
	p2t.y -= roi.y1;
	line.precision = FLOAT_INT_PRECISION;
	const cv::Point2f vec(p2t.x - p1t.x, p2t.y - p1t.y);
	if (abs(vec.x) < 0.1f && abs(vec.y) < 0.1f) {
		line.na = line.nb = line.nc = 0;
	}
	else {
		float a = vec.x;
		float b = -vec.y;
		float c = -(a * p1t.y + b * p1t.x);
		float norm = sqrt(a*a + b*b);
		line.na = (a/norm) * line.precision;
		line.nb = (b/norm) * line.precision;
		line.nc = (c/norm) * line.precision;
	}
}

void SegmentBasedDetector::initSensorInfo(float zonLengthPix, float zonLengthKm,
										  const TraffPoint &z1enterP1, const TraffPoint &z1enterP2,
										  const TraffPoint &z2exitP1, const TraffPoint &z2exitP2,
										  const TraffPoint &z2enterP1, const TraffPoint &z2enterP2,
										  const TraffRect &z1rr, const TraffRect &z2rr) {
	clear();
	init = true;

	if (zonLengthPix > 0) {
		zoneLengthPix = SCALE_COEF * zonLengthPix;
	}
	if (zonLengthKm > 0) {
		zoneLengthKm = zonLengthKm;
	}

	const TraffRect z1r(SCALE_COEF * z1rr.x1, SCALE_COEF * z1rr.y1, SCALE_COEF * z1rr.x2, SCALE_COEF * z1rr.y2),
					z2r(SCALE_COEF * z2rr.x1, SCALE_COEF * z2rr.y1, SCALE_COEF * z2rr.x2, SCALE_COEF * z2rr.y2);

	const cv::Point2f p1(SCALE_COEF * z2exitP1.x, SCALE_COEF * z2exitP1.y),
					  p2(SCALE_COEF * z2exitP2.x, SCALE_COEF * z2exitP2.y);
	calcNormLineCoefs(exitLine_z1, p1, p2, z1r);
	calcNormLineCoefs(exitLine_z2, p1, p2, z2r);

	const cv::Point2f p1e1(SCALE_COEF * z1enterP1.x, SCALE_COEF * z1enterP1.y),
					  p2e1(SCALE_COEF * z1enterP2.x, SCALE_COEF * z1enterP2.y);
	calcNormLineCoefs(enterLine_z1, p1e1, p2e1, z1r);

	const cv::Point2f p1e2(SCALE_COEF * z2enterP1.x, SCALE_COEF * z2enterP1.y),
					  p2e2(SCALE_COEF * z2enterP2.x, SCALE_COEF * z2enterP2.y);
	calcNormLineCoefs(enterLine_z2, p1e2, p2e2, z2r);
}

void SegmentBasedDetector::prepareImages(cv::Mat &zBin, TraffRect &zRoi,
										 const uchar *inz, const TraffRect &inzRoi) {
	if (inz) {
		zRoi = TraffRect(inzRoi.x1*SCALE_COEF, inzRoi.y1*SCALE_COEF, inzRoi.x2*SCALE_COEF, inzRoi.y2*SCALE_COEF);
		zBin = cv::Mat(zRoi.height, zRoi.width, CV_8U);
		cv::resize(cv::Mat(inzRoi.height, inzRoi.width, CV_8U, (uchar*)inz), zBin, zBin.size(), 0, 0, cv::INTER_NEAREST);
#ifdef FeatureDetExtCV
cv::imshow("z IN", zBin);
cv::waitKey(1);
#endif
		cv::Mat t(zRoi.height, zRoi.width, CV_8U);
		cv::Mat el = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(MORPH_SIZE_D, MORPH_SIZE_D));
		cv::dilate(zBin, t, el);
		el = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(MORPH_SIZE_E, MORPH_SIZE_E));
		cv::erode(t, zBin, el);
	}
}

bool SegmentBasedDetector::getSegm(CSegParamsRect &segm, cv::Mat &zone) {
	CMarking<C8Conn, CSegParamsRect> mrk(zone.cols);
	const uchar *pB(zone.data);
	for (int yy = 0; yy < zone.rows; ++yy) {
		for (int xx = 0; xx < zone.cols; ++xx) {
			mrk.Marking(*pB);
			++pB;
		}
	}
	std::vector<CSegParamsRect> segms = mrk.GetNewSgs();
	if (segms.empty()) {
		return false;
	}

	int maxNum(segms[0].num);
	CSegParamsRect *pSeg(&segms[0]);
	for (size_t ii = 1; ii < segms.size(); ++ii) {
		if (segms[ii].num > maxNum) {
			maxNum = segms[ii].num;
			pSeg = &segms[ii];
		}
	}

	if (pSeg->num < MIN_SEGM_PT_NUM) {
		return false;
	}
	else {
		segm = *pSeg;
		for (size_t ii = 0; ii < segms.size(); ++ii) {
			if (pSeg != &segms[ii] && segms[ii].num >= MIN_SEGM_PT_NUM)
				segm.AddSeg(segms[ii]);
		}

		uchar *pB;
		for (int yy = segm.t; yy <= segm.b; ++yy) {
			if (yy == 0 || yy == zone.rows-1)
				continue;
			pB = zone.data + yy * zone.cols + segm.l;
			for (int xx = segm.l; xx <= segm.r; ++xx) {
				if (*pB++) {
					if (xx == 0 || xx == zone.cols-1) {
						continue;
					}
					bool brk(false);
					for (int yy2 = yy-1; yy2 <= yy+1; ++yy2) {
						const uchar *pB2 = zone.data + yy2 * zone.cols + xx-1;
						for (int xx2 = xx-1; xx2 <= xx+1; ++xx2) {
							if ((*pB2++) == 0) {
								brk = true;
								break;
							}
						}
						if (brk) {
							break;
						}
					}
					if (brk == false) {
						*(pB - 1) = 1;
					}
				}
			}
		}
#ifdef FeatureDetExtCV
cv::imshow("getSegm", zone);
cv::waitKey(1);
#endif
		return true;
	}
}

float SegmentBasedDetector::segm2lineDist(const CSegParamsRect &segm, const cv::Mat &zone, const NormLineCoefs &line) {
	if (segm.l < 0) {
		return -1.0f;
	}
	int minMDist(INT_MAX), mdist;
	const uchar *pB;
	for (int yy = segm.t; yy <= segm.b; ++yy) {
		pB = zone.data + yy * zone.cols + segm.l;
		for (int xx = segm.l; xx <= segm.r; ++xx) {
			if ((*pB++) == 255) {
				mdist = line.mult_dist(yy, xx);
				if (mdist < minMDist) {
					minMDist = mdist;
				}
			}
		}
	}
	return line.mult_dist2float(minMDist);
}

TrackResult SegmentBasedDetector::procState(SensorState st, const timeval &eventTime,
											const uchar *z1motionMask, const TraffRect &inz1rr,
											const uchar *z2motionMask, const TraffRect &inz2rr) {
	if (init == false) {
		return NONE;
	}

	if (st != z1Enter && st != newFrame && st != z1Exit) {
		return NONE;
	}

#ifdef PRINT_TIME
	timeval tv_start, tv_end;
	int msecPrep, msecMark, msecSegmDist1, msecOther;
#endif

	TrackResult t_r(NONE);
	stopDetected = false;
	if (st == z1Enter && preState != z1Enter) {
#ifdef PRINT_TIME
		gettimeofday(&tv_start, NULL);
#endif
		if (!trckVec.empty()) {
			t_r = calcResult();
			resetTrack();
		}

		cv::Mat sZ1;
		TraffRect sZ1Roi;
		prepareImages(sZ1, sZ1Roi, z1motionMask, inz1rr);

#ifdef FeatureDetExtCV
		cv::imshow("z1 morph", sZ1);
		cv::waitKey(1);
#endif

		CSegParamsRect segm(-1,-1);
		if (false == getSegm(segm, sZ1)) {
			return t_r;
		}

		TrackData newData(eventTime);
		newData.init(segm2lineDist(segm, sZ1, enterLine_z1), segm2lineDist(segm, sZ1, exitLine_z1), NULL);
		if (notValid(newData.distExit, zoneLengthPix) || isValid(newData.distEnter)) {
			return t_r;
		}
		else {
			trckVec.push_back(newData);
		}
#ifdef PRINT_TIME
		gettimeofday(&tv_end, NULL);
		msecPrep = (tv_end.tv_sec * 1000 + tv_end.tv_usec / 1000.0) - (tv_start.tv_sec * 1000 + tv_start.tv_usec / 1000.0);
		LOGINFO("-- enter obj: %d ms\n", msecPrep);
#endif
	}
	else if (st == z1Exit && preState != z1Exit) {
		t_r = calcResult();
		resetTrack();
	}
	else if (!trckVec.empty()) {

#ifdef PRINT_TIME
		gettimeofday(&tv_start, NULL);
#endif
		cv::Mat sZ;
		TraffRect sZRoi;
		prepareImages(sZ, sZRoi, z1motionMask, inz1rr);
#ifdef PRINT_TIME
		gettimeofday(&tv_end, NULL);
		msecPrep = (tv_end.tv_sec * 1000 + tv_end.tv_usec / 1000.0) - (tv_start.tv_sec * 1000 + tv_start.tv_usec / 1000.0);
		gettimeofday(&tv_start, NULL);
#endif
#ifdef FeatureDetExtCV
		cv::imshow("z1 morph", sZ);
		cv::waitKey(1);
		double min, max;
		cv::minMaxLoc(sZ, &min, &max);
#endif
		CSegParamsRect segm(-1, -1);
		if (false == getSegm(segm, sZ)) {
			t_r = calcResult();
			resetTrack();
			return t_r;
		}
#ifdef PRINT_TIME
		gettimeofday(&tv_end, NULL);
		msecMark = (tv_end.tv_sec * 1000 + tv_end.tv_usec / 1000.0) - (tv_start.tv_sec * 1000 + tv_start.tv_usec / 1000.0);
		gettimeofday(&tv_start, NULL);
#endif
		TrackData newData(eventTime);
		float distEnter = segm2lineDist(segm, sZ, enterLine_z1),
			  distExit  = segm2lineDist(segm, sZ, exitLine_z1);
#ifdef PRINT_TIME
		gettimeofday(&tv_end, NULL);
		msecSegmDist1 = (tv_end.tv_sec * 1000 + tv_end.tv_usec / 1000.0) - (tv_start.tv_sec * 1000 + tv_start.tv_usec / 1000.0);
		gettimeofday(&tv_start, NULL);
#endif
		if (false == z2enter) { 
			if (notValid(distExit, zoneLengthPix)) {
				if (trckVec.size() == 1) {
					resetTrack();
					return t_r;
				}
				prepareImages(sZ, sZRoi, z2motionMask, inz2rr);
#ifdef FeatureDetExtCV
				cv::imshow("z2 morph", sZ);
				cv::waitKey(1);
#endif
				if (getSegm(segm, sZ)) {
					float distEnter2 = segm2lineDist(segm, sZ, enterLine_z2);
					if (notValid(distEnter2)) {
						float distExit2 = segm2lineDist(segm, sZ, exitLine_z2);
						if (isValid(distExit2)) {
							distExit = distExit2;
							z2enter = true;
							z2enterIdx = trckVec.size();
						}
					}
				}
			}
		}
		else {
			prepareImages(sZ, sZRoi, z2motionMask, inz2rr);
#ifdef FeatureDetExtCV
			cv::imshow("z2 morph", sZ);
			cv::waitKey(1);
#endif
			bool falseZ2enter(false);
			if (false == getSegm(segm, sZ)) {
				falseZ2enter = true;
			}
			
			if (falseZ2enter) {
				if (z2enterIdx >= 0) {
					trckVec.erase(trckVec.begin() + z2enterIdx, trckVec.end());
				}
				z2enter = false;
				z2enterIdx = -1;
			}
			else {
				distExit = segm2lineDist(segm, sZ, exitLine_z2);
			}
		}
		newData.init(distEnter, distExit, &(trckVec.back()));
		stopDetected = newData.stopped;

		if ((isValid(newData.distEnter) || isValid(newData.distExit)) && !(stopDetected && trckVec.size() > 300)) {
			trckVec.push_back(newData);
		}
		
		if (newData.distEnter > Z1_EXIT_COEF*zoneLengthPix) {
			t_r = calcResult();
			resetTrack();
			return t_r;
		}

#ifdef PRINT_TIME
		gettimeofday(&tv_end, NULL);
		msecOther = (tv_end.tv_sec * 1000 + tv_end.tv_usec / 1000.0) - (tv_start.tv_sec * 1000 + tv_start.tv_usec / 1000.0);
		LOGINFO("-- segPrep: %d ms; segMark: %d ms; segDistZ1x2: %d ms; segOther: %d ms\n", msecPrep, msecMark, msecSegmDist1, msecOther);
#endif

	}
	preState = st;
	return t_r;
}

bool SegmentBasedDetector::isStopDetected() const {
	return stopDetected;
}

TrackResult SegmentBasedDetector::calcResult() {
	TrackResult out(NONE);
	if (trckVec.size() < 2) {
		return out;
	}

	const float kmInPix = (zoneLengthKm > 0) ? zoneLengthKm / zoneLengthPix : 1.0f;

	float sumSpd(0), sumLen(0), spd(-1.0f), sumSpdNonStop(0), sec;
	int numSpd(0), numLen(0), numSpdNonStop(0);
	timeval resultTime;
	for (size_t ii = 1; ii < trckVec.size(); ++ii) {
		TrackData &curD = trckVec[ii];
		timersub(&curD.tt, &trckVec[ii - 1].tt, &resultTime);
		sec = resultTime.tv_sec + resultTime.tv_usec * ONE_MILLIONTH;
		if (curD.deltaEnter < 0 && curD.deltaExit < 0) {
			if (numSpdNonStop <= 0 || curD.distExit > 0) {
				continue;
			}

			sumSpdNonStop /= numSpdNonStop;
			curD.deltaExit = sumSpdNonStop * sec;
			curD.distExit = -curD.deltaExit;
			for (size_t jj = ii+1; jj < trckVec.size(); ++jj) {
				if (trckVec[jj].deltaEnter < 0) {
					timersub(&trckVec[jj].tt, &trckVec[jj - 1].tt, &resultTime);
					float sec2 = resultTime.tv_sec + resultTime.tv_usec * ONE_MILLIONTH;
					trckVec[jj].deltaExit = sumSpdNonStop * sec2;
				}
				else {
					trckVec[jj].deltaExit = trckVec[jj].deltaEnter;
				}
				trckVec[jj].distExit = trckVec[jj - 1].distExit - trckVec[jj].deltaExit;
			}
			sumSpdNonStop = 0;
			numSpdNonStop = 0;
		}

		if (curD.deltaEnter < 0) {
			spd = curD.deltaExit / sec;
		}
		else if (curD.deltaExit < 0) {
			spd = curD.deltaEnter / sec;
		}
		else {
			spd = (curD.deltaEnter + curD.deltaExit) / 2.0f / sec;
			sumLen += 2.0f*zoneLengthPix - curD.distEnter - curD.distExit;
			++numLen;
		}

		if (curD.stopped == false) {
			sumSpdNonStop += spd;
			++numSpdNonStop;
		}
		sumSpd += spd;
		++numSpd;
	}

	if (numSpd > 0) {
		out.speedKmH = (sumSpd / numSpd) * kmInPix * 3600.0f;
	}
	else {
		out.speedKmH = 0;
	}

	if (numLen > 0) {
		out.lengthM = (sumLen / numLen) * kmInPix * 1000.0f;
	}
	else {
		out.lengthM = 0;
	}

	out.st = RESULT;
	return out;
}

#endif //TraffiXtream, TraffiX_TRACKER

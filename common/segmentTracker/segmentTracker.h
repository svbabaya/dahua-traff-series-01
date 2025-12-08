#ifndef __SEGMENT_BASED_TRACKER_H__
#define __SEGMENT_BASED_TRACKER_H__

#include "_common.h"
#if defined(TraffiXtream) && defined(TraffiX_TRACKER)

#include "datastructs.h"
#include "utils_traff.h"
#include "marking.h"

#include <opencv2/core/core.hpp>

#if defined(_ProjCam_) && !defined(WIN32)
	#include <sys/time.h>
#else
	#include "systime.h"
#endif

class SegmentBasedDetector {
public:
	struct NormLineCoefs {
		int na, nb, nc;
		float precision;
		inline int mult_dist(int y, int x) const { 
			return abs(na*y + nb*x + nc);
		}
		inline float mult_dist2float(int multDist) const {
			return float(multDist) / precision;
		}
	};
private:
	struct TrackData {
		float distEnter, distExit, deltaEnter, deltaExit;
		bool stopped;
		timeval tt;
		TrackData(const timeval &t);
		void init(const float &distEnt, const float &distExi, const TrackData *pPreData);
	};
	bool init;
	float zoneLengthPix, zoneLengthKm;
	NormLineCoefs exitLine_z1, exitLine_z2, enterLine_z1, enterLine_z2;
	std::vector<TrackData> trckVec;
	bool stopDetected, z2enter;
	SensorState preState;
	int z2enterIdx;
public:
	SegmentBasedDetector();
	void resetTrack();
	void clear();
	void initSensorInfo(float zonLengthPix, float zonLengthKm,
						const TraffPoint &z1enterP1, const TraffPoint &z1enterP2,
						const TraffPoint &z2exitP1, const TraffPoint &z2exitP2,
						const TraffPoint &z2enterP1, const TraffPoint &z2enterP2,
						const TraffRect &z1rr, const TraffRect &z2rr);
	TrackResult procState(SensorState st, const timeval &eventTime,
						  const uchar *z1motionMask, const TraffRect &inz1rr,
						  const uchar *z2motionMask, const TraffRect &inz2rr);
	bool isStopDetected() const;

private:
	TrackResult calcResult();
	static void calcNormLineCoefs(NormLineCoefs &line, const cv::Point2f &p1, const cv::Point2f &p2, const TraffRect &roi);
	static void prepareImages(cv::Mat &zBin, TraffRect &zRoi, const uchar *inz, const TraffRect &inzRoi);
	static bool getSegm(CSegParamsRect &segm, cv::Mat &zone);
	static float segm2lineDist(const CSegParamsRect &segm, const cv::Mat &zone, const NormLineCoefs &line);
};

#endif //TraffiXtream, TraffiX_TRACKER
#endif //__SEGMENT_BASED_TRACKER_H__

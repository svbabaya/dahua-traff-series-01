#ifndef __FEATURE_BASED_TRACKER_H__
#define __FEATURE_BASED_TRACKER_H__

#include "_common.h"
#if defined(TraffiXtream) && defined(TraffiX_TRACKER)

#include "datastructs.h"
#include "featureReg.h"

class FeatureBasedDetector {
private:
	float sensorDirAngle, zoneLengthPix, zoneLengthKm;
	cv::Point2f z1exitLineP1, z1exitLineP2;
	float tempLengthKm;
	RowMat<uchar> zoneMask, zoneImage;
	TraffRect rrFZ;
	FeatureRegister freg1;
	bool enter, stopDetected;
	SensorState preState;

#ifdef FeatureDetExtCV
	RowMat<uchar> image1, image2;
	TraffRect rr1, rr2;
#endif

public:
	FeatureBasedDetector();
	void clear();
	void initSensorInfo(float sensDirAngle, float zonLengthPix, float zonLengthKm,
						float z1exitY1, float z1exitX1, float z1exitY2, float z1exitX2);
	void initFeatureDet(const std::vector<TraffPoint> &pointList, float tempLength_Km);
	void initExitLine();
	TrackResult procState(SensorState st, const timeval &eventTime, const RowMat<uchar> &frame,
						  const RowMat<uchar> &z1motionMask, const TraffRect &z1rr,
						  const RowMat<uchar> &z2motionMask, const TraffRect &z2rr);
	bool isStopDetected() const;
private:
	void clearSensorInfo();
	void clearFeatureDet();
	TraffRect rectFromBinMask(cv::Mat &mask4detect,
							  int frameH, int frameW, const uchar *inpMask, const TraffRect &rrm);
	TrackResult procTrack(SensorState st, const timeval &eventTime,
						  const RowMat<uchar> &frame,
						  const RowMat<uchar> &z1, const TraffRect &z1rr,
						  const RowMat<uchar> &z2, const TraffRect &z2rr);

#ifdef FeatureDetExtCV
	void procMatch(SensorState st, const timeval &eventTime,
				   const RowMat<uchar> &frame,
				   const RowMat<uchar> &z1, const TraffRect &z1rr,
				   const RowMat<uchar> &z2, const TraffRect &z2rr);
#endif

};

#endif //TraffiXtream, TraffiX_TRACKER
#endif //__FEATURE_BASED_TRACKER_H__

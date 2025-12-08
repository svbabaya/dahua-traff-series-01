#ifndef __FEATURE_REGISTER_H__
#define __FEATURE_REGISTER_H__

#include "_common.h"
#if defined(TraffiXtream) && defined(TraffiX_TRACKER)

#include "utils_traff.h"

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#if defined(_ProjCam_) && !defined(WIN32)
	#include <sys/time.h>
#else
	#include "systime.h"
#endif

const float PI = 3.14159265f;
const float halfPI = PI / 2.0f;

inline float dist(float dy, float dx) {
	return sqrt(dy * dy + dx * dx);
}

class FeatureRegister {
public:
	enum Detector {
		dSURF, dSIFT, dORB, dBRISK, dFAST, dSTAR, dMSER, dGFTT, dHARRIS
	};
	enum Extractor {
		eSURF, eSIFT, eORB, eBRISK, eBRIEF, eFREAK, eNone
	};
	enum Matcher {
		mBruteForce, mFlann, mFlannKnn, mNone
	};
private:
	Detector parDet;
	float parDet_FirstParam;
	cv::Size parTrack_wnd;
	int parTrack_pyrLvl, parTrack_frameProcFreq;
	bool parTrack_ForwardBackward;
	float parTrack_sensorDirAngle;
	int trackCounter;
	std::vector<cv::KeyPoint> match_keypoints1;

#ifdef FeatureDetExtCV
	Extractor parExt;
	Matcher parMatch;
	float parMatch_FeatureDistThr;
	int parMatch_BFNorm;
	cv::Mat image1, image2;
	cv::Mat match_descriptors1, match_descriptors2;
	std::vector<cv::KeyPoint> match_keypoints2;
#endif

	struct TrackPoint {
		cv::Point2f pt;
		timeval time;
		size_t initIdx;
		cv::Point2f delta;
		cv::Point2i dir;
		float dirAngle, distance;
		float sumSpeed_PixSec, sumSpeedAfterStop_PixSec;
		size_t frameNum, frameNumAfterStop;
		TrackPoint(cv::Point2f pt, size_t initIdx, const timeval& curtime);
		TrackPoint(cv::Point2f pt, const TrackPoint &prePt, const timeval& curtime);
		bool isStop() const;
		bool isStop(float minShiftForMove) const;
		void incDirections(int &p_y, int &n_y, int &z_y, int &p_x, int &n_x, int &z_x) const;
	private:
		static int direction(float delta);
	};

	struct TrackFrameBase {
		cv::Mat img;
		std::vector<TrackPoint> tpoints;
	};

	struct TrackFrame {
		cv::Mat img;
		std::vector<cv::Mat> pyr;
		std::vector<cv::Point2f> points;
		std::vector<TrackPoint> tpoints;
		void swap(TrackFrame &other);
	};

	TrackFrame timage1, timage2;

	std::vector<TrackFrameBase> trackFrames;
	size_t trackFrameN, trackEtalonPtsNum;
	struct TrackParams {
		bool init, forward, lost, stop;
		float angle;
		TrackParams();
		void updateAngle(const std::vector<TrackPoint> &tps, const std::vector<TrackPoint> &tpsOnStart, float sensorDirAngle);
	} trackZoneParams;
public:
	FeatureRegister();
	~FeatureRegister();
	void setParameters(Detector det, float detFirstPar,
						Extractor ext,
						Matcher mtch, float thrFeatureDistance,
						cv::Size trckWnd, int trckPyrLvl, int trckFrameProcFreq,
						bool trckForwardBackward, float trckSensorDirAngle);

#ifdef FeatureDetExtCV
	void setImage1(bool fObj, const cv::Mat &img);
	void matchImage2(bool fObj, const cv::Mat &img);
#endif

	void setTrackerObj(const cv::Mat &objImg, const cv::Mat &zoneImg, cv::Point2f shft, const cv::Mat &mask4detector,
					   const timeval &curtime);
	TrackState trackObj(const cv::Mat &zoneImg, const timeval &curtime, size_t idx = 0);
	void backTrackObj(const cv::Mat &objImg, const cv::Mat &zoneImg, cv::Point2f shft);
	void stopTrack();
	TrackState getTrackState() const;
	bool getDirectionAngle(float &angle) const;
	void calcResultParams(float &speedKmH, float &lengthM, float kmInPix,
						  const cv::Point2f &z1exitP1, const cv::Point2f &z1exitP2, const timeval &curtime);
private:
	void calcKpAndDesc(std::vector<cv::KeyPoint>& kp, cv::Mat& desc, const cv::Mat& img, const cv::Mat &mask) const;
	static void keyPoints2points(const std::vector<cv::KeyPoint>& kpts, std::vector<cv::Point2f>& pts, cv::Point2f shft);
	static void points2keyPoints(const std::vector<cv::Point2f>& pts, std::vector<cv::KeyPoint>& kpts, cv::Point2f shft);
	void initTrackerObj(const cv::Mat &objImg, const cv::Mat &zoneImg, cv::Point2f shft, const cv::Mat &mask4detector,
						const timeval &curtime);
	static void filtOnStop(std::vector<TrackPoint> &myPoints);
	static void filtOnMove(std::vector<TrackPoint> &myPoints);
	static void filtOnDirection(std::vector<TrackPoint> &myPoints, char coord, int dir);
	static void filtOnDirectionAngle(std::vector<TrackPoint> &myPoints, float etalonAngle);
	static void filtAnomalMove(std::vector<TrackPoint> &myPoints);
	static void resetSpeedOnStop(std::vector<TrackPoint> &myPoints);
};

#endif //TraffiXtream, TraffiX_TRACKER
#endif //__FEATURE_REGISTER_H__

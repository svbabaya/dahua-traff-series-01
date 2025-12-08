#include "featureReg.h"

#if defined(TraffiXtream) && defined(TraffiX_TRACKER)

#include <opencv2/video/video.hpp>

#ifdef FeatureDetExtCV
	#include <opencv2/highgui/highgui.hpp>
#endif

#ifdef WIN32
	#include "global_saver.h"
	#include "systime.h"
	#include <iostream>
	CTimeAndPause global_timer2;
#endif

const float ONE_MILLIONTH = 1.0f / 1e6f;
const float track_pointNumCoef(0.6f);
const float track_pointNumLostCoef(0.3f);
const float track_deltaAngleRadian(0.35f);
const float track_deltaSensorAngleRadian(0.7f);
const size_t track_minPointsForCalcAngle(7);
const float track_minCoordShift(1.0f);
const float track_maxShiftStdDev(4.0f);


#define max(a,b) (((a) > (b)) ? (a) : (b))
#define min(a,b) (((a) < (b)) ? (a) : (b))

bool boxIntersection(float minX1, float maxX1, float minX2, float maxX2,
					 float minY1, float maxY1, float minY2, float maxY2) {
	if (minX1 > maxX2 || minX2 > maxX1 || minY1 > maxY2 || minY2 > maxY1) {
		return false;
	}
	else {
		return true;
	}
}

bool lineSegmentsIntersect(float &x, float &y,
						   float x1, float y1, float x2, float y2,
						   float x3, float y3, float x4, float y4) {
	x = 0;
	y = 0;
	if (false == boxIntersection(min(x1, x2), max(x1, x2), min(x3, x4), max(x3, x4),
								 min(y1, y2), max(y1, y2), min(y3, y4), max(y3, y4))) {
		x = 0;
		y = 0;
		return false;
	}
	const float denom  = (y4 - y3) * (x2 - x1) - (x4 - x3) * (y2 - y1);
	const float numerA = (x4 - x3) * (y1 - y3) - (y4 - y3) * (x1 - x3);
	const float numerB = (x2 - x1) * (y1 - y3) - (y2 - y1) * (x1 - x3);

	if (abs(numerA) < 2 * FLT_EPSILON && abs(numerB) < 2 * FLT_EPSILON && abs(denom) < 2 * FLT_EPSILON) {
		std::vector<float> xx(4);
		xx[0] = x1; xx[1] = x2; xx[2] = x3; xx[3] = x4;
		std::sort(xx.begin(), xx.end());
		if (xx[0] == xx[1]) {
			x = xx[2];
		}
		else if (xx[2] == xx[3]) {
			x = xx[1];
		}
		else {
			x = (xx[1] + xx[2]) / 2;
		}
		
		std::vector<float> yy(4);
		yy[0] = y1; yy[1] = y2; yy[2] = y3; yy[3] = y4;
		std::sort(yy.begin(), yy.end());
		if (yy[0] == yy[1]) {
			y = yy[2];
		}
		else if (yy[2] == yy[3]) {
			y = yy[1];
		}
		else {
			y = (yy[1] + yy[2]) / 2;
		}
		return true;
	}

	if (abs(denom) < 2 * FLT_EPSILON) {
		x = 0;
		y = 0;
		return false;
	}

	const float muA = numerA / denom;
	const float muB = numerB / denom;

	if (muA < 0 || muA > 1 || muB < 0 || muB > 1) {
		x = 0;
		y = 0;
		return false;
	}

	x = x1 + muA * (x2 - x1);
	y = y1 + muA * (y2 - y1);
	return true;
}

#ifdef FeatureDetExtCV
	VideoWriter vmfiles;
	void drawPoints(const cv::Mat &image1, const std::vector<cv::KeyPoint> &keypoints1, const cv::Mat &image2, const std::vector<cv::KeyPoint> &keypoints2,
					const std::vector<DMatch> &good_matches, const std::string &wndName, bool keyPress);
	void videomatchesInit(cv::Size iSz);
	void videomatchesWrite(const cv::Mat& frame);
	void videomatchesStop();
#endif

FeatureRegister::FeatureRegister() {

#ifdef LOGINFO
	LOGINFO("---- PARALLEL: NumTh=%d; NumCPU=%d; useOptimized=%d\n", cv::getNumThreads(), cv::getNumberOfCPUs(), cv::useOptimized());
#endif

	parDet = dORB;
	parDet_FirstParam = 200;

#ifdef FeatureDetExtCV
	parExt = Extractor::eSURF;
	parMatch = Matcher::mBruteForce;
	parMatch_FeatureDistThr = 0.2f;
	parMatch_BFNorm = cv::NORM_L2;
#endif

	parTrack_wnd = cv::Size(15, 15);
	parTrack_pyrLvl = 3;
	parTrack_frameProcFreq = 1;
	parTrack_ForwardBackward = false;
	parTrack_sensorDirAngle = 0;
	trackFrameN = 0;
	trackEtalonPtsNum = 0;
}

FeatureRegister::~FeatureRegister() {
#ifdef FeatureDetExtCV
	videomatchesStop();
#endif
}

void FeatureRegister::setParameters(Detector det, float detFirstPar,
									Extractor ext,
									Matcher mtch, float thrFeatureDistance,
									cv::Size trckWnd, int trckPyrLvl, int trckFrameProcFreq,
									bool trckForwardBackward, float trckSensorDirAngle) {
	parDet = det;
	parDet_FirstParam = detFirstPar; //SURF 200, SIFT 150, ORB 100, dGFTT 100
	
#ifdef FeatureDetExtCV
	parExt = ext;
	parMatch = mtch;
	parMatch_FeatureDistThr = thrFeatureDistance;
	if (ext == eORB || ext == eBRISK || ext == eBRIEF || ext == eFREAK) {
		parMatch_BFNorm = cv::NORM_HAMMING;
	}
	else {
		parMatch_BFNorm = cv::NORM_L2;
	}
#endif

	parTrack_wnd = trckWnd;
	parTrack_pyrLvl = trckPyrLvl;
	parTrack_frameProcFreq = trckFrameProcFreq;
	parTrack_ForwardBackward = trckForwardBackward;
	parTrack_sensorDirAngle = trckSensorDirAngle;
}

#ifdef FeatureDetExtCV
void FeatureRegister::setImage1(bool fObj, const cv::Mat &img) {
	if (fObj) {
		image1 = img;
		calcKpAndDesc(match_keypoints1, match_descriptors1, image1, cv::Mat());
	}
	else {
		image2 = img;
		calcKpAndDesc(match_keypoints2, match_descriptors2, image2, cv::Mat());
	}
}

void FeatureRegister::matchImage2(bool fObj, const cv::Mat &img) {
	setImage1(fObj, img);
	std::vector<DMatch> good_matches;
	switch (parMatch) {
		case mBruteForce: {
			std::vector<DMatch> matches;
			cv::BFMatcher matcher(parMatch_BFNorm, true);
			matcher.match(match_descriptors1, match_descriptors2, matches);
			for (int i = 0; i < matches.size(); ++i) {
				if (matches[i].distance < parMatch_FeatureDistThr) {
					good_matches.push_back(matches[i]);
				}
			}
			break;
		}
		case mFlann:
		case mFlannKnn: {
			cv::Ptr<flann::IndexParams> pf = new flann::KDTreeIndexParams(5);
			cv::Ptr<flann::SearchParams> ps = new flann::SearchParams(50);
			const float knn_thresh(0.7f);

			cv::FlannBasedMatcher matcher(pf, ps);
			if (parMatch == mFlann) {
				std::vector<DMatch> matches;
				matcher.match(match_descriptors1, match_descriptors2, matches);
				for (int i = 0; i < matches.size(); ++i) {
					if (matches[i].distance < parMatch_FeatureDistThr) {
						good_matches.push_back(matches[i]);
					}
				}
			}
			else {
				std::vector<std::vector<DMatch>> matches;
				matcher.knnMatch(match_descriptors1, match_descriptors2, matches, 2);
				for (size_t i = 0; i < matches.size(); ++i) {
					if (matches[i][0].distance < knn_thresh * matches[i][1].distance) {
						good_matches.push_back(matches[i][0]);
					}
				}
			}
			break;
		}
		default:
			break;
	}

 	drawPoints(image1, match_keypoints1, image2, match_keypoints2, good_matches, "Good Matches", false);
}
#endif

int FeatureRegister::TrackPoint::direction(float delta) {
	if (abs(delta) < track_minCoordShift) {
		return 0;
	}
	else if (delta < 0) {
		return -1;
	}
	else {
		return 1;
	}
}

FeatureRegister::TrackPoint::TrackPoint(cv::Point2f _pt, size_t _initIdx, const timeval& curtime)
	: pt(_pt), time(curtime), initIdx(_initIdx), delta(0, 0), dir(0, 0), dirAngle(0), distance(0),
	sumSpeed_PixSec(0), sumSpeedAfterStop_PixSec(0), frameNum(0), frameNumAfterStop(0) {}

FeatureRegister::TrackPoint::TrackPoint(cv::Point2f _pt, const TrackPoint &prePt, const timeval& curtime)
	: pt(_pt), time(curtime) {
	initIdx = prePt.initIdx;
	delta.y = pt.y - prePt.pt.y;
	delta.x = pt.x - prePt.pt.x;

	dir.y = direction(delta.y);
	dir.x = direction(delta.x);
	if (isStop()) {
		dirAngle = 0;
	}
	else {
		dirAngle = atan2(-delta.y, delta.x);
	}
	distance = dist(delta.y, delta.x);

	float sp(0);
	timeval resultTime;
	timersub(&time, &prePt.time, &resultTime);
	if (resultTime.tv_sec != 0 || resultTime.tv_usec != 0) {
		sp = distance / (resultTime.tv_sec + resultTime.tv_usec * ONE_MILLIONTH);
	}

	sumSpeed_PixSec = prePt.sumSpeed_PixSec + sp;
	sumSpeedAfterStop_PixSec = prePt.sumSpeedAfterStop_PixSec + sp;
	frameNum = prePt.frameNum + 1;
	frameNumAfterStop = prePt.frameNumAfterStop + 1;
}

bool FeatureRegister::TrackPoint::isStop() const {
	return (dir.y == 0 && dir.x == 0);
}

bool FeatureRegister::TrackPoint::isStop(float minShiftForMove) const {
	return (abs(delta.y) < minShiftForMove) && (abs(delta.x) < minShiftForMove);
}

void FeatureRegister::TrackPoint::incDirections(int &p_y, int &n_y, int &z_y, int &p_x, int &n_x, int &z_x) const {
	switch (dir.y) {
	case 0:
		++z_y;
		break;
	case -1:
		++n_y;
		break;
	case 1:
		++p_y;
		break;
	}

	switch (dir.x) {
	case 0:
		++z_x;
		break;
	case -1:
		++n_x;
		break;
	case 1:
		++p_x;
		break;
	}
}

FeatureRegister::TrackParams::TrackParams()
	: init(0), forward(1), lost(0), stop(0), angle(0) {}

void FeatureRegister::TrackFrame::swap(TrackFrame &other) {
	cv::swap(img, other.img);
	pyr.swap(other.pyr);
	points.swap(other.points);
	tpoints.swap(other.tpoints);
}

void FeatureRegister::TrackParams::updateAngle(const std::vector<TrackPoint> &tps, const std::vector<TrackPoint> &tpsOnStart, float sensorDirAngle) {
	float angle1(0);
	for (size_t ip = 0; ip < tps.size(); ++ip) {
		const TrackPoint &p(tps[ip]);
		angle1 += atan2(tpsOnStart[p.initIdx].pt.y - p.pt.y, p.pt.x - tpsOnStart[p.initIdx].pt.x);
	}
	angle1 /= float(tps.size());

	if (angle1 > sensorDirAngle + track_deltaSensorAngleRadian || angle1 < sensorDirAngle - track_deltaSensorAngleRadian) {
		return;
	}
	if (init == false) {
		init = true;
		angle = angle1;
	}
	else {
		angle = (angle + angle1) / 2.0f;
	}
}

void FeatureRegister::filtOnStop(std::vector<TrackPoint> &myPoints) {
	if (myPoints.empty()) {
		return;
	}
	std::vector<TrackPoint> tmp;
	tmp.reserve(myPoints.size());
	const float thr(track_minCoordShift * 2.0f);
	for (size_t ip = 0; ip < myPoints.size(); ++ip) {
		if (myPoints[ip].isStop(thr) == true) {
			tmp.push_back(myPoints[ip]);
		}
	}
	tmp.swap(myPoints);
}

void FeatureRegister::filtOnMove(std::vector<TrackPoint> &myPoints) {
	if (myPoints.empty()) {
		return;
	}
	std::vector<TrackPoint> tmp;
	tmp.reserve(myPoints.size());
	for (size_t ip = 0; ip < myPoints.size(); ++ip) {
		if (myPoints[ip].isStop() == false) {
			tmp.push_back(myPoints[ip]);
		}
	}
	tmp.swap(myPoints);
}

void FeatureRegister::filtOnDirection(std::vector<TrackPoint> &myPoints, char coord, int dir) {
	if (myPoints.empty()) {
		return;
	}
	std::vector<TrackPoint> tmp;
	tmp.reserve(myPoints.size());
	if (coord == 0) {
		for (size_t ip = 0; ip < myPoints.size(); ++ip) {
			if (myPoints[ip].dir.y == dir) {
				tmp.push_back(myPoints[ip]);
			}
		}
	}
	else {
		for (size_t ip = 0; ip < myPoints.size(); ++ip) {
			if (myPoints[ip].dir.x == dir) {
				tmp.push_back(myPoints[ip]);
			}
		}
	}
	tmp.swap(myPoints);
}

void FeatureRegister::filtOnDirectionAngle(std::vector<TrackPoint> &myPoints, float etalonAngle) {
	if (myPoints.empty()) {
		return;
	}
	std::vector<TrackPoint> tmp;
	tmp.reserve(myPoints.size());
	for (size_t ip = 0; ip < myPoints.size(); ++ip) {
		if (myPoints[ip].dirAngle == 0 ||
			(myPoints[ip].dirAngle < etalonAngle+track_deltaAngleRadian && myPoints[ip].dirAngle > etalonAngle-track_deltaAngleRadian)) {
			tmp.push_back(myPoints[ip]);
		}
	}
	tmp.swap(myPoints);
}

void FeatureRegister::filtAnomalMove(std::vector<TrackPoint> &myPoints) {
	if (myPoints.empty()) {
		return;
	}
	std::vector<float> shift(myPoints.size());
	for (size_t ip = 0; ip < myPoints.size(); ++ip) {
		shift[ip] = myPoints[ip].distance;
	}
	cv::Scalar sM, sStdv;
	meanStdDev(shift, sM, sStdv);
	while (!myPoints.empty() && sStdv[0] > track_maxShiftStdDev) {
		float shiftMax(-1.0f);
		size_t idx(shift.size());
		for (size_t ip = 0; ip < shift.size(); ++ip) {
			const float delta = abs(shift[ip] - sM[0]);
			if (delta > shiftMax) {
				shiftMax = delta;
				idx = ip;
			}
		}
		if (idx < shift.size()) {
			myPoints.erase(myPoints.begin() + idx);
			shift.erase(shift.begin() + idx);
			meanStdDev(shift, sM, sStdv);
		}
		else {
			break;
		}
	}
}

void FeatureRegister::resetSpeedOnStop(std::vector<TrackPoint> &myPoints) {
	for (size_t ip = 0; ip < myPoints.size(); ++ip) {
		myPoints[ip].sumSpeedAfterStop_PixSec = 0;
		myPoints[ip].frameNumAfterStop = 0;
	}
}

void FeatureRegister::initTrackerObj(const cv::Mat &objImg, const cv::Mat &zoneImg, cv::Point2f shft,
									 const cv::Mat &mask4detector, const timeval &curtime) {
	zoneImg.copyTo(timage1.img);
	cv::Mat t;
	try {
		calcKpAndDesc(match_keypoints1, t, objImg, mask4detector);
	}
	catch(cv::Exception& e) {

#ifdef LOGINFO
		const char* err_msg = e.what();
		LOGINFO("---- calcKP exception: %s\n", err_msg);
		LOGINFO("---- oH=%d, oW=%d; mH=%d, mW=%d\n", objImg.size().height, objImg.size().width, mask4detector.size().height, mask4detector.size().width);
#endif

	}
	catch (...) {

#ifdef LOGINFO
		LOGINFO("---- calcKP exception\n");
#endif

	}
	
	keyPoints2points(match_keypoints1, timage1.points, shft);

	const std::vector<cv::Point2f> &points = timage1.points;
	std::vector<TrackPoint> &tpoints = timage1.tpoints;
	tpoints.clear();
	tpoints.reserve(points.size());
	for (size_t ip = 0; ip < points.size(); ++ip) {
		tpoints.push_back(TrackPoint(points[ip], ip, curtime));
	}
	trackFrames.push_back(TrackFrameBase());
	trackFrames.back().tpoints = timage1.tpoints;
	if (parTrack_ForwardBackward) {
		timage1.img.copyTo(trackFrames.back().img);
	}
	else {
		buildOpticalFlowPyramid(timage1.img, timage1.pyr, parTrack_wnd, parTrack_pyrLvl, false);
	}

#ifdef FeatureDetExtCV
	imshow(trackZoneParams.forward ? "ForwardTrackObj" : "BackwardTrackObj", objImg);
	/*del*/points2keyPoints(timage1.points, match_keypoints1, cv::Point2f(0, 0));
	/*del*/drawPoints(timage1.img, match_keypoints1, timage1.img, match_keypoints1, std::vector<DMatch>(), "PyrKLT", true);
#endif

}

void FeatureRegister::setTrackerObj(const cv::Mat &objImg, const cv::Mat &zoneImg, cv::Point2f shft, const cv::Mat &mask4detector,
									const timeval &curtime) {
	stopTrack();
	trackCounter = 0;
	trackZoneParams.forward = true;
	initTrackerObj(objImg, zoneImg, shft, mask4detector, curtime);
}

TrackState FeatureRegister::trackObj(const cv::Mat &zoneImg, const timeval &curtime, size_t bIdx) {
	TrackState obj_state(NONE);
	if (trackFrames.empty() || timage1.points.empty() || (!parTrack_ForwardBackward && trackZoneParams.lost)) {
		return NONE;
	}
	++trackCounter;
	if (trackCounter % parTrack_frameProcFreq == 0) {
		trackCounter = 0;
	}
	else {
		return getTrackState();
	}

#ifdef PRINT_TIME
	timeval     tv_start, tv_end;
	int                msec;
#endif

	if (parTrack_ForwardBackward) {
		if (trackZoneParams.forward) {
			if (trackZoneParams.lost) {
				trackFrames.push_back(TrackFrameBase());
				zoneImg.copyTo(trackFrames.back().img);
				return NONE;
			}
			else {
				buildOpticalFlowPyramid(timage1.img, timage1.pyr, parTrack_wnd, parTrack_pyrLvl, false);
				zoneImg.copyTo(timage2.img);
				buildOpticalFlowPyramid(timage2.img, timage2.pyr, parTrack_wnd, parTrack_pyrLvl, false);
			}
		}
		else {
			buildOpticalFlowPyramid(timage1.img, timage1.pyr, parTrack_wnd, parTrack_pyrLvl, false);
			timage2.img = trackFrames[bIdx].img;
			buildOpticalFlowPyramid(timage2.img, timage2.pyr, parTrack_wnd, parTrack_pyrLvl, false);
		}
	}
	else {
		zoneImg.copyTo(timage2.img);
		buildOpticalFlowPyramid(timage2.img, timage2.pyr, parTrack_wnd, parTrack_pyrLvl, false);
	}

#ifdef PRINT_TIME
	gettimeofday(&tv_start, NULL);
#endif

	std::vector<uchar> status;
	std::vector<float> error;
	timage2.points.resize(timage1.points.size());
	for (size_t ii = 0; ii < timage1.points.size(); ++ii) {
		timage2.points[ii].x = timage1.points[ii].x + timage1.tpoints[ii].delta.x / 1.2f;
		timage2.points[ii].y = timage1.points[ii].y + timage1.tpoints[ii].delta.y / 1.2f;
	}
	calcOpticalFlowPyrLK(timage1.pyr, timage2.pyr, timage1.points, timage2.points,
						 status, error, parTrack_wnd,
						 (timage1.pyr.size() < timage2.pyr.size()) ? timage1.pyr.size()-1 : timage2.pyr.size()-1,
						 cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01), cv::OPTFLOW_USE_INITIAL_FLOW);

#ifdef PRINT_TIME
	gettimeofday(&tv_end, NULL);
	msec = (tv_end.tv_sec * 1000 + tv_end.tv_usec / 1000.0) - (tv_start.tv_sec * 1000 + tv_start.tv_usec / 1000.0);
	LOGINFO("----- tr_klt %d pts: %d ms\n", status.size(), msec);
#endif

	std::vector<TrackPoint> &tpoints2 = timage2.tpoints;
	tpoints2.clear();
	tpoints2.reserve(timage2.points.size());
	int p_y(0), n_y(0), z_y(0), p_x(0), n_x(0), z_x(0);
	for (size_t ip = 0; ip < status.size(); ++ip) {
		if (status[ip] == 1) {
			tpoints2.push_back(TrackPoint(timage2.points[ip], timage1.tpoints[ip], curtime));
			tpoints2.back().incDirections(p_y, n_y, z_y, p_x, n_x, z_x);
		}
	}
	const int all_num(tpoints2.size());
	const int all_thr(track_pointNumCoef*all_num);

	if (z_y > all_thr && z_x > all_thr) {
		if (trackFrameN < 2) {
			stopTrack();
			return NONE;
		}
		else {
			obj_state = STOP;
			trackZoneParams.stop = true;
			if (z_y != all_num || z_x != all_num) filtOnStop(tpoints2);
			resetSpeedOnStop(tpoints2);
		}
	}
	else if (tpoints2.size() > 0) {
		filtOnMove(tpoints2);
		if (trackZoneParams.init == false) {
			if (p_y > all_thr) {
				if (p_y != all_num) {
					filtOnDirection(tpoints2, 0, 1);
				}
			}
			else if (n_y > all_thr) {
				if (n_y != all_num) {
					filtOnDirection(tpoints2, 0, -1);
				}
			}
			else if (z_y > all_thr) {
				if (z_y != all_num) {
					filtOnDirection(tpoints2, 0, 0);
				}
			}

			if (p_x > all_thr) {
				if (p_x != all_num) {
					filtOnDirection(tpoints2, 1, 1);
				}
			}
			else if (n_x > all_thr) {
				if (n_x != all_num) {
					filtOnDirection(tpoints2, 1, -1);
				}
			}
			else if (z_x > all_thr) {
				if (z_x != all_num) {
					filtOnDirection(tpoints2, 1, 0);
				}
			}
			filtAnomalMove(tpoints2);
		}
		else {
			filtAnomalMove(tpoints2);
			filtOnDirectionAngle(tpoints2, trackZoneParams.forward ? trackZoneParams.angle 
																   : trackZoneParams.angle > 0 ? trackZoneParams.angle - PI	
																							   : trackZoneParams.angle + PI);
		}
		
		if (trackZoneParams.forward && trackFrameN > 1 && tpoints2.size() >= track_minPointsForCalcAngle)
			trackZoneParams.updateAngle(tpoints2, trackFrames[0].tpoints, parTrack_sensorDirAngle);

		if (tpoints2.empty() || tpoints2.size() < trackEtalonPtsNum) {
			obj_state = NONE;
			trackZoneParams.lost = true;
		}
		else {
			obj_state = MOVE;
			trackZoneParams.stop = false;
			++trackFrameN;
		}
	}
	else {
		obj_state = NONE;
		trackZoneParams.lost = true;
	}

	if (obj_state != NONE) {
		timage2.points.resize(tpoints2.size());
		for (size_t ip = 0; ip < tpoints2.size(); ++ip) {
			timage2.points[ip] = tpoints2[ip].pt;
		}

#ifdef FeatureDetExtCV
		/*del*/points2keyPoints(timage1.points, match_keypoints1, cv::Point2f(0, 0));
		/*del*/points2keyPoints(timage2.points, match_keypoints2, cv::Point2f(0, 0));
		/*del*/drawPoints(timage1.img, match_keypoints1, timage2.img, match_keypoints2, std::vector<DMatch>(), "PyrKLT", obj_state == 213123); //obj_state != STOP
#endif
		
		if (trackEtalonPtsNum == 0) {
			trackEtalonPtsNum = size_t(tpoints2.size() * track_pointNumLostCoef);
		}
	}

	timage1.swap(timage2);

	if (parTrack_ForwardBackward && trackZoneParams.forward && obj_state != STOP && !trackFrames.empty()) {
		trackFrames.push_back(TrackFrameBase());
		timage1.img.copyTo(trackFrames.back().img);
		trackFrames.back().tpoints = timage1.tpoints;
	}

	return obj_state;
}

void FeatureRegister::backTrackObj(const cv::Mat &objImg, const cv::Mat &zoneImg, cv::Point2f shft) {
	if (getTrackState() == NONE || !parTrack_ForwardBackward) {
		return;
	}
	trackZoneParams.forward = false;
	initTrackerObj(objImg, zoneImg, shft, cv::Mat(), timeval());

	if (trackFrames.size() < 2) {
		stopTrack();
		return;
	}

	trackEtalonPtsNum = 0;
	size_t iF(trackFrames.size() - 2);
	while (iF >= 0) {
		TrackState res = trackObj(cv::Mat(), timeval(), iF);
		if (res == NONE || iF == 0) {
			break;
		}
		else {
			--iF;
		}
	}

	stopTrack();
}

void FeatureRegister::stopTrack() {
	trackFrameN = trackEtalonPtsNum = 0;
	trackZoneParams.lost = trackZoneParams.stop = false;
	trackFrames.clear();
	timage1.points.clear();
	timage1.tpoints.clear();
	timage2.points.clear();
	timage2.tpoints.clear();
}

TrackState FeatureRegister::getTrackState() const {
	if (trackZoneParams.lost) {
		return LOST;
	}
	else if (trackZoneParams.stop) {
		return STOP;
	}
	else {
		return trackFrames.empty() ? NONE : MOVE;
	}
}

bool FeatureRegister::getDirectionAngle(float &angle) const {
	angle = trackZoneParams.angle;
	return trackZoneParams.init;
}

void FeatureRegister::calcResultParams(float &speedKmH, float &lengthM, float kmInPix,
									   const cv::Point2f &z1exitP1, const cv::Point2f &z1exitP2, const timeval &curtime) {
	speedKmH = lengthM = 0;
	std::vector<TrackPoint> *pPts;
	TrackState tst = getTrackState();
	switch (tst) {
	case LOST:
		pPts = &timage2.tpoints;
		break;
	case STOP:
	case MOVE:
		pPts = &timage1.tpoints;
		break;
	default:
		pPts = NULL;
		break;
	}

	if (pPts == NULL || pPts->size() < 1 || (*pPts)[0].frameNum < 1) {
		stopTrack();
		return;
	}

	float x1(0), y1(0), x2(0), y2(0);
	for (size_t ii = 0; ii < pPts->size(); ++ii) {
		x1 += trackFrames[0].tpoints[(*pPts)[ii].initIdx].pt.x;
		y1 += trackFrames[0].tpoints[(*pPts)[ii].initIdx].pt.y;
		x2 += (*pPts)[ii].pt.x;
		y2 += (*pPts)[ii].pt.y;
	}
	x1 /= float(pPts->size());
	y1 /= float(pPts->size());
	x2 /= float(pPts->size());
	y2 /= float(pPts->size());

	cv::Point2f z1exitPt(0, 0);
	bool ires = lineSegmentsIntersect(z1exitPt.x, z1exitPt.y,
										x1, y1, x2, y2,
										z1exitP1.x, z1exitP1.y, z1exitP2.x, z1exitP2.y);
	if (false == ires) {
		stopTrack();
		return;
	}

	for (size_t ii = 0; ii < pPts->size(); ++ii) {
		speedKmH += (*pPts)[ii].sumSpeed_PixSec / (*pPts)[ii].frameNum;
		lengthM += dist((*pPts)[ii].pt.y - z1exitPt.y, (*pPts)[ii].pt.x - z1exitPt.x);
	}
	speedKmH /= float(pPts->size());
	lengthM /= float(pPts->size());
	
	if (tst == LOST) {
		timeval resultTime;
		timersub(&curtime, &(*pPts)[0].time, &resultTime);
		if (resultTime.tv_sec != 0 || resultTime.tv_usec != 0) {
			float sp(0);
			if ((*pPts)[0].frameNumAfterStop > 0) {
				for (size_t ii = 0; ii < pPts->size(); ++ii) {
					sp += (*pPts)[ii].sumSpeedAfterStop_PixSec / (*pPts)[ii].frameNumAfterStop;
				}
				sp /= float(pPts->size());
			}
			else {
				sp = speedKmH;
			}
			lengthM += sp * (resultTime.tv_sec + resultTime.tv_usec * ONE_MILLIONTH);
		}
	}

	lengthM *= (kmInPix * 1000.0f);
	speedKmH = speedKmH * kmInPix * 3600.0f;
	stopTrack();
}

void FeatureRegister::calcKpAndDesc(std::vector<cv::KeyPoint>& kp, cv::Mat& ds, const cv::Mat& img, const cv::Mat &mask) const {
	kp.clear();
	ds.release();

#ifdef WIN_GLOBAL_IMG_SAVER
	global_timer2.tic();
#endif

	switch (parDet) {

#ifdef FeatureDetExtCV
	case dSURF: {
		float pdfp = parDet_FirstParam < 0 ? 400 : parDet_FirstParam;
		#ifdef OPENCV_FEATURES_2D_HPP //like opencv 4
			//TODO opencv_contrib
		#else
		cv::SURF detector(pdfp, 3, 2);
		detector.detect(img, kp, mask);
		#endif
		break;
	}
	case dSIFT: {
		//TODO opencv_contrib
		int pdfp = parDet_FirstParam < 0 ? 100 : int(parDet_FirstParam);
		#ifdef OPENCV_FEATURES_2D_HPP //like opencv 4
		cv::Ptr<cv::SIFT> detector = cv::SIFT::create(pdfp, 3);
		detector->detect(img, kp, mask);
		#else
		cv::SIFT detector(pdfp, 3);
		detector.detect(img, kp, mask);
		#endif
		break;
	}
#endif

	case dORB: {
		int pdfp = parDet_FirstParam < 0 ? 100 : int(parDet_FirstParam);
		int sz = max(5, round(min(img.rows, img.cols) / 5.0));

		#ifdef OPENCV_FEATURES_2D_HPP //like opencv 4
		cv::Ptr<cv::ORB> detector = cv::ORB::create(pdfp, 1.1f, 5, sz, 0, 2, cv::ORB::ScoreType::HARRIS_SCORE, sz);
		detector->detect(img, kp, mask);
		#else
		cv::ORB detector(pdfp, 1.1f, 5, sz, 0, 2, 0, sz);
		detector.detect(img, kp, mask);
		#endif
		break;
	}
	case dBRISK: {
		cv::BRISK detector;
		detector.detect(img, kp, mask);
		break;
	}
	case dFAST: {
		#ifdef OPENCV_FEATURES_2D_HPP //like opencv 4
		cv::Ptr<cv::FastFeatureDetector> detector = cv::FastFeatureDetector::create();
		detector->detect(img, kp, mask);
		#else
		cv::FastFeatureDetector detector;
		detector.detect(img, kp, mask);
		#endif
		break;
	}
	case dSTAR: {
		#ifdef OPENCV_FEATURES_2D_HPP //like opencv 4
			//TODO cv::xfeatures2d
		#else
		cv::StarFeatureDetector detector;
		detector.detect(img, kp, mask);
		#endif
		break;
	}
	case dMSER: {
		#ifdef OPENCV_FEATURES_2D_HPP //like opencv 4
		cv::Ptr<cv::MSER> detector = cv::MSER::create();
		detector->detect(img, kp, mask);
		#else
		cv::MserFeatureDetector detector;
		detector.detect(img, kp, mask);
	 	#endif
		break;
	}
	case dGFTT: {
		int pdfp = parDet_FirstParam < 0 ? 100 : int(parDet_FirstParam);
		#ifdef OPENCV_FEATURES_2D_HPP //like opencv 4
		cv::Ptr<cv::GFTTDetector> detector = cv::GFTTDetector::create(pdfp);
		detector->detect(img, kp, mask);
		#else
		cv::GoodFeaturesToTrackDetector detector(pdfp);
		detector.detect(img, kp, mask);
		#endif
		break;
	}
	case dHARRIS: {
		int pdfp = parDet_FirstParam < 0 ? 100 : int(parDet_FirstParam);
		#ifdef OPENCV_FEATURES_2D_HPP //like opencv 4
		cv::Ptr<cv::GFTTDetector> detector = cv::GFTTDetector::create(pdfp, 0.01, 1.0, 3, true);
		detector->detect(img, kp, mask);
		#else
		cv::GoodFeaturesToTrackDetector detector(pdfp, 0.01, 1.0, 3, true);
		detector.detect(img, kp, mask);
		#endif
		break;
	}
	}

#ifdef FeatureDetExtCV
	switch (parExt) {
	case eSURF: {
		#ifdef OPENCV_FEATURES_2D_HPP //like opencv 4
			//TODO opencv_contrib
		#else
		cv::SURF extractor;
		extractor.compute(img, kp, ds);
		#endif
		break;
	}
	case eSIFT: {
		cv::SIFT extractor;
		extractor.compute(img, kp, ds);
		break;
	}
	case eORB: {
		#ifdef OPENCV_FEATURES_2D_HPP //like opencv 4
		cv::Ptr<cv::ORB> extractor = cv::ORB::create();
		extractor->compute(img, kp, ds);
		#else
		cv::ORB extractor;
		extractor.compute(img, kp, ds);
		#endif
		break;
	}
	case eBRISK: {
		cv::BRISK extractor;
		extractor.compute(img, kp, ds);
		break;
	}
	case eBRIEF: {
		#ifdef OPENCV_FEATURES_2D_HPP //like opencv 4
			//TODO opencv_contrib
		#else
		cv::BriefDescriptorExtractor extractor;
		extractor.compute(img, kp, ds);
		#endif
		break;
	}
	case eFREAK: {
		#ifdef OPENCV_FEATURES_2D_HPP //like opencv 4
			//TODO cv::xfeatures2d
		#else
		cv::FREAK extractor;
		extractor.compute(img, kp, ds);
		#endif
		break;
	}

	case eNone:
	default:
		break;
	}
#endif
}

void FeatureRegister::keyPoints2points(const std::vector<cv::KeyPoint>& kpts, std::vector<cv::Point2f>& pts, cv::Point2f shft) {
	pts.resize(kpts.size());
	for (size_t ik = 0; ik < pts.size(); ++ik) {
		pts[ik] = kpts[ik].pt + shft;
	}
}

void FeatureRegister::points2keyPoints(const std::vector<cv::Point2f>& pts, std::vector<cv::KeyPoint>& kpts, cv::Point2f shft) {
	kpts.clear();
	kpts.resize(pts.size());
	for (size_t ik = 0; ik < pts.size(); ++ik) {
		kpts[ik].pt = pts[ik] + shft;
	}
}

#ifdef FeatureDetExtCV
void drawPoints(const cv::Mat &image1, const std::vector<cv::KeyPoint> &keypoints1, const cv::Mat &image2, const std::vector<cv::KeyPoint> &keypoints2,
				const std::vector<DMatch> &good_matches, const std::string &wndName, bool keyPress) {
	cv::Mat img_matches;
	drawMatches(image1, keypoints1, image2, keypoints2, good_matches, img_matches,
				cv::Scalar::all(-1), cv::Scalar::all(-1),
				std::vector<char>(), DrawMatchesFlags::DEFAULT); //::NOT_DRAW_SINGLE_POINTS
	resize(img_matches, img_matches, Size(), 2, 2);
	putText(img_matches, std::to_string(keypoints1.size()) + " - " + std::to_string(keypoints2.size()), cv::Point(10, 100), FONT_HERSHEY_SIMPLEX, 2, cv::Scalar::all(255));
	imshow(wndName, img_matches);
	waitKey(keyPress ? 0 : 1);
}

void videomatchesInit(Size iSz) {
	if (!vmfiles.isOpened()) {
		vmfiles.open("matches.avi", cv::VideoWriter::fourcc('X', 'V', 'I', 'D'), 2.0, iSz);
	}
}

void videomatchesWrite(const cv::Mat &frame) {
	vmfiles << frame;
}

void videomatchesStop() {
	vmfiles.release();
}
#endif

#endif //TraffiXtream, TraffiX_TRACKER

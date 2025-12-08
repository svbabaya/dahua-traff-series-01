#include "featureTracker.h"

#if defined(TraffiXtream) && defined(TraffiX_TRACKER)

#include "utils_cam.h"

const float scale_coef(0.5f);
const int MORPH_SIZE = 3;
const int MIN_BIN_POINT_NUM = 5;

FeatureBasedDetector::FeatureBasedDetector() {
	clear();
}

void FeatureBasedDetector::clearSensorInfo() {
	sensorDirAngle = zoneLengthPix = zoneLengthKm = 0;
	z1exitLineP1 = z1exitLineP2 = cv::Point2f(0, 0);
}

void FeatureBasedDetector::clearFeatureDet() {
	tempLengthKm = 0;
	zoneMask.release();
	zoneImage.release();
	rrFZ = TraffRect();
	freg1 = FeatureRegister();
	enter = stopDetected = false;
	preState = newFrame;

#ifdef FeatureDetExtCV
	image1.release();
	image2.release();
	rr1 = rr2 = TraffRect();
#endif
}

void FeatureBasedDetector::clear() {
	clearSensorInfo();
	clearFeatureDet();
}

void FeatureBasedDetector::initSensorInfo(float sensDirAngle, float zonLengthPix, float zonLengthKm,
										  float z1exitY1, float z1exitX1, float z1exitY2, float z1exitX2) {
	clearSensorInfo();
	sensorDirAngle = sensDirAngle;
	if (zonLengthPix > 0) {
		zoneLengthPix = scale_coef * zonLengthPix;
	}
	if (zonLengthKm > 0) {
		zoneLengthKm = zonLengthKm;
	}

	z1exitLineP1 = cv::Point2f(scale_coef * z1exitX1, scale_coef * z1exitY1);
	z1exitLineP2 = cv::Point2f(scale_coef * z1exitX2, scale_coef * z1exitY2);
}

void FeatureBasedDetector::initFeatureDet(const std::vector<TraffPoint> &pointList, float tempLength_Km) {
	clearFeatureDet();

	if (pointList.empty()) {
		return;
	}

	if (tempLength_Km > 0) {
		tempLengthKm = tempLength_Km;
	}

	TraffPolygon polygon;
	polygon.setPointList(pointList);
	rrFZ = polygon.getBoundingRect();
	rrFZ = TraffRect(scale_coef * rrFZ.x1, scale_coef * rrFZ.y1, scale_coef * rrFZ.x2, scale_coef * rrFZ.y2);
	zoneImage.create(rrFZ.height, rrFZ.width);
}

void FeatureBasedDetector::initExitLine() {
	if (rrFZ.height < 1 || rrFZ.width < 1) {
		return;
	}

	z1exitLineP1.x -= rrFZ.x1;
	z1exitLineP1.y -= rrFZ.y1;
	z1exitLineP2.x -= rrFZ.x1;
	z1exitLineP2.y -= rrFZ.y1;

	const cv::Point2f p1(z1exitLineP1), p2(z1exitLineP2);
	const cv::Point2f vec(p2.x - p1.x, p2.y - p1.y);

	if (abs(vec.x) < 2 * FLT_EPSILON && abs(vec.y) < 2 * FLT_EPSILON) {
		return;
	}

	const float a(vec.x), b(-vec.y);
	const float c = -(a * p1.y + b * p1.x);

	if (abs(vec.x) > abs(vec.y)) {
		if (vec.x > 0) {
			z1exitLineP1.x = 0;
			z1exitLineP2.x = rrFZ.width - 1;
		}
		else {
			z1exitLineP1.x = rrFZ.width - 1;
			z1exitLineP2.x = 0;
		}
		z1exitLineP1.y = (-b*z1exitLineP1.x - c) / a;
		z1exitLineP2.y = (-b*z1exitLineP2.x - c) / a;
	}
	else {
		if (vec.y > 0) {
			z1exitLineP1.y = 0;
			z1exitLineP2.y = rrFZ.height - 1;
		}
		else {
			z1exitLineP1.y = rrFZ.height - 1;
			z1exitLineP2.y = 0;
		}
		z1exitLineP1.x = (-a*z1exitLineP1.y - c) / b;
		z1exitLineP2.x = (-a*z1exitLineP2.y - c) / b;
	}
}

TrackResult FeatureBasedDetector::procState(SensorState st, const timeval &eventTime, const RowMat<uchar> &frame,
											const RowMat<uchar> &z1motionMask, const TraffRect &z1rr,
											const RowMat<uchar> &z2motionMask, const TraffRect &z2rr) {
	if (zoneImage.empty()) {
		return NONE;
	}

	TrackResult res(NONE);

#ifdef FeatureDetExtCV
	//procMatch(st, eventTime, frame, z1motionMask, z1rr, z2motionMask, z2rr);
#endif

	res = procTrack(st, eventTime, frame, z1motionMask, z1rr, z2motionMask, z2rr);
	return res;
}

bool FeatureBasedDetector::isStopDetected() const {
	return stopDetected;
}

struct isect {
	bool is;
	float y, x;
	isect(float _y, float _x)
		: is(false), y(_y), x(_x) {}
};

inline void findIsect(isect &is, float _min, float _max, bool cY) {
	if (cY) {
		is.is = (is.y >= _min-0.1f && is.y <= _max+0.1f);
	}
	else {
		is.is = (is.x >= _min-0.1f && is.x <= _max+0.1f);
	}
}

inline bool calcRectExpand(float &back, float &forw, float dist, float etalonDist) {
	back = forw = 0;
	if (fabs(etalonDist - dist) < 1.0f) {
		return false;
	}

	if (dist > etalonDist) {
		if (dist > 1.5f * etalonDist) {
			forw = -0.5f * etalonDist;
			back = 0.7f * etalonDist;
		}
		else {
			back = 0.5f * etalonDist;
		}
	}
	else {
		if (dist < 0.4f * etalonDist) {
			forw = 0.4f * etalonDist - dist;
			dist = 0.4f * etalonDist;
		}
		back = etalonDist - dist;
	}
	return true;
}

inline TraffRect shiftRect(const TraffRect &rr, const TraffRect &shift) {
	return TraffRect(rr.x1 + shift.x1, rr.y1 + shift.y1, rr.x2 + shift.x2, rr.y2 + shift.y2);
}

inline void fillPolyFromRect(std::vector<cv::Point> &poly, const TraffRect &rr) {
	// 0 ---- 1
	// |      |
	// 3 ---- 2
	if (poly.size() < 4) {
		poly.push_back(cv::Point(rr.x1, rr.y1));
		poly.push_back(cv::Point(rr.x2, rr.y1));
		poly.push_back(cv::Point(rr.x2, rr.y2));
		poly.push_back(cv::Point(rr.x1, rr.y2));
	}
	else {
		poly[0] = cv::Point(rr.x1, rr.y1);
		poly[1] = cv::Point(rr.x2, rr.y1);
		poly[2] = cv::Point(rr.x2, rr.y2);
		poly[3] = cv::Point(rr.x1, rr.y2);
	}
}

inline int checkQuadrant(float angle) {
	if (angle > -PI && angle < -halfPI) {
		return 3;
	}
	else if (angle > -halfPI && angle < 0) {
		return 4;
	}
	else if (angle > 0 && angle < halfPI) {
		return 1;
	}
	else {
		return 2;
	}
}

TraffRect FeatureBasedDetector::rectFromBinMask(cv::Mat &mask4detect,
												int frameH, int frameW, const uchar *inpMask, const TraffRect &rrm) {
	float cx(0), cy(0);
	TraffRect out;
		{
		cv::Mat m;
		cv::Mat(rrm.height, rrm.width, CV_8U, (uchar*)inpMask).copyTo(m);

#ifdef FeatureDetExtCV
		cv::imshow("morph1", m);
#endif

		{
			cv::Mat el = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(MORPH_SIZE, MORPH_SIZE));
			cv::Mat t(rrm.height, rrm.width, CV_8U);
			cv::erode(m, t, el);
			cv::dilate(t, m, el);
		}
#ifdef FeatureDetExtCV
		cv::imshow("morph2", m);
#endif

		const uchar *pMask(m.data);
		int minY(frameH), minX(frameW), maxY(-1), maxX(-1), num(0);
		for (int yy = 0; yy < rrm.height; ++yy) {
			for (int xx = 0; xx < rrm.width; ++xx) {
				if (*pMask) {
					++num;
					cy += yy;
					cx += xx;
					if (yy < minY) {
						minY = yy;
					}
					if (yy > maxY) {
						maxY = yy;
					}
					if (xx < minX) {
						minX = xx;
					}
					if (xx > maxX) {
						maxX = xx;
					}
				}
				++pMask;
			}
		}

		if (num < MIN_BIN_POINT_NUM) {
			return TraffRect(0, 0, 0, 0);
		}

		cy /= num;
		cx /= num;
		out = TraffRect(minX + rrm.x1, minY + rrm.y1, maxX + rrm.x1, maxY + rrm.y1);
		cy += rrm.y1;
		cx += rrm.x1;
	}
	
	float etalonDist;
	if (zoneLengthKm > 0) {
		etalonDist = 1.5f * (zoneLengthPix / (zoneLengthKm * 1000.0f));
	}
	else {
		etalonDist = 0.65f * zoneLengthPix;
	}

	float angle;
	if (false == freg1.getDirectionAngle(angle)) {
		angle = sensorDirAngle;
	}
	
	float distPix, back, forw;
	std::vector<cv::Point> poly;
	fillPolyFromRect(poly, out);
	if (fabs(fabs(angle) - halfPI) < 0.001f) { //~ +90 or -90 deg
		distPix = dist(out.y2 - out.y1, 0);
		if (calcRectExpand(back, forw, distPix, etalonDist)) {
			out = (angle > 0) ? shiftRect(out, TraffRect(0, -forw, 0, back)) //+90
							  : shiftRect(out, TraffRect(0, -back, 0, forw)); //-90
			fillPolyFromRect(poly, out);
		}
	}
	else { // y = -kx+b x = (y-b)/-k
		float k = tan(angle);
		if (fabs(k) < 0.001f) { //~ 0 or +-180 deg
			distPix = dist(0, out.x2 - out.x1);
			if (calcRectExpand(back, forw, distPix, etalonDist)) {
				out = (fabs(angle) > 1.0f) ? shiftRect(out, TraffRect(-forw, 0, back, 0)) //+-180
										  : shiftRect(out, TraffRect(-back, 0, forw, 0)); //0
				fillPolyFromRect(poly, out);
			}
		}
		else {
			distPix = 0;
			{
				float b = cy + k*cx;
				std::vector<isect> isv; isv.reserve(4);
				isv.push_back(isect(out.y1, (out.y1 - b) / -k));
				isv.push_back(isect(-k * out.x2 + b, out.x2));
				isv.push_back(isect(out.y2, (out.y2 - b) / -k));
				isv.push_back(isect(-k * out.x1 + b, out.x1));
				findIsect(isv[0], out.x1, out.x2, 0); //up
				findIsect(isv[1], out.y1, out.y2, 1); //right
				findIsect(isv[2], out.x1, out.x2, 0); //bottom
				findIsect(isv[3], out.y1, out.y2, 1); //left
				for (size_t i1 = 0; i1 < isv.size() - 1; ++i1) {
					if (isv[i1].is == false) {
						continue;
					}
					for (size_t i2 = i1 + 1; i2 < isv.size(); ++i2) {
						if (isv[i2].is) {
							float d = dist(isv[i1].y - isv[i2].y, isv[i1].x - isv[i2].x);
							if (d > distPix) {
								distPix = d;
							}
						}
					}
				}
			}

			if (calcRectExpand(back, forw, distPix, etalonDist)) {
				poly.clear();
				float cosx = cos(angle);
				float cosy = (angle < 0) ? sqrt(1 - cosx * cosx) : -sqrt(1 - cosx * cosx);
				out = shiftRect(out, TraffRect(forw * cosx, forw * cosy, forw * cosx, forw * cosy));
				float newBack = back + forw;
				int quad = checkQuadrant(angle);
				if (quad == 3 || quad == 1) {
					// 0 ---- 1
					// |      |
					// 3 ---- 2
					poly.push_back(cv::Point(out.x1, out.y2)); //3
					poly.push_back(cv::Point(out.x1, out.y1)); //0
					poly.push_back(cv::Point(out.x1 - newBack * cosx, out.y1 - newBack * cosy)); //0'
					poly.push_back(cv::Point(out.x2, out.y1)); //1
					poly.push_back(cv::Point(out.x2 - newBack * cosx, out.y2 - newBack * cosy)); //2'
					poly.push_back(cv::Point(out.x2, out.y2)); //2
				}
				else { //4 || 2
					poly.push_back(cv::Point(out.x2, out.y2)); //2
					poly.push_back(cv::Point(out.x1, out.y2)); //3
					poly.push_back(cv::Point(out.x1 - newBack * cosx, out.y2 - newBack * cosy)); //3'
					poly.push_back(cv::Point(out.x1, out.y1)); //0
					poly.push_back(cv::Point(out.x2 - newBack * cosx, out.y1 - newBack * cosy)); //1'
					poly.push_back(cv::Point(out.x2, out.y1)); //1
				}
				if (quad == 3 || quad == 4) {
					poly[3] += cv::Point(-newBack * cosx, -newBack * cosy); //1' || 0'
				}
				else { //1 || 2
					poly[0] += cv::Point(-newBack * cosx, -newBack * cosy); //3' || 2'
					std::swap(poly[1], poly[2]);
					std::swap(poly[4], poly[5]);
				}

				cv::Point p = poly[0];
				int minx(p.x), maxx(p.x), miny(p.y), maxy(p.y);
				for (size_t ii = 1; ii < poly.size(); ++ii) {
					p = poly[ii];
					if (p.x < minx) {
						minx = p.x;
					}
					else if (p.x > maxx) {
						maxx = p.x;
					}
					if (p.y < miny) {
						miny = p.y;
					}
					else if (p.y > maxy) {
						maxy = p.y;
					}
				}
				out = TraffRect(minx, miny, maxx, maxy);
			}
		}
	}

	for (size_t ii = 0; ii < poly.size(); ++ii) {
		poly[ii] -= cv::Point(out.x1, out.y1);
	}
	mask4detect = cv::Mat::zeros(out.height, out.width, CV_8U);
	cv::fillConvexPoly(mask4detect, poly, cv::Scalar(255));

#ifdef FeatureDetExtCV
	cv::imshow("mask4detector", mask4detect);
	cv::waitKey(10);
#endif

	int dy1 = (out.y1 < rrFZ.y1) ? rrFZ.y1 - out.y1 : 0, 
		dx1 = (out.x1 < rrFZ.x1) ? rrFZ.x1 - out.x1 : 0,
		dy2 = (out.y2 > rrFZ.y2) ? out.y2 - rrFZ.y2 : 0,
		dx2 = (out.x2 > rrFZ.x2) ? out.x2 - rrFZ.x2 : 0;
	if (dy1 != 0 || dx1 != 0 || dy2 != 0 || dx2 != 0) {
		cv::Rect roi(dx1, dy1, out.width - (dx1 + dx2), out.height - (dy1 + dy2));
		cv::Mat t;
		mask4detect(roi).copyTo(t);
		cv::swap(t, mask4detect);
		out = shiftRect(out, TraffRect(dx1, dy1, -dx2, -dy2));
	}
	return out;
}

inline void prepareImages(cv::Mat &frame, cv::Mat &sensorZone, TraffRect &sensorZoneRoi,
						  const RowMat<uchar> &inframe,
						  const RowMat<uchar> &inz, const TraffRect &inzRoi) {
	if (!inframe.empty()) {
		const int h(inframe.height()*scale_coef);
		const int w(inframe.width()*scale_coef);
		frame = cv::Mat(h, w, CV_8U);
		cv::resize(cv::Mat(inframe.height(), inframe.width(), CV_8U, (uchar*)inframe[0]), frame, frame.size(), 0, 0, cv::INTER_NEAREST);
	}

	if (!inz.empty()) {
		sensorZoneRoi = TraffRect(inzRoi.x1 * scale_coef, inzRoi.y1 * scale_coef, inzRoi.x2 * scale_coef, inzRoi.y2 * scale_coef);
		sensorZone = cv::Mat(sensorZoneRoi.height, sensorZoneRoi.width, CV_8U);
		cv::resize(cv::Mat(inzRoi.height, inzRoi.width, CV_8U, (uchar*)inz[0]), sensorZone, sensorZone.size(), 0, 0, cv::INTER_NEAREST);
	}
}

TrackResult FeatureBasedDetector::procTrack(SensorState st, const timeval &eventTime,
										    const RowMat<uchar> &inframe,
										    const RowMat<uchar> &inz1, const TraffRect &inz1rr,
										    const RowMat<uchar> &inz2, const TraffRect &inz2rr) {
	TrackResult res(NONE);
	if (st != z1Enter && st != newFrame && st != z1Exit) {
		return NONE;
	}
	const bool forwardBackward = 1;
	freg1.setParameters(FeatureRegister::dORB, 30, FeatureRegister::eNone, FeatureRegister::mNone, -1,
						cv::Size(15, 15), 2, 2, forwardBackward, sensorDirAngle);

#ifdef PRINT_TIME
	timeval     tv_start, tv_end;
	int                msec;
#endif

	cv::Mat frameImg, sZoneBin;
	TraffRect sZoneRoi;
	stopDetected = false;
	if (st == z1Enter && preState != z1Enter) {

#ifdef PRINT_TIME
		gettimeofday(&tv_start, NULL);
#endif

		prepareImages(frameImg, sZoneBin, sZoneRoi, inframe, inz1, inz1rr);
		cv::Mat mask4detector;
		TraffRect rrObj = rectFromBinMask(mask4detector, frameImg.rows, frameImg.cols, sZoneBin.data, sZoneRoi);

#ifdef PRINT_TIME
		gettimeofday(&tv_end, NULL);
		msec = (tv_end.tv_sec * 1000 + tv_end.tv_usec / 1000.0) - (tv_start.tv_sec * 1000 + tv_start.tv_usec / 1000.0);
		LOGINFO("prep+rect: %d ms\n", msec);
#endif

		if (rrObj.height < 1) {
			return NONE;
		}
		RowMat<uchar> objImg(rrObj.height, rrObj.width);
		copyZoneImageFromFullFrame(objImg, rrObj, RowMat<uchar>(frameImg.data, frameImg.rows, frameImg.cols, false));
		copyZoneImageFromFullFrame(zoneImage, rrFZ, RowMat<uchar>(frameImg.data, frameImg.rows, frameImg.cols, false));

#ifdef PRINT_TIME
		gettimeofday(&tv_start, NULL);
#endif

		freg1.setTrackerObj(cv::Mat(rrObj.height, rrObj.width, CV_8U, objImg[0]),
							cv::Mat(rrFZ.height, rrFZ.width, CV_8U, zoneImage[0]),
							cv::Point2f(rrObj.x1-rrFZ.x1, rrObj.y1-rrFZ.y1), mask4detector, eventTime);

#ifdef PRINT_TIME
		/*gettimeofday(&tv_end, NULL);
		msec = (tv_end.tv_sec * 1000 + tv_end.tv_usec / 1000.0) - (tv_start.tv_sec * 1000 + tv_start.tv_usec / 1000.0);
		LOGINFO("setobj: %d ms\n", msec);*/
#endif
#ifdef WIN_GLOBAL_IMG_SAVER
		global_saver.setTime(&eventTime);
		global_saver.saveImage(frameImg.data, frameImg.rows, frameImg.cols, "enter_full");
		global_saver.saveImage(zoneImage[0], rrFZ.height, rrFZ.width, "enter_zoneImage");
		global_saver.saveImage(objImg[0], rrObj.height, rrObj.width, "enter_Obj");
		global_saver.saveImage(sZoneBin.data, sZoneRoi.height, sZoneRoi.width, "enter_z1motionMask");
#endif

		res.st = MOVE;
	}
	else if (st == z1Exit && preState != z1Exit && freg1.getTrackState() != NONE) {
		if (forwardBackward) {
			TraffRect z2Roi = TraffRect(inz2rr.x1*scale_coef, inz2rr.y1*scale_coef, inz2rr.x2*scale_coef, inz2rr.y2*scale_coef);
			TraffRect rrObj = shiftRect(z2Roi, TraffRect(0, 0, 0, 0));
			RowMat<uchar> objImg(rrObj.height, rrObj.width);
			TraffRect tmp;
			prepareImages(frameImg, sZoneBin, tmp, inframe, RowMat<uchar>(), TraffRect());
			copyZoneImageFromFullFrame(objImg, rrObj, RowMat<uchar>(frameImg.data, frameImg.rows, frameImg.cols, false));
			copyZoneImageFromFullFrame(zoneImage, rrFZ, RowMat<uchar>(frameImg.data, frameImg.rows, frameImg.cols, false));
			freg1.backTrackObj(cv::Mat(rrObj.height, rrObj.width, CV_8U, objImg[0]),
							   cv::Mat(rrFZ.height, rrFZ.width, CV_8U, zoneImage[0]),
							   cv::Point2f(rrObj.x1-rrFZ.x1, rrObj.y1-rrFZ.y1));
		}
		else {
			float kmInPix = (zoneLengthKm > 0) ? zoneLengthKm / zoneLengthPix : 1.0f;
			freg1.calcResultParams(res.speedKmH, res.lengthM, kmInPix, z1exitLineP1, z1exitLineP2, eventTime);
		}
		res.st = RESULT;
	}
	else if (freg1.getTrackState() != NONE) {
		TraffRect tmp;
		prepareImages(frameImg, sZoneBin, tmp, inframe, RowMat<uchar>(), TraffRect());
		copyZoneImageFromFullFrame(zoneImage, rrFZ, RowMat<uchar>(frameImg.data, frameImg.rows, frameImg.cols, false));
		res.st = freg1.trackObj(cv::Mat(rrFZ.height, rrFZ.width, CV_8U, zoneImage[0]), eventTime);
	}
	preState = st;
	stopDetected = (res.st == STOP);
	return res;
}

#ifdef FeatureDetExtCV
	void FeatureBasedDetector::procMatch(SensorState st, const timeval &eventTime,
									 	 const RowMat<uchar> &frame,
										 const RowMat<uchar> &z1, const TraffRect &z1rr,
										 const RowMat<uchar> &z2, const TraffRect &z2rr) {
		if (st != z1Enter && st != z1Exit) {
			return;
		}

		freg1.setParameters(FeatureRegister::dGFTT, -1,
							FeatureRegister::eFREAK,
							FeatureRegister::mBruteForce, 500,
							cv::Size(0, 0), 0, 0, 0, 0);

		switch (st) {
		case SensorState::z1Enter:
			enter = true;
			if (image1.empty()) {
				rr1 = TraffRect(rrFZ.x1, rrFZ.y1, rrFZ.x2, z1rr.y1);
				image1.create(rr1.height, rr1.width);
			}
			copyZoneImageFromFullFrame(image1, rr1, frame);
			freg1.setImage1(false, cv::Mat(rr1.height, rr1.width, CV_8U, image1[0]));
			break;
		case SensorState::z1Exit:
			if (enter == false) {
				break;
			}
			enter = false;
			if (image2.empty()) {
				rr2 = TraffRect(rrFZ.x1, z1rr.y2, rrFZ.x2, rrFZ.y2);
				image2.create(rr2.height, rr2.width);
			}
			copyZoneImageFromFullFrame(image2, rr2, frame);
			freg1.matchImage2(true, cv::Mat(rr2.height, rr2.width, CV_8U, image2[0]));
			break;
		}
	}
#endif

#endif //TraffiXtream, TraffiX_TRACKER

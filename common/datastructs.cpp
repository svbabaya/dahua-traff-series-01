#include "datastructs.h"
#include "utils_cam.h"

#include <fstream>

using std::string;

//Camera params
string CamParams::params2text(const CamParams &pars) {
	string config, stmp;
	config += "FrameW=" + i2str(pars.FrameW, stmp) + "\n";
	config += "FrameH=" + i2str(pars.FrameH, stmp) + "\n";
	config += "RGB_FRAME=" + i2str(pars.RGB_FRAME, stmp) + "\n";
	config += "MAX_STATISTICS_FILE_SIZE_MB=" + i2str(pars.MAX_STATISTICS_FILE_SIZE_MB, stmp) + "\n";
	config += "ID_CAM=" + pars.ID_CAM + "\n";
	config += "CAPT_TYPE=" + pars.CAPT_TYPE + "\n";
	return config;
}

CamParams CamParams::readParamsFromFile(bool &isOpened, std::string fname) {
	CamParams out;
	isOpened = false;
	//LOGINFO("readParamsFromFile() CAM:in\n");
	try {
		std::ifstream parametersFile;
		parametersFile.open(fname.c_str());
		if (parametersFile.is_open()) {
			//LOGINFO("readParamsFromFile() CAM:file is_open\n");
			isOpened = true;
			string row;
			while (getline(parametersFile, row)) {
				size_t equalIndex = row.find("=");
				if (equalIndex != string::npos) {
					string key = row.substr(0, equalIndex),
						value = row.substr(equalIndex + 1);
					if (key == "FrameW")
						out.FrameW = atoi(value.c_str());
					else if (key == "FrameH")
						out.FrameH = atoi(value.c_str());
					else if (key == "RGB_FRAME")
						out.RGB_FRAME = atoi(value.c_str());
					else if (key == "MAX_STATISTICS_FILE_SIZE_MB")
						out.MAX_STATISTICS_FILE_SIZE_MB = atoi(value.c_str());
					else if (key == "ID_CAM")
						out.ID_CAM = value;
					else if (key == "CAPT_TYPE")
						out.CAPT_TYPE = value;
				}
			}
			parametersFile.close();
		}
	}
	catch (const std::exception &e) {
		//LOGERR("Camera parameters file parsing error!\n");
	}
	//LOGINFO("readParamsFromFile() CAM:out\n");
	return out;
}

// Data storage
Frame::Frame()
	: rgb(false), yuv(false) {
	t.tv_sec = 0;
	t.tv_usec = 0;
}

Frame Frame::clone() const {
	Frame out;
	out.mats = RowMatX<uchar>::cloneData();
	out.rgb = this->rgb;
	out.yuv = this->yuv;
	out.t = this->t;
	return out;
}

void Frame::getRowPts(FramePt &st, size_t row, size_t col) const {
	if (st.C.size() != mats.size())
		st.C.resize(mats.size());
	for (size_t cc = 0; cc < mats.size(); ++cc)
		st.C[cc] = mats[cc][row] + col;
}

// Polygon
void TraffPolygon::setPointList(const std::vector<TraffPoint> &list) {
	this->pointList = list;
	computeBoundingRect();
}

TraffRect TraffPolygon::getBoundingRect() const {
	return boundingRect;
}

bool TraffPolygon::containsPoint(const TraffPoint &pt) const {
	if (pointList.empty()) {
		return false;
	}
	int winding_number(0);
	TraffPoint last_pt = pointList[0];
	TraffPoint last_start = pointList[0];
	for (size_t ii = 1; ii < pointList.size(); ++ii) {
		const TraffPoint &e = pointList[ii];
		isect_line(winding_number, last_pt, e, pt);
		last_pt = e;
	}

	// implicitly close last subpath
	if ((last_pt.x != last_start.x) && (last_pt.y != last_start.y)) {
		isect_line(winding_number, last_pt, last_start, pt);
	}
	return ((winding_number % 2) != 0);
}

void TraffPolygon::isect_line(int &winding, const TraffPoint &p1, const TraffPoint &p2, const TraffPoint &pos) const {
	float x1(p1.x), y1(p1.y), x2(p2.x), y2(p2.y), y(pos.y);
	int dir(1);
	if (fabs(y1 - y2) * 100000.f <= fmin(fabs(y1), fabs(y2))) {
		// ignore horizontal lines according to scan conversion rule
		return;
	}
	else if (y2 < y1) {
		float tmp(x2);
		x2 = x1; x1 = tmp;
		tmp = y2;
		y2 = y1; y1 = tmp;
		dir = -1;
	}

	if (y >= y1 && y < y2) {
		float x = x1 + ((x2 - x1) / (y2 - y1)) * (y - y1);
		// count up the winding number if we're
		if (x <= pos.x)
			winding += dir;
	}
}

void TraffPolygon::computeBoundingRect() {
	if (pointList.empty()) {
		boundingRect = TraffRect(0, 0, 0, 0);
		return;
	}
	TraffPoint p = pointList[0];
	int minx(p.x), maxx(p.x), miny(p.y), maxy(p.y);

	for (size_t ii = 1; ii < pointList.size(); ++ii) {
		p = pointList[ii];
		if (p.x < minx)
			minx = p.x;
		else if (p.x > maxx)
			maxx = p.x;
		if (p.y < miny)
			miny = p.y;
		else if (p.y > maxy)
			maxy = p.y;
	}
	boundingRect = TraffRect(minx, miny, maxx, maxy);
}

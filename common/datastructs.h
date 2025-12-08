#ifndef __DATASTRUCTS_H__
#define __DATASTRUCTS_H__

#include "rowMatClass.h"

#include <vector>
#include <string>
#include <sys/time.h>

// Camera params
struct CamParams {
	int FrameW, FrameH, RGB_FRAME, MAX_STATISTICS_FILE_SIZE_MB;
	std::string ID_CAM, CAPT_TYPE;
	CamParams() {
		FrameW = 640;
		FrameH = 360;
		RGB_FRAME = 1; //0 - gray, 1 - color yuv/rgb
		MAX_STATISTICS_FILE_SIZE_MB = 5; //2 5 10 15
		ID_CAM = "Traffic Detector";
		CAPT_TYPE = "I420_YUV"; //NAT, NATOLD, NV12, I420_YUV, I420_RGB, Y800, JPEG_RGB
	}
	static std::string params2text(const CamParams &pars);
	static CamParams readParamsFromFile(bool &isOpened, std::string fname);
};

// Data storage
class Frame : public RowMatX<uchar> {
public:
	struct FramePt {
		std::vector<const uchar*> C;
		inline void operator++() {
			for (size_t cc = 0; cc < C.size(); ++cc)
				++C[cc];
		}
		inline const uchar* operator[](size_t channel) const { 
			return C[channel]; 
		};
	}; 
	bool rgb, yuv;
	struct timeval t;
	Frame();
	Frame clone() const;
	void getRowPts(FramePt &st, size_t row, size_t col = 0) const;
};

// Polygon
struct TraffPoint {
	int x, y;
	TraffPoint()
		: x(0), y(0) {};
	TraffPoint(int xNew, int yNew)
		: x(xNew), y(yNew) {};
};

struct TraffRect {
	int x1, y1, x2, y2, width, height;
	TraffRect()
		: x1(0), y1(0), x2(0), y2(0), width(0), height(0) {};
	TraffRect(int x1New, int y1New, int x2New, int y2New)
		: x1(x1New), y1(y1New), x2(x2New), y2(y2New), width(x2New - x1New + 1), height(y2New - y1New + 1) {};
};

class TraffPolygon {
public:
	void setPointList(const std::vector<TraffPoint> &list);
	TraffRect getBoundingRect() const;
	bool containsPoint(const TraffPoint &pt) const;
private:
	TraffRect boundingRect;
	std::vector<TraffPoint> pointList;
	void isect_line(int &winding, const TraffPoint &p1, const TraffPoint &p2, const TraffPoint &pos) const;
	void computeBoundingRect();
};

#endif //__DATASTRUCTS_H__

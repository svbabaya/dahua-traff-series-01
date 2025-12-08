#ifndef __ROWMAT_CLASS_H__
#define __ROWMAT_CLASS_H__

#include "_common.h"

#ifndef _NO_OCV_
	#include <opencv2/core/core.hpp>
#endif

#include "mytypedefs.h" //_FloatType

#include <cstring>
#include <math.h>
#include <vector>

struct PointYX {
	int y, x;
	PointYX() : y(0), x(0) { };
	PointYX(int _y, int _x) : y(_y), x(_x) { };
	#if defined(__OPENCV_CORE_HPP__) || defined(OPENCV_CORE_HPP)
		PointYX(const cv::Point &cvPt) : y(cvPt.y), x(cvPt.x) { };
		operator cv::Point() const
		{ return cv::Point(x,y); }
	#endif

	inline const PointYX operator-()
	{ return PointYX(-y, -x); }

	inline PointYX& operator+=(const PointYX& rght)
	{ y += rght.y; x += rght.x; return *this; }

	inline PointYX& operator+=(int rght)
	{ y += rght; x += rght; return *this; }

	inline PointYX& operator-=(const PointYX& rght)
	{ y -= rght.y; x -= rght.x; return *this; }

	inline PointYX& operator-=(int rght)
	{ y -= rght; x -= rght; return *this; }

	inline PointYX& operator*=(const PointYX& rght)
	{ y *= rght.y; x *= rght.x; return *this; }

	inline PointYX& operator*=(int rght)
	{ y *= rght; x *= rght; return *this; }

	friend inline const PointYX operator+(const PointYX& left, const PointYX& rght)
	{ return PointYX(left.y + rght.y, left.x + rght.x); }

	friend inline const PointYX operator+(int left, const PointYX& rght)
	{ return PointYX(left + rght.y, left + rght.x); }

	friend inline const PointYX operator+(const PointYX& left, int rght)
	{ return PointYX(left.y + rght, left.x + rght); }

	friend inline const PointYX operator-(const PointYX& left, const PointYX& rght)
	{ return PointYX(left.y - rght.y, left.x - rght.x); }

	friend inline const PointYX operator-(int left, const PointYX& rght)
	{ return PointYX(left - rght.y, left - rght.x); }

	friend inline const PointYX operator-(const PointYX& left, int rght)
	{ return PointYX(left.y - rght, left.x - rght); }

	friend inline const PointYX operator*(const PointYX& left, const PointYX& rght)
	{ return PointYX(left.y * rght.y, left.x * rght.x); }

	friend inline const PointYX operator*(int left, const PointYX& rght)
	{ return PointYX(left * rght.y, left * rght.x); }

	friend inline const PointYX operator*(const PointYX& left, int rght)
	{ return PointYX(left.y * rght, left.x * rght); }

	friend inline bool operator==(const PointYX& left, const PointYX& rght)
	{ return (left.y == rght.y && left.x == rght.x); }

	friend inline bool operator!=(const PointYX& left, const PointYX& rght)
	{ return (left.y != rght.y || left.x != rght.x); }
};

struct RectInt {
	int yT, xL, h, w;
	RectInt() : yT(0), xL(0), h(0), w(0) { };
	RectInt(int _yT, int _xL, int _h, int _w) : yT(_yT), xL(_xL), h(_h), w(_w) {};
	inline int yB() const
	{ return yT+h-1; }
	inline int xR() const
	{ return xL+w-1; }
};

template <typename TElem>
class RowMat {
protected:
	size_t *refcounter;
	bool external_data;
	TElem *data;
	TElem **rows;
	size_t hh, ww;
public:
	RowMat();
	RowMat(size_t h, size_t w);
	RowMat(const TElem *img, size_t h, size_t w, bool copydata = false);
	RowMat(const TElem *img4roi, size_t h, size_t w,
		   size_t rY, size_t rX, size_t rH, size_t rW, bool copydata = false);
	virtual ~RowMat();
	RowMat(const RowMat&); 
	RowMat& operator=(const RowMat&);
	inline bool empty() const { return (refcounter == NULL); };
	void release();
	bool create(size_t h, size_t w);
	bool create(const TElem *img, size_t h, size_t w, bool copydata);
	RowMat clone() const;
	bool copyTo(RowMat<TElem> &out) const;

	template <typename TOut>
	bool difTypeCopy(RowMat<TOut> &out) const;

	RowMat roi(size_t rY, size_t rX, size_t rH, size_t rW) const;
	void zeros();
	void zeroBorders();
	void fill(TElem val);
	void fillBorders(TElem val);
	void fillBordersNeighborhood();
	void getMin(TElem &minVal) const;
	void getMax(TElem &maxVal) const;
	void getMinMax(TElem &minVal, TElem &maxVal) const;
	void getMean(_FloatType &mean) const;
	void getMeanStdDev(_FloatType &mean, _FloatType &stdDev) const;
	
	inline size_t height() const { return hh; };
	inline size_t width() const { return ww; };

	inline		 TElem& elem(size_t yy, size_t xx)	    { return rows[yy][xx]; };
	inline const TElem& elem(size_t yy, size_t xx)const { return rows[yy][xx]; };
	inline		 TElem& elem(PointYX pt)				{ return rows[pt.y][pt.x]; };
	inline const TElem& elem(PointYX pt)		  const { return rows[pt.y][pt.x]; };	

	inline       TElem* row(size_t yy)				{ return rows[yy]; };
	inline const TElem* row(size_t yy)		  const { return rows[yy]; };
	inline		 TElem* operator[](size_t yy)		{ return rows[yy]; };
	inline const TElem* operator[](size_t yy) const { return rows[yy]; };


	#if defined(__OPENCV_CORE_HPP__) || defined(OPENCV_CORE_HPP)
		RowMat(const cv::Mat &img, bool copydata = false);
		bool create(const cv::Mat &img, bool copydata);
		cv::Mat cvMat() const;
		inline TElem& elem(cv::Point pt)		{ return rows[pt.y][pt.x]; };
		inline TElem  elem(cv::Point pt) const	{ return rows[pt.y][pt.x]; };
	#endif
protected:
	void cleanup();
};

//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//
//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//
template <typename TElem>
void RowMat<TElem>::cleanup() {
	if (refcounter) {
		if (*refcounter == 0) {
			if (!external_data)
				delete [] data;
			delete [] rows;
			delete refcounter;
		}
		else
			--(*refcounter);
	}
	refcounter = NULL;
	data = NULL;
	rows = NULL;
	external_data = false;
	hh = ww = 0;
}


//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//
//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//
template <typename TElem>
RowMat<TElem>::RowMat()
	: refcounter(NULL), external_data(false), data(NULL), rows(NULL), hh(0), ww(0) {}

template <typename TElem>
RowMat<TElem>::RowMat(size_t h, size_t w)
	: refcounter(NULL), external_data(false), data(NULL), rows(NULL), hh(0), ww(0)
{ create(h, w); }

template <typename TElem>
RowMat<TElem>::RowMat(const TElem *img, size_t h, size_t w, bool copydata /*= false*/)
	: refcounter(NULL), external_data(false), data(NULL), rows(NULL), hh(0), ww(0)
{ create(img, h, w, copydata); }

template <typename TElem>
RowMat<TElem>::RowMat(const TElem *img4roi, size_t h, size_t w,
					  size_t rY, size_t rX, size_t rH, size_t rW, bool copydata /*= false*/)
	: refcounter(NULL), external_data(false), data(NULL), rows(NULL), hh(0), ww(0) {
	if (!img4roi || h == 0 || w == 0 || rH == 0 || rW == 0
		|| rY+rH > h || rX+rW > w)
		return;

	refcounter = new size_t;
	*refcounter = 0;
	external_data = !copydata;

	hh = rH;
	ww = rW;
	rows = new TElem*[hh];
	if (copydata) {
		data = new TElem[hh * ww];
		for (size_t yy = 0; yy < hh; ++yy) {
			rows[yy] = data + yy * ww;
			memcpy(rows[yy], img4roi + (yy+rY)*w + rX, ww * sizeof(TElem));
		}
	}
	else {
		data = (TElem*) (img4roi + rY*w + rX);
		for (size_t yy = 0; yy < hh; ++yy)
			rows[yy] = (TElem*) (img4roi + (yy+rY)*w + rX);
	}
}

template <typename TElem>
RowMat<TElem>::~RowMat()
{ cleanup(); }

template <typename TElem>
RowMat<TElem>::RowMat(const RowMat& robj) {
	refcounter = robj.refcounter;
	external_data = robj.external_data;
	data = robj.data;
	rows = robj.rows;
	hh = robj.hh;
	ww = robj.ww;
	if (refcounter)
		++(*refcounter);
}

template <typename TElem>
RowMat<TElem> &RowMat<TElem>::operator=(const RowMat& robj) {
	if (this != &robj) {
		cleanup();
		refcounter = robj.refcounter;
		external_data = robj.external_data;
		data = robj.data;
		rows = robj.rows;
		hh = robj.hh;
		ww = robj.ww;
		if (refcounter)
			++(*refcounter);
	}
	return *this;
}

template <typename TElem>
void RowMat<TElem>::release()
{ cleanup(); }


//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//
//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//
template <typename TElem>
bool RowMat<TElem>::create(size_t h, size_t w) {
	cleanup();

	if (h == 0 || w == 0)
		return false;
	hh = h;
	ww = w;
	refcounter = new size_t;
	*refcounter = 0;
	external_data = false;
	data = new TElem[hh * ww] ();
	rows = new TElem*[hh];
	for (size_t yy = 0; yy < hh; ++yy)
		rows[yy] = data + yy * ww;
	return true;
}

template <typename TElem>
bool RowMat<TElem>::create(const TElem *img, size_t h, size_t w, bool copydata) {
	cleanup();
	if (!img || h == 0 || w == 0) {
		return false;
	}
	hh = h;
	ww = w;
	refcounter = new size_t;
	*refcounter = 0;
	external_data = !copydata;
	rows = new TElem*[hh];
	if (copydata) {
		data = new TElem[hh * ww];
		for (size_t yy = 0; yy < hh; ++yy) {
			rows[yy] = data + yy * ww;
			memcpy(rows[yy], img + yy * ww, ww * sizeof(TElem));
		}
	}
	else {
		data = (TElem*) img;
		for (size_t yy = 0; yy < hh; ++yy)
			rows[yy] = (TElem*) (img + yy * ww);
	}
	return true;
}

template <typename TElem>
RowMat<TElem> RowMat<TElem>::clone() const {
	RowMat<TElem> out(hh, ww);
	for (size_t yy = 0; yy < hh; ++yy) {
		memcpy(out.rows[yy], rows[yy], ww * sizeof(TElem));
	}
	return out;
}

template <typename TElem>
bool RowMat<TElem>::copyTo(RowMat<TElem> &out) const {
	if (hh != out.height() || ww != out.width()) {
		return false;
	}
	for (size_t yy = 0; yy < hh; ++yy) {
		memcpy(out.rows[yy], rows[yy], ww * sizeof(TElem));
	}
	return true;
}

template <typename TElem> template <typename TOut>
bool RowMat<TElem>::difTypeCopy(RowMat<TOut> &out) const {
	if (hh != out.height() || ww != out.width()) {
		return false;
	}
	const TElem *pCur;
	TOut *pOut;
	for (size_t xx, yy = 0; yy < hh; ++yy) {
		pCur = rows[yy];
		pOut = out.row(yy);
		for (xx = 0; xx < ww; ++xx)
			*pOut++ = *pCur++;
	}
	return true;
}

template <typename TElem>
RowMat<TElem> RowMat<TElem>::roi(size_t rY, size_t rX, size_t rH, size_t rW) const {
	RowMat<TElem> out;
	if (rH == 0 || rW == 0 || rY+rH > hh || rX+rW > ww) {
		return out;
	}
	if (refcounter) {
		out.refcounter = new size_t;
		*(out.refcounter) = 0;
		out.external_data = true;
		out.data = rows[rY] + rX;
		out.rows = new TElem*[rH];
		out.hh = rH;
		out.ww = rW;
		for (size_t yy = 0; yy < rH; ++yy) {
			out.rows[yy] = rows[yy+rY] + rX;
		}
	}
	return out;
}

template <typename TElem>
void RowMat<TElem>::zeros() {
	if (refcounter) {
		for (size_t yy = 0; yy < hh; ++yy) {
			memset(rows[yy], 0, ww * sizeof(TElem));
		}
	}
}

template <typename TElem>
void RowMat<TElem>::zeroBorders() {
	if (refcounter) {
		memset(rows[0],    0, ww * sizeof(TElem));
		memset(rows[hh-1], 0, ww * sizeof(TElem));
		for (size_t yy = 1; yy < hh - 1; ++yy) {
			rows[yy][0] = rows[yy][ww-1] = 0;
		}
	}
}

template <typename TElem>
void RowMat<TElem>::fill(TElem val) {
	if (refcounter) {
		TElem *pCur;
		for (size_t xx, yy = 0; yy < hh; ++yy) {
			pCur = rows[yy];
			for (xx = 0; xx < ww; ++xx) {
				*pCur++ = val;
			}
		}
	}
}

template <typename TElem>
void RowMat<TElem>::fillBorders(TElem val) {
	if (refcounter) {
		for (size_t xx = 0; xx < ww; ++xx) {
			rows[0][xx] = rows[hh-1][xx] = val;
		}
		for (size_t yy = 1; yy < hh - 1; ++yy) {
			rows[yy][0] = rows[yy][ww-1] = val;
		}
	}
}

template <typename TElem>
void RowMat<TElem>::fillBordersNeighborhood() {
	if (refcounter) {
		if (ww > 2) {
			for (size_t yy = 1; yy < hh - 1; ++yy) {
				rows[yy][0]    = rows[yy][1];
				rows[yy][ww-1] = rows[yy][ww-2];
			}
		}
		if (hh > 2) {
			memcpy(rows[0],    rows[1],    ww * sizeof(TElem));
			memcpy(rows[hh-1], rows[hh-2], ww * sizeof(TElem));
		}
	}
}

template <typename TElem>
void RowMat<TElem>::getMin(TElem &minVal) const {
	if (refcounter) {
		minVal = rows[0][0];
		const TElem *pCur;
		for (size_t xx, yy = 0; yy < hh; ++yy) {
			pCur = rows[yy];
			for (xx = 0; xx < ww; ++xx) {
				if (*pCur < minVal) {
					minVal = *pCur;
				}
				++pCur;
			}
		}
	}
}

template <typename TElem>
void RowMat<TElem>::getMax(TElem &maxVal) const {
	if (refcounter) {
		maxVal = rows[0][0];
		const TElem *pCur;
		for (size_t xx, yy = 0; yy < hh; ++yy) {
			pCur = rows[yy];
			for (xx = 0; xx < ww; ++xx) {
				if (*pCur > maxVal) {
					maxVal = *pCur;
				}
				++pCur;
			}
		}
	}
}

template <typename TElem>
void RowMat<TElem>::getMinMax(TElem &minVal, TElem &maxVal) const {
	if (refcounter) {
		minVal = maxVal = rows[0][0];
		const TElem *pCur;
		for (size_t xx, yy = 0; yy < hh; ++yy) {
			pCur = rows[yy];
			for (xx = 0; xx < ww; ++xx) {
				if (*pCur > maxVal) {
					maxVal = *pCur;
				}
				else if (*pCur < minVal) {
					minVal = *pCur;
				}
				++pCur;
			}
		}
	}
}

template <typename TElem>
void RowMat<TElem>::getMean(_FloatType &mean) const {
	if (refcounter) {
		mean = 0;
		_FloatType sum;
		const TElem *pCur;
		for (size_t xx, yy = 0; yy < hh; ++yy) {
			pCur = rows[yy];
			sum = 0;
			for (xx = 0; xx < ww; ++xx) {
				sum += *pCur++;
			}
			mean += sum / ww;
		}
		mean /= hh;
	}
}

template <typename TElem>
void RowMat<TElem>::getMeanStdDev(_FloatType &mean, _FloatType &stdDev) const {
	if (refcounter) {
		mean = stdDev = 0;
		_FloatType tmp, sum, sq_sum;
		const TElem *pCur;
		for (size_t xx, yy = 0; yy < hh; ++yy) {
			pCur = rows[yy];
			sum = sq_sum = 0;
			for (xx = 0; xx < ww; ++xx) {
				tmp = *pCur++;
				sum += tmp;
				sq_sum += tmp * tmp;
			}
			mean += sum / ww;
			stdDev += sq_sum / ww;
		}

		mean   /= hh;
		stdDev /= hh;
		stdDev -= mean * mean;
		stdDev = (stdDev > 0) ? sqrt(stdDev) : 0;
	}
}

//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//
//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//
#if defined(__OPENCV_CORE_HPP__) || defined(OPENCV_CORE_HPP)

	template <typename TElem>
	RowMat<TElem>::RowMat(const cv::Mat &img, bool copydata/* = false*/)
		: refcounter(NULL), external_data(false), data(NULL), rows(NULL), hh(0), ww(0)
	{ create(img, copydata); }

	template <typename TElem>
	bool RowMat<TElem>::create(const cv::Mat &img, bool copydata) {
		cleanup();
		if (!img.data || img.rows <= 0 || img.cols <= 0) {
			return false;
		}
		hh = img.rows;
		ww = img.cols;
		refcounter = new size_t;
		*refcounter = 0;
		external_data = !copydata;

		rows = new TElem*[hh];
		if (copydata) {
			data = new TElem[hh*ww];
			for (size_t yy = 0; yy < hh; ++yy) {
				rows[yy] = data + yy * ww;
				memcpy(rows[yy], img.ptr<TElem>(yy), ww * sizeof(TElem));
			}
		}
		else {
			data = (TElem*) img.data;
			for (size_t yy = 0; yy < hh; ++yy) {
				rows[yy] = (TElem*) img.ptr<TElem>(yy);
			}
		}
		return true;
	}

	template <typename TElem>
	cv::Mat RowMat<TElem>::cvMat() const {
		cv::Mat out;
		if (refcounter) {
			out = cv::Mat(hh, ww, cv::DataType<TElem>::type);
			for (size_t yy = 0; yy < hh; ++yy) {
				memcpy(out.ptr<TElem>(yy), rows[yy], ww * sizeof(TElem));
			}
		}
		return out;
	}

#endif //__OPENCV_CORE_HPP__

template <typename TElem>
class RowMatX {
protected:
	std::vector< RowMat<TElem> > mats;
public:
	inline bool empty()  const { return mats.empty(); };
	inline size_t size() const { return mats.size(); };
	void release() { mats.clear(); }
	void resize(size_t _size) { mats.resize(_size); };
	void push(const RowMat<TElem> &_mat) { mats.push_back(_mat); };
	inline		 RowMat<TElem>& mat(size_t channel)	  			 { return mats[channel]; };
	inline const RowMat<TElem>& mat(size_t channel)		   const { return mats[channel]; };
	inline		 RowMat<TElem>& operator[](size_t channel)	     { return mats[channel]; };
	inline const RowMat<TElem>& operator[](size_t channel) const { return mats[channel]; };
	RowMatX clone() const;
	std::vector<RowMat<TElem> > cloneData() const;
};

template <typename TElem>
RowMatX<TElem> RowMatX<TElem>::clone() const {
	RowMatX<TElem> out;
	out.resize(mats.size());
	for (size_t ss = 0; ss < out.size(); ++ss) {
		out[ss] = mats[ss].clone();
	}
	return out;
}

template <typename TElem>
std::vector<RowMat<TElem> > RowMatX<TElem>::cloneData() const {
	std::vector<RowMat<TElem> > out;
	out.resize(mats.size());
	for (size_t ss = 0; ss < out.size(); ++ss) {
		out[ss] = mats[ss].clone();
	}
	return out;
}

#endif //__ROWMAT_CLASS_H__

#ifndef CAPTURE_HANDLER_H
#define CAPTURE_HANDLER_H

// #include <capture.h> // axis

#include "datastructs.h"

#include "dhop_sys.h"
#include "dhop_log.h"


class CaptureBase {
protected:
	Frame data;
public:
	CaptureBase() {};
	virtual ~CaptureBase();
	virtual void close();
	virtual bool open(int frameW, int frameH, bool color = false) = 0;
	virtual const Frame handle() = 0;
private:
	CaptureBase(const CaptureBase&);
	CaptureBase& operator=(const CaptureBase&);
};


class CaptureNat : public CaptureBase { // No support for RGB/YUV
private:
	// media_native *source; // axis
	bool old_native;
public:
	CaptureNat(bool oldNative = false);
	~CaptureNat();
	void close();
	bool open(int frameW, int frameH, bool color = false);
	const Frame handle();
private:
	CaptureNat(const CaptureNat&);
	CaptureNat& operator=(const CaptureNat&);
};


class CaptureNV12 : public CaptureBase {
protected:
	// media_stream *source; / axis
	bool init_media_stream_uncompressed(const char cap_prop[128], int frameW, int frameH, bool color);
public:
	CaptureNV12() = default; // axis
	~CaptureNV12();
	void close();
	bool open(int frameW, int frameH, bool color = false);
	const Frame handle();
private:
	CaptureNV12(const CaptureNV12&);
	CaptureNV12& operator=(const CaptureNV12&);
};


class CaptureI420_YUV : public CaptureNV12 {
public:
	CaptureI420_YUV() {};
	bool open(int frameW, int frameH, bool color = false);
	const Frame handle();
private:
	CaptureI420_YUV(const CaptureI420_YUV&);
	CaptureI420_YUV& operator=(const CaptureI420_YUV&);
};


class CaptureY800 : public CaptureNV12 {
public:
	CaptureY800() {};
	bool open(int frameW, int frameH, bool color = false);
	const Frame handle();
private:
	CaptureY800(const CaptureY800&);
	CaptureY800& operator=(const CaptureY800&);
};

#endif // CAPTURE_HANDLER_H

#include "capturehandler.h"

#include "_common.h"

#include <cstring>

CaptureBase::~CaptureBase() {
	close();
}

void CaptureBase::close() {
	data.release();
	data.t.tv_sec = data.t.tv_usec = 0;
	data.rgb = data.yuv = false;
}

CaptureNat::CaptureNat(bool oldNative /* = false */)
	: /* source(NULL), */ old_native(oldNative) {} // axis

CaptureNat::~CaptureNat() {
	close();
}

void CaptureNat::close() {
	CaptureBase::close();
	// if (source != NULL) { // axis
	// 	capture_close_native(source); // axis
	// 	source = NULL; // axis
	// } //axis
}

bool CaptureNat::open(int frameW, int frameH, bool color) {
	// color = data.rgb = data.yuv = false;
	// close();
	// if (frameW <= 0 || frameH <= 0) {
	// 	return false;
	// }

	// //LOGINFO("Capture image size: %d x %d", frameW, frameH);
	// source = capture_open_native(frameW, frameH);
	// if (source == NULL) {
	// 	LOGERR("Failed to open capture stream!");
	// 	return false;
	// }
	// capture_start_native(source);
	// data.push(RowMat<uchar>(frameH, frameW));
	// return true;

	return false; // stub for test without axis sdk functiont
}

const Frame CaptureNat::handle() {
	// if (source == NULL) {
	// 	return Frame();
	// }
	// const unsigned char *fData = (const unsigned char*)capture_get_image_native(source);
	// unsigned char *pOut = data[0][0];
	// const int sz = data[0].height() * data[0].width();
	// if (old_native) {
	// 	const unsigned char *pIn = fData + 2 * sz - 1;
	// 	for (int ii = 0; ii < sz; ++ii) {
	// 		*pOut = *pIn;
	// 		++pOut;
	// 		pIn -= 2;
	// 	}
	// }
	// else {
	// 	const unsigned char *pIn = fData + 1;
	// 	for (int ii = 0; ii < sz; ++ii) {
	// 		*pOut = *pIn;
	// 		++pOut;
	// 		pIn += 2;
	// 	}
	// }
	// gettimeofday(&(data.t), NULL);
	// return data;

	return Frame(); // stub for test without axis sdk functiont
}

// CaptureNV12::CaptureNV12() // axis
// 	: source(NULL) {} // axis

CaptureNV12::~CaptureNV12() {
	close();
}

void CaptureNV12::close() {
	CaptureBase::close();
	// if (source != NULL) { // axis
	// 	capture_close_stream(source); // axis
	// 	source = NULL; // axis
	// } // axis
}

bool CaptureNV12::init_media_stream_uncompressed(const char cap_prop[128], int frameW, int frameH, bool color) {
	// source = capture_open_stream(IMAGE_UNCOMPRESSED, cap_prop);
	// if (source == NULL) {
	// 	LOGERR("Failed to open capture stream: %s", cap_prop);
	// 	return false;
	// }

	// if (color) {
	// 	data.push(RowMat<uchar>(frameH, frameW));
	// 	data.push(RowMat<uchar>(frameH, frameW));
	// 	data.push(RowMat<uchar>(frameH, frameW));
	// }
	// else {
	// 	data.push(RowMat<uchar>(frameH, frameW));
	// }
	// return true;

	return false; // stub for test without axis sdk functiont
}

bool CaptureNV12::open(int frameW, int frameH, bool color) {
	// close();
	// if (frameW <= 0 || frameH <= 0) {
	// 	return false;
	// }
	// char cap_prop[128];
	// snprintf(cap_prop, 128, "resolution=%dx%d&sdk_format=NV12", frameW, frameH);
	// bool out = init_media_stream_uncompressed(cap_prop, frameW, frameH, color);

	// if (out) {
	// 	data.rgb = false;
	// 	data.yuv = color;
	// }
	// return out;

	return false; // stub for test without axis sdk functiont
}

const Frame CaptureNV12::handle() {
	// if (source == NULL) {
	// 	return Frame();
	// }

	// media_frame *frame = capture_get_frame(source);
	// const unsigned char* fData = (const unsigned char*)capture_frame_data(frame);
	// if (fData == NULL) {
	// 	return Frame();
	// }
	// capture_time ct = capture_frame_timestamp(frame);
	// data.t.tv_sec = ct / 1000000000;
	// data.t.tv_usec = (ct - data.t.tv_sec * 1000000000) / 1000;

	// const size_t rowsize(data[0].width() * sizeof(unsigned char));
	// const int stride(capture_frame_stride(frame));
	// if (stride == 0) {
	// 	memcpy(data[0][0], fData, data[0].height() * rowsize);
	// }
	// else {
	// 	const unsigned char *pIn(fData);
	// 	for (size_t yy = 0; yy < data[0].height(); ++yy) {
	// 		memcpy(data[0][yy], pIn, rowsize);
	// 		pIn += stride;
	// 	}
	// }

	// if (data.size() == 3) {
	// 	//>>>>> NV12 to full resolution UV
	// 	const int H = data[0].height(),
	// 			  W = data[0].width();
	// 	const unsigned char *pIn(fData + H*W);
	// 	for (int yy = 0; yy < H; yy += 2) {
	// 		unsigned char *pU1(data[1][yy]), *pU2(data[1][yy+1]),
	// 					  *pV1(data[2][yy]), *pV2(data[2][yy+1]);
	// 		for (int xx = 0; xx < W; xx += 2) {
	// 			unsigned char val = *pIn++;
	// 			*pU1++ = val;
	// 			*pU1++ = val;
	// 			*pU2++ = val;
	// 			*pU2++ = val;

	// 			val = *pIn++;
	// 			*pV1++ = val;
	// 			*pV1++ = val;
	// 			*pV2++ = val;
	// 			*pV2++ = val;
	// 		}
	// 	}
	// 	//>>>>> NV12 to full resolution UV
	// }
	// capture_frame_free(frame);
	// return data;

	return Frame(); // stub for test without axis sdk functiont
}

bool CaptureI420_YUV::open(int frameW, int frameH, bool color) {
	// close();
	// if (frameW <= 0 || frameH <= 0) {
	// 	return false;
	// }
	// char cap_prop[128];
	// snprintf(cap_prop, 128, "resolution=%dx%d&sdk_format=I420", frameW, frameH);
	// const bool res = init_media_stream_uncompressed(cap_prop, frameW, frameH, color);
	// if (res) {
	// 	data.rgb = false;
	// 	data.yuv = color;
	// }
	// // LOGINFO("CaptureI420_YUV. cap_prop=%s\n", cap_prop);
	// return res;

	return false; // stub for test without axis sdk functiont
}

const Frame CaptureI420_YUV::handle() {
	// if (source == NULL) {
	// 	return Frame();
	// }
	// media_frame *frame = capture_get_frame(source);
	// const unsigned char* fData = (const unsigned char*)capture_frame_data(frame);
	// if (fData == NULL) {
	// 	return Frame();
	// }
	// capture_time ct = capture_frame_timestamp(frame);
	// data.t.tv_sec = ct / 1000000000;
	// data.t.tv_usec = (ct - data.t.tv_sec * 1000000000) / 1000;

	// const size_t rowsize(data[0].width() * sizeof(unsigned char));
	// const int stride(capture_frame_stride(frame));
	// if (stride == 0) {
	// 	memcpy(data[0][0], fData, data[0].height() * rowsize);
	// }
	// else {
	// 	const unsigned char *pIn(fData);
	// 	for (size_t yy = 0; yy < data[0].height(); ++yy) {
	// 		memcpy(data[0][yy], pIn, rowsize);
	// 		pIn += stride;
	// 	}
	// }

	// if (data.size() == 3) {
	// 	//>>>>> I420 to full resolution UV
	// 	const int H = data[0].height(),
	// 			  W = data[0].width();
	// 	const unsigned char *pUin(fData + H*W), *pVin(fData + H*W + H*W / 4);
	// 	for (int yy = 0; yy < H; yy += 2) {
	// 		unsigned char *pU1(data[1][yy]), *pU2(data[1][yy+1]),
	// 					  *pV1(data[2][yy]), *pV2(data[2][yy+1]);
	// 		for (int xx = 0; xx < W; xx += 2) {
	// 			unsigned char val = *pUin++;
	// 			*pU1++ = val;
	// 			*pU1++ = val;
	// 			*pU2++ = val;
	// 			*pU2++ = val;

	// 			val = *pVin++;
	// 			*pV1++ = val;
	// 			*pV1++ = val;
	// 			*pV2++ = val;
	// 			*pV2++ = val;
	// 		}
	// 	}
	// 	//>>>>> I420 to full resolution UV
	// }
	// capture_frame_free(frame);
	// return data;

	return Frame(); // stub for test without axis sdk functiont
}

bool CaptureY800::open(int frameW, int frameH, bool color) {
	// color = data.rgb = data.yuv = false;
	// close();
	// if (frameW <= 0 || frameH <= 0) {
	// 	return false;
	// }
	// char cap_prop[128];
	// snprintf(cap_prop, 128, "resolution=%dx%d&sdk_format=Y800", frameW, frameH);
	// const bool res = init_media_stream_uncompressed(cap_prop, frameW, frameH, color);
	// //LOGINFO("CaptureY800. cap_prop=%s\n", cap_prop);
	// return res;

	return false; // stub for test without axis sdk functiont
}

const Frame CaptureY800::handle() {
	// if (source == NULL) {
	// 	return Frame();
	// }
	// media_frame *frame = capture_get_frame(source);
	// const unsigned char* fData = (const unsigned char*)capture_frame_data(frame);
	// if (fData == NULL) {
	// 	return Frame();
	// }
	// capture_time ct = capture_frame_timestamp(frame);
	// data.t.tv_sec = ct / 1000000000;
	// data.t.tv_usec = (ct - data.t.tv_sec * 1000000000) / 1000;
	// const size_t rowsize(data[0].width() * sizeof(unsigned char));
	// const int stride(capture_frame_stride(frame));
	// if (stride == 0) {
	// 	memcpy(data[0][0], fData, data[0].height() * rowsize);
	// }
	// else {
	// 	const unsigned char *pIn(fData);
	// 	for (size_t yy = 0; yy < data[0].height(); ++yy) {
	// 		memcpy(data[0][yy], pIn, rowsize);
	// 		pIn += stride;
	// 	}
	// }
	// capture_frame_free(frame);
	// return data;

	return Frame(); // stub for test without axis sdk functiont
}

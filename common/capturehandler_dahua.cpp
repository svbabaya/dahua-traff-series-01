#include "capturehandler.h"

#include "_common.h"

#include <cstring>

#include "dhop_sys.h"
#include "dhop_log.h"




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
	close();
	if (frameW <= 0 || frameH <= 0) {
		return false;
	}

	///////////////////////////////From sample
	DH_Int32 ret = -1;
    DHOP_YUV_Option yuvOption;
    //DHOP_YUV_FrameData2 yuvFrame;
    //DHOP_YUV_FrameData2 copyFrame;
    DHOP_YUV_CopyReq2 copyReq;
    DHOP_YUV_ResizeReq resizeReq;
    DHOP_YUV_FrameData2 resizeFrame;
    DH_Uint32 dataSize = 0;
    DH_Uint8 *pPhyAddr, *pVirAddr = NULL;
    DH_Uint8 dumpPath[] = "/home/";

  
    //2.Get YUV channel capability
	DHOP_YUV_CapInfo yuvCap;
	DH_Int32 yuvChn = 0;
    memset(&yuvCap, 0, sizeof(DHOP_YUV_CapInfo));
    ret = DHOP_YUV_getChnCaps(yuvChn, &yuvCap);
    if(0 != ret)
    {
        DHOP_LOG_ERROR("DHOP_YUV_getChnCaps fail with %#x\n", ret);
        return ret;
    }
	DHOP_LOG_INFO("DHOP_YUV_getChnCaps  yuvChn:%d\n",yuvChn);
    /************************************************************/
    /* YUV use default parameters after opening.                */
    /* Default parameters:                                      */
    /*         Resolution - yuvCap.maxWidth x yuvCap.maxHeight  */
    /*         Format     - DHOP_YUV_FMT_420SP_VU               */
    /*         Fps        - yuvCap.maxFps                       */
    /*         Depth      - 1(Supported in v1.2.0 and above)    */
    /************************************************************/
    //3.Open YUV channel
	DHOP_YUV_OpenParam yuvOpenPrm;
    DHOP_YUV_FormatParam yuvFmtPrm;
	DH_Handle yuvHdl = NULL;
	
    memset(&yuvOpenPrm, 0, sizeof(yuvOpenPrm));
    yuvOpenPrm.channel = 0;
    ret = DHOP_YUV_open(&yuvOpenPrm, &yuvHdl);
    if(0 != ret)
    {
        DHOP_LOG_ERROR("DHOP_YUV_open fail with %#x\n", ret);
        return ret;
    }
	DHOP_LOG_INFO("DHOP_YUV_open  success\n");
    /************************************************************/
    /* You can use DHOP_YUV_setFormat() & DHOP_YUV_setOption()  */
    /* to adjust parameters according to actual needs.          */
    /************************************************************/
    //4.Set YUV channel parameters
    memset(&yuvFmtPrm, 0, sizeof(yuvFmtPrm));
    yuvFmtPrm.format = DHOP_YUV_FMT_420SP_VU;
    yuvFmtPrm.fps = yuvCap.maxFps;
    yuvFmtPrm.width = frameW;//yuvCap.maxWidth;
    yuvFmtPrm.height = frameH;//yuvCap.maxHeight;
    ret = DHOP_YUV_setFormat(yuvHdl, &yuvFmtPrm);
    if(0 != ret)
    {
        DHOP_LOG_ERROR("DHOP_YUV_setFormat fail with %#x\n", ret);
        DHOP_YUV_close(&yuvHdl);
        return ret;
    }
	DHOP_LOG_INFO("DHOP_YUV_setFormat  success\n");
	
    //5.Set YUV queue depth(Supported in v1.2.0 and above)
    memset(&yuvOption, 0, sizeof(yuvOption));
    yuvOption.cbSize = sizeof(yuvOption);
    yuvOption.type = DHOP_YUV_OPT_DEPTH;
    yuvOption.option.depth = 1;
    ret = DHOP_YUV_setOption(yuvHdl, &yuvOption);
    if(0 != ret)
    {
        DHOP_LOG_ERROR("DHOP_YUV_setOption fail with %#x\n", ret);
        DHOP_YUV_close(&yuvHdl);
        return ret;
    }
	DHOP_LOG_INFO("DHOP_YUV_setOption  success\n");


	// char cap_prop[128];
	// snprintf(cap_prop, 128, "resolution=%dx%d&sdk_format=I420", frameW, frameH);
	// const bool res = init_media_stream_uncompressed(cap_prop, frameW, frameH, color);
	if (res==0) {
		data.rgb = false;
	 	data.yuv = color;
	}
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

#ifndef __UTILS_CAM_H_
#define __UTILS_CAM_H_

#include "_common.h"
#include "datastructs.h"
#include "utils_traff.h"
#include "traffsensor.h"

#ifdef _ProjCam_

#include <pthread.h>
#include <string>
#include <vector>
#include <cstring>
#include <climits>
#include <math.h>
#include <algorithm>

// Copy data
inline void copyZoneImageFromFullFrame(RowMat<uchar> &dst, const TraffRect &rR, const RowMat<uchar> &src) {
	const size_t rowsize(rR.width * sizeof(dst[0][0]));
	for (int yy = 0; yy < rR.height; ++yy) {
		memcpy(dst[yy], src[rR.y1+yy] + rR.x1, rowsize);
	}
}

inline void copyZoneImageFromFullFrame(Frame &dst, const TraffRect &rR, const Frame &src) {
	const int chnls = dst.size() < src.size() ? dst.size() : src.size();
	for (int cc = 0; cc < chnls; ++cc) {
		copyZoneImageFromFullFrame(dst[cc], rR, src[cc]);
	}
}

class CamFileWorker {
public:
	CamFileWorker();
	~CamFileWorker();

	// All functions work using a mutex (lock flag for control)
	bool writeTextToFile(bool lock, const std::string &data, std::string fileName);
	bool readTextFromFile(bool lock, std::string &data, std::string fileName);
	bool writeImg2PGM(bool lock, std::string fileName, const void *data, int width, int height, int stride);
	void setCommon(std::string nameAuth, std::string nameStatistics, std::string nameCamConfig);

	// bool readAuthFile(bool lock, std::vector<std::string> &LogPass);

	/***** New declaration */
	bool readAuthFile(bool lock, std::string& login, std::string& password);
	/*****  */

	std::string getName_CamConfig();
	bool readCamParams(bool lock, CamParams &cPar, int frameW = 0, int frameH = 0);
	std::string getCamId();
	void setUpdConfigFlag(bool flag);
	bool checkUpdConfig(bool reset); // Returns the current value and resets the flag

	// It is necessary to reinitialize all algorithms after calling setCommon, since the resolution may change
	void setTraffix(std::string nameTraffParam, std::string nameTraffConfig);
	std::string getName_TraffParam();
	std::string getName_TraffConfig();
	std::vector<TraffSensor> readTraffix(bool lock, bool &configFileOpened, TraffAlgParams &tPar, TraffAvgParams &avg);
	bool writeStatistics(bool lock, const std::vector<TraffStat> &sensors);
private:
	pthread_mutex_t mutex_files;
	std::string fileAuth;
	std::string fileStatistics;
	std::string fileCamConfig;
	CamParams camPar;
	std::string fileTraffParam;
	std::string fileTraffConfig;
	bool updConfig;
	CamFileWorker(const CamFileWorker&);
	CamFileWorker& operator=(const CamFileWorker&);
};

class Masker {
public:
	Masker(int H, int W);
	~Masker();
	void addNewPolygon(const std::vector<TraffPoint> &pointList);
	void write2PGM(std::string fname) const;
private:
	Masker(const Masker&);
	Masker& operator=(const Masker&);
	uchar *data;
	int hh, ww;
};

#endif //_ProjCam_

// i2str для double
/* We can cut double precision numbers to two integers, numbers before and after the decimal separator.
Then its a matter of converting these integers and concatenating both with decimal separator.
The only edge case we need to handle here is when the number after decimal separator has leading 0s, like 1.02.
One idea is to multiply the number after decimal separator with power of 10 (better to take 10^maximum precision).
If the number of digits is less than the power of 10(maximum precision), then we have to prepend the number with 0s.
*/

// Function for converting int to std::string
// Maximum number of digits in the decimal system for the int type
const int DEC_NUM_MAX = ceil(log10(pow(2, sizeof(int)*CHAR_BIT)));
inline std::string& i2str(int val, std::string &str) {
	if (val == 0) {
		return (str = '0');
	}
	str.clear();
	str.reserve(DEC_NUM_MAX + 1);
	if (val > 0) {
		while (val) {
			str += '0' + (val % 10);
			val /= 10;
		}
	}
	else {
		// Fix problem abs(INT_MIN) == (INT_MAX + 1)
		if (val == INT_MIN) { 
			int res(-(val + 1) % 0);
			str += (res == 9) ? '0' : ('0' + (res + 1));
			val /= 10;
		}
		val = -val;
		while (val) {
			str += '0' + (val % 10);
			val /= 10;
		}
		str += '-';
	}
	if (str.length() > 1) {
		reverse(str.begin(), str.end());
	}
	return str;
}

#endif // __UTILS_CAM_H_

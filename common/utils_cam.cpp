#include "utils_cam.h"

#ifdef _ProjCam_

#include <fstream>
#include <sys/stat.h>

using std::string;
using std::vector;

CamFileWorker::CamFileWorker()
	: updConfig(false) {
	pthread_mutex_init(&mutex_files, NULL);
}

CamFileWorker::~CamFileWorker() {
	pthread_mutex_destroy(&mutex_files);
}

bool CamFileWorker::writeTextToFile(bool lock, const string &data, string fileName) {
	bool out;
	if (lock) {
		pthread_mutex_lock(&mutex_files);
	}
	FILE *fini = fopen(fileName.c_str(), "w");
	if (fini == NULL) {
		out = false;
	}
	else {
		fputs(data.c_str(), fini);
		fclose(fini);
		out = true;
	}	
	if (lock) {
		pthread_mutex_unlock(&mutex_files);
	}
	return out;
}

bool CamFileWorker::readTextFromFile(bool lock, string &data, string fileName) {
	data = "";
	bool out;
	if (lock) {
		pthread_mutex_lock(&mutex_files);
	}
	FILE *fini = fopen(fileName.c_str(), "r");
	if (fini == NULL) {
		out = false;
	}
	else {
		data.reserve(1024);
		char str[260];
		while (fgets(str, 256, fini) != NULL) {
			data += str;
		}
		fclose(fini);
		out = true;
	}
	if (lock) {
		pthread_mutex_unlock(&mutex_files);
	}
	return out;
}

bool CamFileWorker::writeImg2PGM(bool lock, string fileName, const void *data, int width, int height, int stride) {
	bool out;
	if (lock) {
		pthread_mutex_lock(&mutex_files);
	}
	FILE *fp = fopen(fileName.c_str(), "w");
	if (fp == NULL) {
		out = false;
	}
	else {
		fprintf(fp, "P5\n");
		fprintf(fp, "# CREATOR: FileWriter\n");
		fprintf(fp, "%d %d\n", width, height);
		fprintf(fp, "%d\n", 255);
		for (int row = 0; row < height; ++row) {
			for (int column = 0; column < width; ++column) {
				fputc(((const unsigned char*)data)[row * stride + column], fp);
			}
		}
		fclose(fp);
		out = true;
	}
	if (lock) {
		pthread_mutex_unlock(&mutex_files);
	}
	return out;
}

void CamFileWorker::setCommon(string nameAuth, string nameStatistics, string nameCamConfig) {
	pthread_mutex_lock(&mutex_files);
	fileAuth = nameAuth;
	fileStatistics = nameStatistics;
	fileCamConfig = nameCamConfig;
	readCamParams(false, camPar);
  	pthread_mutex_unlock(&mutex_files);
}

/** Read auth.txt */
bool CamFileWorker::readAuthFile(bool lock, string& login, string& password) {
	bool out;
	if (lock) {
		pthread_mutex_lock(&mutex_files);
	}
	std::ifstream af;
	af.open(fileAuth.c_str());
	if (af.is_open() == false) {
		out = false;
		//LOGERR("File auth.txt isn't open\n");
	}
	else {
		out = true;
		if (af.peek() == EOF) {
			out = false;
			//LOGERR("File auth.txt is empty\n");
		}
		else {
			af >> login >> password;
		}
		af.close();
	}
	if (lock) {
		pthread_mutex_unlock(&mutex_files);
	}
	return out;
}
/** Read auth.txt end */


string CamFileWorker::getName_CamConfig() {
	string out;
	pthread_mutex_lock(&mutex_files);
	out = fileCamConfig;
	pthread_mutex_unlock(&mutex_files);
	return out;
}

bool CamFileWorker::readCamParams(bool lock, CamParams &par, int frameW, int frameH) {
	if (lock) {
		pthread_mutex_lock(&mutex_files);
	}

	// params
	bool fFileOk;
	camPar = CamParams::readParamsFromFile(fFileOk, fileCamConfig);
	if (camPar.MAX_STATISTICS_FILE_SIZE_MB < 1) {
		camPar.MAX_STATISTICS_FILE_SIZE_MB = 1;
	}

	if (frameW > 0 && frameH > 0 && (camPar.FrameW != frameW || camPar.FrameH != frameH)) {
		camPar.FrameW = frameW;
		camPar.FrameH = frameH;
		fFileOk = false;
	}
	if (!fFileOk) {
		if (writeTextToFile(false, CamParams::params2text(camPar), fileCamConfig)) {
			fFileOk = true;
		}
	}
	par = camPar;
	if (lock) {
		pthread_mutex_unlock(&mutex_files);
	}
	return fFileOk;
}

string CamFileWorker::getCamId() {
	string out;
	pthread_mutex_lock(&mutex_files);
	out = camPar.ID_CAM;
	pthread_mutex_unlock(&mutex_files);
	return out;
}

void CamFileWorker::setUpdConfigFlag(bool flag) {
	pthread_mutex_lock(&mutex_files);
	updConfig = flag;
	pthread_mutex_unlock(&mutex_files);
}

bool CamFileWorker::checkUpdConfig(bool reset) {
	bool out;
	pthread_mutex_lock(&mutex_files);
	out = updConfig;
	if (reset) {
		updConfig = false;
	}
	pthread_mutex_unlock(&mutex_files);
	return out;
}

void CamFileWorker::setTraffix(string nameTraffParam, string nameTraffConfig) {
	pthread_mutex_lock(&mutex_files);
	fileTraffParam = nameTraffParam;
	fileTraffConfig = nameTraffConfig;
	pthread_mutex_unlock(&mutex_files);
}

string CamFileWorker::getName_TraffParam() {
	string out;
	pthread_mutex_lock(&mutex_files);
	out = fileTraffParam;
	pthread_mutex_unlock(&mutex_files);
	return out;
}

string CamFileWorker::getName_TraffConfig() {
	string out;
	pthread_mutex_lock(&mutex_files);
	out = fileTraffConfig;
	pthread_mutex_unlock(&mutex_files);
	return out;
}

vector<TraffSensor> CamFileWorker::readTraffix(bool lock, bool &configFileOpened, TraffAlgParams &tPar, TraffAvgParams &avg) {
	if (lock) {
		pthread_mutex_lock(&mutex_files);
	}

	// params
	bool fFileOk;
	tPar = TraffAlgParams::readAlgParamsFromFile(fFileOk, fileTraffParam);
	if (camPar.RGB_FRAME == 0) {
		tPar.RGB_FRAME = 0;
	}
	if (!fFileOk) {
		writeTextToFile(false, TraffAlgParams::algParams2text(tPar), fileTraffParam);
	}

	// config
	vector<TraffSensor> sensors = TraffSensor::readSensorsFromFile(avg, fFileOk, fileTraffConfig, camPar, tPar);
	configFileOpened = fFileOk;
	if (lock) {
		pthread_mutex_unlock(&mutex_files);
	}
	return sensors;
}

bool CamFileWorker::writeStatistics(bool lock, const std::vector<TraffStat> &sensors) {
	bool out(false);
	if (lock) {
		pthread_mutex_lock(&mutex_files);
	}
	FILE *fd = fopen(fileStatistics.c_str(), "a");
	if (fd) {
		struct stat st;
		stat(fileStatistics.c_str(), &st);
		int mbSize = st.st_size / (1000 * 1000);
		if (mbSize >= camPar.MAX_STATISTICS_FILE_SIZE_MB) {
			fclose(fd);
			fd = fopen(fileStatistics.c_str(), "w");
		}
	}
	if (fd) {
		fseek(fd, 0L, SEEK_END);
		for (size_t ii = 0; ii < sensors.size(); ++ii) {
			fprintf(fd, "%s;%d;%d;%d;%d;%d;%d;%d;\n",
					sensors[ii].time.c_str(),
					sensors[ii].period,
					sensors[ii].id,
					sensors[ii].counter,
					sensors[ii].speed,
					sensors[ii].occup,
					sensors[ii].headway,
					sensors[ii].lastAvgCounter);
		}
		fclose(fd);
		out = true;
	}
	if (lock) {
		pthread_mutex_unlock(&mutex_files);
	}
	return out;
}

Masker::Masker(int H, int W) {
	hh = H;
	ww = W;
	if (H > 0 && W > 0) {
		data = new uchar[H*W]();
	}
	else {
		data = NULL;
	}
}

Masker::~Masker() {
	if (data) {
		delete[] data;
	}
}

void Masker::addNewPolygon(const vector<TraffPoint> &pointList) {
	if (data == NULL) {
		return;
	}
	TraffPolygon polygon;
	polygon.setPointList(pointList);
	TraffRect rr = polygon.getBoundingRect();
	for (int dy = 0; dy < rr.height; ++dy) {
		for (int dx = 0; dx < rr.width; ++dx) {
			TraffPoint p(rr.x1 + dx, rr.y1 + dy);
			if (polygon.containsPoint(p)) {
				data[p.y*ww + p.x] = 255;
			}
		}
	}
}

void Masker::write2PGM(string fname) const {
	if (data == NULL || fname.empty()) {
		return;
	}
	CamFileWorker().writeImg2PGM(false, fname, data, ww, hh, ww);
}

#endif //_ProjCam_

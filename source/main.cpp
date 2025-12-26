#include "_common.h"

#include "globals_cam.h"

#include "capturehandler.h"
#include "signalhandler.h"
#include "tcpServerSource.h"
#include "traffcounter.h"

#include <stdlib.h>
#include <unistd.h>
#include <syslog.h>
#include <libgen.h>
// #include <licensekey.h> // axis sdk library

#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ptrace.h>
#include <netinet/in.h>
#include <sys/time.h>

#include <iostream>
#include <fstream>
#include <string>
#include <vector>

void globalsInit() {
	std::string tmp;
	camApp::global_gvStr = "TraffiXtream " + i2str(MAJOR_VERSION, tmp) + "." + i2str(MINOR_VERSION, tmp);
	camApp::global_files.setCommon("auth.txt", "statistics.txt", "config_cam.txt");
	pthread_mutex_init(&camApp::global_mutex, NULL);
	camApp::global_frame.release();
	camApp::global_files.setTraffix("parameters_traff.txt", "config_traff.txt");
	traffixApp::global_clrSensorsFlag = false;
	traffixApp::global_clrSensIDs.clear();
	traffixApp::global_sensors.clear();
	camApp::global_initFlag = true;
}

void globalsClear() {
	pthread_mutex_destroy(&camApp::global_mutex);
	camApp::global_frame.release();
}

inline const int& max(const int &v1, const int &v2) { 
	return (v1 > v2) ? v1 : v2; 
}

bool commonInit(CaptureBase* &pCapt);
bool traffiXInit(TraffCounter &traffixCounter);

/* Syslog */
static void openSyslog(const char *app_name);
static void closeSyslog();
static void logCurDir();

static void daemonize();

/**
* Binding for MAC version 3.0-1
*/
/* Make hash of macCamera */
long long int encrypt(const std::string& s, long long int p, long long int mod) {
	long long int h = 0;
	for (int i = 0; i < s.size(); i++) {
		h = (h * p + (long long int)s[i]) % mod;
	}
	return h;
}
/* Read MAC address from file and compare with customer's MAC address */
bool macIsCorrect() {
	/* Data for checking */
	const long long int macCustomer = 608684250766;
	const long long int p = 911503;
	const long long int mod = 914185130743;
	const std::string pathOfAddress = "//sys//class//net//eth0//address";
	std::ifstream fin(pathOfAddress.c_str(), std::ifstream::in);
	if (!fin.is_open()) {
		// LOGINFO("The file /sys/class/net/eth0/address wasn't opened (ver. 3)\n");
		return false;
	}
	else {
		// LOGINFO("The file /sys/class/net/eth0/address is open (ver. 3)\n");
		std::string macCamera = "";
		fin >> macCamera;
		/* Delete colon from MAC address */
		const std::string to_delete = ":";
		size_t start(macCamera.find(to_delete));
		while (start != std::string::npos) {
			macCamera.erase(start, to_delete.length());
			start = macCamera.find(to_delete, start + to_delete.length());
		}
		// LOGINFO("The MAC address from the system whithout a colon: %s (ver. 3)\n", macCamera.c_str());
		if (encrypt(macCamera, p, mod) != macCustomer) {
			return false;
		}
	}
	return true;
}
/**
* Binding for MAC end
*/

DH_Void exitCbFunc(DH_Void) {
    return;
}

int main(int argc, char *argv[]) {
	const char *app_name = basename(argv[0]);
	openSyslog(app_name);
	const char *app_path = dirname(argv[0]);
	LOGINFO("%s: START. dirname: %s\n", app_name, app_path);

// Initialization for Dahua
#if DAHUA
	DH_Int32 ret = -1;
    DHOP_SYS_InitParam sysInitPrm;
	DHOP_YUV_CapInfo yuvCaps;

	// Initialize Dhop system and register callback function
    memset(&sysInitPrm, 0, sizeof(sysInitPrm));
    sysInitPrm.onExitCallback = exitCbFunc;
    ret = DHOP_SYS_init(&sysInitPrm);
    if(0 != ret) {
        DHOP_LOG_ERROR("DHOP_SYS_init fail with %#x\n", ret);
        return ret;
    }
	DHOP_LOG_setLevel(DHOP_LOG_LEVEL_DEBUG, DHOP_LOG_DEST_WEB);
	DHOP_LOG_INFO("DHOP_SYS_init success\n");
#endif

	/* Anti-debugging based on the ptrace system call */
	if (ptrace(PTRACE_TRACEME, 0, 0, 0) == -1) {
		LOGERR("%s: application is currently being debugged\n", app_name);
		closeSyslog();
		return EXIT_FAILURE;
	}

	/* Check MAC address from OS and MAC address from customer
	if (!macIsCorrect()) {
		// LOGINFO("The file /sys/class/net/eth0/address wasn't opened or the MAC address isn't correct (ver. 3)\n");
		LOGINFO("This software doesn't fit for the camera!\n");
		return EXIT_FAILURE;
	}
		// LOGINFO("The MAC address is correct (ver. 3)\n"); */

	daemonize();
	chdir(app_path);
	logCurDir();
	SignalHandler::init();
	globalsInit();

	/* Start TCP server */
	TCPServerClass _serv;
	if (_serv.start(8080, 3, 2, 5, 30, 8192, 300, 5) == 0) {
		LOGINFO("%s: TCP server is started\n", app_name);
	}
	else {
		LOGERR("%s: TCP server start failed\n", app_name);
		closeSyslog();
		return EXIT_FAILURE;
	}

	/* Capture and image processing initialization */
	CaptureBase *pCapture(NULL);
	if (false == commonInit(pCapture)) {
		LOGERR("%s: Capture initialization failed\n", app_name);
		closeSyslog();
		return EXIT_FAILURE;
	}

	TraffCounter traffixCounter;
	if (false == traffiXInit(traffixCounter)) {
		LOGERR("%s: Traffix counter initialization failed\n", app_name);
		closeSyslog();
		return EXIT_FAILURE;
	}
	std::vector<TraffStat> curTraffStats;

	/*** Work ***/
	struct timeval     tv_start, tv_end;
	int                msecF, msecPT;
	int frameCounter_traffstat(0), frameCounter_config(0);
	LOGINFO("%s: Work is started\n", app_name);

	/* Application loop */
	while (SignalHandler::getExitSignal() == 0) {
		gettimeofday(&tv_start, NULL);
		const Frame frame = pCapture->handle();
		gettimeofday(&tv_end, NULL);
		msecF = (tv_end.tv_sec * 1000 + tv_end.tv_usec / 1000.0) - (tv_start.tv_sec * 1000 + tv_start.tv_usec / 1000.0);
		if (frame.size() < 1) {
			break;
		}

		gettimeofday(&tv_start, NULL);
		bool fAvg = traffixCounter.processImage(frame);
		gettimeofday(&tv_end, NULL);
		msecPT = (tv_end.tv_sec * 1000 + tv_end.tv_usec / 1000.0) - (tv_start.tv_sec * 1000 + tv_start.tv_usec / 1000.0);

		if (msecF > 5000 || msecPT > 5000) {
			LOGERR("CriticalTime: %d ms capt; %d ms proc\n", msecF, msecPT);
		}

		pthread_mutex_lock(&camApp::global_mutex);
		if (traffixApp::global_clrSensorsFlag) { // rq command
			for (size_t ii = 0; ii < traffixApp::global_clrSensIDs.size(); ++ii)
				traffixCounter.resetSensorStat(traffixApp::global_clrSensIDs[ii]);
			traffixCounter.toTraffStat(traffixApp::global_sensors, fAvg);
			traffixApp::global_clrSensorsFlag = false;
			traffixApp::global_clrSensIDs.clear();
			LOGINFO("rq command done");
		}
		pthread_mutex_unlock(&camApp::global_mutex);

		const int change = max(traffixCounter.carInChanged(curTraffStats, fAvg), traffixCounter.counterChanged(curTraffStats, fAvg));
		if (change >= 0 || fAvg || frameCounter_traffstat >= GLOBAL_UPDATE_STAT_FRAMENUM) {
			pthread_mutex_lock(&camApp::global_mutex);
			traffixApp::global_sensors = curTraffStats;
			pthread_mutex_unlock(&camApp::global_mutex);
			if (change >= 0)
				_serv.smModeWakeup();
			if (fAvg || frameCounter_traffstat >= GLOBAL_UPDATE_STAT_FRAMENUM) {
				if (fAvg)
					camApp::global_files.writeStatistics(true, curTraffStats);
				frameCounter_traffstat = 0;
			}
		}
		++frameCounter_traffstat;

		if (frameCounter_config >= GLOBAL_UPDATE_CONFIG_FRAMENUM) {
			if (camApp::global_files.checkUpdConfig(true)) {
				if (!commonInit(pCapture) || !traffiXInit(traffixCounter)) {
					break;
				}
			}
			frameCounter_config = 0;
		}

		++frameCounter_config;
	} // Application loop

	/* Clean up */
	traffixCounter.clear();
	if (pCapture) {
		pCapture->close();
		delete pCapture;
	}
	_serv.stop();

	globalsClear();

	LOGINFO("%s: Successful Exit\n", app_name);
	closeSyslog();

	return EXIT_SUCCESS;
}


bool commonInit(CaptureBase* &pCapt) {
	if (pCapt) {
		pCapt->close();
		delete pCapt;
		pCapt = NULL;
	}

	CamParams camPar;
	if (!camApp::global_files.readCamParams(true, camPar)) { 
		LOGINFO("readCamParams FAILED\n");
		return false;
	}

	if (camPar.CAPT_TYPE == "NAT")
		pCapt = new CaptureNat(false);
	else if (camPar.CAPT_TYPE == "NATOLD")
		pCapt = new CaptureNat(true);
	else if (camPar.CAPT_TYPE == "NV12")
		pCapt = new CaptureNV12();
	else if (camPar.CAPT_TYPE == "I420_YUV")
		pCapt = new CaptureI420_YUV();
	else if (camPar.CAPT_TYPE == "Y800")
		pCapt = new CaptureY800();
	else { //I420_RGB, JPEG_RGB ...
		LOGINFO("Invalid Capture TYPE\n");
		return false;
	}

	/* Capture initialization */
	if (pCapt->open(camPar.FrameW, camPar.FrameH, camPar.RGB_FRAME)) {
		const Frame frame = pCapt->handle();
		pthread_mutex_lock(&camApp::global_mutex);
		camApp::global_frame.release();
		camApp::global_frame.push(frame[0].clone());
		pthread_mutex_unlock(&camApp::global_mutex);
		LOGINFO("Capture is opened\n");
		LOGINFO("FrameW: %d, FrameH: %d, RGB_FRAME: %d\n", camPar.FrameW, camPar.FrameH, camPar.RGB_FRAME);
		return true;
	}
	else {
		delete pCapt;
		pCapt = NULL;
		LOGERR("Capture initialization failed\n");
		return false;
	}
}

bool traffiXInit(TraffCounter &traffixCounter) {
	traffixCounter.clear();
	pthread_mutex_lock(&camApp::global_mutex);
	traffixApp::global_clrSensorsFlag = false;
	traffixApp::global_clrSensIDs.clear();
	traffixApp::global_sensors.clear();
	pthread_mutex_unlock(&camApp::global_mutex);

	/* Image processing initialization */
	TraffAlgParams traffixPar;
	TraffAvgParams avgPar;
	bool configOk;
	std::vector<TraffSensor> sens = camApp::global_files.readTraffix(true, configOk, traffixPar, avgPar);
	if (configOk == false) {
		LOGINFO("Config (TraffiX) parsing FAILED. File does not exists or corrupted\n");
		return false;
	}
	else {
		traffixCounter.initTraffSensors(sens, avgPar,
										traffixPar.SENSOR_PARALLEL_WORK, traffixPar.SM_DATA_WAKEUP);
		LOGINFO("Config (TraffiX) parsing is done\n");
		return true;
	}
}

static void openSyslog(const char *app_name) {
	openlog(app_name, LOG_PID, LOG_LOCAL4);
	syslog(LOG_INFO, "Starting!");
}

static void closeSyslog() {
	syslog(LOG_INFO, "Exiting!");
}

static void logCurDir() {
	char cwd[PATH_MAX];
	if (getcwd(cwd, PATH_MAX) != NULL) {
		LOGINFO("Current working dir: %s\n", cwd);
	}
	else {
		LOGINFO("getcwd() error");
	}
}

static void daemonize() {
	if (daemon(1, 0) < 0) {
    syslog(LOG_CRIT, "Failed to daemonize!");
    exit(EXIT_FAILURE);
	}
}
 
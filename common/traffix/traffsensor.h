#ifndef _TRAFFIX_SENSOR_H_
#define _TRAFFIX_SENSOR_H_

#include "_common.h"

#if defined(TraffiXtream) && defined(_ProjCam_)

#include "datastructs.h"
#include "utils_traff.h"
#ifdef TraffiX_TRACKER
#include "segmentTracker.h"
#include "featureTracker.h"
#endif

#include <string>
#include <vector>
#include <pthread.h>

#if defined(_ProjCam_) && !defined(WIN32)
	#include <sys/time.h>
#else
	#include "systime.h"
	#include "global_saver.h"
	#include <iostream>
#endif

class TraffSensor : private TraffStat {
	struct AveragingValues {
		float carInSensorTime;
		int counter, sumSpeed, Cnt[5], sumSpd[5];
		AveragingValues()
			: carInSensorTime(0), counter(0), sumSpeed(0) {
			memset(Cnt, 0, sizeof(Cnt));
			memset(sumSpd, 0, sizeof(sumSpd));
		}
	};

	//ZONE
	enum ZoneWorkMode {
		CATCH,
		NORMAL
#if USE_GLARE
		//Glare modes
		,GLARE_NORMAL
#endif
	};
	enum ZoneCatchState {
		FIND_MOTION,
		WAIT_OTHER_ZONE,
		FIND_NO_MOTION,
		CHECK_JAM,
		FIND_STABLE_INTERVAL
		//Glare states
		//GLARE_WAIT_OTHER_ZONE
	};
	enum ZoneState {
		ROAD,
		CAR,
		SHADOW,
		LIGHT_BEAM
	};
	struct TraffZone {
	public:
		bool carDetected;
		bool carPassedZone;
		timeval lastFDJumpTime;
		timeval carEnterTime;
		timeval carExitTime;

#ifdef TraffiX_TRACKER
		RowMat<uchar> motionMask;
		TraffRect rr;
#else
	private:
		TraffRect rr;
#endif

    public:
        TraffAlgParams algPar;
    
	private:
		// TraffAlgParams algPar;
		RowMat<uchar> imageMask;
		Frame bgFrame, refFrame, midFrame, preFrame;
		double dImgMean=0, dRefMean=0, dBgMean=0, dMidMean=0;
		int sensorId;
		ZoneWorkMode workMode;
		ZoneCatchState catchModeState;
		ZoneState zoneState;
		bool isSecond;
		bool thresholdsWereIncreased/*Флаг использования увеличенных порогов*/, fdJumpWasDetected, noMotionWasDetected;
		bool bGlareDetect=false;
		//int objectEnterPointsCount, objectExitPointsCount; Заменены на функции getObjectEnterPointsCount, getObjectExitPointsCount
		int getObjectEnterPointsCount() {
			return thresholdsWereIncreased ? algPar.INCREASED_OBJECT_ENTER_POINTS_COUNT : algPar.OBJECT_ENTER_POINTS_COUNT;
		}
		int getObjectExitPointsCount() {
			return thresholdsWereIncreased ? algPar.INCREASED_OBJECT_EXIT_POINTS_COUNT : algPar.OBJECT_EXIT_POINTS_COUNT;
		}
		int motionCounter, stableCounter, iNoMotionCounter;
		timeval zoneLockedBeginTime;
		timeval fdJumpTime;
		timeval motionWasFoundTime;
		timeval noMotionTime;
		timeval _CurrentTime_;
	public:
		TraffZone();
		bool isInit() const;
		void clear();
		void init(const TraffAlgParams &aparams, bool bSecond, const std::vector<TraffPoint> &pointList, int sensid);
		int processImage(const Frame &frame, TraffZone *pOtherZone, bool stopDetected = false);
		void resetCatchMode();
		void resetZone();
#if DEBUG_MSGS
		mutable cv::Mat oDbgZoneImg = cv::Mat(100, 100, CV_8UC3);
		mutable cv::Mat oDbgPreFrame;
		cv::Mat oDbgNormalHigherLower;
		int iDbgMotionCounter;
		int iDbgGlareCounter;
		int iDbgStableCounter;
		int iDbgNormalHigher=0, iDbgNormalLower=0;
		int iDbgShadowPointsCount;
		int iDbgFindStableIntervalDiffCount = 0;
		int iDbgSpecialShadowPointsCount=0;
		int iDbgSpecialLightBeamPointsCount = 0;
		void GetDbgInfo(cv::Mat*,std::string& sOut);
#endif

#if COMPENSATE_LIGHTCHANGE
		int iTotalCenteredCount = 0; //Счетчик точек, превышающих порог после центрирования изображений
#endif

	private:
		int absDifferenceAndBinarization(const RowMat<uchar> &currentFullFrame, const RowMat<uchar> &frame, int* piNumGlare,double *pdMean) const;
		void filterImage(const RowMat<uchar>& currentFullFrame,  RowMat<uchar>& frame, double k,double*pdMean);
		void catchMode(const Frame &curFullFrame, TraffZone *pOtherZone, bool stopDetected = false, bool carOut = false);
		int detectCarInNormalMode(const Frame &curFullFrame);
		int detectCarInGlareNormalMode(const Frame& curFullFrame);
		bool processRoadState(int totalPointsCount, bool shadowDetected, bool lightBeamDetected);
		bool processCarState(int totalPointsCount, bool shadowDetected, bool lightBeamDetected);
		bool processShadowState(int totalPointsCount, int specialShadowPointsCount);
		bool processLightBeamState(int totalPointsCount, int specialLightBeamPointsCount);
		bool processCarEnter();
	};
private:
	//Параметры алгоритма. Не должны копироваться, чтобы была возможность изменения на лету
	TraffAlgParams algPar;
public:
	//Задать параметры сенсоров на лету
	void setParams(const TraffAlgParams& _algPar) {
		algPar = _algPar;
		firstZone.algPar = algPar;
		secondZone.algPar = algPar;
	}
	//Получить параметры сенсора
	TraffAlgParams getParams() const {
		return algPar;
	}
private:
	int rotation;
    TraffZone firstZone, secondZone;
	TraffAvgParams avg;
	int prevCurrentSpeed;
	float prevCurrentLength;
	// Средняя скорость
	int sumCurSpeed, sumCurSpd[5];
	// Время занятости сенсора
	float sumCarInSensorTime;
	// Вектор усредненных значений за предыдущие маленькие интервалы (smallAvgInterval)
	std::vector<AveragingValues> avgValuesBuffer;
	int avgValuesBufferSize;
	timeval lastAvgTime, carDetTime;

#ifdef TraffiX_TRACKER
	FeatureBasedDetector featureTracker;
	TrackResult preTrRes;
	SegmentBasedDetector segmTracker;
	timeval stopDetTime;
#endif

private: //для параллельной обработки
	struct ThreadSt {
		#define THR_ST_STOP -1
		#define THR_ST_NONE -2
		int id;
		TraffSensor *parent;
		const Frame *pFrame;
		pthread_t tid;
		bool condf_W;
		pthread_cond_t cond_W;
		pthread_mutex_t mutex_W;
		bool init_sync();
		void clear_sync();
		ThreadSt()
			: id(THR_ST_NONE) {}
	};
	ThreadSt thrSt;
	static void* work_thread(void *arg);
	void par_stop();
public: //для параллельной обработки
	void par_init();
	TraffSensor();
	bool isInit() const;
	void resetStat();
	void clear();
	void init(const TraffAlgParams &aparams,
			  const std::vector<TraffPoint> &pointList1, const std::vector<TraffPoint> &pointList2, int sensid, float dist, int rot);
	int getRotation() const;
	void initAvg(const TraffAvgParams &avgParams);
	void processImage(const Frame &frame);
	bool processFinished();
	void doAveraging(const timeval &curtime);
	bool isBigAvgComplete() const;
	void toTraffStat(TraffStat &out, const std::string &stattime) const;
	int getId() const;
	static std::vector<TraffSensor> readSensorsFromFile(TraffAvgParams &avg, bool &isOpened, std::string fname, const CamParams &cparams, const TraffAlgParams &aparams);
private:
	void process(const Frame &frame);
	void calcCounter_Speed_Length(bool fVal, float valSpeed, float valLength);
	void calcOccup(const timeval &curtime);
	void doBigAveraging();
	static void calcAverage(int &cnt, int &spd, float &totalCarInSensorTime, const std::vector<AveragingValues> &avgBuf, int idx);
};

#endif //TraffiXtream && _ProjCam_
#endif // _TRAFFIX_SENSOR_H_

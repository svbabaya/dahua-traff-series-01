#ifndef __COMMON_H_
#define __COMMON_H_

#define _ProjCam_

#define TraffiXtream

#define COMPENSATE_LIGHTCHANGE 1

#define TraffiX_TRACKER // For arm

// #define _FixPoint_ // What is it for?
// #define _NO_OCV_ // What is it for?
// #define LOGINFO // What is it for?
// #define PRINT_TIME // What is it for?
// #define USE_GLARE // What is it for?
// #define SRVTEST_PRINT // What is it for?

#define APP_ID 50001

#if defined(TraffiX_TRACKER)
	#define MAJOR_VERSION	2
	#define MINOR_VERSION	0
	#define VERSION_SUFFIX	"TRACKER"
#else
	#define MAJOR_VERSION	1
	#define MINOR_VERSION	0
	#define VERSION_SUFFIX	"ORDINARY"
#endif

#define GLOBAL_UPDATE_STAT_FRAMENUM		1  // Global statistics update interval (frames)
#define GLOBAL_UPDATE_CONFIG_FRAMENUM	10 // Configuration check

#include <stdio.h>
#include <syslog.h>

#define LOGINFO(fmt, args...) { syslog(LOG_INFO, fmt, ## args); printf(fmt, ## args); }
#define LOGERR(fmt, args...) { syslog(LOG_CRIT, fmt, ## args); fprintf(stderr, fmt, ## args); }

#define DAHUA 1

#endif // __COMMON_H_

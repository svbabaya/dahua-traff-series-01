#include "signalhandler.h"
#include "_common.h"

#include <signal.h>
#include <unistd.h>
#include <execinfo.h>
#include <stdlib.h>

sig_atomic_t SignalHandler::exit_signal = 0;

void SignalHandler::init() {
	exit_signal = 0;
	struct sigaction sa;
	sa.sa_flags = 0;
	sigemptyset(&sa.sa_mask);
	sa.sa_handler = handleSigterm;
	sigaction(SIGTERM, &sa, NULL);
	sigaction(SIGINT, &sa, NULL);
	sa.sa_handler = handleSigabort;
	sigaction(SIGABRT, &sa, NULL);
}

sig_atomic_t SignalHandler::getExitSignal() {
	return exit_signal;
}

void SignalHandler::handleSigterm(sig_atomic_t signo) {
	exit_signal = 1;
}

void PrintStackTrace() {
	void *arr[256];
	size_t size = backtrace(arr, 256);
	char **strings = backtrace_symbols(arr, 256);
	if (strings) {
		LOGINFO("--Stack trace follows (%zd frames):\n", size);
		for (size_t i = 0; i < size; i++) {
			LOGINFO("  %s\n", strings[i]);
		}
		LOGINFO("--End Stack trace\n");
		free(strings);
	}
	else {
		LOGINFO("PrintStackTrace:  Error, could not generate stack trace!\n");
	}
}

void SignalHandler::handleSigabort(sig_atomic_t signo) {
	exit_signal = 1;
	LOGINFO("CrashSignalHandler called with signal %i\n", signo);
	PrintStackTrace();
	fflush(stdout);
	signal(SIGABRT, SIG_DFL);
	abort();
}

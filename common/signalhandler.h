#ifndef SIGNAL_HANDLER_H
#define SIGNAL_HANDLER_H

#include <signal.h>

class SignalHandler {
public:
	static void init();
	static sig_atomic_t getExitSignal();
private:
	SignalHandler();
	static sig_atomic_t exit_signal;
	static void handleSigterm(sig_atomic_t signo);
	static void handleSigabort(sig_atomic_t signo);
};

#endif // SIGNAL_HANDLER_H

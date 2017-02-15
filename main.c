#include "compass.h"
#include <stdio.h>
#include <stdbool.h>
#include <signal.h>

bool cont;

void signalHandler(int signo)
{
	cont = false;
}

int main(void)
{
	signal(SIGINT, signalHandler);

	if(compass_init()) {
		fprintf(stderr, "compass_init() fail");
		return -1;
	}

	cont = true;

	while(cont) {
		printf("%f\n", compass_read());
		sleep(1);
	}

	return 0;
}

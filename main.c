#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <football.h>

int main(void)
{
	play();
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}

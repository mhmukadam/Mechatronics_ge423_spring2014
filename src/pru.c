
#include "evmomapl138.h"
#include "pru.h"

void PRU_stop(pru_regs_t *pru) {
	PRU_wakeup(pru);
	pru->CONTROL &= ~ENABLE;
	while(pru->CONTROL & RUNSTATE) {}
}

void PRU_run(pru_regs_t *pru) {
	pru->CONTROL |= ENABLE;
}

void PRU_load(uint32_t *prog_base, uint32_t prog[], uint32_t size) {
	uint32_t i;
	for(i = 0; i < size; i++)
		*(prog_base + i) = prog[i];
}

void PRU_reset(pru_regs_t *pru) {
	pru->CONTROL &= ~RESET;
}

void PRU_restart(pru_regs_t *pru) {
	PRU_stop(pru);
	PRU_reset(pru);
	PRU_run(pru);
}

void PRU_wakeup(pru_regs_t *pru) {
	pru->CONTROL &= ~SLEEP;
}

//-----------------------------------------------------------------------------
// \file    mark_pru.h
// \brief   PRU register defines
//
//-----------------------------------------------------------------------------

#ifndef MARK_PRU_H
#define MARK_PRU_H

#define PRU0_REG_BASE 0x01C37000
#define PRU1_REG_BASE 0x01C37800

#define PRU0_PROG_BASE 0x01C38000
#define PRU1_PROG_BASE 0x01C3C000

//-----------------------------------------------------------------------------
// Register Structure & Defines
//-----------------------------------------------------------------------------
typedef struct
{
   volatile uint32_t CONTROL;
   volatile uint32_t STATUS;
   volatile uint32_t WAKEUP;
   volatile uint32_t CYCLECNT;
   volatile uint32_t STALLCNT;
   volatile uint32_t CONTABBLKIDX0;
   volatile uint32_t CONTABPROPTR0;
   volatile uint32_t CONTABPROPTR1;
   volatile uint32_t INTGPR[32];
   volatile uint32_t INTCTER[32];
} pru_regs_t;

#define PRU0 ((pru_regs_t *)PRU0_REG_BASE)
#define PRU1 ((pru_regs_t *)PRU1_REG_BASE)

#define PRU0_PROG ((uint32_t *)PRU0_PROG_BASE)
#define PRU1_PROG ((uint32_t *)PRU1_PROG_BASE)

#define RUNSTATE (0x00008000)
#define ENABLE (0x00000002)
#define RESET (0x00000001)
#define SLEEP (0x00000004)

//-----------------------------------------------------------------------------
// Public Function Prototypes
//-----------------------------------------------------------------------------
void PRU_stop(pru_regs_t *pru);
void PRU_load(uint32_t *prog_base, uint32_t prog[], uint32_t size);
void PRU_run(pru_regs_t *pru);
void PRU_reset(pru_regs_t *pru);
void PRU_restart(pru_regs_t *pru);
void PRU_wakeup(pru_regs_t *pru);

// To clear interrupt:
// *EVTCLR0 |= PRU_INTCLR;
//#define EVTCLR0 ((uint32_t *)0x01800040)
#define PRU_INTCLR (0x00000040)

#endif

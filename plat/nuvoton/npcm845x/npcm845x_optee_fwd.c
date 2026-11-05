/*
 * Copyright (C) 2026 Nuvoton Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/*
 * Forwarding of OP-TEE interrupts through the EHF.
 *
 * GICv2 Group 0 is for EL3 on this platform, so every Group 0 interrupt that
 * OP-TEE owns is an EL3 interrupt (OP-TEE would otherwise get them as S-EL1
 * interrupts from the OP-TEE dispatcher). While the non-secure world runs, the
 * interrupt is taken by BL31 and dispatched by the EHF to optee_fwd_handler(),
 * which forwards it to OP-TEE (S-EL1). While OP-TEE itself runs, the interrupt
 * is taken natively at S-EL1 and BL31 is not involved.
 *
 * The handler is registered at PLAT_OPTEE_FWD_PRI, the EHF priority index that
 * covers the GIC priority OP-TEE gives its interrupts (0x1) and the default
 * one of interrupts BL31 does not configure (0x80). So any interrupt OP-TEE
 * configures, SGI, PPI or SPI, is forwarded without listing its ID here.
 *
 * The opposite direction is a small SiP service, for the interrupts
 * that BL31 owns but that were taken while OP-TEE runs (watchdog pre-timeout
 * and the stop SGI of the reset code in npcm845x_psci.c). With the GICv2
 * those are Group 0 as well, so they reach OP-TEE at S-EL1 and BL31 is not
 * involved. OP-TEE (plat-nuvoton/optee_wdg_fwd.c) recognises them and calls
 * BL31 (secure callers only; both calls never return):
 *   0x82000112  watchdog pre-timeout: stop the other CPUs, then this one
 *   0x82000113  stop SGI: stop this CPU
 *
 * The context switch into OP-TEE is not reimplemented here: it is done by the
 * OP-TEE dispatcher's own S-EL1 interrupt handler. Its address is picked up
 * when the dispatcher registers it (see __wrap_register_interrupt_type_handler()
 * at the end of this file), so this file follows the dispatcher if that handler
 * changes in a later TF-A.
 */

#include <errno.h>
#include <stdint.h>

#include <arch_helpers.h>
#include <bl31/ehf.h>
#include <bl31/interrupt_mgmt.h>
#include <common/debug.h>
#include <common/runtime_svc.h>
#include <drivers/arm/gicv2.h>
#include <lib/el3_runtime/context_mgmt.h>
#include <lib/mmio.h>
#include <lib/smccc.h>
#include <plat/common/platform.h>
#include <platform_def.h>
#include <plat_npcm845x.h>

/* OP-TEE dispatcher state (optee_vector_table) */
#include "opteed_private.h"

#define OPTEE_FWD_FID_WDG_PRETIMEOUT	0x82000112U
#define OPTEE_FWD_FID_WDG_STOP_CPU	0x82000113U

/*
 * The OP-TEE dispatcher's S-EL1 interrupt handler, opteed_sel1_interrupt_handler(),
 * saved when the dispatcher registers it (see the wrapper at the end of the file).
 */
static interrupt_type_handler_t optee_sel1_handler;

/* CPU interface ID of the source CPU of an SGI, in the acknowledged value */
#define IAR_SGI_SRC_CPU(raw)	(((raw) >> 10) & 0x7U)

/*
 * The EHF acknowledged the interrupt, but OP-TEE acknowledges it itself. Give
 * the interrupt back to the GIC: complete it and make it pending again, so
 * that OP-TEE reads it (edge-triggered sources would be lost otherwise).
 */
static void optee_fwd_release(uint32_t intr_raw)
{
	unsigned int id = plat_ic_get_interrupt_id(intr_raw);

	plat_ic_end_of_interrupt(intr_raw);

	if (id < MIN_PPI_ID) {
		/*
		 * An SGI is pended again from its source CPU. GICD_SPENDSGIR
		 * is byte accessible: write only the byte of this SGI.
		 */
		mmio_write_8(BASE_GICD_BASE + GICD_SPENDSGIR + id,
			     (uint8_t)(1U << IAR_SGI_SRC_CPU(intr_raw)));
	} else {
		gicv2_set_interrupt_pending(id);
	}
}

/*
 * Enter OP-TEE to handle an interrupt by calling the OP-TEE dispatcher's own
 * S-EL1 interrupt handler (opteed_sel1_interrupt_handler(), saved by the
 * wrapper below): it saves the non-secure context and makes the next exception
 * return enter OP-TEE at its FIQ entry. OP-TEE acknowledges the interrupt itself
 * and returns with TEESMC_OPTEED_RETURN_FIQ_DONE, which the dispatcher turns into
 * a return to the interrupted non-secure code.
 */
static int optee_fwd_enter_optee(uint32_t flags, void *handle)
{
	/* The dispatcher has not registered its handler: nothing to forward to */
	if (optee_sel1_handler == NULL) {
		return -ENODEV;
	}

	/* OP-TEE has not completed its initialisation, nothing to forward to */
	if (optee_vector_table == NULL) {
		return -ENODEV;
	}

	/* EL3 interrupts are routed to EL3 from the non-secure world only */
	if ((get_interrupt_src_ss(flags) != NON_SECURE) ||
	    (handle != cm_get_context(NON_SECURE))) {
		return -EINVAL;
	}

	/*
	 * Called as the interrupt management framework calls it for an interrupt
	 * that was not acknowledged. The interrupt is released and OP-TEE
	 * acknowledges it itself, so no interrupt ID is passed. The return value (the
	 * context to return to) is not used: the handler selects it with
	 * cm_set_next_eret_context().
	 */
	(void)optee_sel1_handler(INTR_ID_UNAVAILABLE, flags, handle, NULL);

	return 0;
}

static int optee_fwd_handler(uint32_t intr_raw, uint32_t flags, void *handle,
			     void *cookie)
{
	unsigned int id = plat_ic_get_interrupt_id(intr_raw);

	(void)cookie;

	optee_fwd_release(intr_raw);

	if (optee_fwd_enter_optee(flags, handle) != 0) {
		/* Nobody to forward to: don't leave it pending forever */
		if (id >= MIN_PPI_ID) {
			gicv2_clear_interrupt_pending(id);
		}
	}

	return 0;
}

static uintptr_t optee_fwd_smc_handler(uint32_t smc_fid, u_register_t x1,
				       u_register_t x2, u_register_t x3,
				       u_register_t x4, void *cookie,
				       void *handle, u_register_t flags)
{
	(void)x1;
	(void)x2;
	(void)x3;
	(void)x4;
	(void)cookie;

	switch (smc_fid) {
	case OPTEE_FWD_FID_WDG_PRETIMEOUT:
	case OPTEE_FWD_FID_WDG_STOP_CPU:
		/*
		 * These park the calling CPU: only OP-TEE may ask for it, the
		 * non-secure world must not be able to stop CPUs.
		 */
		if (!is_caller_secure(flags)) {
			SMC_RET1(handle, SMC_UNK);
		}

		npcm845x_wdg_from_secure(smc_fid ==
					 OPTEE_FWD_FID_WDG_PRETIMEOUT);

	default:
		SMC_RET1(handle, SMC_UNK);
	}
}

/*
 * Linked in place of register_interrupt_type_handler() (--wrap, see
 * platform.mk), so that no common code needs to be changed.
 *
 * With GICV2_G0_FOR_EL3=1 there are no S-EL1 interrupts: Group 0 interrupts
 * are EL3 interrupts, which optee_fwd_handler() forwards to OP-TEE. The OP-TEE
 * dispatcher would still register its S-EL1 handler, which is not supported
 * here and would end in a panic, so do not register it. Its address is saved
 * instead: optee_fwd_enter_optee() calls that handler, so the dispatcher's own
 * code does the switch into OP-TEE. Everything else is passed on.
 */
int32_t __real_register_interrupt_type_handler(uint32_t type,
					       interrupt_type_handler_t handler,
					       uint32_t flags);

int32_t __wrap_register_interrupt_type_handler(uint32_t type,
					       interrupt_type_handler_t handler,
					       uint32_t flags)
{
	if (type == INTR_TYPE_S_EL1) {
		/* Like TF-A, accept one S-EL1 handler only */
		if (optee_sel1_handler != NULL) {
			return -EALREADY;
		}
		optee_sel1_handler = handler;
		return 0;
	}

	return __real_register_interrupt_type_handler(type, handler, flags);
}

static int32_t optee_fwd_setup(void)
{
	ehf_register_priority_handler(PLAT_OPTEE_FWD_PRI, optee_fwd_handler);

	return 0;
}

DECLARE_RT_SVC(optee_fwd_svc, OEN_SIP_START, OEN_SIP_END, SMC_TYPE_FAST,
	       optee_fwd_setup, optee_fwd_smc_handler);

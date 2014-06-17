/****************************************************************************
 *
 *   Copyright (c) 2012, 2013 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file drv_hrt.c
 *
 * High-resolution timer callouts and timekeeping.
 *
 * This can use any general or advanced STM32 timer.
 *
 * Note that really, this could use systick too, but that's
 * monopolised by NuttX and stealing it would just be awkward.
 *
 * We don't use the NuttX STM32 driver per se; rather, we
 * claim the timer and then drive it directly.
 */

#include <nuttx/config.h>
#include <nuttx/arch.h>
#include <nuttx/irq.h>

#include <sys/types.h>
#include <stdbool.h>

#include <assert.h>
#include <debug.h>
#include <time.h>
#include <queue.h>
#include <errno.h>
#include <string.h>
#include <math.h>
#include <stdio.h>

#include <board_config.h>
#include <drivers/drv_pwmin.h>
#include <drivers/drv_hrt.h>

#include "chip.h"
#include "up_internal.h"
#include "up_arch.h"

#include "stm32.h"
#include "stm32_gpio.h"
#include "stm32_tim.h"
#include <systemlib/err.h>
//#include <drivers/device/device.h>

#ifdef PWMIN_TIMER

#if HRT_TIMER == PWMIN_TIMER
#error cannot share timer between HRT and PWMIN
#endif

/* PWMIN configuration */
#if PWMIN_TIMER == 1
# define PWMIN_TIMER_BASE		STM32_TIM1_BASE
# define PWMIN_TIMER_POWER_REG	STM32_RCC_APB2ENR
# define PWMIN_TIMER_POWER_BIT	RCC_APB2ENR_TIM1EN
# define PWMIN_TIMER_VECTOR	    STM32_IRQ_TIM1CC
# define PWMIN_TIMER_CLOCK	    STM32_APB2_TIM1_CLKIN
# define PWMIN_TIMER_LIMIT      UINT16_MAX
# if CONFIG_STM32_TIM1 
#  error must not set CONFIG_STM32_TIM1=y and PWMIN_TIMER=1
# endif
#elif PWMIN_TIMER == 2
# define PWMIN_TIMER_BASE		STM32_TIM2_BASE
# define PWMIN_TIMER_POWER_REG	STM32_RCC_APB1ENR
# define PWMIN_TIMER_POWER_BIT	RCC_APB1ENR_TIM2EN
# define PWMIN_TIMER_VECTOR	    STM32_IRQ_TIM2
# define PWMIN_TIMER_CLOCK	    STM32_APB1_TIM2_CLKIN
# define PWMIN_TIMER_LIMIT      UINT32_MAX
# if CONFIG_STM32_TIM2
#  error must not set CONFIG_STM32_TIM2=y and PWMIN_TIMER=2
# endif
#elif PWMIN_TIMER == 3
# define PWMIN_TIMER_BASE		STM32_TIM3_BASE
# define PWMIN_TIMER_POWER_REG	STM32_RCC_APB1ENR
# define PWMIN_TIMER_POWER_BIT	RCC_APB2ENR_TIM3EN
# define PWMIN_TIMER_VECTOR	STM32_IRQ_TIM3
# define PWMIN_TIMER_CLOCK	STM32_APB1_TIM3_CLKIN
# define PWMIN_TIMER_LIMIT UINT16_MAX
# if CONFIG_STM32_TIM3
#  error must not set CONFIG_STM32_TIM3=y and PWMIN_TIMER=3
# endif
#elif PWMIN_TIMER == 4
# define PWMIN_TIMER_BASE		STM32_TIM4_BASE
# define PWMIN_TIMER_POWER_REG	STM32_RCC_APB1ENR
# define PWMIN_TIMER_POWER_BIT	RCC_APB2ENR_TIM4EN
# define PWMIN_TIMER_VECTOR	STM32_IRQ_TIM4
# define PWMIN_TIMER_CLOCK	STM32_APB1_TIM4_CLKIN
# define PWMIN_TIMER_LIMIT UINT16_MAX
# if CONFIG_STM32_TIM4
#  error must not set CONFIG_STM32_TIM4=y and PWMIN_TIMER=4
# endif
#elif PWMIN_TIMER == 5
# define PWMIN_TIMER_BASE		STM32_TIM5_BASE
# define PWMIN_TIMER_POWER_REG	STM32_RCC_APB1ENR
# define PWMIN_TIMER_POWER_BIT	RCC_APB2ENR_TIM5EN
# define PWMIN_TIMER_VECTOR	STM32_IRQ_TIM5
# define PWMIN_TIMER_CLOCK	STM32_APB1_TIM5_CLKIN
# define PWMIN_TIMER_LIMIT UINT32_MAX
# if CONFIG_STM32_TIM5
#  error must not set CONFIG_STM32_TIM5=y and PWMIN_TIMER=5
# endif
#elif PWMIN_TIMER == 8
# define PWMIN_TIMER_BASE		STM32_TIM8_BASE
# define PWMIN_TIMER_POWER_REG	STM32_RCC_APB2ENR
# define PWMIN_TIMER_POWER_BIT	RCC_APB2ENR_TIM8EN
# define PWMIN_TIMER_VECTOR	STM32_IRQ_TIM8CC
# define PWMIN_TIMER_CLOCK	STM32_APB2_TIM8_CLKIN
# define PWMIN_TIMER_LIMIT UINT16_MAX
# if CONFIG_STM32_TIM8
#  error must not set CONFIG_STM32_TIM8=y and PWMIN_TIMER=8
# endif
#endif

/*
 * HRT clock must be at least 1MHz
 */
#if PWMIN_TIMER_CLOCK <= 1000000
# error PWMIN_TIMER_CLOCK must be greater than 1MHz
#endif

#define PWMIN_MAX_PERIOD_SECONDS 0.005


/*
 * Timer register accessors
 */
#define REG(_reg)	(*(volatile uint32_t *)(PWMIN_TIMER_BASE + _reg))

#define rCR1     	REG(STM32_GTIM_CR1_OFFSET)
#define rCR2     	REG(STM32_GTIM_CR2_OFFSET)
#define rSMCR    	REG(STM32_GTIM_SMCR_OFFSET)
#define rDIER    	REG(STM32_GTIM_DIER_OFFSET)
#define rSR      	REG(STM32_GTIM_SR_OFFSET)
#define rEGR     	REG(STM32_GTIM_EGR_OFFSET)
#define rCCMR1   	REG(STM32_GTIM_CCMR1_OFFSET)
#define rCCMR2   	REG(STM32_GTIM_CCMR2_OFFSET)
#define rCCER    	REG(STM32_GTIM_CCER_OFFSET)
#define rCNT     	REG(STM32_GTIM_CNT_OFFSET)
#define rPSC     	REG(STM32_GTIM_PSC_OFFSET)
#define rARR     	REG(STM32_GTIM_ARR_OFFSET)
#define rCCR1    	REG(STM32_GTIM_CCR1_OFFSET)
#define rCCR2    	REG(STM32_GTIM_CCR2_OFFSET)
#define rCCR3    	REG(STM32_GTIM_CCR3_OFFSET)
#define rCCR4    	REG(STM32_GTIM_CCR4_OFFSET)
#define rDCR     	REG(STM32_GTIM_DCR_OFFSET)
#define rDMAR    	REG(STM32_GTIM_DMAR_OFFSET)

/*
 * Specific registers and bits used by HRT sub-functions
 */
#if PWMIN_TIMER_CHANNEL == 1
// Registers
    #define rCCR_PWMIN_A	    rCCR1			    /* compare register for PWMIN */
    #define DIER_PWMIN_A	    (GTIM_DIER_CC1IE) // | GTIM_DIER_UIE | GTIM_DIER_TIE)	/* interrupt enable for PWMIN */
    #define SR_INT_PWMIN_A	    GTIM_SR_CC1IF		/* interrupt status for PWMIN */
    #define rCCR_PWMIN_B	    rCCR2               /* compare register for PWMIN */
    #define SR_INT_PWMIN_B	    GTIM_SR_CC2IF		/* interrupt status for PWMIN */
    #define CCMR1_PWMIN         ((0x02 << GTIM_CCMR1_CC2S_SHIFT) | (0x01 << GTIM_CCMR1_CC1S_SHIFT))
    #define CCMR2_PWMIN         0 // Other two channels disabled.
    #define CCER_PWMIN          (GTIM_CCER_CC2P | GTIM_CCER_CC1E | GTIM_CCER_CC2E) // Rising edge on CH1, falling edge on CH2, enable both.
    #define SR_OVF_PWMIN        (GTIM_SR_CC1OF | GTIM_SR_CC2OF)
    #define SMCR_PWMIN_1        (0x05 << GTIM_SMCR_TS_SHIFT)
    #define SMCR_PWMIN_2        ((0x04 << GTIM_SMCR_SMS_SHIFT) | SMCR_PWMIN_1)
#elif PWMIN_TIMER_CHANNEL == 2
    #define rCCR_PWMIN_A	    rCCR2			    /* compare register for PWMIN */
    #define DIER_PWMIN_A	    (GTIM_DIER_CC2IE)		/* interrupt enable for PWMIN */
    #define SR_INT_PWMIN_A	    GTIM_SR_CC2IF		/* interrupt status for PWMIN */
    #define rCCR_PWMIN_B	    rCCR1               /* compare register for PWMIN */
    #define DIER_PWMIN_B	    GTIM_DIER_CC1IE		/* interrupt enable for PWMIN */
    #define SR_INT_PWMIN_B	    GTIM_SR_CC1IF		/* interrupt status for PWMIN */
    #define CCMR1_PWMIN         ((0x01 << GTIM_CCMR1_CC2S_SHIFT) | (0x02 << GTIM_CCMR1_CC1S_SHIFT))
    #define CCMR2_PWMIN         0 // Other two channels disabled.
    #define CCER_PWMIN          (GTIM_CCER_CC1P | GTIM_CCER_CC1E | GTIM_CCER_CC2E) // Rising edge on CH2, falling edge on CH1, enable both
    #define SR_OVF_PWMIN        (GTIM_SR_CC1OF | GTIM_SR_CC2OF)
    #define SMCR_PWMIN_1        (0x06 << GTIM_SMCR_TS_SHIFT)
    #define SMCR_PWMIN_2        ((0x04 << GTIM_SMCR_SMS_SHIFT) | SMCR_PWMIN_1)
#else
    #error PWMIN_TIMER_CHANNEL must be a value between 1 and 2 in order to use the slave mode controller
#endif


/* timer-specific functions */
static void		pwmin_tim_init(void);
static int		pwmin_tim_isr(int irq, void *context);

__EXPORT uint16_t num_pwmin_interrupts = 0;
//__EXPORT uint16_t pwmin_buffer_width = 0;
//__EXPORT uint16_t pwmin_buffer_period = 0;
//__EXPORT uint64_t pwmin_last_valid_decode = 0;

//__EXPORT uint16_t pwmin_data[3];

static void	pwmin_decode(uint16_t status);

#include <uORB/uORB.h>
#include <uORB/topics/pwm_input.h>
#include <uORB/topics/subsystem_info.h>


ORB_DEFINE(pwm_input, struct pwm_input_s);
orb_advert_t _pwm_pub = -1;

/*
 * Initialise the timer we are going to use.
 */
static void pwmin_tim_init(void) {
	/* claim our interrupt vector */
	irq_attach(PWMIN_TIMER_VECTOR, pwmin_tim_isr);

	/* Clear no bits, set timer enable bit.*/
    modifyreg32(PWMIN_TIMER_POWER_REG, 0, PWMIN_TIMER_POWER_BIT); 

	/* disable and configure the timer */
	rCR1 = 0;
	rCR2 = 0;
	rSMCR = 0;
	rDIER = DIER_PWMIN_A;
	rCCER = 0;		/* unlock CCMR* registers */
	rCCMR1 = CCMR1_PWMIN;
	rCCMR2 = CCMR2_PWMIN;
    rSMCR = SMCR_PWMIN_1; // Set up mode
    rSMCR = SMCR_PWMIN_2; // Enable slave mode controller
	rCCER  = CCER_PWMIN;
	rDCR = 0;

    
    // Calculate the prescaler, rounding up to ensure that the timer period definitely exceeds the signal period.
    double prescaler_fl = ceil((PWMIN_MAX_PERIOD_SECONDS * PWMIN_TIMER_CLOCK) / (PWMIN_TIMER_LIMIT + 1.0));
 //   printf("max_period(x1M) = %f\ntimer_clock = %f\ntimer_limit=%f\n",(double) PWMIN_MAX_PERIOD_SECONDS, (double) PWMIN_TIMER_CLOCK,PWMIN_TIMER_LIMIT + 1.0);
 //   printf("prescaler_fl = %f\n",prescaler_fl);
//    printf("PWMIN_MAX_PERIOD_SECONDS = %f\nPWMIN_TIMER_CLOCK = %f\nTIMER_MAX = %f\n",PWMIN_MAX_PERIOD_SECONDS, PWMIN_TIMER_CLOCK, TIMER_MAX + 1.0);
    uint64_t prescaler_in = (uint64_t) (prescaler_fl);
 //   printf("prescaler_in = %lld\n",prescaler_in);
    if (prescaler_in > (UINT16_MAX + 1)) {
        // Unexpected. For now, just keep going anyway and hope that most captures won't be as long as the claimed maximum period.
        // It detects (and ignores) overcaptures anyway.
        prescaler_in = UINT16_MAX + 1;
    } else if (prescaler_in < 1) {
        prescaler_in = 1;
    }
    // Requested prescaler = PWMIN_MAX_PERIOD / TIMER_MAX * PWMIN_TIMER_CLOCK
    // Define the clock speed. We want the highest possible clock speed that avoids overflows.
	rPSC = prescaler_in - 1;	
 //   printf("PWMIN timer clock = %d, prescaler = %d\n",PWMIN_TIMER_CLOCK,rPSC);

	/* run the full span of the counter */
	rARR = PWMIN_TIMER_LIMIT;

	/* generate an update event; reloads the counter, all registers */
	rEGR = GTIM_EGR_UG;

	/* enable the timer */
	rCR1 = GTIM_CR1_CEN;
    num_pwmin_interrupts = 34;

	/* enable interrupts */
	up_enable_irq(PWMIN_TIMER_VECTOR);
}


static void pwmin_decode(uint16_t status)
{
    
    static uint64_t error_count = 0;
    uint32_t period = rCCR_PWMIN_A;
	uint32_t pulse_width  = rCCR_PWMIN_B;

    num_pwmin_interrupts++;
 //   pwmin_data[0] = period;
 //   pwmin_data[1] = pulse_width;
 //   pwmin_data[2] = num_pwmin_interrupts;
    
	/* if we missed an edge, we have to give up */
	if (status & SR_OVF_PWMIN) {
        error_count++;
		return;
    }

    struct pwm_input_s pwmin_report;
    pwmin_report.timestamp = hrt_absolute_time();
    pwmin_report.error_count = error_count;
    pwmin_report.period = period;
    pwmin_report.pulse_width = pulse_width;
    
    

    if (_pwm_pub != -1) {
        /* publish for subscribers */
        orb_publish(ORB_ID(pwm_input), _pwm_pub, &pwmin_report);
    }
    
	return;
}


/*
 * Handle the compare interupt by calling the callout dispatcher
 * and then re-scheduling the next deadline.
 */
static int pwmin_tim_isr(int irq, void *context)
{
	uint16_t status;
    
	/* copy interrupt status */
	status = rSR;

	/* ack the interrupts we just read */
	rSR = ~status;

    pwmin_decode(status);
	return OK;
}

#define rCR1     	REG(STM32_GTIM_CR1_OFFSET)
#define rCR2     	REG(STM32_GTIM_CR2_OFFSET)
#define rSMCR    	REG(STM32_GTIM_SMCR_OFFSET)
#define rDIER    	REG(STM32_GTIM_DIER_OFFSET)
#define rSR      	REG(STM32_GTIM_SR_OFFSET)
#define rEGR     	REG(STM32_GTIM_EGR_OFFSET)
#define rCCMR1   	REG(STM32_GTIM_CCMR1_OFFSET)
#define rCCMR2   	REG(STM32_GTIM_CCMR2_OFFSET)
#define rCCER    	REG(STM32_GTIM_CCER_OFFSET)
#define rCNT     	REG(STM32_GTIM_CNT_OFFSET)
#define rPSC     	REG(STM32_GTIM_PSC_OFFSET)
#define rARR     	REG(STM32_GTIM_ARR_OFFSET)
#define rCCR1    	REG(STM32_GTIM_CCR1_OFFSET)
#define rCCR2    	REG(STM32_GTIM_CCR2_OFFSET)
#define rCCR3    	REG(STM32_GTIM_CCR3_OFFSET)
#define rCCR4    	REG(STM32_GTIM_CCR4_OFFSET)
#define rDCR     	REG(STM32_GTIM_DCR_OFFSET)
#define rDMAR    	REG(STM32_GTIM_DMAR_OFFSET)

void print_registers() {
    int tmp = rCR1;
    printf("rCR1 = %04X\n",tmp);
    tmp = rCR2;
    printf("rCR2 = %04X\n",tmp);
    tmp = rSMCR;
    printf("rSMRC = %04X\n",tmp);
    tmp = rDIER;
    printf("rDIER = %04X\n",tmp);
    tmp = rSR;
    printf("rSR = %04X\n",tmp);
    tmp = rEGR;
    printf("rEGR = %04X\n",tmp);
    tmp = rCCMR1;
    printf("rCCMR1 = %04X\n",tmp);
    tmp = rCCMR2;
    printf("rCCMR2 = %04X\n",tmp);
    tmp = rCCER;
    printf("rCCER = %04X\n",tmp);
    tmp = rCNT;
    printf("rCNT = %04X\n",tmp);
    tmp = rPSC;
    printf("rPSC = %04X\n",tmp);
    tmp = rARR;
    printf("rARR = %04X\n",tmp);
    tmp = num_pwmin_interrupts;
    printf("msg_retval = %05d\n",tmp);
}
    
    
    
    
    
    

/**
 * Initalise the high-resolution timing module.
 */
void pwmin_init(void)
{
    
    static uint8_t start_number = 0;
    
    if (start_number > 0) {
        print_registers();
        return;
        // Timer clearly not running.
    } 
    start_number++;
    
	pwmin_tim_init();
    
    stm32_configgpio(GPIO_PWM_IN);
    //20140604: Lock the port so nobody else can use it. 
 //   putreg32(0x00010002,STM32_GPIOA_LCKR);
 //   putreg32(0x00000002,STM32_GPIOA_LCKR);
 //   putreg32(0x00010002,STM32_GPIOA_LCKR);
 //   uint32_t port_value = getreg32(STM32_GPIOA_LCKR);
 //   port_value = getreg32(STM32_GPIOA_LCKR);port_value = getreg32(STM32_GPIOA_LCKR);
 //   printf("Port locked: 0x%08X\n",port_value);
    
	struct pwm_input_s zero_report;
	memset(&zero_report, 0, sizeof(zero_report));
	_pwm_pub = orb_advertise(ORB_ID(pwm_input), &zero_report);
    printf("PWMIN publisher advertised, %d\n",_pwm_pub);

	if (_pwm_pub < 0) {
		warnx("failed to create PWM input object. Did you start uOrb?");
    }
    
    struct subsystem_info_s info = {
		true,
		true,
		true,
		SUBSYSTEM_TYPE_AERODYNAMIC
	};
	static orb_advert_t pub = -1;

	if (pub > 0) {
		orb_publish(ORB_ID(subsystem_info), pub, &info);
	} else {
		pub = orb_advertise(ORB_ID(subsystem_info), &info);
	}
}
#endif

int pwm_input_main(int argc, char * argv[]) {
    #ifdef PWMIN_TIMER
    printf("PWMIN init\n");
    pwmin_init();
    printf("PWMIN init complete\n");
    return 0;
    #else
    return 1;
    #endif
}


 /* PWMIN_TIMER */

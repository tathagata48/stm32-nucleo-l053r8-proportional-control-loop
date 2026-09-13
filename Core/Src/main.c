/* =====================================================================
 * LAB 4 - CLOSED-LOOP POSITION CONTROL + TELEMETRY
 * Board: NUCLEO-L053R8 @ 32 MHz (Cortex-M0+, bare-metal)
 *
 * WHAT THIS DOES:
 *   Two potentiometers on PA0/PA1 simulate a target and a measured
 *   position. A 1 kHz control ISR reads both via ADC+DMA, computes a
 *   proportional control effort, drives it out as PWM (TIM2 on PA5, the
 *   green LED), sets a direction pin (PA6), and streams telemetry
 *   (Target,Position,PWM) over serial at 50 Hz for Serial Studio/Simulink.
 * ===================================================================== */

#include <stdint.h>
#include "stm32l0xx.h"

/* Telemetry text buffer + ADC DMA target array.
 * adc_buffer[0] = Target (PA0), adc_buffer[1] = Position (PA1).
 * 'volatile' because DMA (hardware) writes adc_buffer behind the CPU's
 * back, and the ISR updates telemetry_tick asynchronously to main. */
char tx_buf[64];
volatile uint16_t adc_buffer[2];
volatile uint32_t telemetry_tick = 0;

/* ---------------------------------------------------------------------
 * uint_to_str - tiny integer->ASCII (no printf, saves flash/RAM).
 * Builds digits least-significant first into temp[], then reverses them
 * into buf[]. Returns how many characters were written.
 * ------------------------------------------------------------------- */
static int uint_to_str(uint16_t val, char *buf) {
    int idx = 0;
    char temp[10];
    if (val == 0) { buf[0] = '0'; return 1; }  /* special-case zero */
    while (val > 0) {
        temp[idx++] = '0' + (val % 10);        /* next digit */
        val /= 10;
    }
    for (int i = 0; i < idx; i++) {            /* reverse into output */
        buf[i] = temp[idx - 1 - i];
    }
    return idx;
}

/* ---------------------------------------------------------------------
 * send_telemetry - format "Target,Position,PWM\r\n" and blast it out.
 * CSV so Serial Studio / Simulink can plot the three channels directly.
 * ------------------------------------------------------------------- */
static void send_telemetry(uint16_t target, uint16_t position, uint16_t pwm) {
    int i = 0;
    i += uint_to_str(target, &tx_buf[i]);
    tx_buf[i++] = ',';
    i += uint_to_str(position, &tx_buf[i]);
    tx_buf[i++] = ',';
    i += uint_to_str(pwm, &tx_buf[i]);
    tx_buf[i++] = '\r';                        /* frame terminator */
    tx_buf[i++] = '\n';

    /* Blocking transmit: for each byte wait for TXE (data reg empty),
     * then write it. Simple and fine at 50 Hz update rate. */
    for (int j = 0; j < i; j++) {
        while (!(USART2->ISR & USART_ISR_TXE));
        USART2->TDR = tx_buf[j];
    }
}

/* ---------------------------------------------------------------------
 * SYSTEM CLOCK -> 32 MHz using HSI16 through the PLL.
 * (Simpler than Lab 3: HSI-only, no HSE attempt, since this lab doesn't
 * need the extra accuracy of an external reference.)
 * ------------------------------------------------------------------- */
void system_clock_32mhz(void) {
    RCC->APB1ENR |= RCC_APB1ENR_PWREN;                 /* PWR clock for VOS */
    PWR->CR = (PWR->CR & ~PWR_CR_VOS) | PWR_CR_VOS_0;  /* range 1 (>16 MHz) */
    FLASH->ACR |= FLASH_ACR_LATENCY;                   /* 1 flash wait state */

    RCC->CR |= RCC_CR_HSION;                            /* start HSI16 */
    while (!(RCC->CR & RCC_CR_HSIRDY));                 /* wait stable */

    /* PLL src = HSI16, *4 /2 = 32 MHz. */
    RCC->CFGR = RCC_CFGR_PLLSRC_HSI | RCC_CFGR_PLLMUL4 | RCC_CFGR_PLLDIV2;
    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY));                 /* wait PLL lock */

    RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW) | RCC_CFGR_SW_PLL;  /* select PLL */
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);    /* confirm */
}

/* ---------------------------------------------------------------------
 * init_hardware - clocks, GPIO, USART, ADC+DMA, PWM timer, 1 kHz timer.
 * ------------------------------------------------------------------- */
void init_hardware(void) {
    /* Enable every peripheral clock this lab touches, in one shot each bus. */
    RCC->IOPENR  |= RCC_IOPENR_GPIOAEN;                 /* port A */
    RCC->AHBENR  |= RCC_AHBENR_DMA1EN;                  /* DMA1 */
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;                 /* ADC1 */
    RCC->APB1ENR |= (RCC_APB1ENR_TIM2EN |              /* PWM timer */
                     RCC_APB1ENR_TIM6EN |              /* 1 kHz control tick */
                     RCC_APB1ENR_USART2EN);            /* serial */

    /* ---- 1. GPIO ----
     * PA0, PA1 -> analog (11) for the two ADC pot inputs. Setting the full
     * MODE0/MODE1 fields (both bits) gives 11. */
    GPIOA->MODER |= GPIO_MODER_MODE0 | GPIO_MODER_MODE1;

    /* PA2 -> AF4 = USART2_TX. Clear the field, set high bit (10=alt func),
     * then program nibble 2 of AFR[0] to 4. */
    GPIOA->MODER = (GPIOA->MODER & ~GPIO_MODER_MODE2) | GPIO_MODER_MODE2_1;
    GPIOA->AFR[0] = (GPIOA->AFR[0] & ~GPIO_AFRL_AFSEL2) | (4U << GPIO_AFRL_AFSEL2_Pos);

    /* PA5 -> AF5 = TIM2_CH1 (PWM). PA5 is the Nucleo green LED, so the duty
     * cycle is visible as brightness. Alt-func mode + AF5 in nibble 5. */
    GPIOA->MODER = (GPIOA->MODER & ~GPIO_MODER_MODE5) | GPIO_MODER_MODE5_1;
    GPIOA->AFR[0] = (GPIOA->AFR[0] & ~GPIO_AFRL_AFSEL5) | (5U << GPIO_AFRL_AFSEL5_Pos);

    /* PA6 -> plain output (01), used as a motor DIRECTION pin (simulated). */
    GPIOA->MODER = (GPIOA->MODER & ~GPIO_MODER_MODE6) | GPIO_MODER_MODE6_0;

    /* ---- 2. USART2 @ 115200 ----
     * BRR = 32 MHz / 115200 = 277.7 -> 278. TE+UE only (TX-only stream). */
    USART2->BRR = 278;
    USART2->CR1 = USART_CR1_TE | USART_CR1_UE;

    /* ---- 3. ADC + DMA: continuously scan CH0 and CH1 into adc_buffer ----
     * DMA channel 1 moves each conversion result from ADC->DR to memory.
     *   CPAR = source (ADC data register)
     *   CMAR = dest (adc_buffer)
     *   CNDTR = 2 (two channels)
     *   MINC : increment memory address each transfer (buffer[0], buffer[1])
     *   MSIZE_0/PSIZE_0 : 16-bit transfers (ADC result is 12-bit in 16)
     *   CIRC : circular, so it restarts and keeps both values fresh
     *   EN  : enable channel */
    DMA1_Channel1->CPAR = (uint32_t)&ADC1->DR;
    DMA1_Channel1->CMAR = (uint32_t)adc_buffer;
    DMA1_Channel1->CNDTR = 2;
    DMA1_Channel1->CCR = DMA_CCR_MINC | DMA_CCR_MSIZE_0 | DMA_CCR_PSIZE_0
                       | DMA_CCR_CIRC | DMA_CCR_EN;

    /* ADC config: CONT = continuous conversion, DMACFG = circular DMA,
     * DMAEN = generate DMA requests. Then choose channels 0 and 1. */
    ADC1->CFGR1 |= ADC_CFGR1_CONT | ADC_CFGR1_DMACFG | ADC_CFGR1_DMAEN;
    ADC1->CHSELR = ADC_CHSELR_CHSEL0 | ADC_CHSELR_CHSEL1;   /* PA0, PA1 */

    /* Calibrate the ADC BEFORE enabling it (required on L0). ADCAL self-
     * clears when done. Then ADEN=enable, ADSTART=begin conversions. */
    ADC1->CR |= ADC_CR_ADCAL;
    while (ADC1->CR & ADC_CR_ADCAL);
    ADC1->CR |= ADC_CR_ADEN;
    ADC1->CR |= ADC_CR_ADSTART;

    /* ---- 4. TIM2 PWM @ 1 kHz on CH1 ----
     * PSC=31 -> 32 MHz/(31+1) = 1 MHz timer tick.
     * ARR=999 -> 1 MHz/(999+1) = 1 kHz PWM period. Duty = CCR1/1000.
     * OC1M=110 (PWM mode 1) + OC1PE (preload) in CCMR1; CC1E enables the
     * output; CEN starts the timer. */
    TIM2->PSC = 31;
    TIM2->ARR = 999;
    TIM2->CCMR1 = (6U << TIM_CCMR1_OC1M_Pos) | TIM_CCMR1_OC1PE;
    TIM2->CCER = TIM_CCER_CC1E;
    TIM2->CR1 |= TIM_CR1_CEN;

    /* ---- 5. TIM6 @ 1 kHz control-loop tick ----
     * Same PSC/ARR math -> 1 kHz. UIE raises an interrupt each period;
     * CEN starts it. This is the deterministic heartbeat of the controller. */
    TIM6->PSC = 31;
    TIM6->ARR = 999;
    TIM6->DIER |= TIM_DIER_UIE;
    TIM6->CR1 |= TIM_CR1_CEN;

    NVIC_SetPriority(TIM6_DAC_IRQn, 1);        /* M0+ levels 0-3 */
    NVIC_EnableIRQ(TIM6_DAC_IRQn);
}

/* ---------------------------------------------------------------------
 * CONTROL LOOP ISR - runs 1000x/sec. Proportional controller.
 * ------------------------------------------------------------------- */
void TIM6_DAC_IRQHandler(void) {
    if (TIM6->SR & TIM_SR_UIF) {               /* update event? */
        TIM6->SR = ~TIM_SR_UIF;                /* clear flag (direct write) */

        /* Latest values DMA has deposited (no CPU polling needed). */
        uint16_t target   = adc_buffer[0];
        uint16_t position = adc_buffer[1];

        /* ERROR = where I want to be minus where I am. */
        int32_t error = target - position;

        /* DIRECTION pin (PA6) from the sign of the error. BSRR sets/resets
         * a bit atomically: BS_6 drives PA6 high, BR_6 drives it low.
         * After setting direction, take the magnitude of the error so the
         * PWM duty is always positive. */
        if (error > 0) {
            GPIOA->BSRR = GPIO_BSRR_BS_6;      /* forward */
        } else {
            GPIOA->BSRR = GPIO_BSRR_BR_6;      /* reverse */
            error = -error;                    /* |error| */
        }

        /* PROPORTIONAL term: effort = Kp * error, Kp=2. Saturate at the ARR
         * value (999 = 100% duty) so we never command more than full scale. */
        int32_t control_effort = error * 2;
        if (control_effort > 999) control_effort = 999;

        TIM2->CCR1 = control_effort;           /* update PWM duty -> LED */

        /* TELEMETRY at 50 Hz: the ISR runs at 1 kHz, so send every 20th
         * tick (1000/20 = 50 Hz). Sending every tick would flood the link
         * and lag the plotter. */
        telemetry_tick++;
        if (telemetry_tick >= 20) {
            send_telemetry(target, position, control_effort);
            telemetry_tick = 0;
        }
    }
}

/* ---------------------------------------------------------------------
 * MAIN - all the real work is in the 1 kHz ISR, so main just idles.
 * Keeping control in a fixed-rate ISR makes the loop timing deterministic,
 * which matters for stable control.
 * ------------------------------------------------------------------- */
int main(void) {
    system_clock_32mhz();
    init_hardware();

    while (1) {
        /* intentionally empty - controller runs in TIM6_DAC_IRQHandler */
    }
}

/* =====================================================================
 * Proportional control loop with ADC sampling and CSV telemetry
 * NUCLEO-L053R8, bare metal (no HAL)
 *
 * Two analog inputs act as a setpoint and a position reading. A
 * proportional controller runs at 1 kHz, drives the on-board LED with
 * the resulting effort, and reports the three values over the serial
 * port as comma separated text.
 *
 * Both analog pins are left floating in this build - touching a header
 * pin couples in enough hum and body capacitance to move the reading,
 * which is what the demo does. Wire a potentiometer per pin as a
 * divider if a controlled input is wanted; nothing here changes.
 *
 * Clock
 *   HSI16 -> PLL x4 /2 -> 32 MHz, the maximum for this part.
 *   Voltage range 1 and one flash wait state are set beforehand.
 *
 * Signals
 *   PA0  target input          ADC channel 0, floating
 *   PA1  position input        ADC channel 1, floating
 *   PA5  PWM effort            TIM2 CH1, AF5, the green LD2 user LED
 *   PA6  direction             high when the target is above position
 *   PA2  telemetry out         USART2 TX, virtual COM port
 *
 * Sampling
 *   ADC1 scans both channels continuously. DMA1 channel 1 moves each
 *   result into adc_buffer in circular mode, so no interrupt is needed
 *   and the newest pair is always there to read.
 *
 * Control, once per TIM6 tick
 *   error  = target - position
 *   PA6    = sign of the error
 *   effort = |error| * 2, clamped to the 999 count PWM period
 *
 * Telemetry, every 20th tick (50 Hz)
 *   target,position,effort<CR><LF> at 115200 8N1, transmit only.
 *
 * Note: the transmit loop is polled, and a full frame occupies the line
 * for longer than one tick, so that tick runs late. Harmless at this
 * scale, but DMA would be the fix.
 * ===================================================================== */

#include <stdint.h>
#include "stm32l0xx.h"

/* Telemetry string buffer and ADC DMA target array */
char tx_buf[64];
volatile uint16_t adc_buffer[2]; /* [0] = Target (PA0), [1] = Position (PA1) */
volatile uint32_t telemetry_tick = 0;

/* Standard C helper to convert integer to string without heavy printf */
static int uint_to_str(uint16_t val, char *buf) {
    int idx = 0;
    char temp[10];
    if (val == 0) {
        buf[0] = '0';
        return 1;
    }
    while (val > 0) {
        temp[idx++] = '0' + (val % 10);
        val /= 10;
    }
    for (int i = 0; i < idx; i++) {
        buf[i] = temp[idx - 1 - i];
    }
    return idx;
}

/* Formats and sends: Target,Position,PWM\r\n */
static void send_telemetry(uint16_t target, uint16_t position, uint16_t pwm) {
    int i = 0;

    /* Add data (Comma separated values only) */
    i += uint_to_str(target, &tx_buf[i]);
    tx_buf[i++] = ',';
    i += uint_to_str(position, &tx_buf[i]);
    tx_buf[i++] = ',';
    i += uint_to_str(pwm, &tx_buf[i]);

    /* End frame (Just carriage return and newline) */
    tx_buf[i++] = '\r';
    tx_buf[i++] = '\n';

    /* Blocking transmit at 115200 bps */
    for (int j = 0; j < i; j++) {
        while (!(USART2->ISR & USART_ISR_TXE));
        USART2->TDR = tx_buf[j];
    }
}

void system_clock_32mhz(void) {
    /* Use HSI16 multiplied to 32MHz (max for STM32L053) */
    RCC->APB1ENR |= RCC_APB1ENR_PWREN;
    PWR->CR = (PWR->CR & ~PWR_CR_VOS) | PWR_CR_VOS_0;
    FLASH->ACR |= FLASH_ACR_LATENCY;

    RCC->CR |= RCC_CR_HSION;
    while (!(RCC->CR & RCC_CR_HSIRDY));

    RCC->CFGR = RCC_CFGR_PLLSRC_HSI | RCC_CFGR_PLLMUL4 | RCC_CFGR_PLLDIV2;
    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY));

    RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW) | RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);
}

void init_hardware(void) {
    /* Enable Clocks: GPIOA, DMA1, ADC, TIM2, TIM6, USART2 */
    RCC->IOPENR |= RCC_IOPENR_GPIOAEN;
    RCC->AHBENR |= RCC_AHBENR_DMA1EN;
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
    RCC->APB1ENR |= (RCC_APB1ENR_TIM2EN | RCC_APB1ENR_TIM6EN | RCC_APB1ENR_USART2EN);

    /* 1. GPIO Configuration */
    /* PA0, PA1: Analog (ADC inputs) */
    GPIOA->MODER |= GPIO_MODER_MODE0 | GPIO_MODER_MODE1;

    /* PA2: AF4 (USART2 TX) */
    GPIOA->MODER = (GPIOA->MODER & ~GPIO_MODER_MODE2) | GPIO_MODER_MODE2_1;
    GPIOA->AFR[0] = (GPIOA->AFR[0] & ~GPIO_AFRL_AFSEL2) | (4U << GPIO_AFRL_AFSEL2_Pos);

    /* PA5: AF5 (TIM2_CH1 PWM output on the Nucleo Green LED) */
    GPIOA->MODER = (GPIOA->MODER & ~GPIO_MODER_MODE5) | GPIO_MODER_MODE5_1;
    GPIOA->AFR[0] = (GPIOA->AFR[0] & ~GPIO_AFRL_AFSEL5) | (5U << GPIO_AFRL_AFSEL5_Pos);

    /* PA6: General Output (Direction pin simulation) */
    GPIOA->MODER = (GPIOA->MODER & ~GPIO_MODER_MODE6) | GPIO_MODER_MODE6_0;

    /* 2. USART2 (115200 bps for stability) */
    USART2->BRR = 278; /* 32MHz / 115200 = 277.7 (rounded to 278) */
    USART2->CR1 = USART_CR1_TE | USART_CR1_UE;

    /* 3. ADC & DMA Setup (Continuous Scan of CH0 and CH1) */
    DMA1_Channel1->CPAR = (uint32_t)&ADC1->DR;
    DMA1_Channel1->CMAR = (uint32_t)adc_buffer;
    DMA1_Channel1->CNDTR = 2;
    DMA1_Channel1->CCR = DMA_CCR_MINC | DMA_CCR_MSIZE_0 | DMA_CCR_PSIZE_0 | DMA_CCR_CIRC | DMA_CCR_EN;

    ADC1->CFGR1 |= ADC_CFGR1_CONT | ADC_CFGR1_DMACFG | ADC_CFGR1_DMAEN;
    ADC1->CHSELR = ADC_CHSELR_CHSEL0 | ADC_CHSELR_CHSEL1; /* Select PA0, PA1 */
    ADC1->CR |= ADC_CR_ADCAL;
    while(ADC1->CR & ADC_CR_ADCAL);
    ADC1->CR |= ADC_CR_ADEN;
    ADC1->CR |= ADC_CR_ADSTART;

    /* 4. PWM Motor Simulation (TIM2, 1 kHz) */
    TIM2->PSC = 31;         /* 32MHz / 32 = 1MHz tick */
    TIM2->ARR = 999;        /* 1MHz / 1000 = 1kHz PWM frequency */
    TIM2->CCMR1 = (6U << TIM_CCMR1_OC1M_Pos) | TIM_CCMR1_OC1PE; /* PWM Mode 1 */
    TIM2->CCER = TIM_CCER_CC1E;
    TIM2->CR1 |= TIM_CR1_CEN;

    /* 5. 1 kHz Interrupt Timer (TIM6) */
    TIM6->PSC = 31;
    TIM6->ARR = 999;
    TIM6->DIER |= TIM_DIER_UIE;
    TIM6->CR1 |= TIM_CR1_CEN;

    NVIC_SetPriority(TIM6_DAC_IRQn, 1);
    NVIC_EnableIRQ(TIM6_DAC_IRQn);
}

/* Control Loop (Runs 1000 times per second) */
void TIM6_DAC_IRQHandler(void) {
    if (TIM6->SR & TIM_SR_UIF) {
        TIM6->SR = ~TIM_SR_UIF;

        uint16_t target = adc_buffer[0];
        uint16_t position = adc_buffer[1];

        /* Simple Proportional Control Logic */
        int32_t error = target - position;

        /* Set Direction (PA6) */
        if (error > 0) {
            GPIOA->BSRR = GPIO_BSRR_BS_6;
        } else {
            GPIOA->BSRR = GPIO_BSRR_BR_6;
            error = -error; /* Absolute value for PWM */
        }

        /* Apply a simple Gain (Kp=2) and cap at 100% duty cycle (999) */
        int32_t control_effort = error * 2;
        if (control_effort > 999) control_effort = 999;

        TIM2->CCR1 = control_effort; /* Output to LED */

        /* Transmit telemetry at 50 Hz (every 20ms) to avoid lagging Serial Studio */
        telemetry_tick++;
        if (telemetry_tick >= 20) {
            send_telemetry(target, position, control_effort);
            telemetry_tick = 0;
        }
    }
}

int main(void) {
    system_clock_32mhz();
    init_hardware();

    while (1) {
        /* Main stays empty, control logic runs securely in the 1kHz ISR */
    }
}

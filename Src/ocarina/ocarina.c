
#include <stdio.h>
#include <stdlib.h>

#include <stm32f0xx.h>
#include <stm32f0xx_hal.h>

#include <eforce/tx.h>
#include <stm32f042x6.h>
#include <stm32f0xx_hal_conf.h>

#include "ocarina/ocarina.h"
#include "ocarina/protocol.h"

#if BOARD == 3001
static const char interface_id[] = "Ocarina III r1";
enum { HW_VERSION = 3, SW_VERSION = 1, REVISION = 1 };

/*
 * main loop:
 *   if detecting -> need next hop ? then hop, set silent & baudrate
 *
 *   rx:
 *   if detecting -> ocarinaSetBitrate(detecting=false, set non-silent, baudrate)
 */
enum { enableBitrateAutodetect = 1 };
enum { autodetectHopInterval = 100 };
enum { autodetectMin = 125000 };
enum { autodetectMax = 1000000 };

static int detectingBitrate = 0;
static volatile int detected = 0;
#elif BOARD == 2001
static const char interface_id[] = "Ocarina II rev.1";
enum { HW_VERSION = 2, SW_VERSION = 1, REVISION = 1 };
#else
#error Unknown board revision!
#endif

static volatile int init_done = 0;
static volatile unsigned int rx_this_second = 0;

// TODO: this needs to be renamed
volatile uint8_t ocarinaRxEnabled;
uint16_t ocarinaErrorFlags;
uint32_t ocarinaStatusFlags;

static void setLED(GPIO_TypeDef* gpio, uint16_t mask, int enable) {
    if (enable) {
        gpio->BSRR = mask;
    }
    else {
        gpio->BRR = mask;
    }
}

#if BOARD >= 3000
static void waitUntilInitDone3(void) {
    uint32_t shift_reg = 0;

    while (shift_reg != 0xff) {
        shift_reg = (shift_reg << 1) | 1;
        ocarinaSetLEDs(shift_reg & 0xff);
        HAL_Delay(50);
    }

    HAL_Delay(500);
    ocarinaSetLEDs(0);

    while (!init_done) {
        setLED(LED_READY_GPIO, LED_READY_MASK, 1);
        HAL_Delay(50);
        setLED(LED_READY_GPIO, LED_READY_MASK, 0);
        HAL_Delay(1000-50);
    }

    setLED(LED_READY_GPIO, LED_READY_MASK, 1);
}

static void initRxTxLeds(void) {
    // Set up TIM3_CH3 - PB0
    __HAL_RCC_TIM3_CLK_ENABLE();
    TIM3->PSC = 48-1;
    TIM3->ARR = 50000-1;

    // Set up TIM3 PWM
    TIM3->CCER = TIM_CCER_CC3E;
    TIM3->CCR3 = 25000;

    GPIO_InitTypeDef init;
    init.Pin = (1 << 0);
    init.Mode = GPIO_MODE_AF_PP;
    init.Pull = GPIO_NOPULL;
    init.Alternate = GPIO_AF1_TIM3;
    init.Speed = GPIO_SPEED_MEDIUM;
    HAL_GPIO_Init(GPIOB, &init);

    // Set up TIM17_CH1 - PA7
    __HAL_RCC_TIM17_CLK_ENABLE();
    TIM17->PSC = 48-1;
    TIM17->ARR = 50000-1;

    // Set up TIM17 PWM
    TIM17->CCER = TIM_CCER_CC1E;
    TIM17->CCR1 = 1;

    init.Pin = (1 << 7);
    init.Mode = GPIO_MODE_AF_PP;
    init.Pull = GPIO_NOPULL;
    init.Alternate = GPIO_AF0_TIM17;
    init.Speed = GPIO_SPEED_MEDIUM;
    HAL_GPIO_Init(GPIOA, &init);
}
#else
static void waitUntilInitDone2(void) {
    uint32_t shift_reg = 0x80000000;

    while (!init_done) {
        shift_reg = (shift_reg << 1) | (shift_reg >> 15);
        ocarinaSetLEDs(shift_reg & 0xff);
        HAL_Delay(5);
    }
}
#endif

void ocarinaSetLEDs(uint8_t on) {
#if BOARD >= 3000
    setLED(LED_READY_GPIO, LED_READY_MASK, (on & 0x01));
    setLED(LED_RX_GPIO, LED_RX_MASK, (on & 0x02));
    setLED(LED_TX_GPIO, LED_TX_MASK, (on & 0x04));
    setLED(LED_ERROR_GPIO, LED_ERROR_MASK, (on & 0x08));
    setLED(LED_125K_GPIO, LED_125K_MASK, (on & 0x10));
    setLED(LED_250K_GPIO, LED_250K_MASK, (on & 0x20));
    setLED(LED_500K_GPIO, LED_500K_MASK, (on & 0x40));
    setLED(LED_1M_GPIO, LED_1M_MASK, (on & 0x80));
#else
    GPIOB->ODR = (GPIOB->ODR & 0xff00) | on;
#endif
}

void ocarinaBlinkRx(void) {
#if BOARD >= 3000
    if (!(TIM3->CR1 & TIM_CR1_CEN)) {
        TIM3->CCMR2 = TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_0;
        TIM3->CCMR2 = TIM_CCMR2_OC3M_1;

        TIM3->CNT = 0;
        TIM3->EGR = TIM_EGR_UG;
        TIM3->CR1 |= TIM_CR1_CEN | TIM_CR1_OPM;
    }
#endif
}

void ocarinaBlinkTx(void) {
#if BOARD >= 3000
    if (!(TIM17->CR1 & TIM_CR1_CEN)) {
        TIM17->CCMR1 = TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_0;
        TIM17->CCMR1 = TIM_CCMR1_OC1M_1;

        TIM17->CNT = 0;
        TIM17->EGR = TIM_EGR_UG;
        TIM17->CR1 |= TIM_CR1_CEN | TIM_CR1_OPM;
    }
#endif
}

void ocarinaSetBitrate(int definite, int bitrate) {
    if (definite) {
        detectingBitrate = 0;
    }

    ConfigureCAN(detectingBitrate, 0, bitrate);

    setLED(LED_125K_GPIO, LED_125K_MASK, (bitrate == 125000));
    setLED(LED_250K_GPIO, LED_250K_MASK, (bitrate == 250000));
    setLED(LED_500K_GPIO, LED_500K_MASK, (bitrate == 500000));
    setLED(LED_1M_GPIO, LED_1M_MASK, (bitrate == 1000000));
}

void ocarinaInitDone(void) {
    init_done = 1;
}

int ocarinaSendMessageExtId(uint32_t eid, const void* data, size_t length) {
    txSendCANMessage(EXT_ID(eid), data, length);
    return 0;
}

int ocarinaSendMessageStdId(uint16_t sid, const void* data, size_t length) {
    txSendCANMessage(STD_ID(sid), data, length);
    return 0;
}

static void send_test_message(void) {
    uint8_t data[8] = {0xAA, 13, 0, 0, HW_VERSION, SW_VERSION, 0, 0};
    txSendCANMessage(STD_ID(96), data, sizeof(data));
}

void ocarinaPanic(uint8_t code) {
    for (;;) {
        ocarinaSetLEDs(~code);
        HAL_Delay(100);
        ocarinaSetLEDs(~0);
        HAL_Delay(100);
    }
}

int txHandleCANMessage(uint32_t timestamp, CAN_ID_t id, const void* data, size_t length) {
    if (IS_STD_ID(id)) {
        protocolMessageReceivedStd(GET_STD_ID(id), data, length);
    }
    else {
        protocolMessageReceivedExt(GET_EXT_ID(id), data, length);
    }

    return 1;
}

void ocarinaMain(void) {
    // Initialize Ocarina protocol
    ocarinaErrorFlags = 0;
    ocarinaStatusFlags = 0;
    protocolInit(interface_id);

    // Initialize Tx library
    txInit();

#if BOARD >= 3000
    waitUntilInitDone3();

    // Bit of a hack: we depend on startup LED animation taking long enough
    if ((ISO_SENSE_GPIO->IDR & ISO_SENSE_MASK) != 0)
        ocarinaPanic(PANIC_ISO5V_SENSE);

    initRxTxLeds();

    unsigned int heartbeat_last_sent = 0;

    unsigned int autodetectLastHop = 0;

    ocarinaSetBitrate(0, 500000);

    HAL_NVIC_SetPriority(CEC_CAN_IRQn, 2, 1);
    HAL_NVIC_EnableIRQ(CEC_CAN_IRQn);
    CAN->IER |= CAN_IER_FMPIE0 | CAN_IER_FMPIE1;

    if (enableBitrateAutodetect) {
        detectingBitrate = 1;
        detected = 0;
    }
#else
    waitUntilInitDone2();
    ocarinaSetLEDs(0x01);

    unsigned int heartbeat_counter = 0;
    unsigned int test_message_counter = 0;

    unsigned int rx_last_second = 0;
    //unsigned int tx_last_second = 0;

    unsigned int shift_speed = 0;
    unsigned int shift_counter = 0;

    uint16_t shiftReg = 0;

    int fx_enabled = 1;
#endif

    HAL_NVIC_SetPriority(CEC_CAN_IRQn, 2, 1);
    HAL_NVIC_EnableIRQ(CEC_CAN_IRQn);
    CAN->IER |= CAN_IER_FMPIE0 | CAN_IER_FMPIE1;

    ocarinaRxEnabled = 1;

    for (;;) {
        txProcess();

        // FIXME: also handle RX FIFO overflow etc
        ocarinaStatusFlags = CAN->ESR;

        if ((CAN->ESR & CAN_ESR_LEC) != 0)
            LED_ERROR_GPIO->BSRR = LED_ERROR_MASK;
        else
            LED_ERROR_GPIO->BRR = LED_ERROR_MASK;

        protocolProcess();

#if BOARD >= 3000
        unsigned int tick = HAL_GetTick();

        if (enableBitrateAutodetect && detectingBitrate > 0) {
            if (detected) {
                int newBitrate = autodetectMin * (1 << (detectingBitrate - 1));
                ocarinaSetBitrate(1, newBitrate);
            }
            else if (tick > autodetectLastHop + autodetectHopInterval) {
                detectingBitrate++;

                int newBitrate = autodetectMin * (1 << (detectingBitrate - 1));
                if (newBitrate > autodetectMax) {
                    detectingBitrate = 1;
                    newBitrate = autodetectMin;
                }

                ocarinaSetBitrate(0, newBitrate);
                autodetectLastHop = tick;
            }
        }

        if (tick > heartbeat_last_sent + 1000) {
            protocolSendHeartbeat();
            heartbeat_last_sent = tick;
        }
#else
        if (fx_enabled) {
            if (shiftReg == 0)
                ocarinaSetLEDs(0);
            else if (++shift_counter >= shift_speed) {
                //shiftReg = (shiftReg << 1) | (shiftReg >> 15);
                shiftReg = (shiftReg >> 1) | (shiftReg << 15);
                ocarinaSetLEDs(shiftReg & 0xff);
                shift_counter = 0;

                if (shiftReg == 0xff00 && --rx_last_second == 0)
                    shiftReg = 0;
            }
        }

        timer_sleep(1);

        if (++test_message_counter == 50) {
            //send_test_message();
            test_message_counter = 0;
        }

        // should be approx. every second
        if (++heartbeat_counter == 1000) {
            // TODO: if (needToSendHeartbeat)
            protocolSendHeartbeat();
            heartbeat_counter = 0;

            rx_last_second = rx_this_second;
            rx_this_second = 0;

            if (rx_last_second > 0) {
                shiftReg = 0xff00;

                unsigned int slow = rx_last_second - 1;
                unsigned int min_slow = 4;
                unsigned int max_slow = 32;

                if (slow >= max_slow - min_slow)
                    shift_speed = min_slow;
                else
                    shift_speed = max_slow - slow;
            }
            else
                shiftReg = 0;
        }
#endif
    }
}

void CEC_CAN_IRQHandler(void) {
    if (CAN->RF0R & CAN_RF0R_FMP0) {
        if (ocarinaRxEnabled) {
            uint32_t id;

            if (CAN->sFIFOMailBox[0].RIR & CAN_RI0R_IDE) {
                // extended
                id = EXT_ID(0x1FFFFFFF & (CAN->sFIFOMailBox[0].RIR >> 3));
            }
            else {
                // standard
                id = STD_ID(0x000007FF & (CAN->sFIFOMailBox[0].RIR >> 21));
            }

            size_t length = CAN->sFIFOMailBox[0].RDTR & 0x0f;

            union {
                uint32_t rdr[2];
                uint8_t bytes[8];
            } rx;

            rx.rdr[0] = CAN->sFIFOMailBox[0].RDLR;
            rx.rdr[1] = CAN->sFIFOMailBox[0].RDHR;

            if (txReceiveCANMessage(id, rx.bytes, length) < 0)
                ocarinaErrorFlags |= OCARINA_CANIN_OVERFLOW;
            else
                rx_this_second += 1;
        }

        CAN->RF0R |= CAN_RF0R_RFOM0;
        ocarinaBlinkRx();

        detected = 1;
    }
    else
        ocarinaPanic(PANIC_CAN_BAD_IRQ);
}

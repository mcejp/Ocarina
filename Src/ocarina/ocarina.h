
#include <stdint.h>
#include <stdlib.h>

/*
 * 2001     Ocarina II rev.1
 * 3001     Ocarina III r1
 */
#ifndef BOARD
#error BOARD must be defined!
#endif

#if BOARD >= 2000 && BOARD < 3000
#define OCARINA_2
#elif BOARD >= 3000 && BOARD < 4000
#define OCARINA_3
#endif

#if BOARD >= 3000
#define LED_READY_GPIO      GPIOB
#define LED_READY_PIN       2
#define LED_READY_MASK      (1<<LED_READY_PIN)

#define LED_RX_GPIO         GPIOB
#define LED_RX_PIN          0
#define LED_RX_MASK         (1<<LED_RX_PIN)

#define LED_TX_GPIO         GPIOA
#define LED_TX_PIN          7
#define LED_TX_MASK         (1<<LED_TX_PIN)

#define LED_ERROR_GPIO      GPIOB
#define LED_ERROR_PIN       1
#define LED_ERROR_MASK      (1<<LED_ERROR_PIN)

#define LED_125K_GPIO       GPIOA
#define LED_125K_PIN        6
#define LED_125K_MASK       (1<<LED_125K_PIN)

#define LED_250K_GPIO       GPIOA
#define LED_250K_PIN        4
#define LED_250K_MASK       (1<<LED_250K_PIN)

#define LED_500K_GPIO       GPIOA
#define LED_500K_PIN        3
#define LED_500K_MASK       (1<<LED_500K_PIN)

#define LED_1M_GPIO         GPIOA
#define LED_1M_PIN          5
#define LED_1M_MASK         (1<<LED_1M_PIN)

#define ISO_SENSE_GPIO      GPIOB
#define ISO_SENSE_PIN       5
#define ISO_SENSE_MASK      (1<<ISO_SENSE_PIN)
#endif

enum {
    PANIC_CLOCK_STARTUP =   1,
    PANIC_CAN_BAD_IRQ =     2,
    PANIC_CAN_INIT =        3,
    PANIC_ISO5V_SENSE =     4,
};

extern uint16_t ocarinaErrorFlags;
extern uint32_t ocarinaStatusFlags;
extern volatile uint8_t ocarinaRxEnabled;

void ocarinaMain(void);

void ocarinaInitDone(void);
void ocarinaDataIn(const uint8_t* data, size_t length);
int ocarinaDataOut(const uint8_t* data, size_t length);

void ocarinaBlinkRx(void);
void ocarinaBlinkTx(void);
void ocarinaSetLEDs(uint8_t on);
void ocarinaSysTickHandler(void);

void ocarinaPanic(uint8_t code);

// TODO: hardware specific; where to put this?
void ConfigureCAN(int silent, int loopback, int bitrate);

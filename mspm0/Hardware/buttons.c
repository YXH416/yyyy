#include "buttons.h"

#include <stdint.h>

#include "ti_msp_dl_config.h"

/*
 * Buttons_Update() is called once per main-loop millisecond.  A transition
 * must therefore remain unchanged for about 20 ms before it is accepted.
 * Polling is used deliberately so the motor timer and UART ISRs are untouched.
 */
#define BUTTON_DEBOUNCE_SAMPLES    (20U)

typedef struct {
    bool stable_high;
    bool candidate_high;
    uint8_t candidate_count;
    bool press_pending;
} DebouncedButton;

static DebouncedButton calibrate_button;
static DebouncedButton start_button;

static bool button_read_high(GPIO_Regs *port, uint32_t pin)
{
    return (DL_GPIO_readPins(port, pin) != 0U);
}

static void button_init(
    DebouncedButton *button, GPIO_Regs *port, uint32_t pin)
{
    bool raw_high = button_read_high(port, pin);

    button->stable_high = raw_high;
    button->candidate_high = raw_high;
    button->candidate_count = 0U;
    button->press_pending = false;
}

static void button_update(
    DebouncedButton *button, GPIO_Regs *port, uint32_t pin)
{
    bool raw_high = button_read_high(port, pin);

    if (raw_high != button->candidate_high) {
        button->candidate_high = raw_high;
        button->candidate_count = 1U;
        return;
    }

    if (button->candidate_count < BUTTON_DEBOUNCE_SAMPLES) {
        button->candidate_count++;
    }

    if ((button->candidate_count >= BUTTON_DEBOUNCE_SAMPLES) &&
        (button->stable_high != button->candidate_high)) {
        button->stable_high = button->candidate_high;

        /* High -> low is one debounced press event. */
        if (!button->stable_high) {
            button->press_pending = true;
        }
    }
}

static bool button_take_press(DebouncedButton *button)
{
    bool pending = button->press_pending;
    button->press_pending = false;
    return pending;
}

/*
 * Self-contained GPIO initialization for the two buttons.
 * This module does NOT depend on SysConfig-generated button defines —
 * all pin and pull configuration is local, following the same IOMUX
 * values that the moter project's SysConfig generated.
 */
void Buttons_Init(void)
{
    /* PA17 — calibrate button, input with internal pull-up (active LOW) */
    DL_GPIO_initDigitalInputFeatures(
        IOMUX_PINCM39,
        DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_DISABLE,
        DL_GPIO_WAKEUP_DISABLE);

    /* PB7 — start button, input with internal pull-up (active LOW) */
    DL_GPIO_initDigitalInputFeatures(
        IOMUX_PINCM24,
        DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_DISABLE,
        DL_GPIO_WAKEUP_DISABLE);

    button_init(&calibrate_button,
        CALIBRATE_BUTTON_PORT, CALIBRATE_BUTTON_CAL_KEY_PIN);
    button_init(&start_button,
        START_BUTTON_PORT, START_BUTTON_START_KEY_PIN);
}

void Buttons_Update(void)
{
    button_update(&calibrate_button,
        CALIBRATE_BUTTON_PORT, CALIBRATE_BUTTON_CAL_KEY_PIN);
    button_update(&start_button,
        START_BUTTON_PORT, START_BUTTON_START_KEY_PIN);
}

bool Buttons_TakeCalibratePress(void)
{
    return button_take_press(&calibrate_button);
}

bool Buttons_TakeStartPress(void)
{
    return button_take_press(&start_button);
}



#ifndef BUTTONS_H
#define BUTTONS_H

#include <stdbool.h>

/*
 * MOD-033 button assignments:
 *   CALIBRATE: PA17 -- external normally-open button -- GND
 *   START:     PB7  -- external normally-open button -- GND
 *
 * Both inputs use the MSPM0 internal pull-up and are active low.
 */

/*
 * Local hardware defines — independent of SysConfig generated ti_msp_dl_config.
 * These are the same definitions that the moter project's SysConfig generated,
 * placed here so this module is self-contained and portable.
 */
#define CALIBRATE_BUTTON_PORT               GPIOA
#define CALIBRATE_BUTTON_CAL_KEY_PIN        DL_GPIO_PIN_17
#define START_BUTTON_PORT                   GPIOB
#define START_BUTTON_START_KEY_PIN          DL_GPIO_PIN_7

void Buttons_Init(void);
void Buttons_Update(void);
bool Buttons_TakeCalibratePress(void);
bool Buttons_TakeStartPress(void);

#endif /* BUTTONS_H */



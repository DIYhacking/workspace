/*
    ChibiOS - Copyright (C) 2006..2015 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

#include "ch.h"
#include "hal.h"
#include "test.h"


/*
 * Blinker thread #1.
 */
/*static THD_WORKING_AREA(waThread1, 128);
static THD_FUNCTION(Thread1, arg) {

  (void)arg;

  chRegSetThreadName("blinker");

  while (true) {

	  TestThread(&SD1);


  }
}*/

/*
 * Blinker thread #2.
 */
/*static THD_WORKING_AREA(waThread2, 128);
static THD_FUNCTION(Thread2, arg) {

  (void)arg;

  chRegSetThreadName("blinker");
  while (true) {
    palSetPad(GPIOB, GPIOB_LED3);
    chThdSleepMilliseconds(1000);
    palClearPad(GPIOB, GPIOB_LED3);
    chThdSleepMilliseconds(1000);
	  TestThread2(&SD1);
  }
}*/

/*
 * Application entry point.
 */
int main(void) {

  /*
   * System initializations.
   * - HAL initialization, this also initializes the configured device drivers
   *   and performs the board-specific initializations.
   * - Kernel initialization, the main() function becomes a thread and the
   *   RTOS is active.
   */
  halInit();
  chSysInit();

  /*
   * Activates the serial driver 1 using the driver default configuration.
   * PA9(TX) and PA10(RX) are routed to USART1.
   */
  sdStart(&SD1, NULL);
  palSetPadMode(GPIOA, 9, PAL_MODE_ALTERNATE(7));
  palSetPadMode(GPIOA, 10, PAL_MODE_ALTERNATE(7));
  adcStart(&ADCD1, NULL);

  /*
   * Creates the example threads.
   */
 // chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO+1, Thread1, NULL);
 // chThdCreateStatic(waThread2, sizeof(waThread2), NORMALPRIO+1, Thread2, NULL);

  /*
   * Normal main() thread activity, in this demo it does nothing except
   * sleeping in a loop and check the button state, when the button is
   * pressed the test procedure is launched.
   */
  while (true) {
    //if (palReadPad(GPIOA, GPIOA_BUTTON))
	 /* palSetPad(GPIOB, GPIOB_LED4);
	  palSetPad(GPIOB, 9);//LED4
	  palSetPad(GPIOB, 8);
	  palSetPad(GPIOB, 5);
	  palSetPad(GPIOB, 4);
	  palSetPad(GPIOB, 3);
	  palSetPad(GPIOA, 15);
	  palSetPad(GPIOB, 12);
	  palSetPad(GPIOB, 13);
	  palSetPad(GPIOB, 14);
	  palSetPad(GPIOB, 15);
	  palSetPad(GPIOB, 10);
	  palSetPad(GPIOB, 11);
	  palSetPad(GPIOB, 1);
	  palSetPad(GPIOB, 2);*/
	  palSetPad(GPIOB, 12);
	 /* palSetPad(GPIOA, 2);
	  palSetPad(GPIOC, 13);*/

	  chThdSleepMilliseconds(1000);

	  /*palClearPad(GPIOB, GPIOB_LED4);
	  palClearPad(GPIOB, 9);
	  palClearPad(GPIOB, 8);
	  palClearPad(GPIOB, 5);
	  palClearPad(GPIOB, 4);
	  palClearPad(GPIOB, 3);
	  palClearPad(GPIOA, 15);
	  palClearPad(GPIOB, 12);
	  palClearPad(GPIOB, 13);
	  palClearPad(GPIOB, 14);
	  palClearPad(GPIOB, 15);
	  palClearPad(GPIOB, 10);
	  palClearPad(GPIOB, 11);
	  palClearPad(GPIOB, 1);
	  palClearPad(GPIOB, 2);*/
	  palClearPad(GPIOB, 12);
	 /* palClearPad(GPIOA, 2);
	  palClearPad(GPIOC, 13);*/

	  chThdSleepMilliseconds(1000);
    //chThdSleepMilliseconds(500);
  }
}

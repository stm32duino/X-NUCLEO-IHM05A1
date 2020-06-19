 /**
 ******************************************************************************
 * @file    main.cpp
 * @author  IPC Rennes
 * @version V1.0.0
 * @date    April 13th, 2016
 * @brief   mbed simple application for the STMicroelectronics X-NUCLEO-IHM05A1
 *          Motor Control Expansion Board: control of 1 motor.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2016 STMicroelectronics</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/

/* Arduino specific header files. */
#include "Arduino.h"

/* Component specific header files. */
#include "L6208.h"
/* Definitions ---------------------------------------------------------------*/
#define SerialPort Serial
#ifdef TARGET_NUCLEO_F334R8
#define VREFA_PWM_PIN D11
#define VREFB_PWM_PIN D9
#elif TARGET_NUCLEO_F302R8
#define VREFA_PWM_PIN D11
#define VREFB_PWM_PIN D15 /* HW mandatory patch: bridge manually D9 with D15 */
#else
#define VREFA_PWM_PIN D3
#define VREFB_PWM_PIN D9
#endif

/* Variables -----------------------------------------------------------------*/

/* Initialization parameters of the motor connected to the expansion board. */
l6208_init_t init_s =
{
  1500,            //Acceleration rate in step/s^2 or (1/16)th step/s^2 for microstep modes
  40,              //Acceleration current torque in % (from 0 to 100)
  1500,            //Deceleration rate in step/s^2 or (1/16)th step/s^2 for microstep modes
  30,              //Deceleration current torque in % (from 0 to 100)
  1500,            //Running speed in step/s or (1/16)th step/s for microstep modes
  50,              //Running current torque in % (from 0 to 100)
  20,              //Holding current torque in % (from 0 to 100)
  STEP_MODE_1_16,  //Step mode via enum motorStepMode_t
  FAST_DECAY,      //Decay mode via enum motorDecayMode_t
  0,               //Dwelling time in ms
  FALSE,           //Automatic HIZ STOP
  100000           //VREFA and VREFB PWM frequency (Hz)
};

/* Motor Control Component. */
L6208 *motor;

/* Functions -----------------------------------------------------------------*/

/**
 * @brief  This is an example of user handler for the flag interrupt.
 * @param  None
 * @retval None
 * @note   If needed, implement it, and then attach and enable it:
 *           + motor->attach_flag_irq(&my_flag_irq_handler);
 *           + motor->enable_flag_irq();
 *         To disable it:
 *           + motor->DisbleFlagIRQ();
 */
void my_flag_irq_handler(void)
{
  SerialPort.print("    WARNING: \"FLAG\" interrupt triggered:\r\n");
  motor->disable();
  SerialPort.print("    Motor disabled.\r\n\n");

}
/**
 * @brief  This is an example of error handler.
 * @param[in] error Number of the error
 * @retval None
 * @note   If needed, implement it, and then attach it:
 *           + motor->attach_error_handler(&my_error_handler);
 */
void my_error_handler(uint16_t error)
{
  char report[256];
  sprintf(report, "Error %d detected\r\n\n", error);
  SerialPort.print(report);

  /* Infinite loop */
  while (true) {
  }    
}

void setup()
{
  char report[256];
  /* Initialize Serial Port */
  SerialPort.begin(115200);
    /* Printing to the console. */
  SerialPort.println("STARTING MAIN PROGRAM\r\n");
  SerialPort.println("    Reminder:\r\n");
  SerialPort.println("    The position unit is in agreement to the step mode.\r\n");
  SerialPort.println("    The speed, acceleration or deceleration unit depend on the step mode:\r\n");
  SerialPort.println("    - For normal mode and half step mode, the unit is steps/s or /s^2.\r\n");
  SerialPort.println("    - For microstep modes, the unit is (1/16)steps/s or /s^2.\r\n");

  SerialPort.println("inizio ");
  /* Initializing Motor Control Component. */
  motor = new L6208(D2, D8, D7, D4, D5, D6, VREFA_PWM_PIN, VREFB_PWM_PIN);
  if (motor->init(&init_s) != COMPONENT_OK) {
    exit(EXIT_FAILURE);
  }

    /* Attaching and enabling an interrupt handler. */
  motor->attach_flag_irq(&my_flag_irq_handler);
  motor->enable_flag_irq();

    /* Attaching an error handler */
  motor->attach_error_handler(&my_error_handler);
  /* Printing to the console. */
  SerialPort.println("Motor Control Application Example for 1 Motor\r\n");



//----- run the motor BACKWARD
 /* sprintf(report," StepperMotor::BWD => 0x%x ",  motor->get_deceleration());
  SerialPort.println(report);
 */
  SerialPort.println("--> Running the motor backward.\r\n");
  motor->run(StepperMotor::BWD);
   SerialPort.println("--> dovrebbe FUNZIONARE \r\n");
}

void loop(){
  
  }

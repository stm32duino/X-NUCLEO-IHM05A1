/**
 ******************************************************************************
 * @file    X_NUCLEO_IHM05A1_HelloWorld.ino
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

#if defined(ARDUINO_NUCLEO_F401RE) || defined(ARDUINO_NUCLEO_L152RE) || defined(ARDUINO_NUCLEO_L476RG)
  #define VREFA_PWM_PIN D3
  #define VREFB_PWM_PIN D9
  #define TICKER_INSTANCE TIM5
#elif defined(ARDUINO_NUCLEO_L053R8)
  #define VREFA_PWM_PIN D3
  #define VREFB_PWM_PIN D9
  #define TICKER_INSTANCE TIM21
#else
  #error "Target board not tested! Please, comment this line and define the good pins for VrefA and VrefB and a good instance for the Ticker Hardware Timer"
  /* Like for example */
  /*
  #define VREFA_PWM_PIN D3
  #define VREFB_PWM_PIN D9
  #define TICKER_INSTANCE TIM1
  */
#endif

/* Variables -----------------------------------------------------------------*/

/* Initialization parameters of the motor connected to the expansion board. */
l6208_init_t init_s = {
  1500,            //Acceleration rate in step/s^2 or (1/16)th step/s^2 for microstep modes
  20,              //Acceleration current torque in % (from 0 to 100)
  1500,            //Deceleration rate in step/s^2 or (1/16)th step/s^2 for microstep modes
  20,              //Deceleration current torque in % (from 0 to 100)
  1500,            //Running speed in step/s or (1/16)th step/s for microstep modes
  10,              //Running current torque in % (from 0 to 100)
  5,               //Holding current torque in % (from 0 to 100)
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
  while (1) {
  }
}

void setup()
{
  int32_t pos;
  uint16_t mySpeed;
  /* Initialize Serial Port */
  SerialPort.begin(115200);

  /* Initializing Motor Control Component. */
  SerialPort.println("Initializing Motor Control Component.");
  motor = new L6208(D2, D8, D7, D4, D5, D6, VREFA_PWM_PIN, VREFB_PWM_PIN, TICKER_INSTANCE);
  if (motor->init(NULL) != COMPONENT_OK) {
    exit(EXIT_FAILURE);
  }

  /* Attaching and enabling an interrupt handler. */
  motor->attach_flag_irq(&my_flag_irq_handler);
  motor->enable_flag_irq();

  /* Attaching an error handler */
  motor->attach_error_handler(&my_error_handler);
  /* Printing to the console. */
  SerialPort.println("Motor Control Application Example for 1 Motor\r\n");

  //----- Move of 16000 microsteps in the FW direction

  /* Move device 16000 microsteps in the FORWARD direction */
  SerialPort.println("Move device 16000 microsteps in the FORWARD direction.");
  motor->move(StepperMotor::FWD, 16000);

  /* Wait for the motor ends moving */
  motor->wait_while_active();

  /* Wait for 2 seconds */
  delay(2000);

  //----- Move of 16000 microsteps in the BW direction

  /* Move device 16000 microsteps in the BACKWARD direction*/
  SerialPort.println("Move device 16000 microsteps in the BACKWARD direction.");
  motor->move(StepperMotor::BWD, 16000);

  /* Wait for the motor ends moving */
  motor->wait_while_active();

  /* Get current position */
  pos = motor->get_position();
  SerialPort.print("Position: ");
  SerialPort.println(pos);

  /* Set the current position to be the Home position */
  Serial.println("Set the current position to be the Home position.");
  motor->set_home(pos);

  /* Wait for 2 seconds */
  delay(2000);

  //----- Go to position -6400

  /* Request to go to position -6400 */
  Serial.println("Go to position -6400.");
  motor->go_to(-6400);

  /* Wait for the motor ends moving */
  motor->wait_while_active();

  /* Get current position */
  pos = motor->get_position();
  SerialPort.print("Position: ");
  SerialPort.println(pos);

  if (pos != -6400) {
    my_error_handler(L6208_ERROR_POSITION);
  }

  /* Get current position */
  pos = motor->get_position();
  SerialPort.print("Position: ");
  SerialPort.println(pos);


  /* Set the current position to be the Mark position */
  SerialPort.println("Set the current position to be the Mark position.");
  motor->set_mark(pos);

  /* Wait for 2 seconds */
  delay(2000);

  //----- Go Home

  /* Request to go to Home */
  Serial.println("Go Home.");
  motor->go_home();
  motor->wait_while_active();

  /* Get current position */
  pos = motor->get_position();
  SerialPort.print("Position: ");
  SerialPort.println(pos);

  /* Wait for 2 seconds */
  delay(2000);

  //----- Go to position 6400

  /* Request to go to position 6400 */
  SerialPort.println("Go to position 6400.");
  motor->go_to(StepperMotor::FWD, 6400);

  /* Wait for the motor ends moving */
  motor->wait_while_active();

  /* Get current position */
  pos = motor->get_position();
  SerialPort.print("Position: ");
  SerialPort.println(pos);
  /* Wait for 2 seconds */
  delay(2000);

  //----- Go Mark which was set previously after go to -6400

  /* Request to go to Mark position */
  SerialPort.println("Go to Mark position.");
  motor->go_mark();

  /* Wait for the motor ends moving */
  motor->wait_while_active();

  /* Get current position */
  pos = motor->get_position();
  SerialPort.print("Position: ");
  SerialPort.println(pos);

  /* Wait for 2 seconds */
  delay(2000);

  //----- Run the motor BACKWARD

  /* Request to run BACKWARD */
  SerialPort.println("Run BACKWARD.");
  motor->run(StepperMotor::BWD);
  delay(5000);

  /* Get current speed */
  mySpeed = motor->get_speed();


  //----- Increase the speed while running

  /* Increase speed to 2400 microstep/s */
  motor->set_max_speed(2400);
  delay(5000);

  /* Get current speed */
  mySpeed = motor->get_speed();
  SerialPort.print("Speed: ");
  SerialPort.println(mySpeed);
  //----- Decrease the speed while running

  /* Decrease speed to 1200 microstep/s */
  SerialPort.println(" Decrease speed to 1200 microstep/s.");
  motor->set_max_speed(1200);
  delay(5000);

  /* Get current speed */
  mySpeed = motor->get_speed();
  SerialPort.print("Speed: ");

  SerialPort.println(mySpeed);
  //----- Increase acceleration while running

  /* Increase acceleration to 2000 microstep/s^2 */
  SerialPort.println("Increase acceleration to 2000 microstep/s^2.");
  motor->set_acceleration(2000);
  delay(5000);

  /* Increase speed to 2400 microstep/s */
  SerialPort.println("Increase speed to 2400 microstep/s.");
  motor->set_max_speed(2400);
  delay(5000);

  /* Get current speed */
  mySpeed = motor->get_speed();
  SerialPort.print("Speed: ");
  SerialPort.println(mySpeed);

  if (mySpeed != 2400) {
    my_error_handler(L6208_ERROR_SPEED);
  }

  //----- Increase deceleration while running

  /* Increase deceleration to 2000 microstep/s^2 */
  SerialPort.println("Increase deceleration to 2000 microstep/s^2.");
  motor->set_deceleration(2000);
  delay(5000);

  /* Decrease speed to 1200 microstep/s */
  SerialPort.println("Decrease speed to 1200 microstep/s.");
  motor->set_max_speed(1200);
  delay(5000);

  /* Get current speed */
  mySpeed = motor->get_speed();
  SerialPort.print("Speed: ");
  SerialPort.println(mySpeed);

  //----- Soft stopped required while running

  /* Request soft stop */
  SerialPort.println("soft stop.");
  motor->soft_stop();

  /* Wait for the motor ends moving */
  motor->wait_while_active();

  /* Wait for 2 seconds */
  delay(2000);

  //----- Run stopped by hardstop

  /* Request to run in FORWARD direction */
  SerialPort.println("Run in FORWARD direction.");
  motor->run(StepperMotor::FWD);
  delay(5000);

  /* Request to immediatly stop */
  SerialPort.println("immediatly stop.");
  motor->hard_stop();
  motor->wait_while_active();

  /* Wait for 2 seconds */
  delay(2000);

  //----- GOTO stopped by softstop

  /* Request to go to position 20000  */
  SerialPort.println("Go to position 20000.");
  motor->go_to(20000);
  delay(5000);

  /* Request to perform a soft stop */
  SerialPort.println("soft stop.");
  motor->soft_stop();
  motor->wait_while_active();

  /* Wait for 2 seconds */
  delay(2000);

  //----- Change step mode to full step mode

  /* Select full step mode (normal mode) */
  SerialPort.println("Select full step mode (normal mode).");
  motor->set_step_mode(StepperMotor::STEP_MODE_FULL);

  /* Set speed, acceleration and deceleration to scale with normal mode */
  /* For normal mode and half step mode, these parameters are in steps/s or /s^2 */
  /* For microstep modes, these parameters are either in (1/16)step/s or /s^2 */
  motor->set_max_speed((motor->get_max_speed()) >> 4);
  motor->set_min_speed(L6208_MIN_SPEED);
  motor->set_acceleration((motor->get_acceleration()) >> 4);
  motor->set_deceleration((motor->get_deceleration()) >> 4);

  /* Request to go position 200 (full steps) */
  SerialPort.println("Go position 200 (full steps).");
  motor->go_to(200);

  /* Wait for the motor ends moving */
  motor->wait_while_active();

  /* Get current position */
  pos =  motor->get_position();
  SerialPort.print("Position: ");
  SerialPort.println(pos);
  if (pos != 200) {
    my_error_handler(L6208_ERROR_POSITION);
  }

  /* Wait for 2 seconds */
  delay(2000);

  //----- Change step mode to half step mode
  /* Select half step mode */
  SerialPort.println("Select half step mode.");
  motor->set_step_mode(StepperMotor::STEP_MODE_HALF);

  /* Request to go position -400 (half steps) */
  SerialPort.println("go position -400 (half steps).");
  motor->go_to(-400);

  /* Wait for the motor ends moving */
  motor->wait_while_active();

  /* Get current position */
  pos =  motor->get_position();
  SerialPort.print("Position: ");
  SerialPort.println(pos);

  if (pos != -400) {
    my_error_handler(L6208_ERROR_POSITION);
  }

  /* Wait for 2 seconds */
  delay(2000);

  //----- Change step mode 1/4 microstepping mode
  /* Select 1/4 step mode */
  SerialPort.println("Select 1/4 step mode.");
  motor->set_step_mode(StepperMotor::STEP_MODE_1_4);

  /* Set speed, acceleration and deceleration to scale with microstep mode */
  /* For normal mode and half step mode, these parameters are in steps/s or /s^2 */
  /* For microstep modes, these parameters are either in (1/16)step/s or /s^2 */
  motor->set_max_speed((motor->get_max_speed()) << 4);
  motor->set_min_speed(L6208_MIN_SPEED);
  motor->set_acceleration((motor->get_acceleration()) << 4);
  motor->set_deceleration((motor->get_deceleration()) << 4);

  /* Request to go position 800 (quarter steps) */
  SerialPort.println("Go position 800 (quarter steps).");
  motor->go_to(800);

  /* Wait for the motor ends moving */
  motor->wait_while_active();

  /* Get current position */
  pos =  motor->get_position();
  SerialPort.print("Position: ");
  SerialPort.println(pos);

  if (pos != 800) {
    my_error_handler(L6208_ERROR_POSITION);
  }

  /* Wait for 2 seconds */
  delay(2000);

  //----- Change step mode 1/8 microstepping mode
  /* Select 1/8 step mode */
  SerialPort.println("Select 1/8 step mode.");
  motor->set_step_mode(StepperMotor::STEP_MODE_1_8);

  /* Request to go position -1600 (1/8th steps) */
  SerialPort.println("Go position -1600 (1/8th steps).");
  motor->go_to(-1600);

  /* Wait for the motor ends moving */
  motor->wait_while_active();

  /* Get current position */
  pos =  motor->get_position();
  SerialPort.print("Position: ");
  SerialPort.println(pos);

  if (pos != -1600) {
    my_error_handler(L6208_ERROR_POSITION);
  }

  /* Wait for 2 seconds */
  delay(2000);

  //----- Restore 1/16 microstepping mode

  /* Reset device to 1/16 microstepping mode */
  SerialPort.println("Reset device to 1/16 microstepping mode.");
  motor->set_step_mode(StepperMotor::STEP_MODE_1_16);

  /* Set speed and acceleration at lowest values */
  motor->set_max_speed(L6208_MIN_SPEED);
  motor->set_acceleration(L6208_MIN_ACC_DEC_RATE);
  motor->set_deceleration(L6208_MIN_ACC_DEC_RATE);

  /* Move device 49 microsteps (1/16th steps) in the FORWARD direction*/
  SerialPort.println("Move device 49 microsteps (1/16th steps) in the FORWARD direction.");
  motor->move(StepperMotor::FWD, 49);

  /* Wait for the motor ends moving */
  SerialPort.println("Wait for the motor ends moving.");
  motor->wait_while_active();

  /* Get current position */
  pos =  motor->get_position();
  SerialPort.print("Position: ");
  SerialPort.println(pos);

  if (pos != 49) {
    my_error_handler(L6208_ERROR_POSITION);
  }

  /* Set speed and acceleration from powerspin6208_target_config.h */
  motor->set_max_speed(L6208_CONF_PARAM_RUNNING_SPEED);
  motor->set_acceleration(L6208_CONF_PARAM_ACC_RATE);
  motor->set_deceleration(L6208_CONF_PARAM_DEC_RATE);

  /* Turn off power bridges when motor is stopped */

  motor->set_stop_mode(HIZ_MODE);

  SerialPort.println("--> Infinite Loop...\r\n");

}



/* loop ----------------------------------------------------------------------*/

void loop()
{
  /* Request to run */
  SerialPort.println("Run.");
  if (motor->get_direction() == StepperMotor::FWD) {
    motor->run(StepperMotor::BWD);
  } else {
    motor->run(StepperMotor::FWD);
  }

  delay(5000);

  /* Request soft stop */
  SerialPort.println("Soft stop.");
  motor->soft_stop();
  delay(2000);
}
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

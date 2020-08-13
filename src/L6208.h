/**
 ******************************************************************************
 * @file    L6208.h
 * @author  IPC Rennes
 * @version V1.0.0
 * @date    March 18th, 2016
 * @brief   This file contains the class of a L6208 Motor Control component.
 * @note    (C) COPYRIGHT 2016 STMicroelectronics
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


/* Define to prevent recursive inclusion -------------------------------------*/

#ifndef __L6208_CLASS_H
#define __L6208_CLASS_H


/* Includes ------------------------------------------------------------------*/

/* ACTION 1 ------------------------------------------------------------------*
 * Include here platform specific header files.                               *
 *----------------------------------------------------------------------------*/
#include "Arduino.h"

/* ACTION 2 ------------------------------------------------------------------*
 * Include here component specific header files.                              *
 *----------------------------------------------------------------------------*/
#include "L6208_def.h"
/* ACTION 3 ------------------------------------------------------------------*
 * Include here interface specific header files.                              *
 *                                                                            *
 * Example:                                                                   *
 *   #include "HumiditySensor.h"                                              *
 *   #include "TemperatureSensor.h"                                           *
 *----------------------------------------------------------------------------*/
#include "StepperMotor.h"

/* Typedefs ------------------------------------------------------------------*/
template <typename T>
struct Callback;

template <typename Ret, typename... Params>
struct Callback<Ret(Params...)> {
  template <typename... Args>
  static Ret callback(Args... args)
  {
    return func(args...);
  }
  static std::function<Ret(Params...)> func;
};

template <typename Ret, typename... Params>
std::function<Ret(Params...)> Callback<Ret(Params...)>::func;

typedef void (*TickHandler_Callback)(void);

/* Classes -------------------------------------------------------------------*/

/**
 * @brief Class representing a L6208 component.
 */

class L6208 : public StepperMotor {
  public:
    /*** Constructor and Destructor Methods ***/

    /**
     * @brief Constructor.
     * @param flag_and_enable_pin   pin name of the EN pin of the component.
     * @param reset_pin             pin name of the RESET pin of the component.
     * @param direction_pin         pin name of the CW_CCW pin of the component.
     * @param half_full_pin         pin name of the HALF_FULL pin of the component.
     * @param control_pin           pin name of the CONTROL pin of the component.
     * @param clock_pin             pin name of the CLOCK pin of the component.
     * @param vrefA_pwm_pin         pin name of the PWM connected to the VREFA pin of the component.
     * @param vrefB_pwm_pin         pin name of the PWM connected to the VREFB pin of the component.
     */

    L6208(uint8_t flag_and_enable_pin, uint8_t reset_pin, uint8_t direction_pin, uint8_t half_full_pin, uint8_t control_pin, uint8_t clock_pin, uint8_t vrefA_pwm_pin, uint8_t vrefB_pwm_pin) : StepperMotor(),
      flag_and_enable(flag_and_enable_pin),
      reset_pin(reset_pin),
      direction_pin(direction_pin),
      half_full_pin(half_full_pin),
      control_pin(control_pin),
      clock_pin(clock_pin),
      vrefA_pwm(vrefA_pwm_pin),
      vrefB_pwm(vrefB_pwm_pin)

    {
      pinMode(A5, OUTPUT);
      current_state_ticker = 0;


      current_pwm_periodA = 0;
      current_pwm_periodB = 0;
      current_pwm_pulseWidthA = 0;
      current_pwm_pulseWidthB = 0;


      pinMode(flag_and_enable, OUTPUT);
      pinMode(reset_pin, OUTPUT);
      pinMode(direction_pin, OUTPUT);
      pinMode(control_pin, OUTPUT);
      pinMode(clock_pin, OUTPUT);

      // #if defined(TIM1)
      //     TIM_TypeDef *Instance = TIM1;
      // #else
      //     TIM_TypeDef *Instance = TIM2;
      // #endif
      //  TIM_TypeDef *Instance = TIM4;//F4
      TIM_TypeDef *Instance = TIM9;//or TIM10 L1
      //    TIM_TypeDef *Instance = TIM21; //L0

      Callback<void()>::func = std::bind(&L6208::L6208_TickHandler, this);
      callback_handler = static_cast<TickHandler_Callback>(Callback<void()>::callback);

      vrefA_pwm_instance = (TIM_TypeDef *)pinmap_peripheral(digitalPinToPinName(vrefA_pwm), PinMap_PWM);
      vrefA_pwm_channel = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(vrefA_pwm), PinMap_PWM));


      vrefB_pwm_instance = (TIM_TypeDef *)pinmap_peripheral(digitalPinToPinName(vrefB_pwm), PinMap_PWM);
      vrefB_pwm_channel = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(vrefB_pwm), PinMap_PWM));

      vrefA_pwm_timer = new HardwareTimer(vrefA_pwm_instance);
      vrefA_pwm_timer->setMode(vrefA_pwm_channel, TIMER_OUTPUT_COMPARE_PWM1, vrefA_pwm_pin);
      vrefA_pwm_timer ->setOverflow(current_pwm_periodA, MICROSEC_FORMAT);
      vrefA_pwm_timer->setCaptureCompare(vrefA_pwm_channel, current_pwm_pulseWidthA, MICROSEC_COMPARE_FORMAT);

      vrefB_pwm_timer = new HardwareTimer(vrefB_pwm_instance);
      vrefB_pwm_timer->setMode(vrefB_pwm_channel, TIMER_OUTPUT_COMPARE_PWM1, vrefB_pwm_pin);
      vrefB_pwm_timer ->setOverflow(current_pwm_periodB, MICROSEC_FORMAT);
      vrefB_pwm_timer->setCaptureCompare(vrefB_pwm_channel, current_pwm_pulseWidthB, MICROSEC_COMPARE_FORMAT);

      ticker = new HardwareTimer(Instance);

      errorHandlerCallback = 0;
      memset(&device_prm, 0, sizeof(deviceParams_t));
      deviceInstance = numberOfDevices++;
      tickFreq = TIMER_TICK_FREQUENCY;
      pMicroTable2 = &(microTable1[16]);
    }

    /**
      * @brief Destructor.
      */
    virtual  ~L6208(void)
    {
      free(vrefA_pwm_timer);
      free(vrefB_pwm_timer);

    }

    /*** Public Component Related Methods ***/
    /**
     * @brief Public functions inherited from the Component Class
     */

    /**
     * @brief  Initialize the component.
     * @param  init Pointer to device specific initalization structure.
     * @retval "0" in case of success, an error code otherwise.
     */

    virtual int init(void *init = NULL)
    {
      return (int) L6208_Init((void *) init);
    }

    /**
    * @brief  Getting the ID of the component.
    * @param  id Pointer to an allocated variable to store the ID into.
    * @retval "0" in case of success, an error code otherwise.
    */
    virtual int read_id(uint8_t *id = NULL)
    {
      return (int) L6208_ReadID((uint8_t *) id);
    }

    virtual unsigned int get_status(void)
    {
      return (unsigned int) L6208_GetMotionState();
    }

    /**
     * @brief  Getting the position.
     * @param  None.
     * @retval The position.
     */
    virtual signed int get_position(void)
    {
      return (signed int)L6208_GetPosition();
    }

    /**
     * @brief  Getting the marked position.
     * @param  None.
     * @retval The marked position.
     */
    virtual signed int get_mark(void)
    {
      return (signed int)L6208_GetMark();
    }

    /**
     * @brief  Getting the current speed in pps.
     * @param  None.
     * @retval The current speed in pps.
     */
    virtual unsigned int get_speed(void)
    {
      return (unsigned int)L6208_GetCurrentSpeed();
    }
    /**
     * @brief  Getting the maximum speed in pps.
     * @param  None.
     * @retval The maximum speed in pps.
     */
    virtual unsigned int get_max_speed(void)
    {
      return (unsigned int)L6208_GetMaxSpeed();
    }

    /**
     * @brief  Getting the minimum speed in pps.
     * @param  None.
     * @retval The minimum speed in pps.
     */
    virtual unsigned int get_min_speed(void)
    {
      return (unsigned int)L6208_GetMinSpeed();
    }

    /**
     * @brief  Getting the acceleration in pps^2.
     * @param  None.
     * @retval The acceleration in pps^2.
     */
    virtual unsigned int get_acceleration(void)
    {
      return (unsigned int)L6208_GetAcceleration();
    }

    /**
     * @brief  Getting the deceleration in pps^2.
     * @param  None.
     * @retval The deceleration in pps^2.
     */
    virtual unsigned int get_deceleration(void)
    {
      return (unsigned int)L6208_GetDeceleration();
    }

    /**
     * @brief  Getting the direction of rotation.
     * @param  None.
     * @retval The direction of rotation.
     */
    virtual direction_t get_direction(void)
    {
      if (L6208_GetDirection() != BACKWARD) {
        return FWD;
      } else {
        return BWD;
      }
    }

    /**
     * @brief  Setting the current position to be the home position.
     * @param  None.
     * @retval None.
     */
    virtual void set_home(void)
    {
      L6208_SetHome();
    }

    /**
     * @brief  Setting the current position to be the marked position.
     * @param  None.
     * @retval None.
     */
    virtual void set_mark(void)
    {
      L6208_SetMark();
    }

    /**
     * @brief  Setting the maximum speed in pps.
     * @param  speed The maximum speed in pps.
     * @retval "true" in case of success, "false" otherwise.
     */
    virtual bool set_max_speed(unsigned int speed)
    {
      if (speed <= 0xFFFF) {
        return L6208_SetMaxSpeed((uint16_t) speed);
      } else {
        return false;
      }
    }

    /**
     * @brief  Setting the minimum speed in pps.
     * @param  speed The minimum speed in pps.
     * @retval "true" in case of success, "false" otherwise.
     */
    virtual bool set_min_speed(unsigned int speed)
    {
      if (speed <= 0xFFFF) {
        return L6208_SetMinSpeed((uint16_t) speed);
      } else {
        return false;
      }
    }

    /**
     * @brief  Setting the acceleration in pps^2.
     * @param  acceleration The acceleration in pps/s^2.
     * @retval "true" in case of success, "false" otherwise.
     */
    virtual bool set_acceleration(unsigned int acceleration)
    {
      if (acceleration <= 0xFFFF) {
        return L6208_SetAcceleration((uint16_t) acceleration);
      } else {
        return false;
      }
    }

    /**
     * @brief  Setting the deceleration in pps^2.
     * @param  deceleration The deceleration in pps^2.
     * @retval "true" in case of success, "false" otherwise.
     */
    virtual bool set_deceleration(unsigned int deceleration)
    {
      if (deceleration <= 0xFFFF) {
        return L6208_SetDeceleration((uint16_t) deceleration);
      } else {
        return false;
      }
    }

    /**
     * @brief  Setting the Step Mode.
     * @param  step_mode The Step Mode.
     * @retval "true" in case of success, "false" otherwise.
     * @note   step_mode can be one of the following:
     *           + STEP_MODE_FULL
     *           + STEP_MODE_WAVE
     *           + STEP_MODE_HALF
     *           + STEP_MODE_1_4
     *           + STEP_MODE_1_8
     *           + STEP_MODE_1_16
     */
    virtual bool set_step_mode(step_mode_t step_mode)
    {
      return L6208_SetStepMode((motorStepMode_t) step_mode);
    }

    /**
     * @brief  Going to a specified position.
     * @param  position The desired position.
     * @retval None.
     */
    virtual void go_to(signed int position)
    {
      L6208_GoTo((int32_t)position);
    }

    /**
     * @brief  Going to the home position.
     * @param  None.
     * @retval None.
     */
    virtual void go_home(void)
    {
      L6208_GoHome();
    }

    /**
     * @brief  Going to the marked position.
     * @param  None.
     * @retval None.
     */
    virtual void go_mark(void)
    {
      L6208_GoMark();
    }

    /**
     * @brief  Running the motor towards a specified direction.
     * @param  direction The direction of rotation.
     * @retval None.
     */
    virtual void run(direction_t direction)
    {
      L6208_Run((motorDir_t)(direction == StepperMotor::FWD ? FORWARD : BACKWARD));
    }


    /**
     * @brief  Moving the motor towards a specified direction for a certain number of steps.
     * @param  direction The direction of rotation.
     * @param  steps The desired number of steps.
     * @retval None.
     */
    virtual void move(direction_t direction, unsigned int steps)
    {
      L6208_Move((motorDir_t)(direction == StepperMotor::FWD ? FORWARD : BACKWARD), (uint32_t)steps);
    }

    /**
     * @brief  Stopping the motor through an immediate deceleration up to zero speed.
     * @param  None.
     * @retval None.
     */
    virtual void soft_stop(void)
    {
      L6208_SoftStop();
    }

    /**
     * @brief  Stopping the motor through an immediate infinite deceleration.
     * @param  None.
     * @retval None.
     */
    virtual void hard_stop(void)
    {
      L6208_HardStop();
    }
    /**
    * @brief  Disabling the power bridge after performing a deceleration to zero.
    * @param  None.
    * @retval None.
    */
    virtual void soft_hiz(void)
    {
      motorStopMode_t stopMode = L6208_GetStopMode();
      if (stopMode == HIZ_MODE) {
        L6208_SoftStop();
      } else {
        L6208_SetStopMode(HIZ_MODE);
        L6208_SoftStop();
        L6208_SetStopMode(stopMode);
      }
    }

    /**
     * @brief  Disabling the power bridge immediately.
     * @param  None.
     * @retval None.
     */
    virtual void hard_hiz(void)
    {
      L6208_HardHiZ();
    }

    /**
     * @brief  Waiting while the motor is active.
     * @param  None.
     * @retval None.
     */
    virtual void wait_while_active(void)
    {
      L6208_WaitWhileActive();
    }

    /**
     * @brief Public functions NOT inherited
     */

    /**
     * @brief  Attaching an error handler.
     * @param  fptr An error handler.
     * @retval None.
     */
    virtual void attach_error_handler(void (*fptr)(uint16_t error))
    {
      L6208_AttachErrorHandler((void (*)(uint16_t error)) fptr);
    }

    /**
     * @brief  Checks if the device is disabled or/and has an alarm flag set
     * by reading the EN pin position.
     * @param  None.
     * @retval One if the EN pin is low (the device is disabled or/and
     * has an alarm flag set), otherwise zero.
     */
    virtual unsigned int check_status_hw(void)
    {
      if (!(((uint16_t)digitalRead(flag_and_enable)))) {
        return 0x01;
      } else {
        return 0x00;
      }
    }

    /**
     * @brief  Disabling the device.
     * @param  None.
     * @retval None.
     */
    virtual void disable(void)
    {
      L6208_Disable();
    }

    /**
     * @brief  Enabling the device.
     * @param  None.
     * @retval None.
     */
    virtual void enable(void)
    {
      L6208_Enable();
    }

    /**
     * @brief  Getting the motor decay mode.
     * @param  None.
     * @retval The motor decay mode.
     */
    virtual motorDecayMode_t get_decay_mode()
    {
      return L6208_get_decay_mode();
    }

    /**
     * @brief  Set the frequency of the VREFA and VREFB PWM
     * @param  frequency in Hz
     * @retval None.
     */
    virtual uint32_t get_freq_vref_pwm(void)
    {
      return L6208_VrefPwmGetFreq();
    }

    /**
     * @brief  Getting the version of the firmware.
     * @param  None.
     * @retval The version of the firmware.
     */
    virtual unsigned int get_fw_version(void)
    {
      return (unsigned int) L6208_GetFwVersion();
    }

    /**
     * @brief  Getting the motor step mode.
     * @param  None.L6208_Board_Disable
     * @retval The motor step mode.
     */
    virtual step_mode_t get_step_mode(void)
    {
      return (step_mode_t) L6208_GetStepMode();
    }

    /**
     * @brief  Getting the motor stop mode.
     * @param  None.
     * @retval The motor stop mode.
     */
    virtual motorStopMode_t get_stop_mode(void)
    {
      return L6208_GetStopMode();
    }

    /**
     * @brief  Going to a specified position with a specificied direction.
     * @param  direction The desired direction.
     * @param  position The desired position.
     * @retval None.
     */
    virtual void go_to(direction_t direction, signed int position)
    {
      L6208_GoToDir((motorDir_t)(direction == StepperMotor::FWD ? FORWARD : BACKWARD), (int32_t)position);
    }

    /**
     * @brief  Release the L6208 reset (Reset pin set to high level).
     * @param  None.
     * @retval None.
     */
    virtual void release_reset(void)
    {
      L6208_ReleaseReset();
    }

    /**
     * @brief  Reset the device with current step mode, resets current speed,
     * positions and microstep variables.
     * @param  None.
     * @retval None.
     */
    virtual void reset(void)
    {
      L6208_Reset();
    }

    /**
     * @brief  Reset the L6208 (Reset pin set to low level).
     * @param  None.
     * @retval None.
     */
    virtual void reset_device(void)
    {
      L6208_ResetDevice();
    }

    /**
     * @brief  Set the motor decay mode.
     * @param  decayMode The desired decay mode (SLOW_DECAY or FAST_DECAY).
     * @retval None.
     */
    virtual void set_decay_mode(motorDecayMode_t decayMode)
    {
      L6208_SetDecayMode(decayMode);
    }

    /**
     * @brief  Set the motor direction.
     * @param  direction The desired direction.
     * @retval None.
     */
    virtual void set_direction(direction_t direction)
    {
      L6208_SetDirection((motorDir_t)(direction == StepperMotor::FWD ? FORWARD : BACKWARD));
    }

    /**
     * @brief  Set the frequency of the VREFA and VREFB PWM
     * @param  frequency in Hz
     * @retval None.
     */
    virtual void set_freq_vref_pwm(uint32_t frequency)
    {
      L6208_VrefPwmSetFreq(frequency);
    }

    /**
     * @brief  Set the motor stop mode.
     * @param  stopMode The desired stop mode (HOLD_MODE or HIZ_MODE).
     * @retval None.
     */
    virtual void set_stop_mode(motorStopMode_t stopMode)
    {
      L6208_SetStopMode(stopMode);
    }

    /*** Public Interrupt Related Methods ***/
    /**
    * @brief  Attaching an interrupt handler to the FLAG interrupt.
    * @param  fptr An interrupt handler.
    * @retval None.
    */
    void attach_flag_irq(void (*fptr)(void))
    {
      int_cb = fptr;
    }
    void enable_flag_irq(void)
    {
      pinMode(flag_and_enable, INPUT);
      attachInterrupt(flag_and_enable, int_cb, FALLING);
    }
  protected:

    /*** Protected Component Related Methods ***/

    status_t L6208_Init(void *init);
    status_t L6208_ReadID(uint8_t *id);
    void L6208_AttachErrorHandler(void (*callback)(uint16_t error));
    void L6208_Disable(void);
    void L6208_ErrorHandler(uint16_t error);
    void L6208_Enable(void);
    uint16_t L6208_GetAcceleration(void);
    uint16_t L6208_GetCurrentSpeed(void);
    uint16_t L6208_GetDeceleration(void);
    motorDecayMode_t L6208_get_decay_mode(void);
    motorDir_t L6208_GetDirection(void);
    uint32_t L6208_GetFwVersion(void);
    int32_t L6208_GetMark(void);
    uint16_t L6208_GetMaxSpeed(void);
    uint16_t L6208_GetMinSpeed(void);
    motorState_t L6208_GetMotionState(void);
    int32_t L6208_GetPosition(void);
    motorStepMode_t L6208_GetStepMode(void);
    motorStopMode_t L6208_GetStopMode(void);
    void L6208_GoHome(void);
    void L6208_GoMark(void);
    void L6208_GoTo(int32_t targetPosition);
    void L6208_GoToDir(motorDir_t direction, int32_t targetPosition);
    void L6208_HardHiZ(void);
    void L6208_HardStop(void);
    void L6208_Move(motorDir_t direction, uint32_t stepCount);
    void L6208_ReleaseReset(void);
    void L6208_Reset(void);
    void L6208_ResetDevice(void);
    void L6208_Run(motorDir_t direction);
    bool L6208_SetAcceleration(uint16_t newAcc);
    void L6208_SetDecayMode(motorDecayMode_t decayMode);
    bool L6208_SetDeceleration(uint16_t newDec);
    void L6208_SetDirection(motorDir_t direction);
    void L6208_SetHome(void);
    void L6208_SetMark(void);
    bool L6208_SetMaxSpeed(uint16_t volatile newSpeed);
    bool L6208_SetMinSpeed(uint16_t volatile newSpeed);
    bool L6208_SetStepMode(motorStepMode_t stepMode);
    void L6208_SetStopMode(motorStopMode_t stopMode);
    bool L6208_SoftStop(void);
    void L6208_TickHandler(void);
    uint32_t L6208_VrefPwmGetFreq(void);
    void L6208_VrefPwmSetFreq(uint32_t newFreq);
    void L6208_WaitWhileActive(void);

    /*** Functions intended to be used only internally ***/

    void L6208_ClearSysFlag(uint32_t mask);
    uint32_t L6208_ComputeNbAccOrDecSteps(uint16_t accOrDecRate);
    uint16_t L6208_ConvertAcceDecelRateValue(uint16_t newAccOrDecRate);
    void L6208_DoAccel(void);
    void L6208_DoDecel(void);
    void L6208_DoRun(void);
    uint8_t L6208_GetMicrostepSample2Scale(void);
    void L6208_Indexmodeinit(void);
    bool L6208_IsSysFlag(uint32_t mask);
    void L6208_ResetSteps(void);
    uint32_t L6208_ScaleWaveformSample(uint8_t sampleIndex);
    void L6208_ScaleWaveformTable(void);
    void L6208_SetDeviceParamsToGivenValues(l6208_init_t *pInitDevicePrm);
    void L6208_SetDeviceParamsToPredefinedValues(void);
    void L6208_SetMicrostepSample2Scale(uint8_t value);
    void L6208_SetMicrostepSample2Update(uint8_t value);
    void L6208_SetMotionState(motorState_t newMotionState);
    bool L6208_SetSpeed(uint16_t newSpeed, uint32_t volatile *pSpeed);
    void L6208_SetSysFlag(uint32_t mask);
    bool L6208_StartMovement(void);
    void L6208_UpdateScanWaveformTable(void);
    void L6208_UstepWaveformHandling(void);
    bool L6208_VectorCalc(uint8_t newTorque);
    bool L6208_VrefPwmComputePulseWidth(uint8_t bridgeId, uint16_t value, bool valueIsPwmDutyCycle);
    void L6208_VrefPwmUpdatePulseWidth(void);

    /*** Component's I/O Methods ***/
    /**
    * @brief  Reset the clock pin.
    * @param  None.
    * @retval None.
    */
    void L6208_Board_CLOCK_PIN_Reset(void)
    {
      digitalWrite(clock_pin, 0) ;
    }
    /**
     * @brief  Set the clock pin.
     * @param  None.
     * @retval None.
     */
    void L6208_Board_CLOCK_PIN_Set(void)
    {
      digitalWrite(clock_pin, 1);

    }

    /**
     * @brief  Set the control pin.
     * @param  None.
     * @retval None.
     */
    void L6208_Board_CONTROL_PIN_Set(void)
    {
      digitalWrite(control_pin, 1);
    }

    /**
     * @brief  Reset the control pin.
     * @param  None.
     * @retval None.
     */
    void L6208_Board_CONTROL_PIN_Reset(void)
    {
      digitalWrite(control_pin, 0);
    }


    /**
     * @brief  Making the CPU wait.
     * @param  None.
     * @retval None.
     */
    void L6208_Board_Delay(uint32_t ms_delay)
    {
      delay(ms_delay);
    }
    /**
    * @brief  Reset the dir pin.
    * @param  None.
    * @retval None.
    */
    void L6208_Board_DIR_PIN_Reset(void)
    {
      digitalWrite(direction_pin, 0);
    }

    /**
     * @brief  Set the dir pin.
     * @param  None.
     * @retval None.
     */
    void L6208_Board_DIR_PIN_Set(void)
    {
      digitalWrite(direction_pin, 1);
    }

    /**
     * @brief  Disable the power bridges (leave the output bridges HiZ).
     * @param  None.
     * @retval None.
     */
    void L6208_Board_Disable(void)
    {
      detachInterrupt(flag_and_enable);
      pinMode(flag_and_enable, OUTPUT);
      digitalWrite(flag_and_enable, LOW);


    }

    /**
     * @brief  Enable the power bridges (leave the output bridges HiZ).
     * @param  None.
     * @retval None.
     */
    void L6208_Board_Enable(void)
    {
      pinMode(flag_and_enable, OUTPUT);
      digitalWrite(flag_and_enable, HIGH);
      pinMode(flag_and_enable, INPUT_PULLUP);
      attachInterrupt(flag_and_enable, int_cb, FALLING);

    }

    /**
    * @brief  Reset the half full pin.
    * @param  None.
    * @retval None.
    */
    void L6208_Board_HALF_FULL_PIN_Reset(void)
    {
      digitalWrite(half_full_pin, 0);
    }

    /**
     * @brief  Set the half full pin.
     * @param  None.
     * @retval None.
     */
    void L6208_Board_HALF_FULL_PIN_Set(void)
    {
      digitalWrite(half_full_pin, 1);
    }

    /**
     * @brief  Initialising the the VREFA or VREFB PWM.
     * @param  None.
     * @retval None.
     */
    void L6208_Board_VrefPwmInit(uint8_t bridgeId, uint32_t pwmFreq) {}

    /**
     * @brief  Exit the device from reset mode.
     * @param  None.
     * @retval None.
     */
    void L6208_Board_ReleaseReset(void)
    {
      digitalWrite(reset_pin, 1);
    }

    /**
     * @brief  Put the device in reset mode.
     * @param  None.
     * @retval None.
     */
    void L6208_Board_Reset(void)
    {
      digitalWrite(reset_pin, 0);
    }

    /**
     * @brief  Get the tick timer frequency in Hz.
     * @param  None.
     * @retval The tick timer frequency in Hz.
     */
    uint32_t L6208_Board_TickGetFreq(void)
    {
      return TIMER_TICK_FREQUENCY;
    }



    /**
     * @brief  Initialising the tick.
     * @param  None.
     * @retval None.
     */
    void L6208_Board_TickInit(void) {}

    /**
     * @brief  Starting the tick timer, setting its frequency
     * and attaching a tick handler function to it.
     * @param  frequency the frequency of the tick.
     * @retval None.
     */
    void L6208_Board_TickStart(uint16_t frequency)
    {
      /* Attaching a tick handler function which updates */
      /* the state machine every elapsed period time. */
      ticker->setOverflow(frequency, HERTZ_FORMAT);
      ticker->attachInterrupt(callback_handler);
      ticker->resume();
    }

    /**
     * @brief  Stopping the tick.
     * @param  None.
     * @retval None.
     */
    void L6208_Board_TickStop(void)
    {

      ticker->pause();
      ticker->detachInterrupt();
    }


    /**
     * @brief Set the pulse width of the VREFA PWM.
     * @param[in] pulseWidthInUs pulse width of the PWM in microsecond.
     * @retval None.
     */
    void L6208_Board_VrefPwmSetPulseWidthA(uint8_t pulseWidthInUs)
    {
      if (current_pwm_pulseWidthA != pulseWidthInUs) {
        current_pwm_pulseWidthA = pulseWidthInUs;
        vrefA_pwm_timer->setCaptureCompare(vrefA_pwm_channel, current_pwm_pulseWidthA, MICROSEC_COMPARE_FORMAT);
      }
      vrefA_pwm_timer->refresh();
    }

    /**
     * @brief Set the pulse width of the VREFB PWM.
     * @param[in] pulseWidthInUs pulse width of the PWM in microsecond.
     * @retval None.
     */
    void L6208_Board_VrefPwmSetPulseWidthB(uint8_t pulseWidthInUs)
    {
      if (current_pwm_pulseWidthB != pulseWidthInUs) {
        current_pwm_pulseWidthB = pulseWidthInUs;
        vrefB_pwm_timer->setCaptureCompare(vrefB_pwm_channel, current_pwm_pulseWidthB, MICROSEC_COMPARE_FORMAT);
      }
      vrefB_pwm_timer->refresh();

    }



    /**
     * @brief  Start the timer for the VREFA or VREFB PWM.
     * @param[in] bridgeId
     *            0 for BRIDGE_A
     *            1 for BRIDGE_B
     * @param[in] pwmPeriod period of the PWM used to generate the reference
     * voltage for the bridge.
     * @retval "true" in case of success, "false" otherwise.
     * @note the unit is 1/256th of a microsecond. The VREFA PWM must be started
     * before the VREFB PWM.
     */
    bool L6208_Board_VrefPwmStart(uint8_t bridgeId, uint16_t pwmPeriod)
    {

      pwmPeriod >>= 8;
      /* Setting the period and the duty-cycle of PWM. */
      if (bridgeId == 0) {

        if (current_pwm_periodA != pwmPeriod) {
          current_pwm_periodA = pwmPeriod;
          vrefA_pwm_timer ->setOverflow(current_pwm_periodA, MICROSEC_FORMAT);
        }
        vrefA_pwm_timer->resume();
      } else if (bridgeId == 1) {
        if (current_pwm_periodB != pwmPeriod) {
          current_pwm_periodB = pwmPeriod;
          vrefB_pwm_timer ->setOverflow(current_pwm_periodB, MICROSEC_FORMAT);
        }
        vrefB_pwm_timer->resume();

      } else {
        return false;
      }
      return true;
    }

    /**
     * @brief  Stop the timer for the VREFA or VREFB PWM.
     * @param[in] bridgeId
     *            0 for BRIDGE_A
     *            1 for BRIDGE_B
     * @retval "true" in case of success, "false" otherwise.
     */
    bool L6208_Board_VrefPwmStop(uint8_t bridgeId)
    {
      if (bridgeId == 0) {
        HAL_TIM_PWM_Stop(vrefA_pwm_timer->getHandle(), vrefA_pwm_channel);

      } else if (bridgeId == 1) {
        HAL_TIM_PWM_Stop(vrefB_pwm_timer->getHandle(), vrefB_pwm_channel);
      } else {
        return false;
      }

      return true;
    }

  protected:


    /*** Component's Instance Variables ***/
    /* Flag Interrupt. */
    uint8_t flag_and_enable;
    void (*int_cb)(void);



    /* RESET pin. */
    uint8_t reset_pin;
    /* CW_CCW pin. */
    uint8_t direction_pin;
    /* HALF_FULL pin */
    uint8_t  half_full_pin;
    /* CONTROL pin */
    uint8_t  control_pin;
    /* CLOCK pin */
    uint8_t clock_pin;

    /* Pulse Width Modulation pin for VREFA pin */
    HardwareTimer *vrefA_pwm_timer;
    uint32_t vrefA_pwm_channel;
    uint8_t vrefA_pwm;
    TIM_TypeDef *vrefA_pwm_instance;
    TIM_TypeDef *vrefB_pwm_instance;
    /* Pulse Width Modulation pin for VREFA pin */
    HardwareTimer *vrefB_pwm_timer;
    uint32_t vrefB_pwm_channel;
    uint8_t vrefB_pwm;


    HardwareTimer *ticker;
    TickHandler_Callback callback_handler;

    /* Identity */
    uint8_t who_am_i;

    /* Data. */
    void (*errorHandlerCallback)(uint16_t error);
    deviceParams_t device_prm;
    uint8_t deviceInstance;
    uint32_t tickFreq;
    /// microstepping PWM period and torque scaled waveform samples array
    uint16_t updatedMicroTable[L6208_USTEPS_PER_QUARTER_PERIOD + 1];
    /// waveform scanning microstepping PWM period sample arrays for VREFA wave
    uint16_t microTable1[L6208_USTEPS_PER_QUARTER_PERIOD * 3 + 1];
    /// waveform scanning microstepping PWM period sample array for VREFB wave
    uint16_t *pMicroTable2;
    uint16_t current_pwm_periodA ;
    uint16_t current_pwm_periodB ;
    uint8_t current_pwm_pulseWidthA;
    uint8_t current_pwm_pulseWidthB;

    /* Static data. */
    static uint8_t numberOfDevices;
    static const uint16_t RefMicroTable[L6208_USTEPS_PER_QUARTER_PERIOD * 3];

    volatile uint8_t current_state_ticker;
};

#endif // __L6208_CLASS_H

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

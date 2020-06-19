 
/**
  ******************************************************************************
  * @file    L6208.cpp
  * @author  IPC Rennes
  * @version V1.1.0
  * @date    February 11th, 2016
  * @brief   L6208 product related routines
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

/* Includes ------------------------------------------------------------------*/
#include "L6208.h"


/* Definitions ---------------------------------------------------------------*/
/// Bridge A
#define BRIDGE_A                     (0)
/// Bridge B
#define BRIDGE_B                     (1)

/// Bitmaps for system flags
#define EN_A_set              0x00000001    ///< EN_A pin status
#define HiZstop               0x00000002    ///< motor has to be left in HiZ after stopping
#define busy                  0x00000004    ///< stepper position command executing flag
#define running               0x00000008    ///< running motor flag
#define velocitymode          0x00000010    ///< velocity controlled stepper motor
#define positionmode          0x00000020    ///< position controlled stepper motor
#define fullstep              0x00000040    ///< full step mode controlled    
#define halfstep              0x00000080    ///< half step mode controlled    
#define microstep             0x00000100    ///< micro step mode controlled
#define forward               0x00000200    ///< forward running motor
#define dir2change            0x00000400    ///< direction has to be changed while the motor is running
#define fastdecaymode         0x00000800    ///< decay mode is fast
#define wavestep              0x00001000    ///< wave step mode controlled

/* Variables  ----------------------------------------------------------------*/
/* Number of devices. */
uint8_t L6208::numberOfDevices = 0;

/// RefMicroTable values are 2^L6208_SINE_WAVEFORM_POWER_OF_TWO_MAX_VALUE*|sin(n/16*PI/2)|
/// where n is the index in the table
const uint16_t L6208::RefMicroTable[L6208_USTEPS_PER_QUARTER_PERIOD*3] =
{
  0,3212,6393,9512,12540,15447,18205,20788,23170,25330,27246,28899,30274,31357,32138,32610,
  32768,32610,32138,31357,30274,28899,27246,25330,23170,20788,18205,15447,12540,9512,6393,3212,
  0,3212,6393,9512,12540,15447,18205,20788,23170,25330,27246,28899,30274,31357,32138,32610
};

/* Methods -------------------------------------------------------------------*/
/******************************************************//**
 * @brief Start the L6208 library
 * @param[in] pInit pointer to the initialization data
 * @retval COMPONENT_OK in case of success.
 **********************************************************/
status_t L6208::L6208_Init(void* pInit)
{ 
  if (pInit == NULL)
  {
    /* Set context variables to the predefined values from l6208_target_config.h */
    /* Set GPIO according to these values */
    L6208_SetDeviceParamsToPredefinedValues();
  }
  else
  {
    L6208_SetDeviceParamsToGivenValues((l6208_init_t*) pInit);
  }
  
  /* Initialise the PWMs */
  L6208_Board_VrefPwmInit(BRIDGE_A, device_prm.vrefPwmPeriod);
  L6208_Board_VrefPwmInit(BRIDGE_B, device_prm.vrefPwmPeriod);

  /* Initialise the tick */
  L6208_Board_TickInit();

  /* Reset L6208 */
  L6208_ResetDevice();
  
  /* Align motor mechanical position to driver position */
  L6208_Board_VrefPwmStart(BRIDGE_A, device_prm.vrefPwmPeriod);
  L6208_Board_VrefPwmStart(BRIDGE_B, device_prm.vrefPwmPeriod);
  //L6208_Enable(); /* anche nel file mbed è commentato*/
  
  return COMPONENT_OK;
}

/**********************************************************
 * @brief Read id
 * @param id pointer to the identifier to be read.
 * @retval COMPONENT_OK in case of success.
 **********************************************************/
status_t L6208::L6208_ReadID(uint8_t *id)
{
  *id = deviceInstance;

  return COMPONENT_OK;
}

/**********************************************************
 * @brief  Attaches a user callback to the error Handler.
 * The call back will be then called each time the library 
 * detects an error
 * @param[in] callback Name of the callback to attach 
 * to the error Hanlder
 * @retval None
 **********************************************************/
void L6208::L6208_AttachErrorHandler(void (*callback)(uint16_t error))
{
  errorHandlerCallback = (void (*)(uint16_t error)) callback;
}
/******************************************************//**
 * @brief Disable the power bridges (leave the output bridges HiZ)
 * @retval None
 **********************************************************/
void L6208::L6208_Disable(void)
{
  L6208_Board_Disable();
  L6208_ClearSysFlag(EN_A_set);
}
/******************************************************//**
 * @brief Error handler which calls the user callback (if defined)
 * @param[in] error Number of the error
 * @retval None
 **********************************************************/
void L6208::L6208_ErrorHandler(uint16_t error)
{
  if (errorHandlerCallback != 0)
  {
    errorHandlerCallback(error);
  }
  else   
  {
    while(1)
    {
      /* Infinite loop */
    }
  }
}
/******************************************************//**
 * @brief Enable the power bridges
 * @retval None
 **********************************************************/
void L6208::L6208_Enable(void)
{
    Serial.println("sono in L6208_Enable");
  L6208_Board_Enable();
  L6208_SetSysFlag(EN_A_set);
  Serial.println("fuori da L6208_Enable");
}

/******************************************************//**
 * @brief Get the stepper acceleration rate
 * in step/s^2 for full, half and wave modes
 * in microsteps/s^2 for microstep modes
 * @retval the stepper acceleration rate in step/s^2 or microstep/s^2
 * @note
 **********************************************************/
uint16_t L6208::L6208_GetAcceleration(void)
{
    return device_prm.accelerationSps2;
}
/******************************************************//**
 * @brief Get the current speed
 * in step/s for full, half and wave modes
 * in microsteps/s for microstep modes
 * @retval return the current speed in step/s or microstep/s
 * @note
 **********************************************************/
uint16_t L6208::L6208_GetCurrentSpeed(void)
{
  uint64_t tmp64 = (uint64_t) device_prm.speedSpt * L6208_Board_TickGetFreq();
  
  device_prm.speedSps = (uint16_t)(tmp64 >> 23);
  if (device_prm.speedSps & 0x1)
  {
    device_prm.speedSps = (device_prm.speedSps >> 1) + 1;
  }
  else
  {
    device_prm.speedSps = device_prm.speedSps >> 1;
  }
  return device_prm.speedSps;
}
/******************************************************//**
 * @brief Get the motor decay mode
 * @retval decay mode
 **********************************************************/
motorDecayMode_t L6208::L6208_get_decay_mode(void)
{
  if (L6208_IsSysFlag(fastdecaymode))
  {
      return (FAST_DECAY);
  }
  else
  {
      return (SLOW_DECAY);
  }
}

/******************************************************//**
 * @brief Get the stepper deceleration rate
 * in step/s^2 for full, half and wave modes
 * in microsteps/s^2 for microstep modes
 * @retval the stepper deceleration rate in step/s^2 or microstep/s^2
 * @note
 **********************************************************/
uint16_t L6208::L6208_GetDeceleration(void)
{
    return device_prm.decelerationSps2;
}

/******************************************************//**
 * @brief Get the motor current direction
 * @retval direction
 **********************************************************/
motorDir_t L6208::L6208_GetDirection(void)
{
  if (L6208_IsSysFlag(forward))
  {
    return FORWARD;
  }
  else
  {
    return BACKWARD;
  }
}
/******************************************************//**
 * @brief Return the FW version.
 * @retval FW version
 **********************************************************/
uint32_t L6208::L6208_GetFwVersion(void)
{
  return L6208_FW_VERSION;
}

/******************************************************//**
 * @brief Get the mark position (32b signed) 
 * @retval mark position
 **********************************************************/
int32_t L6208::L6208_GetMark(void)
{
  return device_prm.markPos;
}

/******************************************************//**
 * @brief Get the max speed
 * in step/s for full, half and wave modes
 * in microsteps/s for microstep modes
 * @retval return the max speed in step/s or microstep/s
 * @note
 **********************************************************/
uint16_t L6208::L6208_GetMaxSpeed(void)
{
  return device_prm.maxSpeedSps;
}

/******************************************************//**
 * @brief Get the min speed
 * in step/s for full, half and wave modes
 * in microsteps/s for microstep modes
 * @retval return the min speed in step/s or microstep/s
 * @note
 **********************************************************/
uint16_t L6208::L6208_GetMinSpeed(void)
{
  return device_prm.minSpeedSps;
}

/******************************************************//**
 * @brief Get the stepper state machine index
 * @retval one of the stepper state machine index in the motorState_t enum
 **********************************************************/
motorState_t L6208::L6208_GetMotionState(void)
{
  // gets the new stepper state machine index
  return device_prm.motionState;    
}

/******************************************************//**
 * @brief Get the current position (32b signed) 
 * @retval current absoulte position
 **********************************************************/
int32_t L6208::L6208_GetPosition(void)
{
  return device_prm.absolutePos;
}

/******************************************************//**
 * @brief Get the motor step mode
 * @retval step mode
 **********************************************************/
motorStepMode_t L6208::L6208_GetStepMode(void)
{
  return device_prm.stepMode;
}

/******************************************************//**
 * @brief Get the selected stop mode
 * @retval the selected stop mode
 **********************************************************/
motorStopMode_t L6208::L6208_GetStopMode(void)
{
  if (L6208_IsSysFlag(HiZstop) == FALSE)
  {
      return (HOLD_MODE);
  }
  else
  {
      return (HIZ_MODE);
  }
}

/******************************************************//**
 * @brief Go to the home position
 * @retval None
 **********************************************************/
void L6208::L6208_GoHome(void)
{
  L6208_GoTo(0);
}

/******************************************************//**
 * @brief Go to the Mark position
 * @retval None
 **********************************************************/
void L6208::L6208_GoMark(void)
{
  L6208_GoTo(device_prm.markPos);
}
/******************************************************//**
 * @brief move the motor to the absolute position using the shortest path
 * @param[in] abs_pos 32 bit signed value position
 * @retval None
 * @note The position is at the resolution corresponding to the
 * selected step mode.
 * STEP_MODE_FULL or STEP_MODE_WAVE : step
 * STEP_MODE_HALF                     : 1/2 step
 * STEP_MODE_1_4              : 1/4 step
 * STEP_MODE_1_8              : 1/8 step
 * STEP_MODE_1_16             : 1/16 step 
 **********************************************************/
void L6208::L6208_GoTo(int32_t abs_pos)
{
  uint32_t steps = 0;
  
  if(L6208_IsSysFlag(running))
  {
    L6208_HardStop();
  }
  
  if (abs_pos > device_prm.absolutePos)
  {
    steps = abs_pos - device_prm.absolutePos;
    if (steps < (L6208_POSITION_RANGE>>1))
    {
      L6208_Move(FORWARD, steps);
    }
    else
    {
      L6208_Move(BACKWARD, (L6208_POSITION_RANGE - steps));
    }
  }
  else
  {
    steps = device_prm.absolutePos - abs_pos;
    if (steps < (L6208_POSITION_RANGE>>1))
    {
      L6208_Move(BACKWARD, steps);
    }
    else
    {
      L6208_Move(FORWARD, (L6208_POSITION_RANGE - steps));
    }
  }
}
/******************************************************//**
 * @brief move the motor to the absolute position
 * @param[in] direction FORWARD or BACKWARD
 * @param[in] abs_pos 32 bit signed value position
 * @retval None
 * @note The position is at the resolution corresponding to the
 * selected step mode.
 * STEP_MODE_FULL or STEP_MODE_WAVE : step
 * STEP_MODE_HALF                     : 1/2 step
 * STEP_MODE_1_4              : 1/4 step
 * STEP_MODE_1_8              : 1/8 step
 * STEP_MODE_1_16             : 1/16 step 
 **********************************************************/
void L6208::L6208_GoToDir(motorDir_t direction, int32_t abs_pos)
{
  uint32_t steps = 0;
  
  if(L6208_IsSysFlag(running))
  {
    L6208_HardStop();
  }
  
  if (direction != BACKWARD)
  {
    if (abs_pos > device_prm.absolutePos)
    {
      steps = abs_pos - device_prm.absolutePos;
    }
    else
    {
      steps = L6208_POSITION_RANGE + (abs_pos - device_prm.absolutePos);
    }
  }
  else
  {
    if (abs_pos > device_prm.absolutePos)
    {
      steps = L6208_POSITION_RANGE + (device_prm.absolutePos - abs_pos);
    }
    else
    {
      steps = device_prm.absolutePos - abs_pos;
    }
  }
  L6208_Move(direction, steps);
}

/******************************************************//**
 * @brief Immediately stop the motor and disables the power bridges
 * @retval None
 **********************************************************/
void L6208::L6208_HardHiZ(void)
{
  /* Disables power stage */
  L6208_Disable();
  
  /* Sets inactive state */
  L6208_SetMotionState(INACTIVE);
  
  /* Clears the running motor and the position */
  L6208_ClearSysFlag(running);
 
  /* Disables PWMs */
  L6208_Board_VrefPwmStop(BRIDGE_A);
  L6208_Board_VrefPwmStop(BRIDGE_B);
  
  /* Disables tick timer */
  L6208_Board_TickStop();

   
}

/******************************************************//**
 * @brief Immediately stop the motor and keeps holding torque
 * @retval None
 **********************************************************/
void L6208::L6208_HardStop(void) 
{
  /* Sets inactive state */
  L6208_SetMotionState(INACTIVE);
  
  /* Clears the running motor and the position */
  L6208_ClearSysFlag(running);
  L6208_VectorCalc(device_prm.holdTorque);
    
  /* Disables tick timer */
  L6208_Board_TickStop();
}

/******************************************************//**
 * @brief move the motor by the specified number of steps
 * in the specified direction
 * @param[in] direction FORWARD or BACKWARD
 * @param[in] stepCount 32 bit unsigned step count
 * @retval None
 * @note The step count resolution is corresponding to the
 * selected step mode.
 * STEP_MODE_FULL or STEP_MODE_WAVE : step
 * STEP_MODE_HALF                   : 1/2 step
 * STEP_MODE_1_4                    : 1/4 step
 * STEP_MODE_1_8                    : 1/8 step
 * STEP_MODE_1_16                   : 1/16 step 
 **********************************************************/
void L6208::L6208_Move(motorDir_t direction, uint32_t stepCount)
{
  if(L6208_IsSysFlag(running))
  {
    L6208_HardStop();
  }

  /* clear the velocity driving mode flag */
  L6208_ClearSysFlag(velocitymode);
  
  /* Set the indexing driving mode flag */
  /* and the user command executing flag */
  L6208_SetSysFlag(positionmode);
  
  /* store relative number of steps to move */
  device_prm.positionTarget = stepCount;

  L6208_SetDirection(direction); 
  
  /* Motor activation */
  L6208_StartMovement(); 
  Serial.println("fine start");
}

/******************************************************//**
 * @brief  Release the L6208 reset (Reset pin set to high level)
 * @retval None
 **********************************************************/
void L6208::L6208_ReleaseReset(void)
{ 
  L6208_Board_ReleaseReset(); 
}

/******************************************************//**
 * @brief Reset the L6208 (Reset pin set to low level)
 * @retval None
 **********************************************************/
void L6208::L6208_Reset(void)
{
  L6208_Board_Reset();
}


/******************************************************//**
 * @brief Call L6208_SetStepMode with current step mode, 
 * the L6208_SetStepMode function along with setting the step mode resets
 * the L6208 device
 * @retval None
 **********************************************************/
void L6208::L6208_ResetDevice(void)
{
  L6208_SetStepMode(L6208_GetStepMode());
}

/******************************************************//**
 * @brief run the motor in the specified direction
 * according to the speed profile defined by the minimum speed,
 * maximum speed, and acceleration parameters. 
 * The device accelerates from the minimum speed up to the maximum
 * speed by using the device acceleration.
 * @param[in] direction FORWARD or BACKWARD
 * @retval None
 **********************************************************/
void L6208::L6208_Run(motorDir_t direction)
{
  if(L6208_IsSysFlag(running))
  {
    L6208_HardStop();
  }
  L6208_SetDirection(direction);
  /* Clear the indexing driving mode flag */
  L6208_ClearSysFlag(positionmode);
  /* Set the velocity driving mode flag */
  L6208_SetSysFlag(velocitymode);
  /* Motor activation */
  L6208_StartMovement(); 
}

/******************************************************//**
 * @brief Set the stepper acceleration rate
 * in step/s^2 and step/tick^2 for full, half and wave modes
 * in microsteps/s^2 and microsteps/tick^2 for microstep modes
 * @param[in] newAcc new acceleration rate in step/s^2 or microstep/s^2
 * @retval TRUE
 * @note
 **********************************************************/
bool L6208::L6208_SetAcceleration(uint16_t newAcc)
{
  uint16_t newAccSpt2 = L6208_ConvertAcceDecelRateValue(newAcc);
  if (newAccSpt2)
  {
    device_prm.accelerationSps2 = newAcc;
    device_prm.accelerationSpt2 = newAccSpt2; 
  }
  else
  {
    L6208_ErrorHandler(L6208_ERROR_SET_ACCELERATION);
  }
  return TRUE;
}

/******************************************************//**
 * @brief Select the motor decay mode
 * @param[in] decayMode (SLOW_DECAY or FAST_DECAY)
 * @retval None
 **********************************************************/
void L6208::L6208_SetDecayMode(motorDecayMode_t decayMode)
{
  if ((decayMode & L6208_FAST_DECAY_MODE_MASK) == L6208_FAST_DECAY_MODE_MASK)
  {
    L6208_Board_CONTROL_PIN_Set();
    L6208_SetSysFlag(fastdecaymode);
  }
  else 
  {
    L6208_Board_CONTROL_PIN_Reset();
    L6208_ClearSysFlag(fastdecaymode);
  }
}

/******************************************************//**
 * @brief Set the stepper deceleration rate
 * in step/s^2 and step/tick^2 for full, half and wave modes
 * in microsteps/s^2 and microsteps/tick^2 for microstep modes
 * @param[in] newDec new deceleration rate in step/s^2 or microstep/s^2
 * @retval TRUE
 * @note
 **********************************************************/
bool L6208::L6208_SetDeceleration(uint16_t newDec)
{
  uint16_t newDecSpt2 = L6208_ConvertAcceDecelRateValue(newDec);
  if (newDecSpt2)
  {
    device_prm.decelerationSps2 = newDec;
    device_prm.decelerationSpt2 = newDecSpt2;
  }
  else
  {
    L6208_ErrorHandler(L6208_ERROR_SET_DECELERATION);
  }
  return TRUE;
}

/******************************************************//**
 * @brief Specify the direction
 * @param[in] dir FORWARD or BACKWARD
 * @note In velocity mode a direction change forces the device to stop and 
 * then run in the new direction. In position mode, if the device is 
 * running, a direction change will generate an error.
 * @retval None
 **********************************************************/
void L6208::L6208_SetDirection(motorDir_t dir)
{
  L6208_ClearSysFlag(dir2change);
  if (dir == FORWARD)
  {
    if (!L6208_IsSysFlag(forward))
    {
      if (L6208_IsSysFlag(running))
      {
        /* motor is running */
        if (L6208_IsSysFlag(positionmode))
        {
          L6208_ErrorHandler(L6208_ERROR_SET_DIRECTION);
        }
        else
        {
          /* set the rotation direction to change flag */
          L6208_SetSysFlag(dir2change);
        }
      }
      else /* the motor is stopped, cw direction selected */
      {
        L6208_SetSysFlag(forward);
        L6208_Board_DIR_PIN_Set();
      }
    }
  }
  else
  {
    if (L6208_IsSysFlag(forward))
   {
      if (L6208_IsSysFlag(running))
      {
        /* motor is running */
        if (L6208_IsSysFlag(positionmode))
        {
          L6208_ErrorHandler(L6208_ERROR_SET_DIRECTION);
        }
        else
        {
          /* set the rotation direction to change flag */
          L6208_SetSysFlag(dir2change);
        }
      }
      else /* the motor is stopped, ccw direction selected */
      {
        L6208_ClearSysFlag(forward);
        L6208_Board_DIR_PIN_Reset();
      }
    }
  }
  if(L6208_IsSysFlag(dir2change))
  {
    L6208_VectorCalc(device_prm.decelTorque);     
    L6208_SetMotionState(DECELERATINGTOSTOP);  
  }
}
/******************************************************//**
 * @brief Set current position to be the home position
 * @retval None
 **********************************************************/
void L6208::L6208_SetHome(void)
{
  if (!L6208_IsSysFlag(running))
  {
      device_prm.absolutePos = 0;
  }
  else
  {
      L6208_ErrorHandler(L6208_ERROR_SET_HOME);
  }
}

/******************************************************//**
 * @brief Set current position to be the mark position 
 * @retval None
 **********************************************************/
void L6208::L6208_SetMark(void)
{
  device_prm.markPos = device_prm.absolutePos;
}

/******************************************************//**
 * @brief Set the user selected maximum speed 
 * in step/s and step/tick for full, half and wave modes
 * in microsteps/s and microsteps/tick for microstep modes
 * @param[in] newSpeed speed value (step/s or microstep/s)
 * @retval TRUE
 * @note One microstep is 1/16 step
 **********************************************************/
bool L6208::L6208_SetMaxSpeed(uint16_t newSpeed)
{
  if (L6208_SetSpeed(newSpeed, &device_prm.maxSpeedSpt))
  {
    device_prm.maxSpeedSps = newSpeed;
  }
  else
  {
    L6208_ErrorHandler(L6208_ERROR_SET_MAX_SPEED);
  }
  return TRUE;
}

/******************************************************//**
 * @brief Set the user selected minimum speed 
 * in step/s and step/tick for full, half and wave modes
 * in microsteps/s and microsteps/tick for microstep modes
 * @param[in] newSpeed speed value (step/s or microstep/s)
 * @retval TRUE
 * @note One microstep is 1/16 step
 **********************************************************/
bool L6208::L6208_SetMinSpeed(uint16_t newSpeed)
{
  if (L6208_SetSpeed(newSpeed, &device_prm.minSpeedSpt))
  {
    device_prm.minSpeedSps = newSpeed;
  }
  else
  {
    L6208_ErrorHandler(L6208_ERROR_SET_MIN_SPEED);
  }
  return TRUE;
}

/******************************************************//**
 * @brief Set the step mode
 * @param[in] stepMode
 * @retval true if the command is successfully executed, else false
 * @note Every time the step mode is changed, the step state machine is reset
 **********************************************************/
bool L6208::L6208_SetStepMode(motorStepMode_t stepMode)
{
  device_prm.stepMode = stepMode;
  L6208_ClearSysFlag(fullstep | halfstep | microstep | wavestep);
  switch (stepMode)
  {
    case STEP_MODE_HALF:
      /* Set the Half/Full pin low and Reset and the set the Half/Full pin high*/
      L6208_Board_HALF_FULL_PIN_Reset(); 
      L6208_Board_Reset();
      L6208_Board_HALF_FULL_PIN_Set();
      /* Set system flag */
      L6208_SetSysFlag(halfstep);
      break;
    case STEP_MODE_FULL:
       /* Set the Half/Full pin low and Reset */
      L6208_Board_HALF_FULL_PIN_Reset(); 
      L6208_Board_Reset();
      /* Set system flag */
      L6208_SetSysFlag(fullstep);
      break;
    case STEP_MODE_WAVE:
      /* Set the Half/Full pin low and Reset and the set the Half/Full pin high*/
      L6208_Board_CLOCK_PIN_Reset();
      L6208_Board_HALF_FULL_PIN_Reset();
      L6208_Board_Reset();
      L6208_Board_CLOCK_PIN_Set();
      L6208_Board_HALF_FULL_PIN_Set();
      L6208_Board_Delay(2);
      L6208_Board_CLOCK_PIN_Reset();
      L6208_Board_Delay(2);
      L6208_Board_HALF_FULL_PIN_Reset();
      /* Set system flag */
      L6208_SetSysFlag(wavestep);
      break;
      case STEP_MODE_1_4:
      /* Set the Half/Full pin low and Reset */
      L6208_Board_HALF_FULL_PIN_Reset();
      L6208_Board_Reset();
      /* Set system flag */
      L6208_SetSysFlag(microstep);
      device_prm.uStepInc = 4;
      break;
    case STEP_MODE_1_8:
      /* Set the Half/Full pin low and Reset */
      L6208_Board_HALF_FULL_PIN_Reset();
      L6208_Board_Reset();
      /* Set system flag */
      L6208_SetSysFlag(microstep);
      device_prm.uStepInc = 2;
      break;
      case STEP_MODE_1_16:
      /* Set the Half/Full pin low and Reset */
      L6208_Board_HALF_FULL_PIN_Reset();
      L6208_Board_Reset();
      /* Set system flag */
      L6208_SetSysFlag(microstep);
      device_prm.uStepInc = 1;
      break;
    default:
      return FALSE;
    }
  L6208_Board_Delay(2);
  L6208_Board_ReleaseReset();
  L6208_ResetSteps();
  return TRUE;
}

/******************************************************//**
 * @brief Select the mode to stop the motor. When the motor
 * is stopped, if autoHiZ is TRUE, the power bridges are disabled
 * if autoHiZ is FALSE, the power bridges are kept enabled.
 * @param[in] stopMode HOLD_MODE to let power bridge enabled
 * @retval None
 **********************************************************/
void L6208::L6208_SetStopMode(motorStopMode_t stopMode)
{
  if (stopMode == HOLD_MODE)
  {
    L6208_ClearSysFlag(HiZstop);
  }
  else
  {
    L6208_SetSysFlag(HiZstop);
  }
}

/******************************************************//**
 * @brief  Stop the motor by using the device deceleration and set deceleration torque
 * @retval true if the command is successfully executed, else false
 * @note .
 **********************************************************/
bool L6208::L6208_SoftStop(void)
{   
  L6208_VectorCalc(device_prm.decelTorque);
  L6208_SetMotionState(DECELERATINGTOSTOP);
  return TRUE;
}

/******************************************************//**
 * @brief  Handle the device state machine at each tick timer pulse end.
 * @retval None
 **********************************************************/
void L6208::L6208_TickHandler(void)
{
   
  uint32_t locMaxSpeedSpt = device_prm.maxSpeedSpt;
  uint32_t locMinSpeedSpt = device_prm.minSpeedSpt;
  
  /* Update state, target speed, acceleration and deceleration rates */
  L6208_Board_CLOCK_PIN_Reset();
    
  switch(L6208_GetMotionState())
  {
    /* ============ Velocity control mode states ======================== */
    case ACCELERATING:
      /* velocity mode: acceleration phase */
      /* Increase Speed and update position */
      L6208_DoAccel();
      if(locMaxSpeedSpt < device_prm.speedSpt)
      {  
        /*Target speed reached */
        device_prm.speedSpt = locMaxSpeedSpt;
        L6208_VectorCalc(device_prm.runTorque); 
        L6208_SetMotionState(STEADY);
      }
      break;
    case STEADY:  
      /* velocity mode: constant speed phase */
      /* Update position */
      L6208_DoRun();  
      if(locMaxSpeedSpt != device_prm.speedSpt)
      { 
        /* targeted speed  has changed */
        if(locMaxSpeedSpt< device_prm.speedSpt)
        { 
          /* Slow down the motor */
          L6208_VectorCalc(device_prm.decelTorque);     
          L6208_SetMotionState(DECELERATING); 
        }
        else
        { 
          /* speed up the motor */
          L6208_VectorCalc(device_prm.accelTorque);     
          L6208_SetMotionState(ACCELERATING);
        }
      }
      break;
    case DECELERATING:  
      /* velocity mode: running motor deceleration phase */
      /* Decrease Speed and update position */
      L6208_DoDecel();  
      if(locMaxSpeedSpt > device_prm.speedSpt)
      { 
        /*Target speed reached but motor has still to be run*/
        device_prm.speedSpt = locMaxSpeedSpt;
        L6208_VectorCalc(device_prm.runTorque);   
        L6208_SetMotionState(STEADY);  
      }
      break;
    case DECELERATINGTOSTOP: 
      /* velocity mode: decelerate to stopped phase */
      /* Decrease current speed */
      L6208_DoDecel();
      if(device_prm.speedSpt == locMinSpeedSpt)
      { 
        if (L6208_IsSysFlag(dir2change))
        { 
          L6208_ClearSysFlag(running);
          /* Change direction */
          if (L6208_IsSysFlag(forward))
          {
            /* switch to reverse rotation */
            L6208_SetDirection(BACKWARD);
          }
          else
          {
            /* switch to forward rotation */
            L6208_SetDirection(FORWARD);
          }
          L6208_SetSysFlag(running);
          L6208_SetMotionState(ACCELERATING);
          /* Set VRefA and VRefB to the selected acceleration torque */
          L6208_VectorCalc(device_prm.accelTorque);
        }
        else
        {
          if (L6208_IsSysFlag(HiZstop))
          { 
            L6208_HardHiZ();
          }
          else
          {
            L6208_HardStop();
          }
        }
      }
      break;
    
    /* ============ Position (indexed) control mode states ======================== */

    case INDEX_ACCEL:
      /*  position mode: acceleration state*/
      
      /* Increase Speed and update position */
      L6208_DoAccel();  

      if(device_prm.positionTarget1 <= device_prm.step)
      { 
        /* End of acceleration phase */
        L6208_VectorCalc(device_prm.runTorque); 
        L6208_SetMotionState(INDEX_RUN);    
      }
      break;

      case INDEX_RUN:   
        /* position mode: constant speed phase */
        
        /* Update position */
        L6208_DoRun();  

        if(device_prm.positionTarget2 <= device_prm.step)
        {  
          /* reach position targeted for constant speed */
          L6208_VectorCalc(device_prm.decelTorque);   
          L6208_SetMotionState(INDEX_DECEL); 
        }
        break;

      case INDEX_DECEL: 
        /* position mode: deceleration phase */
        
        /* Decrease Speed and update position */
        L6208_DoDecel();  

        if(device_prm.positionTarget3 <= device_prm.step)
        {  
          /* reach position targeted for deceleration phase */
          /* the motor terminated its run */
          /* the torque will be the deceleration one */
          device_prm.step = device_prm.positionTarget3;
          L6208_SetMotionState(INDEX_DWELL);   
        }
        break;

      case INDEX_DWELL: 
        /* position mode: dwelling state */
        if(device_prm.dwellCounter > 0)
        {
          /* decrease the dwelling wait tick counter */
          device_prm.dwellCounter--;
        }
        if(device_prm.dwellCounter == 0)
        { 
          /* dwelling wait time is elapsed */
          /* so stop the motor */
          if (L6208_IsSysFlag(HiZstop))
          { 
            L6208_HardHiZ();
          }
          else
          {
            L6208_HardStop();
          }
        }
        break;
        
    /* ============ stopped state ======================== */
    case INACTIVE:
    {
      if(L6208_IsSysFlag(running))
      {
        /* clear the user move command executing  */
        /* and the motor running flags */
        L6208_ClearSysFlag(running);
      }
      break;
    }
    default:
      break;
  } /* switch(L6208_GetMotionState()) */
  if(L6208_GetMotionState() != INACTIVE)
  {
    if (L6208_IsSysFlag(microstep))
    { 
      /* Microstep handling */     
      switch(device_prm.uStepInc)
      {
        default:
        case 1:  
          /* 1 microstep increment */
          device_prm.lsbTicks = (uint8_t)(device_prm.ticks>>16);
          break;

        case 2:  
          /* 2 microsteps increment */           
          device_prm.lsbTicks = (uint8_t)(device_prm.ticks>>17);
          break;

        case 4:  
          /* 4 microsteps increment */
          device_prm.lsbTicks = (uint8_t)(device_prm.ticks>>18);
          break;
      }
      device_prm.lsbTicks &= 0x01;
      if(device_prm.lsbOldUSteppingTicks != device_prm.lsbTicks)
      { 
        /*  waveform sample to update */
        device_prm.lsbOldUSteppingTicks = device_prm.lsbTicks;
        device_prm.step++;
        if(L6208_IsSysFlag(forward))
        { 
          /* the motor is going forward */
          device_prm.absolutePos++;
          /* Reset the absolute motor position in step/microsteps */
          /* Get next microstep sample */
          device_prm.uStepSample += device_prm.uStepInc;  
          if(device_prm.uStepSample > 31)
          {
            device_prm.uStepSample = 0;
          }
        }
        else
        { 
         /* the motor is going backward */
          device_prm.absolutePos--;
          if(device_prm.uStepSample >= device_prm.uStepInc)
          {
            /* Get previous microstep sample */
            device_prm.uStepSample -= device_prm.uStepInc; 
          }
          else
          {
            device_prm.uStepSample = 32 - device_prm.uStepInc;
          }
        }
        /* set the PWM to update VRefs */
        L6208_VrefPwmComputePulseWidth(BRIDGE_A, pMicroTable2[device_prm.uStepSample], FALSE);
        L6208_VrefPwmComputePulseWidth(BRIDGE_B, microTable1[device_prm.uStepSample], FALSE);
        if(device_prm.uStepsample2update > 0)
        { 
          /*  the waveform samples table has been recalculated 
          so update the waveform scanning table */
          L6208_UpdateScanWaveformTable();
          device_prm.uStepsample2update = 0;
        }
      }
      /* Microstep: use the bit4 toggling as step clock */
      /* this bit is used because there are 16 microstep samples per quarter period */
      device_prm.lsbTicks = (uint8_t)((device_prm.uStepSample>>4) & 0x01);
      if(device_prm.lsbOldTicks != device_prm.lsbTicks)
      { 
        /* the selected bit status changed ==> get the next motor step
        save the current masked motor tick position for step setting scope ... */
        device_prm.lsbOldTicks = device_prm.lsbTicks;
        L6208_Board_CLOCK_PIN_Set();
      }
    }
    else
    {
      /* Full and half step handling code */ 
      if(!L6208_IsSysFlag(halfstep))
      { 
        /* Full step: use the bit 16 toggling as step clock */
        device_prm.lsbTicks = (uint8_t)((device_prm.ticks>>16) & 0x00000001);
      }
      else
      { 
        /* half step: use the bit 15 toggling as step clock */
        device_prm.lsbTicks = (uint8_t)((device_prm.ticks>>15) & 0x00000001);
      }
      if(device_prm.lsbOldTicks != device_prm.lsbTicks)
      { 
        /* the selected bit status changed ==> get the next motor step */
        device_prm.step++;
        if(L6208_IsSysFlag(forward))
        { 
          /* the motor is going forward */
          device_prm.absolutePos++;
        }
        else
        {
          /* the motor is going backward */
          device_prm.absolutePos--;          
        }
        /* save the current masked motor tick position for step setting scope ... */
        device_prm.lsbOldTicks = device_prm.lsbTicks;
        L6208_Board_CLOCK_PIN_Set();
      }
    }
  }
  L6208_UstepWaveformHandling();
  L6208_VrefPwmUpdatePulseWidth();
}
/******************************************************//**
 * @brief Get the frequency of VREFA and VREFB PWM
 * @retval the frequency of VREFA and VREFB PWM in Hz
 * @note
 **********************************************************/
uint32_t L6208::L6208_VrefPwmGetFreq(void)
{
  return device_prm.vrefPwmFreq;
}
/******************************************************//**
 * @brief Set the frequency of the VREFA and VREFB PWM
 * @param[in] newFreq in Hz
 * @retval None
 * @note
 **********************************************************/
void L6208::L6208_VrefPwmSetFreq(uint32_t newFreq)
{
  device_prm.vrefPwmFreq = newFreq;
  /* Compute the pwm period in 1/256th of a microsecond */
  device_prm.vrefPwmPeriod = (uint16_t)((1000000<<8)/newFreq);
  /* Re-Initialise the PWMs -----------------------------------------------------*/
  L6208_Board_VrefPwmInit(BRIDGE_A, device_prm.vrefPwmPeriod);
  L6208_Board_VrefPwmInit(BRIDGE_B, device_prm.vrefPwmPeriod);
  /* Recompute the waveform samples according to the new PWM frequency */
  L6208_ScaleWaveformTable();
  /* Update the waveform scanning table */
  L6208_UpdateScanWaveformTable();
  if (L6208_IsSysFlag(running))
  {
    L6208_Board_VrefPwmStart(BRIDGE_A, device_prm.vrefPwmPeriod);
    L6208_Board_VrefPwmStart(BRIDGE_B, device_prm.vrefPwmPeriod);
  }
}

/******************************************************//**
 * @brief Lock while motor is running
 * @retval None
 **********************************************************/
void L6208::L6208_WaitWhileActive(void)
{
  /* Wait while motor is running */
  while (L6208_IsSysFlag(running));
}

/* ------------------------------------------------------------------------- */
/* Private functions ------------------------------------------------------- */
/* ------------------------------------------------------------------------- */
/******************************************************//**
 * @brief Clear the bit/s of flags according to the specified mask
 * @param[in] mask flag bit mask
 * @retval None
 **********************************************************/
inline void L6208::L6208_ClearSysFlag(uint32_t mask)
{
  device_prm.flags &= ~mask;    
}

/******************************************************//**
 * @brief Compute the number of steps at the end of the accereration/deceleration phase
 * P = position in steps at the end of the acceleration/deceleration phase
 * T = acceleration/deceleration time in seconds
 * A =  acceleration/deceleration rate in steps per second per second (steps/sec^2)
 * V = peak velocity during acceleration/deceleration phase
 * V1 = average velocity during acceleration/deceleration phase
 * T = V/A
 * V1 = V/2
 * P = V1*T
 * P = V^2/2A
 * @param  accOrDecRate acceleration/deceleration rate in steps per second per second (steps/sec^2)
 * @retval end position or 0xFFFFFFFF on error 
 **********************************************************/
uint32_t L6208::L6208_ComputeNbAccOrDecSteps(uint16_t accOrDecRate)
{
  uint32_t nbAccOrDecSteps;
  uint32_t locMaxSpeedSps = (uint32_t)device_prm.maxSpeedSps;
  
  if (L6208_IsSysFlag(microstep))
  {
    switch(device_prm.uStepInc)
    {
      case 1:  
        locMaxSpeedSps = (uint32_t)device_prm.maxSpeedSps;
        break;
      case 2:            
        locMaxSpeedSps = ((uint32_t)device_prm.maxSpeedSps)>>1;
        accOrDecRate >>= 1;
        break;
      case 4:  
        locMaxSpeedSps = ((uint32_t)device_prm.maxSpeedSps)>>2;
        accOrDecRate >>= 2;
        break;
      default:
        break;
    }
  }
  else if (L6208_IsSysFlag(halfstep))
  {
    locMaxSpeedSps = ((uint32_t)device_prm.maxSpeedSps)<<1;
    accOrDecRate <<= 1;
  }
  
  if(accOrDecRate == 0)
  {
    /* division by 0 error */
    return 0xFFFFFFFF;
  }
  nbAccOrDecSteps = locMaxSpeedSps * locMaxSpeedSps;
  nbAccOrDecSteps /= (uint32_t)accOrDecRate;
  nbAccOrDecSteps /= 2;

  return nbAccOrDecSteps;
}

/******************************************************//**
 * @brief Compute the acceleration/deceleration speed increment value
 * @param[in] newAccOrDecRate acceleration or deceleration value (steps/s^2) greater or equal than 24
 * @retval the speed (step/tick) increment value
 * LSB = 2^-24 step/tick^2 or 2^-20 microstep/tick^2
 * @note return 0 if the rate is too low or if the tick frequency is too small
 * or if the device is running in position mode
 **********************************************************/
uint16_t L6208::L6208_ConvertAcceDecelRateValue(uint16_t newAccOrDecRate)
{
  uint64_t tmp64;
  uint32_t tmp32;

  if (((L6208_IsSysFlag(running))&&(L6208_IsSysFlag(positionmode)))||\
      (newAccOrDecRate < L6208_MIN_ACC_DEC_RATE))
  {
    return 0;
  } 
  /* Compute (tick frequency)^2 */
  tmp32 = (uint32_t)L6208_Board_TickGetFreq();
  tmp32 *= tmp32;
  /* Return 0 if the (tick frequency)^2 is too small */
  if ( tmp32 < (uint32_t)newAccOrDecRate )
  {
    return 0;
  } 
  /* Compute the decimal number of microstep or step per tick^2 */
  /* Decimal part is on 32 bits */
  tmp64 = (uint64_t)newAccOrDecRate << 32;
  tmp64 /= ((uint64_t)tmp32);

  return (uint16_t)((tmp64 & 0x00000000FFFFFFFF)>>8);
}

/******************************************************//**
 * @brief Compute next position and speed according to the acceleration rate
 * @retval None
 **********************************************************/
void L6208::L6208_DoAccel(void)
{
  /* Increase speed by acceleration rate */
  uint32_t locAccelerationSpt2 = (uint32_t)device_prm.accelerationSpt2;
  uint32_t locMinSpeedSpt = device_prm.minSpeedSpt;
  if ((device_prm.speedSpt + locAccelerationSpt2) < locMinSpeedSpt)
  {
    device_prm.speedSpt = locMinSpeedSpt;
  }
  else
  {
    device_prm.speedSpt += locAccelerationSpt2;
  }
  /* Compute next position */
  L6208_DoRun();
}
/******************************************************//**
 * @brief Compute next position and speed according to the deceleration rate
 * @retval None
 **********************************************************/
void L6208::L6208_DoDecel(void)
{
  /* Decrease current speed by deceleration rate */
  uint32_t locDecelerationSpt2 = (uint32_t)device_prm.decelerationSpt2;
  uint32_t locMinSpeedSpt = device_prm.minSpeedSpt;
  if((device_prm.speedSpt - locMinSpeedSpt) > (uint32_t)locDecelerationSpt2)  
  {
    device_prm.speedSpt -= (uint32_t)locDecelerationSpt2;
  }
  else
  {
    /* Set minimum speed */
    device_prm.speedSpt = locMinSpeedSpt;
  }
  /* Compute next position */
  L6208_DoRun(); 
}

/******************************************************//**
 * @brief Compute next position by adding current speed
 * @retval None
 **********************************************************/
void L6208::L6208_DoRun(void)
{
  device_prm.ticks += (device_prm.speedSpt >> 8) & 0x0000FFFF;
}

/******************************************************//**
 * @brief Get number of samples to rescale
 * @retval uStepsample2scale the number of micro stepping waveform samples to rescale
 **********************************************************/
uint8_t L6208::L6208_GetMicrostepSample2Scale(void)
{
  return device_prm.uStepsample2scale;
}

/******************************************************//**
 * @brief  Initialize the system for position mode motor moving command
 *  P = total move distance in steps
 *  P1 = steps required to accel from 0 to V
 *  P2 = steps required to decel from V to 0
 *  V = peak velocity in steps per second (steps/sec)
 *  V1 = average velocity during accel or decel*
 *  A = required accel rate in steps per second per second (steps/sec2)
 *  D = required decel rate in steps per second per second (steps/sec2)
 *  T1 = acceleration time in seconds
 *  T2 = deceleration time in seconds*
 *
 *  1) T1 = V / A
 *  2) V1 = V / 2
 *  3) P1 = V1 T1
 *  Substituting 1 and 2 into 3 yields:
 *  4) P1 = V2 / 2A
 *  In the same manner we have:
 *  5) P2 = V2 / 2D
 *
 *  P1 = PD/(D+A)
 *
 *  \sa Application Note: AN2044  
 * @retval None
 **********************************************************/
void L6208::L6208_Indexmodeinit(void)
{
  uint32_t tmpVal0;
  uint32_t tmpVal1;
  uint32_t locAccelSteps;
  uint32_t locDecSteps;

  /* calculate the number of steps to get the running speed */
  locAccelSteps = L6208_ComputeNbAccOrDecSteps(device_prm.accelerationSps2);
  /* calculate the number of steps to get the motor stopped */
  locDecSteps = L6208_ComputeNbAccOrDecSteps(device_prm.decelerationSps2);
  if(( locAccelSteps + locDecSteps ) > device_prm.positionTarget)
  { 
    /* Triangular move needed */
    /* accelsteps = P1 = PD/(D+A) */
    tmpVal0 = device_prm.positionTarget * device_prm.decelerationSps2;
    tmpVal1 = (uint32_t)device_prm.decelerationSps2;
    tmpVal1 += (uint32_t)device_prm.accelerationSps2;
    locAccelSteps = tmpVal0 / tmpVal1;
    device_prm.positionTarget1 = locAccelSteps;
    device_prm.positionTarget2 = device_prm.positionTarget1 + 1;
    device_prm.positionTarget3 = device_prm.positionTarget;
    if(device_prm.positionTarget1 == 0)
    {
      device_prm.positionTarget1 = 1;
    }
  }
  else
  {  
    /* trapezoidal move needed */
    /* P1 = V^2/2A */
    /* P2 = P - V^2/2D */
    device_prm.positionTarget1 = locAccelSteps;
    device_prm.positionTarget2 = device_prm.positionTarget - locDecSteps;
    device_prm.positionTarget3 = device_prm.positionTarget;
  }
  L6208_SetMotionState(INDEX_ACCEL);  
}

/******************************************************//**
 * @brief Check the bit/s of flags according to the specified mask
 * @param[in] mask flag bit mask
 * @retval TRUE if the bit of the mask are set
 **********************************************************/
inline bool L6208::L6208_IsSysFlag(uint32_t mask)
{
  return (bool)((device_prm.flags & mask) == mask);    
}

/******************************************************//**
 * @brief Stepper driver device step state reset subroutine
 * @retval None
 **********************************************************/
void L6208::L6208_ResetSteps(void)
{
  device_prm.speedSpt = 0;             // reset the current speed value
  device_prm.ticks = 0;                // reset the current ticks counter value
  device_prm.step = 0;                 // reset the current step counter value
  device_prm.lsbOldTicks = 0;          // reset copy of the previous position (tick)
  device_prm.lsbOldUSteppingTicks = 0; // reset copy of the previous position (tick) ( micro stepping )
  device_prm.lsbTicks = 0;             // reset copy of the current position (tick)
  device_prm.absolutePos = 0;          // reset the absolute motor position in step/microsteps
  device_prm.uStepSample = 0;          // reset the microstepping waveform sample index
}

/******************************************************//**
 * @brief Compute the specified micro stepping waveform sample with the
 * current selected torque and pwm period
 * @param[in] sampleIndex sample Index
 * @retval scaled sample value
 **********************************************************/
uint32_t L6208::L6208_ScaleWaveformSample(uint8_t sampleIndex)
{
  uint32_t sample;

  sample = (uint32_t)RefMicroTable[sampleIndex];
  sample *= device_prm.vrefPwmPeriod;
  sample >>= (uint32_t)L6208_SINE_WAVEFORM_POWER_OF_TWO_MAX_VALUE;
  
  sample *= (uint32_t)device_prm.curTorqueScaler; // torque val (%)
  sample /= (uint32_t)100;
  
  return sample;
}

/******************************************************//**
 * @brief Compute the micro stepping waveform sample table samples with the
 * current selected torque and pwm period
 * @retval None
 **********************************************************/
void L6208::L6208_ScaleWaveformTable(void)
{
  uint8_t index;
  for(index=0; index<=L6208_USTEPS_PER_QUARTER_PERIOD; index++)
  { 
    /* Calculate the scaled sample and save its value into the waveform to update table */
    updatedMicroTable[index] = (uint16_t)L6208_ScaleWaveformSample(index);
  }
}

/******************************************************//**
 * @brief  Set the parameters of the device to values of the structure pointed
 * by pInitDevicePrm. Set GPIO according to these values.
 * @param pInitDevicePrm pointer onto the structure containing values to
 * initialize the device parameters.
 * @retval None
 **********************************************************/
void L6208::L6208_SetDeviceParamsToGivenValues(l6208_init_t* pInitDevicePrm)
{
  memset(&device_prm, 0, sizeof(device_prm));
  L6208_SetAcceleration(pInitDevicePrm->accelerationSps2);
  L6208_SetDeceleration(pInitDevicePrm->decelerationSps2);
  L6208_SetMaxSpeed(pInitDevicePrm->maxSpeedSps);
  L6208_SetMinSpeed(L6208_MIN_SPEED);
  device_prm.accelTorque = pInitDevicePrm->accelTorque;
  device_prm.decelTorque = pInitDevicePrm->decelTorque;
  device_prm.runTorque = pInitDevicePrm->runTorque;
  device_prm.holdTorque = pInitDevicePrm->holdTorque;
  /* Only once acceleration, deceleration, min speed and max speed have been */
  /* initialized, set the step mode */
  device_prm.stepMode = pInitDevicePrm->stepMode;
  L6208_SetDecayMode(pInitDevicePrm->decayMode);
  device_prm.moveDwellTime = pInitDevicePrm->moveDwellTime;
  if (L6208_CONF_PARAM_AUTO_HIZ_STOP)
  {
    L6208_SetSysFlag(pInitDevicePrm->autoHiZstop);
  }
  device_prm.vrefPwmFreq = pInitDevicePrm->vrefPwmFreq;
  device_prm.vrefPwmPeriod = (uint16_t)((1000000<<8)/pInitDevicePrm->vrefPwmFreq);
  /* Initialize current stepper state machine index  */
  L6208_SetMotionState(INACTIVE);
}

/******************************************************//**
 * @brief  Set the parameters of the device to predefined values
 * Set GPIO according to these values
 * from l6208_target_config.h
 * @retval None
 **********************************************************/
void L6208::L6208_SetDeviceParamsToPredefinedValues(void)
{
  memset(&device_prm, 0, sizeof(device_prm));
  L6208_SetAcceleration(L6208_CONF_PARAM_ACC_RATE);
  L6208_SetDeceleration(L6208_CONF_PARAM_DEC_RATE);
  L6208_SetMaxSpeed(L6208_CONF_PARAM_RUNNING_SPEED);
  L6208_SetMinSpeed(L6208_MIN_SPEED);
  device_prm.accelTorque = L6208_CONF_PARAM_ACC_CURRENT;
  device_prm.decelTorque = L6208_CONF_PARAM_DEC_CURRENT;
  device_prm.runTorque = L6208_CONF_PARAM_RUNNING_CURRENT;
  device_prm.holdTorque = L6208_CONF_PARAM_HOLDING_CURRENT;
  /* Only once acceleration, deceleration, min speed and max speed have been */
  /* initialized, set the step mode */
  device_prm.stepMode = (motorStepMode_t) L6208_CONF_PARAM_STEP_MODE;
  L6208_SetDecayMode(L6208_CONF_PARAM_DECAY_MODE);
  device_prm.moveDwellTime = L6208_CONF_PARAM_DWELL_TIME;
  if (L6208_CONF_PARAM_AUTO_HIZ_STOP) 
  {
    L6208_SetSysFlag(HiZstop);
  }
  device_prm.vrefPwmFreq = L6208_CONF_VREF_PWM_FREQUENCY;
  device_prm.vrefPwmPeriod = (uint16_t)((1000000<<8)/L6208_CONF_VREF_PWM_FREQUENCY);
  /* Initialize current stepper state machine index  */
  L6208_SetMotionState(INACTIVE);
}

/******************************************************//**
 * @brief Set the number of micro stepping waveform samples to rescale
 * @param[in] value number of micro stepping waveform samples 
 * @retval None
 **********************************************************/
void L6208::L6208_SetMicrostepSample2Scale(uint8_t value)
{
  if(value > L6208_USTEPS_PER_QUARTER_PERIOD)
  {
    value = L6208_USTEPS_PER_QUARTER_PERIOD;  // clamp to maximum number of samples per period/4
  }
  device_prm.uStepsample2scale = value;    
}

/******************************************************//**
 * @brief Set the number of micro stepping waveform samples to update into scanning
 * @param[in] value number of micro stepping waveform samples 
 * @retval None
 **********************************************************/
void L6208::L6208_SetMicrostepSample2Update(uint8_t value)
{
  // clamp to maximum number of samples per period/4
  if(value > L6208_USTEPS_PER_QUARTER_PERIOD)
  {
    value = L6208_USTEPS_PER_QUARTER_PERIOD;  
  }
  // copy the stepper acceleration rate
  device_prm.uStepsample2update = value;    
}

/******************************************************//**
 * @brief Set the stepper state machine index
 * @param[in] newMotionState
 * @retval None
 **********************************************************/
void L6208::L6208_SetMotionState(motorState_t newMotionState)
{
  // sets the new stepper state machine index
  device_prm.motionState = newMotionState;    
}

/******************************************************//**
 * @brief Set the user selected speed in step/tick
 * @param[in] newSpeed speed value (step/s)
 * @param[in] pSpeed pointer to the selected speed field
 * @retval return FALSE if the speed is too low or too high
 * or if the device is running in position mode, else TRUE
 **********************************************************/
bool L6208::L6208_SetSpeed(uint16_t newSpeed, uint32_t volatile *pSpeed)
{
  uint64_t tmp64;
  uint32_t tmp32;

  if (((L6208_IsSysFlag(running))&&(L6208_IsSysFlag(positionmode)))||\
      (newSpeed < L6208_MIN_SPEED))
  {
    return FALSE;
  }
  tmp32 = (uint32_t)L6208_Board_TickGetFreq();
  if (tmp32 < newSpeed)
  {
    return FALSE;
  }
  /* Compute the decimal number of microstep or step per tick */
  /* Decimal part is on 32 bits */
  tmp64 = (uint64_t)newSpeed << 32;
  tmp64 /= ((uint64_t)tmp32);
  /* set the running constant speed value (step/tick) */
  *pSpeed = (uint32_t)((tmp64 & 0x00000000FFFFFFFF)>>8);
  
  return TRUE;
}

/******************************************************//**
 * @brief Set the bit/s of flags according to the specified mask
 * @param[in] mask flag bit mask
 * @retval None
 **********************************************************/
inline void L6208::L6208_SetSysFlag(uint32_t mask)
{
  device_prm.flags |= mask;    
}

/******************************************************//**
 * @brief Stepper motor start command
 * @retval true on correct command execution
 **********************************************************/
bool L6208::L6208_StartMovement(void)
{
  uint32_t tmp;
  if (L6208_IsSysFlag(running))
  {Serial.println("start if 1");
    /* Motor is already running ==> quit */
    return FALSE;    
  }
  if (!L6208_IsSysFlag(positionmode))
  {Serial.println("start if 2");
    /* Set the VREFA and VREFB to the selected acc. torque */
    L6208_VectorCalc(device_prm.accelTorque);
    
    /* If the speed control mode is selected */
    /* setup the motor acceleration for velocity mode driving */
    L6208_SetMotionState(ACCELERATING);  
  }
  else
  {  Serial.println("start else 1");
    /* if position control mode is selected, reset the current step counter  */
    device_prm.step = 0;    
    if(device_prm.uStepSample > 31)
    {Serial.println("start if 3");
      /* check the micro stepping waveform sample index */
      device_prm.uStepSample = 0;
    }
    /* Set the position dwelling wait time */
    /* compute number of ticks per millisecond */
    tmp = (uint32_t)L6208_Board_TickGetFreq() / 1000;
    /* Compute the dwelling time in ticks => dwellCounter (ticks) */
    device_prm.dwellCounter = tmp * (uint32_t)device_prm.moveDwellTime;
    if (device_prm.positionTarget == 0)
    {Serial.println("start if 4");
      /* if the position to go is 0 (no move) */
      /* Set the deceleration torque */
      L6208_VectorCalc(device_prm.decelTorque); 
      /* Set the dwelling delay state index */
      L6208_SetMotionState(INDEX_DWELL);       
    }
    else
    {Serial.println("start else 2");
      /* Set the VREFA and VREFB to the selected acc. torque */
      L6208_VectorCalc(device_prm.accelTorque);
      /* go to the selected position */
      L6208_Indexmodeinit();
      L6208_SetMotionState(INDEX_ACCEL);
    }
  }
  /* Sets the motor running flag */
  L6208_SetSysFlag(running);
  /* Start the VREFA and VREFB PWMs */
  L6208_Board_VrefPwmStart(BRIDGE_A, device_prm.vrefPwmPeriod);
  L6208_Board_VrefPwmStart(BRIDGE_B, device_prm.vrefPwmPeriod);
  if (!(L6208_IsSysFlag(EN_A_set)))
  {Serial.println("start if 5");
    /* Enable power bridges */
    L6208_Enable();
    Serial.println("stoppete");
  }
  /* Start the tick */
  L6208_Board_TickStart(L6208::tickFreq);
  
  return TRUE;
}

/******************************************************//**
 * @brief Update the micro stepping waveform samples table with the
 * values previously scaled with current selected torque and tick period
 * @retval None
 **********************************************************/
void L6208::L6208_UpdateScanWaveformTable(void)
{
  uint8_t index;

  for(index=0; index<=L6208_USTEPS_PER_QUARTER_PERIOD; index++)
  {
    microTable1[index] = updatedMicroTable[index];
    microTable1[L6208_USTEPS_PER_QUARTER_PERIOD*2 - index] = microTable1[index];
    microTable1[index + L6208_USTEPS_PER_QUARTER_PERIOD*2] = updatedMicroTable[index];
  }
  /* clear the number of samples to update */
  L6208_SetMicrostepSample2Update(0); 
}

/******************************************************//**
 * @brief Check if there are waveform samples to rescale and if so, perform the rescaling
 * @retval None
 **********************************************************/
void L6208::L6208_UstepWaveformHandling(void)
{
  /* micro stepper waveform samples rescaling ... and updating */
  uint8_t nbSamplesToRescale = L6208_GetMicrostepSample2Scale();
  if(nbSamplesToRescale > 0)
  { 
    /* Current torque value has been changed, so recalculate the waveform table */
    L6208_ScaleWaveformTable();
    
    /* Set the number of samples to update */
    L6208_SetMicrostepSample2Update(L6208_USTEPS_PER_QUARTER_PERIOD);

    /* Reset the number of samples to rescaled afer rescaling */
    L6208_SetMicrostepSample2Scale(0);
  }
}

/******************************************************//**
 * @brief Set the current torque value (Vref)
 * @param[in] newTorque Selected torque value
 * @retval always TRUE
 **********************************************************/
bool L6208::L6208_VectorCalc(uint8_t newTorque)
{
  /* save current selected torque value */
  device_prm.curTorqueScaler = (uint16_t)newTorque;

  if(!L6208_IsSysFlag(microstep))
  {
    /* full/half step mode or the motor is not running */
    /* set the PWM duty cycle according to the current torque value (%). */
    /* The TON value will be calculated inside the TIMx_PWM_duty_setup f(). */
    L6208_VrefPwmComputePulseWidth(BRIDGE_A, device_prm.curTorqueScaler, TRUE);
    L6208_VrefPwmComputePulseWidth(BRIDGE_B, device_prm.curTorqueScaler, TRUE);
    device_prm.vRefAVal = device_prm.curTorqueScaler; // save current VREFA value
    device_prm.vRefBVal = device_prm.curTorqueScaler; // save current VREFB value
  }
  else
  { 
    /* microstep mode */
    if(L6208_IsSysFlag(running))
    {
      /* set the number of waveform sample to rescale according current selected */
      /* torque value */
      L6208_SetMicrostepSample2Scale(L6208_USTEPS_PER_QUARTER_PERIOD);
    }
    else
    { 
      /* micro stepping mode motor stopped */
      L6208_ScaleWaveformTable();
      L6208_UpdateScanWaveformTable();
      /* Set the VREF timer PWM TON to update VREFA and VREFB */
      L6208_VrefPwmComputePulseWidth(BRIDGE_A, pMicroTable2[device_prm.uStepSample], FALSE);      
      L6208_VrefPwmComputePulseWidth(BRIDGE_B, microTable1[device_prm.uStepSample], FALSE);
    }
  }
  return TRUE;
}

/******************************************************//**
 * @brief Compute the pulse width of VREFA or VREFB PWM
 * @param[in] bridgeId
 *            0 for BRIDGE_A
 *            1 for BRIDGE_B
 * @param[in] value pulse length in 1/256th of microsecond
 * or PWM duty cycle: 0 - 100 %
 * @param[in] valueIsPwmDutyCycle must be TRUE if value is a PWM duty cycle
 * @retval FALSE if wrong timer handle is used, else TRUE
 **********************************************************/
bool L6208::L6208_VrefPwmComputePulseWidth(uint8_t bridgeId, uint16_t value, bool valueIsPwmDutyCycle)
{
  if(valueIsPwmDutyCycle)
  {
    if (value > 100)
    {
      value = 100;
    }
    value = (uint16_t)(((uint32_t)device_prm.vrefPwmPeriod * (uint32_t)value) / 100); 
  }
  if (bridgeId == 0)
  {
    device_prm.vrefPwmPulseWidthTargetA = value;
    device_prm.vrefPwmPulseWidthToBeGeneratedA = 0;
  } 
  else if (bridgeId == 1)
  {
    device_prm.vrefPwmPulseWidthTargetB = value;
    device_prm.vrefPwmPulseWidthToBeGeneratedB = 0;
  }
  else
  {
    return FALSE;
  }
  return TRUE;
}

/******************************************************//**
 * @brief Update the pulse width of VREFA or VREFB PWM
 * @param None
 * @retval None
 **********************************************************/
void L6208::L6208_VrefPwmUpdatePulseWidth(void)
{
  uint16_t pulseWidthUs;
  
  device_prm.vrefPwmPulseWidthToBeGeneratedA += device_prm.vrefPwmPulseWidthTargetA;
  pulseWidthUs = device_prm.vrefPwmPulseWidthToBeGeneratedA>>8;
  if (pulseWidthUs!=0)
  {
    L6208_Board_VrefPwmSetPulseWidthA(pulseWidthUs);
    device_prm.vrefPwmPulseWidthToBeGeneratedA -= (pulseWidthUs<<8);
  }
  else
  {
    L6208_Board_VrefPwmSetPulseWidthA(0);
  }
  
  device_prm.vrefPwmPulseWidthToBeGeneratedB += device_prm.vrefPwmPulseWidthTargetB;
  pulseWidthUs = device_prm.vrefPwmPulseWidthToBeGeneratedB>>8;
  if (pulseWidthUs!=0)
  {
    L6208_Board_VrefPwmSetPulseWidthB(pulseWidthUs);
    device_prm.vrefPwmPulseWidthToBeGeneratedB -= (pulseWidthUs<<8);
  }
  else
  {
    L6208_Board_VrefPwmSetPulseWidthB(0);
  }
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

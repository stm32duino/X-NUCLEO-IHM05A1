/******************************************************//**
  * @file    L6208_def.h 
  * @author  IPC Rennes
  * @version V1.1.0
  * @date    February 11th, 2016
  * @brief   Header for l6208.c module
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
#ifndef _L6208_H_INCLUDED
#define _L6208_H_INCLUDED

#ifdef __cplusplus
 extern "C" {
#endif 

/* Includes ------------------------------------------------------------------*/
#include "L6208_config.h"
#include "motor_def.h" 

/* Definitions ---------------------------------------------------------------*/

/** @addtogroup Components
 * @{
 */
    
/** @defgroup L6208
  * @{
  */

/** @defgroup L6208_Exported_Defines L6208 Exported Defines
  * @{
  */
/// Current FW major version
#define L6208_FW_MAJOR_VERSION (uint8_t)(1)
/// Current FW minor version
#define L6208_FW_MINOR_VERSION (uint8_t)(1)
/// Current FW patch version
#define L6208_FW_PATCH_VERSION (uint8_t)(0)
/// Current FW version
#define L6208_FW_VERSION       (uint32_t)((L6208_FW_MAJOR_VERSION<<16)|\
                                          (L6208_FW_MINOR_VERSION<<8)|\
                                          (L6208_FW_PATCH_VERSION))

/// Exponent used to scale the sine function for the RefMicroTable
#define L6208_SINE_WAVEFORM_POWER_OF_TWO_MAX_VALUE (15)

/// Tick frequency (Hz)
#define TIMER_TICK_FREQUENCY         (10000)

/// MCU wait time after power bridges are enabled
#define BRIDGE_TURN_ON_DELAY         (10)

/// The maximum number of devices
#define MAX_NUMBER_OF_DEVICES        (1)

/// Max position
#define L6208_MAX_POSITION           (0x7FFFFFFF)

/// Min position
#define L6208_MIN_POSITION           (0x80000000)

/// Position range
#define L6208_POSITION_RANGE         ((uint32_t)(L6208_MAX_POSITION -\
                                                        L6208_MIN_POSITION))
/// micro step samples per period/4 
#define L6208_USTEPS_PER_QUARTER_PERIOD        (16)

/// minimum speed
#define L6208_MIN_SPEED                        (16)

/// minimum acceleration and deceleration rate
#define L6208_MIN_ACC_DEC_RATE                 (24)

/// Mask for HiZ bit in motorDecayMode_t enum
#define L6208_FAST_DECAY_MODE_MASK             (0x1)

/// L6208 error base number
#define L6208_ERROR_BASE                       (0x9000)
/**
  * @}
  */

/* Types ---------------------------------------------------------------------*/

/** @defgroup L6208_Exported_Types L6208 Exported Types
  * @{
  */

/** @defgroup Error_Types Error Types
  * @{
  */
/// Errors
typedef enum {
  L6208_ERROR_SET_HOME         = L6208_ERROR_BASE,     /// Error while setting home position
  L6208_ERROR_SET_MAX_SPEED    = L6208_ERROR_BASE + 1, /// Error while setting max speed
  L6208_ERROR_SET_MIN_SPEED    = L6208_ERROR_BASE + 2, /// Error while setting min speed
  L6208_ERROR_SET_ACCELERATION = L6208_ERROR_BASE + 3, /// Error while setting acceleration
  L6208_ERROR_SET_DECELERATION = L6208_ERROR_BASE + 4, /// Error while setting decelaration
  L6208_ERROR_MCU_OSC_CONFIG   = L6208_ERROR_BASE + 5, /// Error while configuring mcu oscillator
  L6208_ERROR_MCU_CLOCK_CONFIG = L6208_ERROR_BASE + 6, /// Error while configuring mcu clock
  L6208_ERROR_POSITION         = L6208_ERROR_BASE + 7, /// Unexpected current position (wrong number of steps)
  L6208_ERROR_SPEED            = L6208_ERROR_BASE + 8, /// Unexpected current speed
  L6208_ERROR_INIT             = L6208_ERROR_BASE + 9, /// Unexpected number of devices
  L6208_ERROR_SET_DIRECTION    = L6208_ERROR_BASE + 10,/// Error while setting direction
  L6208_ERROR_SET_STEP_MODE    = L6208_ERROR_BASE + 11,/// Attempt to set an unsupported step mode
  L6208_ERROR_SET_PWM          = L6208_ERROR_BASE + 12,/// Error while setting a PWM parameter
}errorTypes_t;
/**
  * @}
  */

/** @defgroup Device_Parameters Device Parameters
  * @{
  */
/// Device Parameters Structure Type
typedef struct
{
  /// dwelling waiting time counter (tick)
  volatile uint32_t dwellCounter;
  /// motor position indicator (tick)
  uint32_t ticks;
  /// LSByte copy of the previous position (tick)
  uint8_t lsbOldTicks;
  /// LSByte copy of the previous position (tick) ( micro stepping )
  uint8_t lsbOldUSteppingTicks;
  /// LSByte copy of the current position (tick)
  uint8_t lsbTicks;
  /// P1 = acceleration phase steps number (motor position control mode)
  uint32_t positionTarget1;
  /// P2 = constant speed steps number (motor position control mode)
  uint32_t positionTarget2;
  /// P3 = deceleration phase steps number (motor position control mode)
  uint32_t positionTarget3;
  /// P = total move distance in steps (motor position control mode)
  uint32_t positionTarget;
  /// absolute motor position in microsteps (motor position control mode)
  volatile int32_t absolutePos;
  /// mark position in microsteps (motor position control mode)
  volatile int32_t markPos;
  /// motor position in microsteps (motor position control mode)
  volatile uint32_t step;
  /// dwelling time after position got (ms)
  volatile uint16_t moveDwellTime;
  /// number of micro stepping waveform samples to be rescaled according to selected torque value
  volatile uint8_t uStepsample2scale;
  /// number of micro stepping waveform samples to be updated into the waveform scanning table  
  volatile uint8_t uStepsample2update;
  /// microstepping waveform sample index  
  volatile uint8_t uStepSample;
  /// system status flags
  volatile uint32_t flags;
  /// current stepper state machine index
  volatile motorState_t motionState;
  /// current step mode
  volatile motorStepMode_t stepMode;
  /// micro stepping waveform scanning sample index increment
  uint8_t uStepInc;
  /// frequency of the VREFA and VREFB PWM
  uint32_t vrefPwmFreq;
  /// period of the VREFA and VREFB PWM in 1/256th of a microsecond
  uint16_t vrefPwmPeriod;
  /// pulse width target of the VREFA PWM in 1/256th of a microsecond
  volatile uint16_t vrefPwmPulseWidthTargetA;
  /// pulse width target of the VREFB PWM in 1/256th of a microsecond
  volatile uint16_t vrefPwmPulseWidthTargetB;
  /// pulse width to be generated for VREFA PWM in 1/256th of a microsecond
  volatile int16_t vrefPwmPulseWidthToBeGeneratedA;
  /// pulse width to be generated for VREFB PWM in 1/256th of a microsecond
  volatile int16_t vrefPwmPulseWidthToBeGeneratedB;
  /// current selected torque value
  volatile uint16_t curTorqueScaler;
  /// selected VREFA value (%)
  volatile uint16_t vRefAVal;
  /// selected VREFB value (%)
  volatile uint16_t vRefBVal;
  /// constant speed phase torque value (%)
  volatile uint8_t runTorque;
  /// acceleration phase torque value (%)
  volatile uint8_t accelTorque;
  /// deceleration phase torque value (%)
  volatile uint8_t decelTorque;
  /// holding phase torque value (%)
  volatile uint8_t holdTorque;
  /// acceleration (steps/s^2)
  volatile uint16_t accelerationSps2;
  /// deceleration (steps/s^2)
  volatile uint16_t decelerationSps2;
  /// acceleration (steps/tick^2)
  volatile uint16_t accelerationSpt2;
  /// deceleration (steps/tick^2)
  volatile uint16_t decelerationSpt2;
  /// maximum speed (steps/s)
  volatile uint16_t maxSpeedSps;
  /// minimum speed (steps/s)
  volatile uint16_t minSpeedSps;
  /// current speed (steps/s)
  volatile uint16_t speedSps;
  /// maximum speed (steps/tick)
  volatile uint32_t maxSpeedSpt;
  /// minimum speed (steps/tick)
  volatile uint32_t minSpeedSpt;
  /// current speed (steps/tick) 
  volatile uint32_t speedSpt;
}deviceParams_t;
/**
  * @}
  */

/// Motor driver initialization structure definition  
typedef struct
{
  /// acceleration (steps/s^2)
  uint16_t accelerationSps2;
  /// acceleration phase torque value (%)
  uint8_t accelTorque;
  /// deceleration (steps/s^2)
  uint16_t decelerationSps2;
  /// deceleration phase torque value (%)
  uint8_t decelTorque;
  /// maximum speed (steps/s)
  uint16_t maxSpeedSps;
  /// constant speed phase torque value (%)
  uint8_t runTorque;
  /// holding phase torque value (%)
  uint8_t holdTorque;
  /// current step mode
  motorStepMode_t stepMode;
  /// current decay mode (SLOW_DECAY or FAST_DECAY)
  motorDecayMode_t decayMode;
  /// dwelling time after position got (ms)
  uint16_t moveDwellTime;
  /// automatic HiZ on stop
  bool autoHiZstop;
  /// frequency of the VREFA and VREFB PWM
  uint32_t vrefPwmFreq;
} l6208_init_t;
/**
  * @}
  */

/* Functions --------------------------------------------------------*/

/** @defgroup MotorControl_Board_Linked_Functions MotorControl Board Linked Functions
  * @{
  */
///Delay of the requested number of milliseconds
extern void L6208_Board_Delay(uint32_t delay);
///Enable Irq
extern void L6208_Board_EnableIrq(void);
///Disable Irq
extern void L6208_Board_DisableIrq(void);
//Initialize the VREFA or VREFB PWM
extern bool L6208_Board_VrefPwmInit(uint8_t bridgeId, uint32_t pwmFreq);
///Initialize the tick
extern void L6208_Board_TickInit(void);
///Release the reset pin 
extern void L6208_Board_Releasereset(void);
///Set the reset pin
extern void L6208_Board_reset(void);
///Set the control pin
extern void L6208_Board_CONTROL_PIN_Set(void);
///Reset the control pin
extern void L6208_Board_CONTROL_PIN_reset(void);
///Set the clock pin
extern void L6208_Board_CLOCK_PIN_Set(void);
///Reset the clock pin
extern void L6208_Board_CLOCK_PIN_reset(void);
///Set the half full pin
extern void L6208_Board_HALF_FULL_PIN_Set(void);
///Reset the half full pin
extern void L6208_Board_HALF_FULL_PIN_reset(void);
///Set the dir pin
extern void L6208_Board_DIR_PIN_Set(void);
///Reset the dir pin
extern void L6208_Board_DIR_PIN_reset(void);
///Enable the power bridges (leave the output bridges HiZ)
extern void L6208_Board_enable(void);
///Disable the power bridges (leave the output bridges HiZ)
extern void L6208_Board_disable(void);
/**
  * @}
  */

  /**
  * @}
  */

/**
  * @}
  */
  
#ifdef __cplusplus
  }
#endif

#endif /* __L6208_H */

/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
 

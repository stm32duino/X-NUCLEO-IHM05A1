/******************************************************//**
  * @file    L6208_config.h 
  * @author  IPC Rennes
  * @version V1.1.0
  * @date    February 11th, 2016
  * @brief   Predefines values for the L6208 parameters
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
#ifndef __L6208_CONFIG_H
#define __L6208_CONFIG_H

#ifdef __cplusplus
 extern "C" {
#endif 

/** @addtogroup L6208
  * @{
  */

/** @addtogroup L6208_Exported_Constants
  * @{
  */

/** @defgroup Predefined_L6208_Parameters_Values Predefined L6208 Parameters Values
  * @{
  */

/// Acceleration rate in step/s^2 or (1/16)th step/s^2 for microstep modes
#define L6208_CONF_PARAM_ACC_RATE        (1000)

/// Acceleration current torque in % (from 0 to 100)
#define L6208_CONF_PARAM_ACC_CURRENT     (10)

/// Deceleration rate in step/s^2 or (1/16)th step/s^2 for microstep modes
#define L6208_CONF_PARAM_DEC_RATE        (1000)

/// Deceleration current torque in % (from 0 to 100)
#define L6208_CONF_PARAM_DEC_CURRENT     (10)

/// Running speed in step/s or (1/16)th step/s for microstep modes
#define L6208_CONF_PARAM_RUNNING_SPEED   (1000)

/// Running current torque in % (from 0 to 100)
#define L6208_CONF_PARAM_RUNNING_CURRENT (10)

/// Holding current torque in % (from 0 to 100)
#define L6208_CONF_PARAM_HOLDING_CURRENT (10)

/// Step mode via enum motorStepMode_t
#define L6208_CONF_PARAM_STEP_MODE       (STEP_MODE_1_16)

/// Decay mode via enum motorDecayMode_t
#define L6208_CONF_PARAM_DECAY_MODE      (FAST_DECAY)

/// Dwelling time in ms
#define L6208_CONF_PARAM_DWELL_TIME      (0)

/// Automatic HIZ STOP
#define L6208_CONF_PARAM_AUTO_HIZ_STOP   (FALSE)

/// VREFA and VREFB PWM frequency (Hz)
#define L6208_CONF_VREF_PWM_FREQUENCY    (100000)

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

#endif /* __L6208_CONFIG_H */

/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/ 

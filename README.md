# X-NUCLEO-IHM05A1
The X-NUCLEO-IHM05A1 is a bipolar stepper motor driver expansion board based on the L6208 driver for bipolar stepper motors. It provides an affordable and easy-to-use solution for driving bipolar stepper motors in STM32 Nucleo projects. The X-NUCLEO-IHM05A1 is compatible with the Arduino UNO R3 connector, and supports the addition of other STM32 expansion boards with a single STM32 Nucleo board. The user can also mount the ST Morpho connector.
Library to handle the X-NUCLEO-IHM05A1 Motor Control Expansion Board based on the L6208 component. It features the: read and write of device parameters, configuration of GPIOs and IRQs (for enabling, direction, current decay and microstepping), control of position, speed, acceleration and deceleration, command locking until the device completes movement, handling of overcurrent and thermal alarms (flag interrupt handling). The API allows to easily: perform various positioning, moves and stops; get/set or monitor the motor positions; set home position and mark; another position; get/set minimum and maximum speed; get current speed; get/set acceleration and deceleration; get/set the step mode (up to 1/16).

# Examples 
X_NUCLEO_IHM05A1_HelloWorld: This application provides a simple example of usage of the X-NUCLEO-IHM05A1 bipolar stepper motor driver expansion board based on the L6208 driver. It shows how to use a stepper motor connected to the board by: running the motor, monitoring the speed and the motor state, setting/getting the speed, setting/getting the step mode. setting/getting the acceleration and deceleration, moving a defined number of steps or microsteps, setting/getting the direction, setting/getting the decay mode.

# Documentation
You can find the source files at

https://github.com/stm32duino/X-NUCLEO-IHM05A1

The L6208 DMOS driver for bipolar stepper motor datasheet is available at

https://www.st.com/content/st_com/en/products/motor-drivers/stepper-motor-drivers/l6208.html

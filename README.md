# STM32F4_Stepper
Template project for evaluating the 28HB30-401A linear stage with TB6600 controller

## Hardware
- 28HB30-401A with TB6600 from Aliexpress (https://www.aliexpress.com/item/4001272086575.html?spm=a2g0o.order_list.order_list_main.29.63d33696rHW8kO#nav-specification)
- STM32F4-Disco (the old but gold) with mini usb cable
- Parallax joystick u-d l-r
- 24V power supply, or enough to power the motor/driver (9V-42V)
- Jumper cables male-female

## Software
- Standard HAL libs and STM32CubeIDE

## Wiring
- Cabling 3V3 from STM to Joystick and analog signals to PA1 (M1) and PA3 (M2)
- Cabling PWM Out (PUL+) PE9 (M1) PE11 (M2)
- Cabling GPIO Out (DIR+) PC5 (M1) and PB1 (M2)

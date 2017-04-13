This logic analyzer polls pins PA0..PA7 at up to a 7.2MHz rate.  Use a SUMP client: https://lxtreme.nl/projects/ols/

Copy the file ols.profile-stm32f103.cfg to your ols plugins directory (there will be other ols.\*.cfg files there)

When capturing, the device type is "STM32F103 Logic Analyzer"

Largest capture size is 10KB

This sump protocol code is based on https://github.com/gillham/logic\_analyzer/blob/master/logic\_analyzer.ino

There is some debug output on uart1 PA9/PA10, but it's not needed to use this code.

There's a 100KHz PWM setup on pin PB4 for testing

Compiling requires make and arm-none-eabi-gcc in your path.  newlib is recommended

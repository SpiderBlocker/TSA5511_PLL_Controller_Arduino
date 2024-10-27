# TSA5511 PLL Controller for Arduino

# Description
My very first Arduino project, being a controller for the TSA5511 PLL, initially intended to replace the proprietary controller for the DRFS06 exciter by Dutch RF Shop.
I think this attempt actually succeeded quite well. Did a lot of testing and fault simulations, including I²C verifications using an I²C decoder. The thing now seems to be quite fool-proof :-)

Using a 3,2 MHz crystal on the TSA5511, the control ranges from 50 kHz up to 1.638,35 MHz, with a step size of 50 kHz or any multiple thereof.
The practical lower and upper limits will be much tighter, as the TSA5511 is rated from 64 MHz up to 1.300 MHz.
Using any other crystal frequency (as far as the TSA5511 may support), valid frequency step size and divisor bytes for the TSA5511 are calculated automatically.
It has a built-in station name editor and the station name and last set frequency are stored in EEPROM.

# Hardware
- The hardware comprises of an Arduino Nano or compatible, a standard 16x2 LCD display (used in 4-bit mode) with backlight and contrast adjustment, three pushbuttons (DOWN/SET/UP, each with a 100 nF debouncing capacitor across its contact) and an optional PLL lock LED with adequate series resistor. The lock status is also shown on the LCD display.
- LCD backlight control is available if you connect it to its reserved digital pin. Refer to code for pin mappings and change if necessary. Note that the digital pin used for the LCD backlight must support PWM. Currently pin 6 is configured, which is valid for all current Arduino boards. The brightness level settings and timings for the LCD backlight can be adjusted as you wish under "_// LCD brightness and dimmer settings_". 
- Pull-up resistors on SDA/SCL are required. Especially if SDA/SCL runs through RF decoupling circuitry, you may want to use lower values for reliable communication, like 1 or 2 kΩ.
- If used with the DRFS06 it is recommended to supply the controller separately from the TSA5511, as it has been proven that slight voltage fluctuations on the TSA5511 will cause a few ppm XTAL frequency deviation accordingly.

# Use
- Verify the actual XTAL frequency and required band edge frequencies under "_// PLL settings_" and "_// VCO frequency settings_" and change if necessary.
- The TSA5511 charge pump is kept high at all times for the DRFS06 exciter. For other platforms, in function "_checkPll()_" set "_data[0] = PLL_CP_LOW_" if required.
- Hold SET during startup to enable the station name editor. Select characters using UP/DOWN and confirm with SET. The new station name will be stored in EEPROM after the last character has been confirmed and the main screen will be displayed.
- Change frequency using UP/DOWN and confirm with SET. The new frequency will be stored in EEPROM. Changing frequency without confirmation will timeout and return to the main screen unchanged. Holding UP/DOWN will auto-scroll through the frequency band with gradual acceleration. 
- During normal operation (PLL locked) the LCD backlight will dim after a preset time. Double-click on SET toggles this function ON/OFF and stores the setting to EEPROM. Press and hold SET to turn off the backlight completely. The LCD backlight will be restored by pressing any button.
- In case of an I²C communication error alert, verify PLL hardware and SDA/SCL connection and press SET to restart. I²C communication will be retried several times before alerting an error.

# TSA5511 PLL Controller for Arduino

# Description
My very first Arduino project, being a controller for the TSA5511 PLL, initially intended to replace the proprietary controller for the DRFS06 exciter by Dutch RF Shop.
I think this attempt actually succeeded quite well. Did a lot of testing and fault simulations, including I2C verifications using an I2C decoder. The thing now seems to be quite fool-proof :-)

Using a 3,2 MHz crystal, the control ranges from 50 kHz up to 1.638,35 MHz, with a step size of 50 kHz or any multiple thereof. 
The practical lower and upper limits will be tighter, as the TSA5511 is rated from 64 MHz up to 1.300 MHz.
It has a built-in station name editor and the station name and last frequency are written to EEPROM.
Using any other crystal frequency (as far as the TSA5511 may support), valid frequency step size and divisor bytes for the TSA5511 are calculated automatically.
Button debouncing: 100 nF || switch contact

# Use
- Verify the actual crystal frequency and required band edge frequencies under "_// PLL settings_" and "_// VCO frequency settings_" and change if necessary.
- The TSA5511 charge pump is kept high at all times for the DRFS06 exciter. For other platforms, in checkPll() set "data[0] = PLL_CP_LOW" if required.
- Change frequency using UP/DOWN-buttons and confirm with SET-button. The new frequency will be stored in EEPROM.
  Changing frequency without confirmation will timeout and return to main screen unchanged. Holding UP/DOWN will auto-scroll through the frequency band with delayed subsequent acceleration. 
- Hold SET-button during startup to enable the station name editor. Select characters using UP/DOWN-buttons and confirm with SET-button.
  The new station name will be stored in EEPROM after the last character has been confirmed and the main screen will be displayed.
- In case of an I2C communication error alert, verify PLL hardware and SDA/SCL connection and press SET-button to restart. I2C communication will be retried several times before alerting an error.

# Hardware
The hardware comprises of an Arduino Nano, a standard 16x2 LCD-display (used in 4-bit mode) with backlighting and contrast adjustment, three pushbuttons (DOWN-SET-UP, each with a 100 nF debouncing cap across them) and a PLL lock-led with series resistor (the lock status also shown on LCD-display). 
Refer to code for pin-mapping and change if necessary. 
Note that pull-up resistors on SDA/SCL are required. Especially if SDA/SCL runs through RF-decoupling circuitry, you may want to use lower values for reliable communication, like 1 kΩ or 2 kΩ.

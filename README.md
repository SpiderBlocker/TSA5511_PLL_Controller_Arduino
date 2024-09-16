# TSA5511 PLL Controller for Arduino

My very first Arduino project, being a controller for the TSA5511 PLL, initially intended to replace the proprietary controller for the DRFS06 exciter by Dutch RF Shop.

Using a 3,2 MHz crystal, the control ranges from 50 kHz up to 1.638,35 MHz, with a step size of 50 kHz or any multiple thereof. The practical lower and upper limits will be tighter, as the TSA5511 is rated from 64 MHz up to 1.300 MHz. Using any other crystal frequency (as far as the TSA5511 will support), valid frequency step size and divisor bytes for the TSA5511 are calculated automatically.

I think this attempt actually succeeded quite well. Did a lot of testing and fault simulations, including I2C verifications using an I2C decoder. The thing now seems to be quite fool-proof :-)

It has a built-in station name editor, which can be called by holding the set-button during startup. Last frequency and station name are written to EEPROM. Otherwise I think everything else is self-explanatory.

The hardware comprises of an Arduino Nano, a standard 16x2 LCD-display (used in 4-bit mode) with backlighting and contrast adjustment, three pushbuttons (DOWN-SET-UP, each with a 100 nF debouncing cap across them) and a PLL lock-led with series resistor (lock status also shown on LCD-display). Refer to code for pin-mapping and adjust if necessary. Charge pump is kept high at all times for the DRFS06 exciter. For other hardware, in checkPll() set "data[0] = PLL_CP_LOW" if required. Note that pull-up resistors on SDA/SCL are required.

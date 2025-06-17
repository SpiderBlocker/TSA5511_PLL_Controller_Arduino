# TSA5511 PLL Controller for Arduino

# Description
PLL controller for the TSA5511, initially intended to replace the proprietary controller for the DRFS06 exciter by Dutch RF Shop, but it can be used for any other TSA5511 based exciter, operating in a VCO frequency range of 64 MHz up to 1,300 MHz as per specification of the TSA5511.
It features an intuitive menu interface for making various system settings as explained in detail below. All configurable settings are stored in EEPROM and automatically recalled upon restart.

# Hardware
- The hardware comprises an Arduino Nano or compatible, a standard 16x2 LCD display (used in 4-bit mode) with backlight and contrast adjustment, three pushbuttons (DOWN/SET/UP, each with a 470 nF debouncing capacitor across its contact) and an optional PLL lock LED which also acts as a blinking fault indicator. The lock status is also shown on the LCD display.
- LCD backlight control is available if you connect it to its reserved digital pin. Refer to code for pin mappings and change if necessary. Note that the digital pin used for the LCD backlight must support PWM. Currently pin 6 is configured, which is valid for all current Arduino boards.
- Pull-up resistors on SDA/SCL are required. Especially if SDA/SCL runs through RF decoupling circuitry, you may want to use lower values for reliable communication, like 1 or 2 kΩ.
- If used with the DRFS06 it is recommended to supply the controller separately from the TSA5511, as slight voltage fluctuations on the TSA5511 supply rail may cause a few ppm XTAL frequency deviation.

# Usage
- Double-clicking SET opens the main configuration menu, in which system settings can be made as follows:
```text

    ■ VCO SUBMENU      => • FREQUENCY BAND  > Various predefined frequency bands are available for selection. The last operating frequency will be stored in EEPROM
                                              for each VCO frequency band separately.
                          • FREQ. PRECISION > This sets the decimal precision at which the VCO frequency can be set and will be displayed. Note that if it is set to
                                              a lower precision than required for the current VCO frequency, confirmation will result in the new VCO frequency to be
                                              rounded and set to the nearest possible value. Since the minimum VCO frequency step size is inherently dependent on the
                                              PLL crystal frequency (25 kHz @ 1.6 MHz and 50 kHz @ 3.2 MHz), the actual frequency precision will default to the
                                              highest possible resolution automatically, i.e. 3 decimals at 1.6 MHz and 2 decimals at 3.2 MHz respectively. This can
                                              be changed to a lower value if so desired. Refer to additional explanation below at PLL SUBMENU > PLL XTAL FREQ.
                          • EXIT SUBMENU    > Returns to the main menu.

    ■ PLL SUBMENU      => • I2C ADDRESS     > This allows selecting the appropriate I²C address based on the actual hardware configuration of the TSA5511.
                                              By DC-biasing pin P3 of the TSA5511, the I²C address can be configured to 0x60, 0x62, or 0x63, while 0x61 is always valid
                                              regardless of the hardware configuration. By default the I²C address is set to 0x61.
                                              Upon confirming a new I²C address, communication is automatically verified. If verification fails, the last known working
                                              I²C address will be restored automatically. In the unlikely event that an incompatible I²C address is stored and can't be
                                              reconfigured through the menu, restart the system while holding SET to restore the default fail-safe I²C address (0x61). 
                          • CHARGE PUMP     > This sets the PLL charge current in locked state (high or low, resp. 220 µA or 50 µA). It should be set to high for the
                                              DRFS06 exciter. For other platforms, set to low if required.
                          • XTAL FREQ.      > This setting must match the actual PLL crystal frequency. The default PLL crystal frequency is 3.2 MHz, resulting in a
                                              theoretical upper VCO frequency of 1,638.35 MHz. If a PLL crystal frequency of 1.6 MHz is used, the theoretical upper
                                              VCO frequency will be 819.175 MHz, in which case any upper band limit exceeding this maximum value will be automatically
                                              adjusted accordingly.
                                              Note that compatibility of the TSA5511 with a 1.6 MHz crystal frequency is not officially supported; however, it has been
                                              empirically confirmed to work.
                          • OUTPUT PORTS    > This setting controls the state of the output ports P2 and P5 in locked state. These ports can be used to control the
                                              RF output and an external lock indicator respectively. By default P2/P5 are set to high during locked state.
                          • EXIT SUBMENMU   > Returns to the main menu.

    ■ STATION NAME     => This sets the radio station name that is shown in quiescent condition (locked state). Select characters using UP/DOWN and confirm each
                          character with SET. Auto-scroll is available when holding DOWN/UP or SET.

    ■ BACKLIGHT DIMMER => This toggles the automatic LCD backlight dimmer function (ON or OFF).

    ■ EXIT MENU        => • save changes    > Stores any changes to EEPROM and returns to the main interface.
                          • discard         > Discards any changes and returns to the main interface.
                          • cancel          > Returns to the first index of the main menu.

```

- The menu interface will timeout after a preset period of inactivity, discarding any unsaved changes and returning to the main screen — except when the exit menu is active, which requires explicit user confirmation.
- Change VCO frequency using UP/DOWN and confirm with SET. Changing the VCO frequency without confirmation will time out and return to the main screen unchanged. Holding UP/DOWN will auto-scroll through the VCO frequency band with gradual acceleration.
- If enabled, the LCD backlight will dim after a preset period in quiescent condition (locked state). Press and hold SET to turn off the backlight completely. The LCD backlight will be restored by pressing any button.
- In case of an I²C communication error alert, verify PLL hardware and SDA/SCL connection and press SET to restart. I²C communication will be retried several times before alerting an error.

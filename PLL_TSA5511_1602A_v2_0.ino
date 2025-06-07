/*
DESCRIPTION
  PLL controller for the TSA5511, initially intended to replace the proprietary controller for the DRFS06 exciter by Dutch RF Shop, but it can be used for any other
  TSA5511 based exciter, operating in a VCO frequency range of 64 MHz up to 1,300 MHz as per specification of the TSA5511.
  It features an intuitive menu interface for making various system settings as explained in detail below.
  All configurable settings are stored in EEPROM and automatically recalled upon restart.

HARDWARE
  • The hardware comprises an Arduino Nano or compatible, a standard 16x2 LCD display (used in 4-bit mode) with backlight and contrast adjustment, three pushbuttons
    (DOWN/SET/UP, each with a 470 nF debouncing capacitor across its contact) and an optional PLL lock LED which also acts as a blinking fault indicator.
    The lock status is also shown on the LCD display.
  • LCD backlight control is available if you connect it to its reserved digital pin. Refer to code for pin mappings and change if necessary. Note that the digital
    pin used for the LCD backlight must support PWM. Currently pin 6 is configured, which is valid for all current Arduino boards.
  • Pull-up resistors on SDA/SCL are required. Especially if SDA/SCL runs through RF-decoupling circuitry, you may want to use lower values for reliable communication,
    like 1 or 2 kΩ.
  • If used with the DRFS06 it is recommended to supply the controller separately from the TSA5511, as it has been proven that slight voltage fluctuations on the
    TSA5511 supply rail will cause a few ppm XTAL frequency deviation accordingly.

USAGE
  • Double-clicking SET opens the main configuration menu, in which system settings can be made as follows:
  
    ■ VCO SUBMENU      => • VCO FREQ. BAND  >  Various predefined frequency bands are available for selection. The last operating frequency will be stored in EEPROM
                                               for each VCO frequency band separately.
                          • FREQ. PRECISION >  This sets the decimal precision at which the VCO frequency can be set and will be displayed. Note that if it is set to
                                               a lower precision than required for the current VCO frequency, confirmation will result in the new VCO frequency to be
                                               rounded and set to the nearest possible value. Since the minimum VCO frequency step size is inherently dependent on the
                                               PLL crystal frequency (25 kHz @ 1.6 MHz and 50 kHz at 3.2 kHz), the actual frequency precision will default to the
                                               highest possible resolution automatically, i.e. 3 decimals at 1.6 MHz and 2 decimals at 3.2 MHz respectively. This can
                                               be changed to a lower value if so desired. Refer to additional explanation at PLL SUBMENU > PLL XTAL FREQ below.
                          • EXIT SUBMENMU   >  Returns to the main menu.

    ■ PLL SUBMENU      => • PLL CHARGE PUMP >  This toggles the PLL charge current in locked state (HIGH or LOW, resp. 220 µA or 50 µA). It should set to HIGH for the
                                               DRFS06 exciter. For other platforms, set to LOW if required.
                          • PLL XTAL FREQ.  >  This setting must match the actual PLL crystal frequency. The default PLL crystal frequency is 3.2 MHz, resulting in a
                                               theoretical upper VCO frequency of 1,638.35 MHz. If a PLL crystal frequency of 1.6 MHz is used, the theoretical upper
                                               VCO frequency will be 819.175 MHz, in which case any upper band limit exceeding the maximum feasible value will be
                                               automatically adjusted accordingly.
                                               Note that compatibility of the TSA5511 with a 1.6 MHz crystal is not officially supported; however, it has been
                                               empirically confirmed to work.
                          • EXIT SUBMENMU   >  Returns to the main menu.

    ■ STATION NAME     => This sets the radio station name that is shown in quiescent condition (PLL locked). Select characters using UP/DOWN and confirm each
                          character with SET. Auto-scroll is available when holding DOWN/UP or SET.

    ■ BACKLIGHT DIMMER => This toggles the automatic LCD backlight dimmer function (ON or OFF).

    ■ EXIT MENU        => • SAVE & EXIT     > Stores any changes to EEPROM and returns to the main interface.
                          • DISCARD & EXIT  > Discards any changes and returns to the main interface.
                          • CANCEL          > Returns to the first index of the main menu.

    The menu interface will timeout after a preset period of inactivity, discarding any unsaved changes and returning to the main screen unchanged — except when the
    exit menu is active, which requires explicit user confirmation.
  • Change VCO frequency using UP/DOWN and confirm with SET. Changing the VCO frequency without confirmation will time out and return to the main screen unchanged.
    Holding UP/DOWN will auto-scroll through the VCO frequency band with gradual acceleration.
  • If enabled, the LCD backlight will dim after a preset period in quiescent condition (PLL locked). Press and hold SET to turn off the backlight completely.
    The LCD backlight will be restored by pressing any button.
  • In case of an I²C communication error alert, verify PLL hardware and SDA/SCL connection and press SET to restart. I²C communication will be retried several times
    before alerting an error.
*/

// === SYSTEM CONSTANTS ===
// version & metadata
#define description "PLL Control"
#define version "V2.0"
#define credits "(C)2025 Loenie"

// libraries
#include <avr/wdt.h>
#include <EEPROM.h>
#include <Wire.h>
#include <LiquidCrystal.h>

// buttons pin mapping
const uint8_t downButton = 2; // DOWN button to ground
const uint8_t setButton = 3; // SET button to ground
const uint8_t upButton = 4; // UP button to ground

// indicators pin mapping (currently sharing same LED for both indicators)
const uint8_t lockIndicator = 5; // lock indicator LED anode
const uint8_t errIndicator = 5; // error indicator LED anode

// LCD display pin mapping
const uint8_t lcdBacklight = 6; // LCD backlight anode
LiquidCrystal lcd(8, 9, 10, 11, 12, 13); // RS, E, D4, D5, D6, D7

// LCD brightness and dimmer settings
const long backlightOffDelay = 1500; // backlight turn-off delay after holding SET
const long dimMessageTime = 2500; // period to show dimmer status message
const long dimDelay = 2500; // brightness dimmer delay
const uint8_t dimStepDelay = 7; // gradual brightness dimming speed
const uint8_t maxBrightness = 255; // maximum brightness
const uint8_t lowBrightness = 30; // dimmed brightness

// EEPROM storage locations
const uint16_t EEPROM_NAME_ADDR = 10; // station name
const uint16_t EEPROM_DIM_ADDR = 30; // backlight dimmer setting
const uint16_t EEPROM_DECIMAL_ADDR = 40; // VCO frequency decimal precision
const uint16_t EEPROM_CP_ADDR = 50; // charge pump setting
const uint16_t EEPROM_FREQBAND_ADDR = 60;
const uint16_t EEPROM_XTAL_ADDR = 70; // 0 = 3.2 MHz, 1 = 1.6 MHz
const uint16_t EEPROM_BAND_FREQ_BASE_ADDR = 80; // VCO frequency per-band (4 bytes per band)

// I²C settings
const long wireTimeout = 5000; // I²C transmission timeout, preventing I²C bus crash in some cases
const long i2cClock = 32000; // low I²C clock frequency, more robust through SDA/SCL RF decoupling circuitry (min. 31,25 kHz for 16 MHz ATmega328P)
const long i2cHealthCheckInterval = 2000; // I²C health check interval
const uint8_t i2cMaxRetries = 10; // maximum number of I²C transmission retries
const uint8_t i2cRetryDelay = 50; // delay between I²C retries

// PLL settings
const long xtalOptions[] = {1600000, 3200000}; // 0 = 1.6 MHz, 1 = 3.2 MHz
uint8_t xtalFreqIndex = 1; // default = 3.2 MHz
const byte PLL_ADDR = 0x30; // 7-bit I²C address
const byte PLL_ADDR_WRITE = (PLL_ADDR << 1); // 8-bit write address
const byte PLL_ADDR_READ = ((PLL_ADDR << 1) | 1); // 8-bit read address
const byte PLL_CP_LOW = 0x8E; // charge pump low
const byte PLL_CP_HIGH = 0xCE; // charge pump high
const byte PLL_ALL_LOW = 0x00; // all outputs (P0-P7) low
const byte PLL_P2_HIGH = 0x04; // P2 high
const byte PLL_P2_P5_HIGH = 0x24; // P2/P5 high
const uint16_t PLL_XTAL_DIVISOR = 512; // crystal frequency divisor
const uint8_t PLL_PRESCALER_DIVISOR = 8; // prescaler divisor
long getPLLRefFreq() { return (xtalOptions[xtalFreqIndex] / PLL_XTAL_DIVISOR) * PLL_PRESCALER_DIVISOR; } // reference frequency (Hz), also equals the minimum VCO frequency and step size
const uint16_t PLL_DIVISOR_LIMIT = 0x7FFF; // cap divisor to 15 bits; MSB of high byte must remain zero
const uint8_t PLL_LOCK_BIT = 6; // lock flag bit

// VCO frequency band settings
const float freqBands[][2] = {
    {65700000, 74000000}, // OIRT FM broadcast (Russia)
    {76000000, 95000000}, // FM broadcast - Japan
    {87000000, 108000000}, // FM broadcast - ITU R1/R2/R3
    {174000000, 240000000}, // DAB/DVB-T - ITU R1/R2/R3
    {144000000, 148000000}, // 2 m amateur band - ITU R1/R2/R3
    {420000000, 450000000}, // 70 cm amateur band - ITU R1/R2/R3
    {470000000, 862000000}, // UHF band - ITU R1/R2/R3 (with XTAL = 1.6 MHz, upper frequency will be lowered to 819.175 MHz accordingly)
    {64000000, 1300000000} // full range as per TSA5511 specification (with XTAL = 1.6 MHz, upper frequency will be lowered to 819.175 MHz accordingly)
};
const byte defaultFreqBandIndex = 2; // default VCO frequency band
const byte numFreqBands = sizeof(freqBands) / sizeof(freqBands[0]); // total number of selectable VCO frequency bands
byte selectedFreqBandIndex; // currently selected VCO frequency band index

// VCO frequency step size multiplier determination (auto-adjusted for visibility, based on XTAL and display precision)
uint8_t numDecimals = 2; // VCO frequency decimal precision [0, 3]
uint8_t getStepSizeMultiplier() {
    float baseStepSizeMHz = (float)getPLLRefFreq() / 1e6; // base PLL step size in MHz
    float displayStepSize = pow(10, -constrain(numDecimals, 0, 3)); // minimum step visible at selected VCO frequency decimal precision
    return (uint8_t)ceil(displayStepSize / baseStepSizeMHz); // multiplier to ensure that display precision is met
}

// VCO frequency validation
long validateFreq(float frequency, bool alignToStepSize = false) {
    frequency = constrain(frequency, getPLLRefFreq(), PLL_DIVISOR_LIMIT * getPLLRefFreq()); // constrain VCO frequency within valid PLL divisor range
    long step = alignToStepSize ? getPLLRefFreq() * getStepSizeMultiplier() : getPLLRefFreq(); // select step size (base PLL step size or visible step size)
    return round(frequency / step) * step; // round to nearest valid VCO frequency step size
}
long lowerFreq = 0;
long upperFreq = 0;

// station name settings
const uint8_t maxNameLength = 16; // maximum station name length
const uint8_t asciiRange[2] = {32, 127}; // allowed ASCII character range
const char defaultName[maxNameLength + 1] = "Station Name"; // +1 for null terminator
char stationName[maxNameLength + 1]; // +1 for null terminator

// timing parameters
const unsigned long splashDelay = 2500; // duration to show splash screen
const unsigned long initialPressDelay = 1000; // delay before auto-repeat when holding button
const unsigned long initialPressInterval = 80; // auto-repeat interval when holding button
const unsigned long charScrollInterval = 200; // interval between character scroll steps
const unsigned long animInterval = 350; // animation speed during PLL unlock status
const unsigned long freqSetTimeout = 5000; // inactivity timeout in frequency set mode
const unsigned long menuTimeout = 20000; // inactivity timeout in menu navigation mode
const unsigned long errBlinkRate = 250; // error indicator blink interval
unsigned long menuUnlockTime = 0; // time after which menu double-click is allowed
unsigned long menuInactivityTimer = 0; // tracks last interaction time in menu mode
const uint16_t setClickInterval = 300; // double-click detection threshold
const uint8_t buttonTimingTolerance = 50; // minimum time window to suppress unwanted button event
const float pressAccelerationBase = 0.7; // initial auto-repeat acceleration when holding UP/DOWN
const uint8_t pressAccelerationLimit = 7; // maximum auto-repeat acceleration when holding UP/DOWN

// display modes
enum {
    SPLASH_SCREEN,
    MAIN_INTERFACE,
    SET_FREQUENCY_INTERFACE,
    MENU_INTERFACE,
    STATION_NAME_EDITOR,
    PLL_LOCK_STATUS,
    LCD_HIBERNATE,
    I2C_ERROR
};

// === RUNTIME STATE VARIABLES ===
// initialization
bool initialized = false; // true if initialization is completed

// buttons
bool buttonDownState = false; // momentary state of DOWN button
bool buttonSetState = false; // momentary state of SET button
bool buttonUpState = false; // momentary state of UP button
bool buttonDownPressed = false; // static state of DOWN button
bool buttonSetPressed = false; // static state of SET button
bool buttonUpPressed = false; // static state of UP button

// station name editor
bool stationNameEditMode = false; // true if station name editor is active
uint8_t editPosition = 0; // actual cursor position in station name editor
char stationNameBuffer[maxNameLength + 1]; // buffer used during editing

// LCD backlight control
bool backlightDimActive = false; // true if dimmer function is enabled
bool backlightOff = false; // true if backlight is off

// frequency control
long currentFreq; // operational VCO frequency
long targetFreq; // target VCO frequency
bool freqSetMode = false; // true if frequency set mode is active
bool pllLock = false; // true if PLL is locked
bool pllCheckPending = false; // true if new PLL divisor value is pending
bool keepChargePumpHigh = true; // PLL charge pump state (high/low) in locked state

// menu
bool menuMode = false; // true if menu session is active
bool menuEditMode = false; // true when editing a setting
bool menuExitConfirmMode = false; // true if SAVE/DISCARD submenu is active
bool ignoreFirstSetInMenu = false; // skip input until SET is released
uint8_t menuIndex = 0; // 0=decimals, 1=dimmer, 2=save/exit
uint8_t menuLevel = 0; // 0 = main menu, 1 = VCO submenu, 2 = PLL submenu


// === MAIN PROGRAM LOGIC ===
void setup() {
    wdt_disable(); // disable watchdog to prevent reset loop
    setupHardware();
    initialize();
}

void loop() {
    readButtons();
    handleBacklightControl();
    handleFrequencyChange();
    handleMenu();
    checkPLL();
    checkI2C();
}

void setupHardware() {
    // I/O pins
    pinMode(downButton, INPUT_PULLUP);
    pinMode(setButton, INPUT_PULLUP);
    pinMode(upButton, INPUT_PULLUP);
    pinMode(lockIndicator, OUTPUT);
    pinMode(errIndicator, OUTPUT);
    pinMode(lcdBacklight, OUTPUT);

    // I²C
    Wire.begin();
    Wire.setClock(i2cClock);
    Wire.setWireTimeout(wireTimeout, true);

    // display
    lcd.begin(16, 2);
}

void initialize() {
    analogWrite(lcdBacklight, maxBrightness);
    display(SPLASH_SCREEN);
    delay(splashDelay);
    readBandIndex();
    readNumDecimals();
    readChargePump();
    readXTALFreq();
    readStationName();
    readDimmerStatus();
    configurePLL();
    display(MAIN_INTERFACE);
    display(PLL_LOCK_STATUS);
    initialized = true;
}

void readButtons() {
    static unsigned long lastMultiButtonTime = 0;
    unsigned long currentMillis = millis();

    buttonDownState = !digitalRead(downButton);
    buttonSetState  = !digitalRead(setButton);
    buttonUpState   = !digitalRead(upButton);

    if ((buttonDownState + buttonSetState + buttonUpState) > 1 ||
        currentMillis - lastMultiButtonTime < buttonTimingTolerance) {
        buttonDownState = buttonSetState = buttonUpState = false;
        lastMultiButtonTime = currentMillis;
    }
}

// read selected VCO frequency band index from EEPROM and apply new range
void readBandIndex() {
    EEPROM.get(EEPROM_FREQBAND_ADDR, selectedFreqBandIndex);
    if (selectedFreqBandIndex >= numFreqBands) selectedFreqBandIndex = defaultFreqBandIndex; // fallback to default band index
    lowerFreq = validateFreq(freqBands[selectedFreqBandIndex][0], true);
    upperFreq = validateFreq(freqBands[selectedFreqBandIndex][1], true);

    // load last-used frequency for this band if valid
    long storedFreq = readBandFrequency(selectedFreqBandIndex);
    if (storedFreq >= lowerFreq && storedFreq <= upperFreq) {
        targetFreq = validateFreq(storedFreq, true);
    } else {
        targetFreq = lowerFreq;
    }
}

// read last-used VCO frequency for specific band index
long readBandFrequency(byte bandIndex) {
    if (bandIndex < numFreqBands) {
        long freq;
        EEPROM.get(EEPROM_BAND_FREQ_BASE_ADDR + bandIndex * sizeof(long), freq);
        return freq;
    }
    return 0;
}

void readNumDecimals() {
    uint8_t val = EEPROM.read(EEPROM_DECIMAL_ADDR);
    if (val <= 3) {
        numDecimals = val;
    }
}

void readChargePump() {
    keepChargePumpHigh = EEPROM.read(EEPROM_CP_ADDR); // no check for invalid stored value required; any non-zero value reads as true
}

void readXTALFreq() {
    uint8_t val = EEPROM.read(EEPROM_XTAL_ADDR);
    xtalFreqIndex = (val <= 1) ? val : 1; // default index 1
}

void readStationName() {
    EEPROM.get(EEPROM_NAME_ADDR, stationName); // read station name from EEPROM
    stationName[maxNameLength] = '\0'; // ensure null terminator

    // check if EEPROM contains valid null terminator
    bool invalid = false;
    if (EEPROM.read(EEPROM_NAME_ADDR + maxNameLength) != 0x00) {
        invalid = true;
    }

    // check for invalid characters or uninitialized EEPROM content
    for (uint8_t i = 0; i < maxNameLength; i++) {
        char c = stationName[i];
        if (c < asciiRange[0] || c > asciiRange[1] || c == 0xFF) {
            invalid = true;
            break;
        }
    }
    if (invalid) {
        strncpy(stationName, defaultName, maxNameLength); // copy default station name to array
        memset(stationName + strlen(defaultName), 32, maxNameLength - strlen(defaultName)); // fill remainder with spaces
        stationName[maxNameLength] = '\0'; // ensure null terminator again
    }
}

void readDimmerStatus() {
    backlightDimActive = EEPROM.read(EEPROM_DIM_ADDR); // no check for invalid stored value required; any non-zero value reads as true
}

void handleBacklightControl() {
    static bool backlightControlActive = false;

    // no LCD backlight control in frequency set mode, menu mode or if unlocked
    if (freqSetMode || menuMode || !pllLock) {
        backlightControlActive = false;
        return;
    }

    static unsigned long buttonHoldStartTime = 0;
    static unsigned long dimmerTimer = 0;
    static unsigned long lastDimmerUpdateTime = 0;
    unsigned long currentMillis = millis();
    static uint8_t currentBrightness = maxBrightness;

    // enable LCD backlight control after SET release
    if (!backlightControlActive) {
        while (!digitalRead(setButton) && digitalRead(downButton) && digitalRead(upButton)); // skip if multiple buttons are pressed simultaneously
        backlightControlActive = true;
        dimmerTimer = currentMillis;
    }

    // turn off backlight by pressing and holding SET
    if (buttonSetState) {
        if (buttonHoldStartTime == 0) {
            buttonHoldStartTime = currentMillis;
        } else if (currentMillis - buttonHoldStartTime > backlightOffDelay && !backlightOff) {
            analogWrite(lcdBacklight, 0);
            backlightOff = true;
            display(LCD_HIBERNATE);
            while (!digitalRead(setButton)); // ensure that SET is released, to prevent backlight from being turned on again
            return;
        }
    } else {
        buttonHoldStartTime = 0;
    }

    // restore brightness by pressing any button
    if (buttonDownState || buttonSetState || buttonUpState) {
        currentBrightness = maxBrightness;
        analogWrite(lcdBacklight, currentBrightness);

        if (backlightOff) {
            backlightOff = false;
            display(LCD_HIBERNATE);

            // determine which button triggered the wake-up
            uint8_t btn = buttonDownState ? downButton : buttonUpState ? upButton : setButton;
            while (!digitalRead(btn)); // wait for release

            // clear state and press flags to suppress immediate action
            if (btn == downButton) buttonDownState = buttonDownPressed = false;
            else if (btn == upButton) buttonUpState = buttonUpPressed = false;
            else buttonSetState = buttonSetPressed = false;
        }
        dimmerTimer = currentMillis;
    }

    // gradual dimming after timeout
    if (backlightDimActive && (currentMillis - dimmerTimer > dimDelay) && !backlightOff && currentBrightness > lowBrightness) {
        if (currentMillis - lastDimmerUpdateTime >= dimStepDelay) {
            currentBrightness--;
            analogWrite(lcdBacklight, currentBrightness);
            lastDimmerUpdateTime = currentMillis;
        }
    }
}

void handleFrequencyChange() {
    if (menuMode) return;
    static bool timedOut = true;
    if (!(buttonDownState || buttonSetState || buttonUpState) && timedOut) return; // avoid redundant processing
    static unsigned long freqSetInactivityTimer = 0;
    unsigned long currentMillis = millis();

    // change VCO frequency
    auto freqChange = [](int8_t direction) { applyFrequencyChange(true, &targetFreq, direction); };
    handleButtonInput(buttonDownState, buttonDownPressed, -1, freqChange);
    handleButtonInput(buttonUpState, buttonUpPressed, 1, freqChange);
    if (buttonDownState || buttonUpState) {
        freqSetInactivityTimer = currentMillis;
        timedOut = false;
    }
    // confirm VCO frequency
    if (freqSetMode && buttonSetState) {
        applyFrequencyChange(false, &targetFreq, 0);
        timedOut = true;
    } else if (currentMillis - freqSetInactivityTimer > freqSetTimeout) { // inactivity timeout
        freqSetMode = false;
        if (!timedOut) { // restore initial status
            targetFreq = currentFreq;
            display(MAIN_INTERFACE);
            timedOut = true;
        }
    }
}

void applyFrequencyChange(bool freqChange, long* targetFreq, int8_t direction) {
    if (freqChange) {
        // UP/DOWN action
        if (freqSetMode) { *targetFreq += direction * (min(getStepSizeMultiplier() * getPLLRefFreq(), upperFreq - lowerFreq)); } // constrain step size within range
        *targetFreq = (*targetFreq < lowerFreq) ? upperFreq : (*targetFreq > upperFreq) ? lowerFreq : *targetFreq;
        freqSetMode = true;
        display(SET_FREQUENCY_INTERFACE);
    } else {
        // SET action
        unsigned long currentMillis = millis();
        configurePLL();
        freqSetMode = false;
        menuUnlockTime = currentMillis + setClickInterval; // block menu entry immediately after freqSetMode
        display(MAIN_INTERFACE);
    }
}

void handleMenu() {
    static unsigned long lastShortClickTime = 0;
    static unsigned long clickStartTime = 0;
    static bool menuTimedOut = true;
    static uint8_t prevMainMenuIndex = 0; // store previous main menu index when entering submenu
    unsigned long currentMillis = millis();

    // skip input until SET is released
    if (ignoreFirstSetInMenu) {
        if (!buttonSetState) {
            ignoreFirstSetInMenu = false;
        }
        return;
    }

    if (!menuMode) {
        // block menu if station name editor is active or still locked after exit
        if (stationNameEditMode || currentMillis < menuUnlockTime) return;

        // detect SET double-click
        if (buttonSetState && clickStartTime == 0) {
            clickStartTime = currentMillis;
        }
        if (!buttonSetState && clickStartTime != 0) {
            unsigned long clickDuration = currentMillis - clickStartTime;
            clickStartTime = 0;
            if (clickDuration < setClickInterval) {
                bool validDoubleClick = currentMillis - lastShortClickTime < setClickInterval &&
                                        currentMillis - lastShortClickTime >= buttonTimingTolerance;

                lastShortClickTime = currentMillis;

                if (validDoubleClick) { // valid double-click detected: enter main menu
                    menuMode = true;
                    menuLevel = 0;
                    menuIndex = 0;
                    menuEditMode = false;
                    menuExitConfirmMode = false;
                    stationNameEditMode = false;
                    strncpy(stationNameBuffer, stationName, maxNameLength); // fill buffer with current station name as safe default
                    stationNameBuffer[maxNameLength] = '\0';
                    display(MENU_INTERFACE);
                    ignoreFirstSetInMenu = true; // skip until SET release
                    menuTimedOut = false;
                    menuInactivityTimer = currentMillis;
                    return;
                }
            }
        }
        return;
    }

    if (stationNameEditMode) {
        // station name editor active as submenu
        handleStationNameEdit();
        return;
    }

    // discard changes and exit menu on timeout if inactive (excluding exit menu)
    if (!menuExitConfirmMode && currentMillis - menuInactivityTimer > menuTimeout) {
        readDimmerStatus();
        readNumDecimals();
        readChargePump();
        readXTALFreq();
        targetFreq = validateFreq(targetFreq, true);
        menuExitConfirmMode = false;
        menuMode = false;
        menuLevel = 0;
        display(MAIN_INTERFACE);
        display(PLL_LOCK_STATUS);
        menuTimedOut = true;
        return;
    }

    if (menuExitConfirmMode) {
        handleButtonInput(buttonDownState, buttonDownPressed, -1, [](int8_t) {
            if (menuIndex > 0) {
                menuIndex--;
                display(MENU_INTERFACE);
            }
        });
        handleButtonInput(buttonUpState, buttonUpPressed, 1, [](int8_t) {
            if (menuIndex < 2) {
                menuIndex++;
                display(MENU_INTERFACE);
            }
        });
        handleButtonInput(buttonSetState, buttonSetPressed, 0, [](int8_t) {
            if (menuIndex == 0) { // save changes
                if (!stationNameEditMode) {
                    strncpy(stationName, stationNameBuffer, maxNameLength); // copy new station name from edit buffer
                    stationName[maxNameLength] = '\0'; // ensure null terminator
                    storeStationName();
                }
                storeBandIndex();
                storeNumDecimals();
                storeDimmerStatus();
                storeChargePump();
                storeXTALFreq();
                lowerFreq = validateFreq(freqBands[selectedFreqBandIndex][0], true);
                upperFreq = validateFreq(freqBands[selectedFreqBandIndex][1], true);
                long storedFreq = readBandFrequency(selectedFreqBandIndex);
                targetFreq = (storedFreq >= lowerFreq && storedFreq <= upperFreq) ? validateFreq(storedFreq, true) : lowerFreq;
                configurePLL(); // apply updated charge pump and XTAL setting
                menuExitConfirmMode = false;
                menuMode = false;
                menuLevel = 0;
                display(MAIN_INTERFACE);
                display(PLL_LOCK_STATUS);
            } else if (menuIndex == 1) { // discard changes
                readNumDecimals();
                readBandIndex(); // restore original band setting
                readDimmerStatus();
                readChargePump();
                readXTALFreq();
                readStationName();
                targetFreq = validateFreq(targetFreq, true);
                menuExitConfirmMode = false;
                menuMode = false;
                menuLevel = 0;
                display(MAIN_INTERFACE);
                display(PLL_LOCK_STATUS);
            } else if (menuIndex == 2) { // cancel and return to main menu
                menuExitConfirmMode = false;
                menuIndex = 0; // return to first main menu option
                display(MENU_INTERFACE);
            }
        });
        menuInactivityTimer = currentMillis;
        menuTimedOut = false;
        return;
    }

    // menu navigation mode: use UP/DOWN to select option, SET to confirm
    uint8_t maxItems = (menuLevel == 0) ? 4 : 3;

    if (!menuEditMode) {
        if (buttonDownState && !buttonDownPressed) {
            buttonDownPressed = true;
            if (menuIndex > 0) menuIndex--;
            display(MENU_INTERFACE);
        } else if (!buttonDownState) {
            buttonDownPressed = false;
        }
        if (buttonUpState && !buttonUpPressed) {
            buttonUpPressed = true;
            if (menuIndex < ((menuLevel == 0) ? 4 : 2)) menuIndex++;
            display(MENU_INTERFACE);
        } else if (!buttonUpState) {
            buttonUpPressed = false;
        }
        handleButtonInput(buttonSetState, buttonSetPressed, 0, [](int8_t) {
            if (menuLevel == 0) {
                switch (menuIndex) {
                    case 0: prevMainMenuIndex = menuIndex; menuLevel = 1; menuIndex = 0; break; // enter VCO SETTINGS
                    case 1: prevMainMenuIndex = menuIndex; menuLevel = 2; menuIndex = 0; break; // enter PLL SETTINGS
                    case 2: // start station name editor
                        editPosition = 0;
                        stationNameEditMode = true;
                        strncpy(stationNameBuffer, stationName, maxNameLength);
                        stationNameBuffer[maxNameLength] = '\0';
                        display(STATION_NAME_EDITOR);
                        while (!digitalRead(setButton)); // wait for SET release to prevent immediate auto-repeat
                        return;
                    case 3: menuEditMode = true; break; // backlight dimmer
                    case 4: menuExitConfirmMode = true; menuIndex = 0; break; // enter exit menu
                }
                display(MENU_INTERFACE);
            } else {
                if (menuIndex == 2) {
                    menuLevel = 0; // return option
                    menuIndex = prevMainMenuIndex; // restore previous menu index
                    display(MENU_INTERFACE);
                } else {
                    menuEditMode = true;
                    display(MENU_INTERFACE);
                }
            }
        });
    } else {
        // edit mode: adjust setting
        if (menuLevel == 0 && menuIndex == 3) {
            // backlight dimmer toggle
            handleButtonInput(buttonDownState, buttonDownPressed, 0, [](int8_t) {
                backlightDimActive = !backlightDimActive;
                display(MENU_INTERFACE);
            });
            handleButtonInput(buttonUpState, buttonUpPressed, 0, [](int8_t) {
                backlightDimActive = !backlightDimActive;
                display(MENU_INTERFACE);
            });
        }
        if (menuLevel == 1) {
            switch (menuIndex) {
                case 0: // VCO frequency band selection
                    handleButtonInput(buttonDownState, buttonDownPressed, -1, [](int8_t) {
                        if (selectedFreqBandIndex > 0) selectedFreqBandIndex--;
                        display(MENU_INTERFACE);
                    });
                    handleButtonInput(buttonUpState, buttonUpPressed, 1, [](int8_t) {
                        if (selectedFreqBandIndex < numFreqBands - 1) selectedFreqBandIndex++;
                        display(MENU_INTERFACE);
                    });
                    break;
                case 1: // VCO frequency decimal precision
                    handleButtonInput(buttonDownState, buttonDownPressed, -1, [](int8_t) {
                        numDecimals = constrain(numDecimals - 1, 0, (xtalFreqIndex == 0 ? 3 : 2));
                        display(MENU_INTERFACE);
                    });
                    handleButtonInput(buttonUpState, buttonUpPressed, 1, [](int8_t) {
                        numDecimals = constrain(numDecimals + 1, 0, (xtalFreqIndex == 0 ? 3 : 2));
                        display(MENU_INTERFACE);
                    });
                    break;
            }
        } else if (menuLevel == 2) {
            switch (menuIndex) {
                case 0: // PLL charge pump setting
                    handleButtonInput(buttonDownState, buttonDownPressed, 0, [](int8_t) {
                        keepChargePumpHigh = !keepChargePumpHigh;
                        display(MENU_INTERFACE);
                    });
                    handleButtonInput(buttonUpState, buttonUpPressed, 0, [](int8_t) {
                        keepChargePumpHigh = !keepChargePumpHigh;
                        display(MENU_INTERFACE);
                    });
                    break;
                case 1: // XTAL frequency selection
                    handleButtonInput(buttonDownState, buttonDownPressed, -1, [](int8_t) {
                        xtalFreqIndex = 0;
                        if (numDecimals < 3) numDecimals = 3; // update visible precision if user switches to 1.6 MHz
                        display(MENU_INTERFACE);
                    });
                    handleButtonInput(buttonUpState, buttonUpPressed, 1, [](int8_t) {
                        xtalFreqIndex = 1;
                        numDecimals = 2; // enforce 2 decimals when switching to 3.2 MHz
                        display(MENU_INTERFACE);
                    });

                    break;
            }
        }

        // SET confirms and returns to menu navigation mode
        handleButtonInput(buttonSetState, buttonSetPressed, 0, [](int8_t) {
            menuEditMode = false;
            display(MENU_INTERFACE);
        });
    }

    // reset timeout timer on any valid input
    if (buttonDownState || buttonUpState || buttonSetState) {
        menuInactivityTimer = currentMillis;
        menuTimedOut = false;
    }
}

void storeBandIndex() {
    EEPROM.put(EEPROM_FREQBAND_ADDR, selectedFreqBandIndex);
}

void storeBandFrequency(byte bandIndex, long frequency) {
    if (initialized && bandIndex < numFreqBands) { // avoid unnecessary write operation during startup to protect EEPROM
        EEPROM.put(EEPROM_BAND_FREQ_BASE_ADDR + bandIndex * sizeof(long), frequency);
    }
}

void storeNumDecimals() {
    EEPROM.update(EEPROM_DECIMAL_ADDR, numDecimals);
}

void storeChargePump() {
    EEPROM.update(EEPROM_CP_ADDR, keepChargePumpHigh);
}

void storeXTALFreq() {
    EEPROM.update(EEPROM_XTAL_ADDR, xtalFreqIndex);
}

void storeDimmerStatus() {
    EEPROM.update(EEPROM_DIM_ADDR, backlightDimActive);
}

void handleStationNameEdit() {
    if (!stationNameEditMode) return;

    // select character
    auto editCharacter = [](int8_t direction) { applyStationNameEdit(true, direction); };
    handleButtonInput(buttonDownState, buttonDownPressed, -1, editCharacter);
    handleButtonInput(buttonUpState, buttonUpPressed, 1, editCharacter);
    
    // confirm selection
    handleButtonInput(buttonSetState, buttonSetPressed, 0, [](int8_t direction) {
        applyStationNameEdit(false, direction);
    });
}

void applyStationNameEdit(bool editCharacter, int8_t direction) {
    if (editCharacter) {
        // UP/DOWN action
        uint8_t range = asciiRange[1] - asciiRange[0] + 1;
        stationNameBuffer[editPosition] = (stationNameBuffer[editPosition] - asciiRange[0] + direction + range) % range + asciiRange[0];
        display(STATION_NAME_EDITOR);
    } else {
        // SET action
        unsigned long currentMillis = millis();
        if (editPosition >= maxNameLength - 1) { // finish station name edit when last character is confirmed
            stationNameEditMode = false; // exit station name editor
            editPosition = 0; // reset edit position after confirmation
            menuEditMode = false; // reset edit flag after editor exit
            menuExitConfirmMode = false; // reset exit submenu state
            stationNameBuffer[maxNameLength] = '\0'; // ensure null terminator
            strncpy(stationName, stationNameBuffer, maxNameLength); // update working copy for menu re-entry
            stationName[maxNameLength] = '\0';
            menuInactivityTimer = currentMillis; // reset menu timeout
            display(MENU_INTERFACE); // show menu again
            menuUnlockTime = currentMillis + setClickInterval; // block menu entry immediately after exit
            ignoreFirstSetInMenu = true; // wait for SET release before accepting new input in menu
        } else {
            editPosition++; // move to next character
            display(STATION_NAME_EDITOR);
        }
    }
}

void storeStationName() {
    // ensure null terminator
    stationName[maxNameLength] = '\0';

    // validate characters before storing
    bool valid = true;
    for (uint8_t i = 0; i < maxNameLength; i++) {
        char c = stationName[i];
        if (c < asciiRange[0] || c > asciiRange[1] || c == 0xFF) {
            valid = false;
            break;
        }
    }

    // fall back to default name if invalid
    if (!valid) {
        strncpy(stationName, defaultName, maxNameLength); // copy default station name
        memset(stationName + strlen(defaultName), 32, maxNameLength - strlen(defaultName)); // pad with spaces
        stationName[maxNameLength] = '\0';
    }
    char storedStationName[maxNameLength + 1]; // +1 for null terminator
    EEPROM.get(EEPROM_NAME_ADDR, storedStationName);

    // avoid unnecessary write operations to protect EEPROM
    if (strncmp(stationName, storedStationName, maxNameLength + 1) != 0) {
        EEPROM.put(EEPROM_NAME_ADDR, stationName);
    }
}

void handleButtonInput(bool buttonState, bool& buttonPressed, int8_t direction, void (*action)(int8_t)) {
    static unsigned long pressStartTime = 0, lastPressTime = 0;
    static unsigned long lastCharEditTime = 0; // for station name editor scroll speed
    unsigned long currentMillis = millis();
    unsigned long totalPressTime = currentMillis - pressStartTime;
    unsigned long fastPressInterval = initialPressInterval;

    if (buttonState) {
        // change on first button press, or - when holding button - continuously after initialPressDelay
        if (!buttonPressed) {
            pressStartTime = currentMillis;
            lastPressTime = currentMillis;
            lastCharEditTime = currentMillis;
            buttonPressed = true;
            action(direction);
        } else {
            if (totalPressTime >= initialPressDelay && freqSetMode) {
                // gradual acceleration in frequency SET mode
                long postDelayTime = totalPressTime - initialPressDelay;
                fastPressInterval = max(
                    initialPressInterval / (pressAccelerationBase + (postDelayTime / initialPressDelay)),
                    initialPressInterval / pressAccelerationLimit
                );
            }

            // auto-repeat action after initialPressDelay
            if (totalPressTime >= initialPressDelay &&
                currentMillis - lastPressTime >= fastPressInterval &&
                (!menuMode || stationNameEditMode)) {

                // fixed scroll speed in station name editor
                if (!stationNameEditMode || currentMillis - lastCharEditTime >= charScrollInterval) {
                    lastPressTime = currentMillis;
                    lastCharEditTime = currentMillis;
                    action(direction);
                }
            }
        }
    } else {
        buttonPressed = false; // reset on button release
    }
}

void configurePLL() {
    if (targetFreq == currentFreq && !menuMode) return;
    long divisor = (targetFreq / getPLLRefFreq()); // calculate divisor
    byte data[4]; // full programming (TSA 5511 datasheet, table 1)
    data[0] = (divisor & 0xFF00) >> 8; // extract high divisor byte
    data[1] = divisor & 0x00FF; // extract low divisor byte
    data[2] = PLL_CP_HIGH; // set charge pump
    data[3] = PLL_ALL_LOW; // set output ports
    if (attemptI2C(false, PLL_ADDR_WRITE, data, 4)) { // I²C transmission with i2cMaxRetries
        storeBandFrequency(selectedFreqBandIndex, targetFreq);
        currentFreq = targetFreq;
        pllCheckPending = true;
    }
}

void checkPLL() {
    if (!pllCheckPending || menuMode) return;
    byte readByte;
    if (attemptI2C(true, PLL_ADDR_READ, &readByte, 1)) {
        pllLock = (readByte >> PLL_LOCK_BIT) & 0x01;
        display(PLL_LOCK_STATUS);
        if (pllLock) {
            byte data[2]; // partial programming, starting with byte 4 (TSA 5511 datasheet, table 1)
            data[0] = keepChargePumpHigh ? PLL_CP_HIGH : PLL_CP_LOW; // set charge pump
            data[1] = PLL_P2_P5_HIGH; // set output ports
            if (attemptI2C(false, PLL_ADDR_WRITE, data, 2)) { // I²C transmission with i2cMaxRetries
                pllCheckPending = false; // PLL lock state not monitored after lock, as PLL lock flag may flicker due to FM modulation
            }
            digitalWrite(lockIndicator, HIGH);
        } else {
            digitalWrite(lockIndicator, LOW);
        }
    }
}

void checkI2C() {
    if (menuMode) { return; } // no interference allowed with menu session
    static unsigned long lastI2cCheckTime = 0;
    unsigned long currentMillis = millis();

    if (currentMillis - lastI2cCheckTime >= i2cHealthCheckInterval) {
        lastI2cCheckTime = currentMillis;
        attemptI2C(false, PLL_ADDR_WRITE, nullptr, 0); // I²C bus is operational if transmission succeeds
    }
}

bool attemptI2C(bool isRead, byte address, byte* buffer, byte length) {
    for (uint8_t i = i2cMaxRetries; i > 0; i--) {
        if (isRead) {
            if (Wire.requestFrom(address, length) == length) {
                for (byte j = 0; j < length; j++) buffer[j] = Wire.read();
                return true;
            }
        } else {
            Wire.beginTransmission(address);
            if (length > 0) Wire.write(buffer, length);
            if (Wire.endTransmission() == 0) return true;
        }
        delay(i2cRetryDelay);
    }
    i2cErrHandler();
    return false;
}

void i2cErrHandler() {
    display(I2C_ERROR);
    digitalWrite(lockIndicator, LOW);
    while (true) {
        while (!digitalRead(setButton)) { blinkLed(errIndicator, errBlinkRate); } // ensure that SET is released, to prevent premature reset
        do { blinkLed(errIndicator, errBlinkRate); } while (digitalRead(setButton)); // reset on SET release, to prevent starting station name editor on restart
        while (!digitalRead(setButton)) blinkLed(errIndicator, errBlinkRate); // continue blinking error indicator while SET is pressed
        digitalWrite(errIndicator, LOW); // reset error indicator
        wdt_enable(WDTO_15MS); // enable watchdog with 15ms timeout
        while (true) {} // wait for watchdog to expire and reset system
    }
}

void blinkLed(uint8_t ledPin, unsigned long interval) {
    static unsigned long lastBlinkTime = 0;
    unsigned long currentMillis = millis();
    static bool ledState = false;

    if (currentMillis - lastBlinkTime >= interval) {
        lastBlinkTime = currentMillis;
        ledState = !ledState;
        digitalWrite(ledPin, ledState);
    }
}

void display(uint8_t mode) {
    // right-align VCO frequency with dynamic precision, and suffix on LCD display
    auto printFreq = [](uint8_t row) {
        uint8_t validatedNumDecimals = constrain(numDecimals, 0, 3); // constrain number of decimals to valid range [0, 3]
        const char* suffix = (validatedNumDecimals == 3) ? "MHz" : " MHz"; // omit leading space if space is tight to fit 16 columns
        uint8_t col = 16 - ((validatedNumDecimals == 3 ? 9 : 8) + strlen(suffix)); // compute starting column for right-alignment
        char buffer[10]; // buffer for max 9 chars and null terminator
        dtostrf(targetFreq / 1000000.0, (validatedNumDecimals == 3 ? 9 : 8), validatedNumDecimals, buffer); // format VCO frequency as right-aligned string
        lcd.setCursor(col, row);
        lcd.print(buffer);
        lcd.print(suffix);
    };

    switch(mode) {
        case SPLASH_SCREEN:
            lcd.clear();
            lcd.print(description);
            lcd.print(" ");
            lcd.print(version);
            lcd.setCursor(0, 1);
            lcd.print(credits);
            break;

        case MAIN_INTERFACE:
            printFreq(0);
            lcd.setCursor(0, 1);
            lcd.print(stationName);
            break;

        case SET_FREQUENCY_INTERFACE:
            lcd.setCursor(0, 1);
            lcd.print("SET ");
            printFreq(1);
            break;

        case MENU_INTERFACE:
            lcd.noCursor();
            lcd.clear();
            if (menuExitConfirmMode) {
                lcd.setCursor(0, 0);
                lcd.print("EXIT MENU");
                lcd.setCursor(0, 1);
                switch (menuIndex) {
                    case 0: lcd.print("> save changes"); break;
                    case 1: lcd.print("> discard"); break;
                    case 2: lcd.print("> cancel"); break;
                }
                break;
            }
            if (menuLevel == 0) {
                switch (menuIndex) {
                    case 0:
                        lcd.print("VCO SUBMENU");
                        lcd.setCursor(0, 1);
                        lcd.print("SET to enter");
                        break;
                    case 1:
                        lcd.print("PLL SUBMENU");
                        lcd.setCursor(0, 1);
                        lcd.print("SET to enter");
                        break;
                    case 2:
                        lcd.print("STATION NAME");
                        lcd.setCursor(0, 1);
                        lcd.print("SET to edit");
                        break;
                    case 3:
                        lcd.print("BACKLIGHT DIMMER");
                        lcd.setCursor(0, 1);
                        if (menuEditMode) {
                            lcd.print("> ");
                            lcd.print(backlightDimActive ? "ON " : "OFF");
                        } else {
                            lcd.print("SET to edit");
                        }
                        break;
                    case 4:
                        lcd.print("EXIT MENU");
                        lcd.setCursor(0, 1);
                        lcd.print("SET to confirm");
                        break;
                }
            } else if (menuLevel == 1) {
                switch (menuIndex) {
                    case 0:
                        lcd.print("VCO FREQ. BAND");
                        lcd.setCursor(0, 1);
                        if (menuEditMode) {
                            long low = validateFreq(freqBands[selectedFreqBandIndex][0], true);
                            long high = validateFreq(freqBands[selectedFreqBandIndex][1], true);
                            lcd.print(low / 1000000);
                            lcd.print("-");
                            lcd.print(high / 1000000);
                            lcd.print(" MHz");
                        } else {
                            lcd.print("SET to edit");
                        }
                        break;
                    case 1:
                        lcd.print("FREQ. PRECISION");
                        lcd.setCursor(0, 1);
                        if (menuEditMode) {
                            lcd.print("> ");
                            lcd.print(numDecimals);
                            lcd.print(numDecimals == 1 ? " decimal" : " decimals");
                        } else {
                            lcd.print("SET to edit");
                        }
                        break;
                    case 2:
                        lcd.print("EXIT SUBMENU");
                        lcd.setCursor(0, 1);
                        lcd.print("SET to confirm");
                        break;
                }
            } else if (menuLevel == 2) {
                switch (menuIndex) {
                    case 0:
                        lcd.print("PLL CHARGE PUMP");
                        lcd.setCursor(0, 1);
                        if (menuEditMode) {
                            lcd.print("> ");
                            lcd.print(keepChargePumpHigh ? "HIGH" : "LOW ");
                        } else {
                            lcd.print("SET to edit");
                        }
                        break;
                    case 1:
                        lcd.print("PLL XTAL FREQ.");
                        lcd.setCursor(0, 1);
                        if (menuEditMode) {
                            lcd.print("> ");
                            lcd.print(xtalFreqIndex == 0 ? "1.6 MHz" : "3.2 MHz");
                        } else {
                            lcd.print("SET to edit");
                        }
                        break;
                    case 2:
                        lcd.print("EXIT SUBMENU");
                        lcd.setCursor(0, 1);
                        lcd.print("SET to confirm");
                        break;
                }
            }
            break;

        case STATION_NAME_EDITOR:
            lcd.clear();
            lcd.print("STATION NAME");
            lcd.setCursor(0, 1);
            lcd.print(stationNameBuffer);
            lcd.setCursor(editPosition, 1);
            lcd.cursor();
            break;

        case PLL_LOCK_STATUS:
            lcd.setCursor(0, 0);
            if (pllLock){
                lcd.print("LOCK ");
            } else {
                // animation if unlocked
                static unsigned long lastCharScrollTime = 0;
                unsigned long currentMillis = millis();
                static uint8_t charPos = 0;
                static bool movingRight = true;
                if (currentMillis - lastCharScrollTime >= animInterval) {
                    lastCharScrollTime = currentMillis;
                    lcd.print("     ");
                    lcd.setCursor(charPos, 0);
                    lcd.print(movingRight ? ">>" : "<<");
                    charPos += movingRight ? 1 : -1;
                    if (charPos == 3 || charPos == 0) { movingRight = !movingRight; }
                }
            }
            break;

        case LCD_HIBERNATE:
            backlightOff ? lcd.clear() : (display(MAIN_INTERFACE), display(PLL_LOCK_STATUS));
            break;

        case I2C_ERROR:
            digitalWrite(lcdBacklight, HIGH);
            lcd.clear();
            lcd.print("I2C ERROR");
            lcd.setCursor(0, 1);
            lcd.print("SET to restart");
            break;
    }
} 
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
  • If used with the DRFS06 it is recommended to supply the controller separately from the TSA5511, as slight voltage fluctuations on the TSA5511 supply rail may cause
    a few ppm XTAL frequency deviation.

USAGE
  • Double-clicking SET opens the main configuration menu, in which system settings can be made as follows:
  
    ■ VCO SETTINGS     => • FREQUENCY BAND   > Various predefined frequency bands are available for selection. The last operating frequency will be stored in EEPROM for
                                               each VCO frequency band separately.
                          • PRECISION        > This sets the decimal precision at which the VCO frequency can be set and will be displayed. Note that if it is set to a
                                               lower precision than required for the current VCO frequency, confirmation will result in the new VCO frequency to be
                                               rounded and set to the nearest possible value. Since the minimum VCO frequency step size is inherently dependent on the
                                               PLL crystal frequency (25 kHz @ 1.6 MHz and 50 kHz @ 3.2 MHz), the actual frequency precision will default to the highest
                                               possible resolution automatically, i.e. 3 decimals at 1.6 MHz and 2 decimals at 3.2 MHz respectively. This can be changed
                                               to a lower value if so desired. Refer to additional explanation below at PLL SUBMENU > XTAL FREQ.
                          • EXIT SETTINGS    > Returns to the main menu.

    ■ PLL SETTINGS     => • I2C ADDRESS      > This allows selecting the appropriate I²C address based on the actual hardware configuration of the TSA5511.
                                               By DC-biasing pin P3 of the TSA5511, the I²C address can be configured to 0x60, 0x62, or 0x63, while 0x61 is always valid
                                               regardless of the hardware configuration. By default the I²C address is set to 0x61.
                                               Upon confirming a new I²C address, communication is automatically verified. If verification fails, the last known working
                                               I²C address will be restored automatically. In the unlikely event that an incompatible I²C address is stored and can't be
                                               reconfigured through the menu, restart the system while holding SET to restore the default fail-safe I²C address (0x61). 
                          • CHARGE PUMP      > This sets the PLL charge current in locked state (high or low, resp. 220 µA or 50 µA). It should be set to high for the 
                                               DRFS06 exciter. For other platforms, set to low if required.
                          • XTAL FREQUENCY   > This setting must match the actual PLL crystal frequency. The default PLL crystal frequency is 3.2 MHz, resulting in a
                                               theoretical upper VCO frequency of 1,638.35 MHz. If a PLL crystal frequency of 1.6 MHz is used, the theoretical upper VCO
                                               frequency will be 819.175 MHz, in which case any upper band limit exceeding this maximum value will be automatically
                                               adjusted accordingly.
                                               Note that compatibility of the TSA5511 with a 1.6 MHz crystal frequency is not officially supported; however, it has been
                                               empirically confirmed to work.
                          • OUTPUT PORTS     > This setting controls the state of the output ports P2 and P5 in locked state. These ports can be used to control the RF
                                               output and an external lock indicator respectively. By default P2/P5 are set to high during locked state.
                          • EXIT SETTINGS    > Returns to the main menu.

    ■ GENERAL SETTINGS => • STATION NAME     > This sets the radio station name that is shown in quiescent condition (locked state). Select characters using UP/DOWN and
                                               confirm each character with SET. Auto-scroll is available when holding UP/DOWN or SET.
                          • BACKLIGHT DIMMER > This toggles the automatic LCD backlight dimmer function (on or off).
                          • EXIT SETTINGS    > Returns to the main menu. 

    ■ EXIT MENU        => • save changes     > Stores any changes to EEPROM and returns to the main interface.
                          • discard          > Discards any changes and returns to the main interface.
                          • cancel           > Returns to the first index of the main menu.

  • The menu interface will timeout after a preset period of inactivity, discarding any unsaved changes and returning to the main screen — except when the
    exit menu is active, which requires explicit user confirmation.
  • Change VCO frequency using UP/DOWN and confirm with SET. Holding UP/DOWN will auto-scroll through the VCO frequency band with gradual acceleration. Changing the
    VCO frequency without confirmation will time out and return to the main screen unchanged.
  • If enabled, the LCD backlight will dim after a preset period in quiescent condition (locked state). Press and hold SET to turn off the backlight completely.
    The LCD backlight will be restored by pressing any button.
  • In case of an I²C communication error alert, verify PLL hardware and SDA/SCL connection and press SET to restart. I²C communication will be retried several times
    before alerting an error.
*/

// === LIBRARIES ===
#include <avr/wdt.h>
#include <EEPROM.h>
#include <Wire.h>
#include <LiquidCrystal.h>


// === AVR STARTUP: DISABLE WATCHDOG ===
#if defined(__AVR__)
__attribute__((naked, section(".init3")))
void disableWatchdogEarly() {
    MCUSR = 0; // clear all reset flags
    wdt_disable(); // disable watchdog to prevent reset loop
}
#endif


// === SYSTEM CONSTANTS ===
// version & metadata
#define description "PLL Control"
#define version "V2.2"
#define credits "(C)2025 Loenie"

// buttons pin mapping
const uint8_t downButton = 2; // DOWN button to ground
const uint8_t setButton = 3; // SET button to ground
const uint8_t upButton = 4; // UP button to ground

// indicators pin mapping (currently sharing same LED for both indicators)
const uint8_t lockIndicator = 5; // lock indicator LED anode
const uint8_t errIndicator = 5; // error indicator LED anode

// LCD display pin mapping
const uint8_t lcdBacklight = 6; // LCD backlight LED anode
LiquidCrystal lcd(8, 9, 10, 11, 12, 13); // RS, E, D4, D5, D6, D7

// LCD brightness and dimmer settings
const unsigned long backlightOffDelay = 1500; // backlight turn-off delay after holding SET
const unsigned long dimDelay = 2500; // brightness dimmer delay
const uint8_t dimStepDelay = 7; // gradual brightness dimming speed
const uint8_t maxBrightness = 255; // maximum brightness
const uint8_t lowBrightness = 30; // dimmed brightness

// EEPROM storage locations
const uint16_t EEPROM_NAME_ADDR = 10; // station name
const uint16_t EEPROM_DIM_ADDR = 30; // backlight dimmer setting
const uint16_t EEPROM_DECIMAL_ADDR = 40; // VCO frequency decimal precision
const uint16_t EEPROM_CP_ADDR = 50; // charge pump setting
const uint16_t EEPROM_FREQBAND_ADDR = 60; // actual VCO frequency band index
const uint16_t EEPROM_XTAL_ADDR = 70; // 0 = 1.6 MHz, 1 = 3.2 MHz
const uint16_t EEPROM_I2C_ADDR = 80; // I²C address setting
const uint16_t EEPROM_OUTPUT_PORTS_ADDR = 85; // PLL output port setting
const uint16_t EEPROM_BAND_FREQ_BASE_ADDR = 90; // VCO frequency per band (4 bytes per band)

// I²C settings
const unsigned long wireTimeout = 5000; // I²C transmission timeout, preventing I²C bus crash in some cases
const unsigned long i2cClock = 32000; // low I²C clock frequency, more robust through SDA/SCL RF decoupling circuitry (min. 31,25 kHz for 16 MHz ATmega328P)
const unsigned long i2cHealthCheckInterval = 2000; // I²C health check interval
const uint8_t i2cMaxRetries = 10; // maximum number of I²C transmission retries
const uint8_t i2cRetryDelay = 50; // delay between I²C retries

// PLL settings
const unsigned long PLL_XTAL_OPTIONS[] = { 1600000, 3200000 }; // PLL crystal frequency (0 = 1.6 MHz, 1 = 3.2 MHz)
const byte PLL_ADDRESSES[] = { 0x60, 0x61, 0x62, 0x63 }; // optional I²C addressing by P3-biasing; refer to TSA5511 datasheet, table 4 (0x61 is always valid)
const byte PLL_CP_LOW = 0x8E; // charge pump low
const byte PLL_CP_HIGH = 0xCE; // charge pump high
const byte PLL_ALL_LOW = 0x00; // all output ports (P0-P7) low
const byte PLL_P2_HIGH = 0x04; // P2 high
const byte PLL_P5_HIGH = 0x20; // P5 high
const byte PLL_P2_P5_HIGH = 0x24; // P2/P5 high
const uint16_t PLL_XTAL_DIVISOR = 512; // crystal frequency divisor
const uint16_t PLL_DIVISOR_LIMIT = 0x7FFF; // cap divisor to 15 bits (MSB of high byte must remain zero)
const uint8_t PLL_PRESCALER_DIVISOR = 8; // prescaler divisor
const uint8_t PLL_LOCK_BIT = 6; // lock flag bit
const uint8_t PLL_OUTPUT_PORTS = 3; // default: P2/P5 high

// VCO frequency band settings
const float freqBands[][2] = {
    { 65700000,   74000000 }, // OIRT FM broadcast
    { 76000000,   95000000 }, // FM broadcast - Japan
    { 87000000,  108000000 }, // FM broadcast - ITU R1/R2/R3
    { 144000000, 148000000 }, // 2 m amateur band - ITU R1/R2/R3
    { 420000000, 450000000 }, // 70 cm amateur band - ITU R1/R2/R3
    { 470000000, 862000000 }, // UHF band - ITU R1/R2/R3 (with XTAL = 1.6 MHz, upper frequency will be capped to 819.175 MHz)
    { 64000000, 1300000000 }  // full range as per TSA5511 specification (with XTAL = 1.6 MHz, upper frequency will be capped to 819.175 MHz)
};
const byte defaultFreqBandIndex = 2; // default VCO frequency band
const byte numFreqBands = sizeof(freqBands) / sizeof(freqBands[0]); // total number of selectable VCO frequency bands

// station name settings
const uint8_t maxNameLength = 16; // maximum station name length
const uint8_t asciiRange[2] = { 32, 127 }; // allowed ASCII character range
const char defaultName[maxNameLength + 1] = "Station Name"; // +1 for null terminator

// timing parameters
    // UI delays and timeouts
    const unsigned long splashDelay = 2500; // duration to show splash screen
    const unsigned long animInterval = 250; // animation speed during PLL unlocked status
    const unsigned long errBlinkRate = 250; // error indicator blink interval
    const unsigned long i2cFallbackDelay = 2500; // duration to show I²C address fallback notification
    const unsigned long freqSetTimeout = 5000; // inactivity timeout in frequency set mode
    const unsigned long menuTimeout = 20000; // inactivity timeout in menu navigation mode

    // button input timings and thresholds
    const unsigned long initialPressDelay = 1000; // delay before auto-repeat when holding button
    const unsigned long charScrollInterval = 200; // interval between character scroll steps
    const unsigned long baseRepeatInterval  = 80; // initial repeat interval before acceleration
    const float pressAccelerationBase = 0.7; // initial auto-repeat acceleration when holding UP/DOWN
    const float pressTargetSweepSpeed = 500.0; // target scroll speed (Hz per ms) - high speeds may be limited by loop/display latencies
    const uint16_t setClickInterval = 300; // double-click detection threshold
    const uint8_t buttonTimingTolerance = 50; // minimum time window to suppress unwanted button event

// menu
const uint8_t menuLength[] = { 4, 3, 5, 3 }; // number of selectable menu items per level: 0 = main, 1 = VCO SETTINGS, 2 = PLL SETTINGS, 3 = GENERAL SETTINGS


// === DISPLAY MODES ===
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
// I²C
uint8_t pllAddrIndex = 1; // default index = 0x61
uint8_t tempPllAddrIndex = pllAddrIndex; // temporary index used for I²C address selection in menu edit mode
byte pllAddress = PLL_ADDRESSES[pllAddrIndex]; // 7-bit I²C address (TSA5511 datasheet mislabels 8-bit values as 7-bit)
bool i2cFallbackActive = false; // true if SET is pressed during startup (I²C address safe fallback)

// PLL settings
uint8_t xtalFreqIndex = 1; // PLL crystal frequency index (default = 3.2 MHz)
uint8_t pllOutputPorts = PLL_OUTPUT_PORTS; // 0 = all output ports low, 1 = P2 high, 2 = P5 high, 3 = P2/P5 high
bool chargePump = true; // PLL charge pump state in locked state (default = high)
bool pllLock = false; // true if PLL is locked
bool pllCheckPending = false; // true if new PLL divisor value is pending

// buttons
bool buttonDownState = false; // momentary state of DOWN button
bool buttonSetState = false; // momentary state of SET button
bool buttonUpState = false; // momentary state of UP button
bool buttonDownPressed = false; // static state of DOWN button
bool buttonSetPressed = false; // static state of SET button
bool buttonUpPressed = false; // static state of UP button

// station name editor
char stationName[maxNameLength + 1]; // current station name (null-terminated)
char stationNameBuffer[maxNameLength + 1]; // buffer used during editing (null-terminated)
uint8_t editPosition = 0; // actual cursor position in station name editor
bool stationNameEditMode = false; // true if station name editor is active

// LCD backlight control
bool backlightDimActive = false; // true if dimmer function is enabled
bool backlightOff = false; // true if backlight is off

// frequency control
unsigned long lowerFreq = 0; // lower VCO frequency band edge
unsigned long upperFreq = 0; // upper VCO frequency band edge
long currentFreq; // operational VCO frequency
long targetFreq; // target VCO frequency
uint8_t numDecimals = 2; // VCO frequency decimal precision (default: 2 decimals)
byte freqBandIndex; // currently selected VCO frequency band index
bool freqSetMode = false; // true if frequency set mode is active

// menu
unsigned long menuUnlockTime = 0; // time after which menu double-click is allowed
unsigned long menuInactivityTimer = 0; // tracks last interaction time in menu mode
uint8_t menuLevel = 0; // 0 = main menu, 1 = VCO submenu, 2 = PLL submenu
uint8_t menuIndex = 0; // selected item index within current menu level
bool menuMode = false; // true if menu session is active
bool menuEditMode = false; // true when editing a setting
bool menuExitConfirmMode = false; // true if SAVE/DISCARD submenu is active
bool ignoreFirstSetInMenu = false; // skip input until SET is released


// === FREQUENCY UTILITIES ===
// PLL reference frequency in Hz (equals minimum VCO frequency and step size)
unsigned long getPLLRefFreq() { return (PLL_XTAL_OPTIONS[xtalFreqIndex] / PLL_XTAL_DIVISOR) * PLL_PRESCALER_DIVISOR; }

// Determine step size multiplier for current display precision and XTAL setting
uint8_t getStepSizeMultiplier() {
    float baseStepSizeMHz = (float)getPLLRefFreq() / 1e6; // base PLL step size in MHz
    float displayStepSize = pow(10, -constrain(numDecimals, 0, 3)); // minimum step visible at selected VCO frequency decimal precision
    return (uint8_t)ceil(displayStepSize / baseStepSizeMHz); // multiplier to ensure that display precision is met
}

// VCO frequency validation
unsigned long validateFreq(float frequency, bool alignToStepSize = false) {
    frequency = constrain(frequency, getPLLRefFreq(), (unsigned long)PLL_DIVISOR_LIMIT * getPLLRefFreq()); // constrain VCO frequency within valid PLL divisor range
    unsigned long step = getPLLRefFreq() * (alignToStepSize ? getStepSizeMultiplier() : 1); // select step size (base PLL step size or visible step size)
    return round(frequency / step) * step; // round to nearest valid VCO frequency step size
}


// === MAIN PROGRAM LOGIC ===
void setup() {
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
    restoreI2CDefaults();
    readSettings();
    configurePLL();
    display(MAIN_INTERFACE);
    display(PLL_LOCK_STATUS);
}

// hold SET during startup to restore safe default I²C address (0x61)
void restoreI2CDefaults() {
    if (!digitalRead(setButton)) {
        pllAddrIndex = 1;
        pllAddress = PLL_ADDRESSES[pllAddrIndex];
        storeI2CAddress();
        i2cFallbackActive = true;
        display(I2C_ERROR);
        i2cFallbackActive = false;
        delay(i2cFallbackDelay);
    }
}

void readSettings() {
    readLastFrequencyBand();
    readNumDecimals();
    readI2CAddress();
    readXTALFreq();
    readChargePump();
    readOutputPortsSetting();
    readStationName();
    readDimmerStatus();
}

// read selected VCO frequency band from EEPROM and apply new range
void readLastFrequencyBand() {
    EEPROM.get(EEPROM_FREQBAND_ADDR, freqBandIndex);
    if (freqBandIndex >= numFreqBands) freqBandIndex = defaultFreqBandIndex; // fallback to default band index
    lowerFreq = validateFreq(freqBands[freqBandIndex][0], true);
    upperFreq = validateFreq(freqBands[freqBandIndex][1], true);

    // load last-used frequency for this band if valid, else default to lower band edge
    long storedFreq = readLastBandFrequency(freqBandIndex);
    targetFreq = (storedFreq >= lowerFreq && storedFreq <= upperFreq) ? validateFreq(storedFreq, false) : lowerFreq;
}

// read last-used VCO frequency for selected band index
long readLastBandFrequency(byte bandIndex) {
    if (bandIndex < numFreqBands) {
        long freq;
        EEPROM.get(EEPROM_BAND_FREQ_BASE_ADDR + bandIndex * sizeof(long), freq);
        return freq;
    }
    return 0;
}

void readNumDecimals() {
    uint8_t val = EEPROM.read(EEPROM_DECIMAL_ADDR);
    if (val <= 3) numDecimals = val;
}

void readI2CAddress() {
    uint8_t val = EEPROM.read(EEPROM_I2C_ADDR);
    pllAddrIndex = (val <= 3) ? val : 1;
    pllAddress = PLL_ADDRESSES[pllAddrIndex];
}

void readXTALFreq() {
    uint8_t val = EEPROM.read(EEPROM_XTAL_ADDR);
    xtalFreqIndex = (val <= 1) ? val : 1;
}

void readChargePump() {
    chargePump = EEPROM.read(EEPROM_CP_ADDR); // no check for invalid stored value required; any non-zero value reads as true
}

void readOutputPortsSetting() {
    uint8_t val = EEPROM.read(EEPROM_OUTPUT_PORTS_ADDR);
    pllOutputPorts = (val <= 3) ? val : 3; // default to P2/P5 high
}

void readStationName() {
    EEPROM.get(EEPROM_NAME_ADDR, stationName); // read station name from EEPROM
    bool invalid = stationName[maxNameLength] != '\0'; // check if station name has a valid null terminator
    stationName[maxNameLength] = '\0'; // ensure null terminator regardless

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

void readButtons() {
    static unsigned long lastMultiButtonTime = 0;
    unsigned long currentMillis = millis();

    buttonDownState = !digitalRead(downButton);
    buttonSetState  = !digitalRead(setButton);
    buttonUpState   = !digitalRead(upButton);

    if ((buttonDownState + buttonSetState + buttonUpState) > 1) {
        buttonDownState = buttonSetState = buttonUpState = false;
        lastMultiButtonTime = currentMillis;
    } else if (currentMillis - lastMultiButtonTime < buttonTimingTolerance) {
        buttonDownState = buttonSetState = buttonUpState = false;
    }
}

void handleBacklightControl() {
    static bool backlightControlActive = false;
    static unsigned long buttonHoldStartTime = 0;
    static unsigned long dimmerTimer = 0;
    static unsigned long lastDimmerUpdateTime = 0;
    static uint8_t currentBrightness = maxBrightness;

    // no LCD backlight control in frequency set mode, menu mode or if unlocked
    if (freqSetMode || menuMode || !pllLock) {
        backlightControlActive = false;
        return;
    }

    unsigned long currentMillis = millis();

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
            uint8_t btn = buttonDownState ? downButton : buttonUpState ? upButton : setButton; // determine which button triggered the wake-up
            while (!digitalRead(btn)); // wait for button release
            switch (btn) { // clear button state and press flags to suppress immediate action
                case downButton: buttonDownState = buttonDownPressed = false; break;
                case upButton: buttonUpState = buttonUpPressed = false; break;
                case setButton: buttonSetState = buttonSetPressed = false; break;
            }
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
    static unsigned long freqSetInactivityTimer = 0;
    static bool timedOut = true;
    
    if (menuMode) return;
    if (!(buttonDownState || buttonSetState || buttonUpState) && timedOut) return; // avoid redundant processing

    unsigned long currentMillis = millis();

    // change VCO frequency
    auto changeFreq = [](int8_t direction) { applyFrequencyChange(true, &targetFreq, direction); };
    handleButtonInput(buttonDownState, buttonDownPressed, -1, changeFreq);
    handleButtonInput(buttonUpState, buttonUpPressed, 1, changeFreq);
    if (buttonDownState || buttonUpState) {
        freqSetInactivityTimer = currentMillis;
        timedOut = false;
    }

    // confirm new frequency on SET, or cancel after timeout
    if (freqSetMode && buttonSetState) {
        applyFrequencyChange(false, &targetFreq, 0);
        timedOut = true;
    } else if (currentMillis - freqSetInactivityTimer > freqSetTimeout) {
        freqSetMode = false;
        if (!timedOut) {
            targetFreq = currentFreq;
            display(MAIN_INTERFACE);
            timedOut = true;
        }
    }
}

void applyFrequencyChange(bool adjusting, long* targetFreq, int8_t direction) {
    if (adjusting) {
        // UP/DOWN action
        if (freqSetMode) { *targetFreq += direction * (min(getStepSizeMultiplier() * getPLLRefFreq(), upperFreq - lowerFreq)); } // constrain step size within range
        *targetFreq = (*targetFreq < lowerFreq) ? upperFreq : (*targetFreq > upperFreq) ? lowerFreq : *targetFreq;
        display(SET_FREQUENCY_INTERFACE);
        freqSetMode = true;
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
    static uint8_t prevMainMenuIndex = 0; // store previous main menu index when entering submenu
    unsigned long currentMillis = millis();

    // reset menu timeout on any button press
    if (buttonDownState || buttonUpState || buttonSetState) menuInactivityTimer = currentMillis;

    // discard changes and exit menu on timeout if inactive (excluding exit menu)
    if (menuMode && !menuExitConfirmMode && currentMillis - menuInactivityTimer > menuTimeout) {
        stationNameEditMode = false;
        display(PLL_LOCK_STATUS);
        restoreSettings();
        menuUnlockTime = currentMillis + setClickInterval;
        ignoreFirstSetInMenu = true;
        return;
    }

    // skip input until SET is released
    if (ignoreFirstSetInMenu) {
        if (!buttonSetState) ignoreFirstSetInMenu = false;
        return;
    }

    if (!menuMode) {
        // block menu if station name editor is active or still locked after exit
        if (stationNameEditMode || currentMillis < menuUnlockTime) return;

        // detect SET double-click
        if (buttonSetState && clickStartTime == 0) clickStartTime = currentMillis;
        if (!buttonSetState && clickStartTime != 0) {
            unsigned long clickDuration = currentMillis - clickStartTime;
            clickStartTime = 0;
            if (clickDuration < setClickInterval) {
                bool validDoubleClick =
                    currentMillis - lastShortClickTime < setClickInterval &&
                    currentMillis - lastShortClickTime >= buttonTimingTolerance;
                lastShortClickTime = currentMillis;
                if (validDoubleClick) { // valid double-click detected: enter main menu
                    menuMode = true;
                    menuLevel = 0;
                    menuIndex = 0;
                    menuEditMode = false;
                    menuExitConfirmMode = false;
                    stationNameEditMode = false;
                    tempPllAddrIndex = pllAddrIndex; // initialize temporary I²C address index
                    strncpy(stationNameBuffer, stationName, maxNameLength); // fill buffer with current station name as safe default
                    stationNameBuffer[maxNameLength] = '\0';
                    display(MENU_INTERFACE);
                    ignoreFirstSetInMenu = true; // skip until SET release
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

    // handle user input in the exit confirmation submenu (save / discard / cancel)
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
                uint8_t prevPllAddrIndex = pllAddrIndex;
                if (tempPllAddrIndex != prevPllAddrIndex) {
                    if (attemptI2C(false, PLL_ADDRESSES[tempPllAddrIndex], nullptr, 0)) { // new I²C address is valid
                        pllAddrIndex = tempPllAddrIndex;
                        pllAddress = PLL_ADDRESSES[pllAddrIndex];
                    } else { // new I²C address is invalid
                        pllAddrIndex = prevPllAddrIndex;
                        pllAddress  = PLL_ADDRESSES[pllAddrIndex];
                        i2cFallbackActive = true;
                        display(I2C_ERROR);
                        i2cFallbackActive = false;
                        unsigned long startTime = millis();
                        while (millis() - startTime < i2cFallbackDelay) {
                            blinkLed(errIndicator, errBlinkRate);
                        }
                        digitalWrite(errIndicator, LOW);
                    }
                }
                storeBandIndex();
                storeNumDecimals();
                storeI2CAddress();
                storeXTALFreq();
                storeChargePump();
                storeOutputPortsSetting();
                storeDimmerStatus();
                lowerFreq = validateFreq(freqBands[freqBandIndex][0], true);
                upperFreq = validateFreq(freqBands[freqBandIndex][1], true);
                long storedFreq = readLastBandFrequency(freqBandIndex);
                targetFreq = (storedFreq >= lowerFreq && storedFreq <= upperFreq) ? validateFreq(storedFreq, true) : lowerFreq;
                configurePLL(); // apply updated charge pump and XTAL setting
                restoreSettings();
            } else if (menuIndex == 1) { // discard changes
                display(PLL_LOCK_STATUS);
                restoreSettings();
            } else if (menuIndex == 2) { // cancel and return to main menu
                menuExitConfirmMode = false;
                menuIndex = 0; // return to first main menu option
                display(MENU_INTERFACE);
            }
        });
        return;
    }

    // menu navigation mode: use UP/DOWN to select option, SET to confirm
    uint8_t maxMenuIndex = menuLength[menuLevel];

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
            if (menuIndex < maxMenuIndex - 1) menuIndex++;
            display(MENU_INTERFACE);
        } else if (!buttonUpState) {
            buttonUpPressed = false;
        }
        handleButtonInput(buttonSetState, buttonSetPressed, 0, [](int8_t) {
            if (menuLevel == 0) {
                switch (menuIndex) {
                    case 0: prevMainMenuIndex = menuIndex; menuLevel = 1; menuIndex = 0; break; // enter VCO settings
                    case 1: prevMainMenuIndex = menuIndex; menuLevel = 2; menuIndex = 0; break; // enter PLL settings
                    case 2: prevMainMenuIndex = menuIndex; menuLevel = 3; menuIndex = 0; break; // enter general settings
                    case 3: menuExitConfirmMode = true; menuIndex = 0; break; // exit menu
                }
                display(MENU_INTERFACE);
            } else {
                if ((menuLevel == 1 && menuIndex == 2) ||
                    (menuLevel == 2 && menuIndex == 4) ||
                    (menuLevel == 3 && menuIndex == 2)) {
                    menuLevel = 0; // return option
                    menuIndex = prevMainMenuIndex;
                    display(MENU_INTERFACE);
                } else if (menuLevel == 3 && menuIndex == 0) {
                    // station name editor
                    editPosition = 0;
                    stationNameEditMode = true;
                    strncpy(stationNameBuffer, stationName, maxNameLength);
                    stationNameBuffer[maxNameLength] = '\0';
                    display(STATION_NAME_EDITOR);
                    return;
                } else {
                    menuEditMode = true;
                    display(MENU_INTERFACE);
                }
            }
        });
    } else {
        // menu edit mode: adjust settings
        if (menuLevel == 1) {
            switch (menuIndex) {
                case 0:
                    handleButtonInput(buttonDownState, buttonDownPressed, -1, [](int8_t) {
                        if (freqBandIndex > 0) freqBandIndex--;
                        display(MENU_INTERFACE);
                    });
                    handleButtonInput(buttonUpState, buttonUpPressed, 1, [](int8_t) {
                        if (freqBandIndex < numFreqBands - 1) freqBandIndex++;
                        display(MENU_INTERFACE);
                    });
                    break;
                case 1:
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
                case 0:
                    handleButtonInput(buttonDownState, buttonDownPressed, -1, [](int8_t) {
                        if (tempPllAddrIndex > 0) tempPllAddrIndex--;
                        display(MENU_INTERFACE);
                    });
                    handleButtonInput(buttonUpState, buttonUpPressed, 1, [](int8_t) {
                        if (tempPllAddrIndex < 3) tempPllAddrIndex++;
                        display(MENU_INTERFACE);
                    });
                    break;
                case 1:
                    handleButtonInput(buttonDownState, buttonDownPressed, -1, [](int8_t) {
                        xtalFreqIndex = 0;
                        if (numDecimals < 3) numDecimals = 3;
                        display(MENU_INTERFACE);
                    });
                    handleButtonInput(buttonUpState, buttonUpPressed, 1, [](int8_t) {
                        xtalFreqIndex = 1;
                        numDecimals = 2;
                        display(MENU_INTERFACE);
                    });
                    break;
                case 2:
                    handleButtonInput(buttonDownState, buttonDownPressed, 0, [](int8_t) { toggleSetting(chargePump); });
                    handleButtonInput(buttonUpState, buttonUpPressed, 0, [](int8_t) { toggleSetting(chargePump); });
                    break;
                case 3:
                    handleButtonInput(buttonDownState, buttonDownPressed, -1, [](int8_t) {
                        if (pllOutputPorts > 0) pllOutputPorts--;
                        display(MENU_INTERFACE);
                    });
                    handleButtonInput(buttonUpState, buttonUpPressed, 1, [](int8_t) {
                        if (pllOutputPorts < 3) pllOutputPorts++;
                        display(MENU_INTERFACE);
                    });
                    break;
            }
        } else if (menuLevel == 3) {
            switch (menuIndex) {
                case 1:
                    handleButtonInput(buttonDownState, buttonDownPressed, 0, [](int8_t) { toggleSetting(backlightDimActive); });
                    handleButtonInput(buttonUpState, buttonUpPressed, 0, [](int8_t) { toggleSetting(backlightDimActive); });
                    break;
            }
        }

        // SET confirms and returns to menu navigation mode
        handleButtonInput(buttonSetState, buttonSetPressed, 0, [](int8_t) {
            menuEditMode = false;
            display(MENU_INTERFACE);
        });
    }
}

// toggle setting and update display
void toggleSetting(bool &setting) {
    setting = !setting;
    display(MENU_INTERFACE);
}

// restore all user settings from EEPROM and return to main interface
void restoreSettings() {
    readSettings();
    targetFreq = validateFreq(targetFreq, true);
    menuExitConfirmMode = false;
    menuMode = false;
    menuLevel = 0;
    display(MAIN_INTERFACE);
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
    } else {
        // SET action
        unsigned long currentMillis = millis();
        if (editPosition >= maxNameLength - 1) { // finish station name edit when last character is confirmed
            stationNameEditMode = false; // exit station name editor
            editPosition = 0; // reset edit position after confirmation
            menuEditMode = false; // reset edit flag after editor exit
            menuExitConfirmMode = false; // reset exit submenu state
            stationNameBuffer[maxNameLength] = '\0'; // ensure null terminator in edit buffer
            strncpy(stationName, stationNameBuffer, maxNameLength); // update working copy for menu re-entry
            stationName[maxNameLength] = '\0'; // ensure null terminator in working copy
            display(MENU_INTERFACE); // show menu again
            menuUnlockTime = currentMillis + setClickInterval; // block menu entry immediately after exit
            ignoreFirstSetInMenu = true; // wait for SET release before accepting new input in menu
            return; // prevent further display call
        } else {
            editPosition++; // move to next character
        }
    }
    display(STATION_NAME_EDITOR);
}

void handleButtonInput(bool buttonState, bool& buttonPressed, int8_t direction, void (*action)(int8_t)) {
    static unsigned long pressStartTime = 0, lastPressTime = 0;
    static unsigned long lastCharEditTime = 0;
    unsigned long currentMillis = millis();
    unsigned long totalPressTime = currentMillis - pressStartTime;
    unsigned long fastPressInterval = baseRepeatInterval ;

    if (buttonState) {
        // change on first button press, or - when holding button - continuously after initialPressDelay
        if (!buttonPressed) {
            pressStartTime = currentMillis;
            lastPressTime = currentMillis;
            lastCharEditTime = currentMillis;
            buttonPressed = true;
            action(direction);
        } else {
            // adaptive gradual acceleration in frequency SET mode
            if (freqSetMode && totalPressTime >= initialPressDelay) {
                fastPressInterval =
                    (getStepSizeMultiplier() * getPLLRefFreq() / pressTargetSweepSpeed) /
                    (pressAccelerationBase + ((totalPressTime - initialPressDelay) / initialPressDelay));
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
    data[2] = PLL_CP_HIGH; // set charge pump high
    data[3] = PLL_ALL_LOW; // set output ports low
    if (attemptI2C(false, pllAddress, data, 4)) {
        storeBandFrequency(freqBandIndex, targetFreq);
        currentFreq = targetFreq;
        pllCheckPending = true;
    }
}

void checkPLL() {
    if (!pllCheckPending) return;
    byte readByte;
    if (attemptI2C(true, pllAddress, &readByte, 1)) {
        pllLock = (readByte >> PLL_LOCK_BIT) & 0x01;
        if (!menuMode) { display(PLL_LOCK_STATUS); }
        if (pllLock) {
            byte data[2]; // partial programming, starting with byte 4 (TSA 5511 datasheet, table 1)
            data[0] = chargePump ? PLL_CP_HIGH : PLL_CP_LOW; // set charge pump
            switch (pllOutputPorts) { // set output ports
                case 0: data[1] = PLL_ALL_LOW; break;
                case 1: data[1] = PLL_P2_HIGH; break;
                case 2: data[1] = PLL_P5_HIGH; break;
                case 3: default: data[1] = PLL_P2_P5_HIGH; break;
            }
            if (attemptI2C(false, pllAddress, data, 2)) {
                pllCheckPending = false; // PLL lock state not monitored after lock, as PLL lock flag may flicker due to FM modulation
            }
            digitalWrite(lockIndicator, HIGH);
        } else {
            digitalWrite(lockIndicator, LOW);
        }
    }
}

void storeBandIndex() {
    EEPROM.update(EEPROM_FREQBAND_ADDR, freqBandIndex);
}

void storeBandFrequency(byte bandIndex, long frequency) {
    if (bandIndex < numFreqBands && frequency != currentFreq) { // avoid unnecessary EEPROM writes
        EEPROM.put(EEPROM_BAND_FREQ_BASE_ADDR + bandIndex * sizeof(long), frequency);
    }
}

void storeNumDecimals() {
    EEPROM.update(EEPROM_DECIMAL_ADDR, numDecimals);
}

void storeI2CAddress() {
    EEPROM.update(EEPROM_I2C_ADDR, pllAddrIndex);
}

void storeXTALFreq() {
    EEPROM.update(EEPROM_XTAL_ADDR, xtalFreqIndex);
}

void storeChargePump() {
    EEPROM.update(EEPROM_CP_ADDR, chargePump);
}

void storeOutputPortsSetting() {
    EEPROM.update(EEPROM_OUTPUT_PORTS_ADDR, pllOutputPorts);
}

void storeStationName() {
    stationName[maxNameLength] = '\0'; // ensure null terminator

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

void storeDimmerStatus() {
    EEPROM.update(EEPROM_DIM_ADDR, backlightDimActive);
}

void checkI2C() {
    static unsigned long lastI2cCheckTime = 0;
    if (menuMode) return; // no interference allowed with menu session

    unsigned long currentMillis = millis();

    if (currentMillis - lastI2cCheckTime >= i2cHealthCheckInterval) {
        lastI2cCheckTime = currentMillis;
        attemptI2C(false, pllAddress, nullptr, 0); // I²C bus is operational if transmission succeeds
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
    if (!menuMode) { handleI2CError(); }
    return false;
}

void handleI2CError() {
    digitalWrite(lockIndicator, LOW);
    display(I2C_ERROR);
    while (true) {
        while (!digitalRead(setButton)) { blinkLed(errIndicator, errBlinkRate); } // ensure that SET is released, to prevent premature reset
        do { blinkLed(errIndicator, errBlinkRate); } while (digitalRead(setButton)); // initiate system reset after confirmation with SET
        while (!digitalRead(setButton)) blinkLed(errIndicator, errBlinkRate); // continue blinking error indicator while SET is pressed
        digitalWrite(errIndicator, LOW); // reset error indicator
        wdt_enable(WDTO_15MS); // reset system via watchdog
        while (true) {} // wait for watchdog to expire and reset system
    }
}

void blinkLed(uint8_t ledPin, unsigned long interval) {
    static unsigned long lastBlinkTime = 0;
    static bool ledState = false;
    unsigned long currentMillis = millis();

    if (currentMillis - lastBlinkTime >= interval) {
        lastBlinkTime = currentMillis;
        ledState = !ledState;
        digitalWrite(ledPin, ledState);
    }
}

void display(uint8_t mode) {
    // format right-aligned VCO frequency with dynamic decimal precision
    auto printFreq = [](uint8_t row) {
        const char* suffix = " MHz";
        char buffer[9]; // buffer for max. 8-character frequency string (incl. decimal point) and null terminator
        dtostrf(targetFreq / 1000000.0, 8, numDecimals, buffer); // format VCO frequency as right-aligned string
        uint8_t col = 16 - (strlen(buffer) + strlen(suffix)); // compute starting column for right-alignment
        lcd.setCursor(col, row);
        lcd.print(buffer);
        lcd.print(suffix);
    };

    // cursor only active in station name editor
    if (mode != STATION_NAME_EDITOR) lcd.noCursor();

    // display modes
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
            if (!freqSetMode) { // avoid time-consuming repeated LCD instructions during auto-repeat VCO frequency change
                lcd.setCursor(0, 1);
                lcd.print("SET ");
            }
            printFreq(1);
            break;

        case MENU_INTERFACE:
            lcd.clear();
            if (menuExitConfirmMode) {
                lcd.setCursor(0, 0);
                lcd.print("EXIT MENU");
                lcd.setCursor(0, 1);
                lcd.print("> ");
                switch (menuIndex) {
                    case 0: lcd.print("save changes"); break;
                    case 1: lcd.print("discard"); break;
                    case 2: lcd.print("cancel"); break;
                }
                break;
            }
            if (menuLevel == 0) {
                switch (menuIndex) {
                    case 0:
                        lcd.print("VCO SETTINGS");
                        lcd.setCursor(0, 1);
                        lcd.print("SET to enter");
                        break;
                    case 1:
                        lcd.print("PLL SETTINGS");
                        lcd.setCursor(0, 1);
                        lcd.print("SET to enter");
                        break;
                    case 2:
                        lcd.print("GENERAL SETTINGS");
                        lcd.setCursor(0, 1);
                        lcd.print("SET to enter");
                        break;
                    case 3:
                        lcd.print("EXIT MENU");
                        lcd.setCursor(0, 1);
                        lcd.print("SET to proceed");
                        break;
                }
            } else if (menuLevel == 1) {
                switch (menuIndex) {
                    case 0:
                        lcd.print("FREQUENCY BAND");
                        lcd.setCursor(0, 1);
                        if (menuEditMode) {
                            long low = validateFreq(freqBands[freqBandIndex][0], true);
                            long high = validateFreq(freqBands[freqBandIndex][1], true);
                            lcd.print("> ");
                            lcd.print(low / 1000000);
                            lcd.print("-");
                            lcd.print(high / 1000000);
                            lcd.print(" MHz");
                        } else {
                            lcd.print("SET to select");
                        }
                        break;
                    case 1:
                        lcd.print("PRECISION");
                        lcd.setCursor(0, 1);
                        if (menuEditMode) {
                            lcd.print("> ");
                            lcd.print(numDecimals);
                            lcd.print(numDecimals == 1 ? " decimal" : " decimals");
                        } else {
                            lcd.print("SET to select");
                        }
                        break;
                    case 2:
                        lcd.print("EXIT SETTINGS");
                        lcd.setCursor(0, 1);
                        lcd.print("SET to confirm");
                        break;
                }
            } else if (menuLevel == 2) {
                switch (menuIndex) {
                    case 0:
                        lcd.print("I2C ADDRESS");
                        lcd.setCursor(0, 1);
                        if (menuEditMode) {
                            lcd.print("> 0x");
                            lcd.print(PLL_ADDRESSES[tempPllAddrIndex], HEX);
                        } else {
                            lcd.print("SET to select");
                        }
                        break;
                    case 1:
                        lcd.print("XTAL FREQUENCY");
                        lcd.setCursor(0, 1);
                        if (menuEditMode) {
                            lcd.print("> ");
                            lcd.print(xtalFreqIndex == 0 ? "1.6 MHz" : "3.2 MHz");
                        } else {
                            lcd.print("SET to select");
                        }
                        break;
                    case 2:
                        lcd.print("CHARGE PUMP");
                        lcd.setCursor(0, 1);
                        if (menuEditMode) {
                            lcd.print("> ");
                            lcd.print(chargePump ? "high" : "low ");
                        } else {
                            lcd.print("SET to select");
                        }
                        break;
                    case 3:
                        lcd.print("OUTPUT PORTS");
                        lcd.setCursor(0, 1);
                        if (menuEditMode) {
                            lcd.print("> ");
                            switch (pllOutputPorts) {
                                case 0: lcd.print("all low"); break;
                                case 1: lcd.print("P2 high"); break;
                                case 2: lcd.print("P5 high"); break;
                                case 3: default: lcd.print("P2/P5 high"); break;
                            }
                        } else {
                            lcd.print("SET to select");
                        }
                        break;
                    case 4:
                        lcd.print("EXIT SETTINGS");
                        lcd.setCursor(0, 1);
                        lcd.print("SET to confirm");
                        break;
                }
            } else if (menuLevel == 3) {
                switch (menuIndex) {
                    case 0:
                        lcd.print("STATION NAME");
                        lcd.setCursor(0, 1);
                        lcd.print("SET to edit");
                        break;
                    case 1:
                        lcd.print("BACKLIGHT DIMMER");
                        lcd.setCursor(0, 1);
                        if (menuEditMode) {
                            lcd.print("> ");
                            lcd.print(backlightDimActive ? "on " : "off");
                        } else {
                            lcd.print("SET to select");
                        }
                        break;
                    case 2:
                        lcd.print("EXIT SETTINGS");
                        lcd.setCursor(0, 1);
                        lcd.print("SET to confirm");
                        break;
                }
            }
            break;

        case STATION_NAME_EDITOR:
            lcd.setCursor(0, 1);
            lcd.print(stationNameBuffer);
            lcd.setCursor(editPosition, 1);
            lcd.cursor();
            break;

        case PLL_LOCK_STATUS:
            static unsigned long lastCharScrollTime = 0;
            static uint8_t charPos = 0;
            static bool movingRight = true;

            lcd.setCursor(0, 0);
            if (pllLock){
                lcd.print("LOCK ");
            } else {
                // animation if unlocked
                unsigned long currentMillis = millis();

                if (currentMillis - lastCharScrollTime >= animInterval) {
                    lastCharScrollTime = currentMillis;
                    lcd.print("     ");
                    lcd.setCursor(charPos, 0);
                    lcd.print(movingRight ? ">>" : "<<");
                    charPos += movingRight ? 1 : -1;
                    if (charPos == 0 || charPos == 3) movingRight = !movingRight;
                }
            }
            break;

        case LCD_HIBERNATE:
            backlightOff ? lcd.clear() : (display(MAIN_INTERFACE), display(PLL_LOCK_STATUS));
            break;

        case I2C_ERROR:
            digitalWrite(lcdBacklight, HIGH);
            lcd.clear();
            lcd.print(i2cFallbackActive ? "I2C SAFE DEFAULT" : "I2C ERROR");
            lcd.setCursor(0, 1);
            if (i2cFallbackActive) {
                lcd.print("restoring 0x");
                lcd.print(pllAddress, HEX);
            } else {
                lcd.print("SET to restart");
            }
            break;
    }
}
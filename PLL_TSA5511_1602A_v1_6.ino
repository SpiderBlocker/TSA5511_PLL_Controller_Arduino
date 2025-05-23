/*
DESCRIPTION
  PLL controller for the TSA5511, initially intended to replace the proprietary controller for the DRFS06 exciter by Dutch RF Shop, but it can be used for any other
  TSA5511 based exciter.
  Using a 3,2 MHz crystal on the TSA5511, the control ranges from 50 kHz up to 1.638,35 MHz, with a step size of 50 kHz or any multiple thereof.
  The practical lower and upper limits will be much tighter, as the TSA5511 is rated from 64 MHz up to 1.300 MHz.
  Using any other crystal frequency (as far as the TSA5511 may support), valid frequency step size and divisor bytes for the TSA5511 are calculated automatically.
  It has a built-in station name editor and LCD backlight control. The station name, backlight dimmer setting and the last operating frequency are stored in EEPROM.

HARDWARE
  • The hardware comprises of an Arduino Nano or compatible, a standard 16x2 LCD display (used in 4-bit mode) with backlight and contrast adjustment, three
    pushbuttons (DOWN/SET/UP, each with a 470 nF debouncing capacitor across its contact) and an optional PLL lock LED which also acts as a blinking fault indicator.
    The lock status is also shown on the LCD display.
  • LCD backlight control is available if you connect it to its reserved digital pin. Refer to code for pin mappings and change if necessary. Note that the digital
    pin used for the LCD backlight must support PWM. Currently pin 6 is configured, which is valid for all current Arduino boards. The brightness level settings and
    timings for the LCD backlight can be adjusted as you wish under "// LCD brightness and dimmer settings".
  • Pull-up resistors on SDA/SCL are required. Especially if SDA/SCL runs through RF-decoupling circuitry, you may want to use lower values for reliable communication,
    like 1 or 2 kΩ.
  • If used with the DRFS06 it is recommended to supply the controller separately from the TSA5511, as it has been proven that slight voltage fluctuations on the
    TSA5511 supply rail will cause a few ppm XTAL frequency deviation accordingly.

USAGE
  • Verify the actual XTAL frequency and frequency band edges under "// PLL settings" and "// VCO frequency and step size settings" respectively and update them if
    necessary.
  • The TSA5511 charge pump is kept high at all times for the DRFS06 exciter. For other platforms, in function "checkPLL()" set "data[0] = PLL_CP_LOW" if required.
  • Press and hold SET during startup to enable the station name editor. Select characters using UP/DOWN and confirm with SET. The new station name will be stored in
    EEPROM after the last character has been confirmed and the main screen will be displayed.
  • Change frequency using UP/DOWN and confirm with SET. The new frequency will be stored in EEPROM. Changing frequency without confirmation will timeout and return
    to the main screen unchanged. Holding UP/DOWN will auto-scroll through the frequency band with gradual acceleration.
  • In quiescent condition (PLL locked) the LCD backlight will dim after a preset period. Double-clicking SET toggles this function ON/OFF and stores the setting in
    EEPROM. Press and hold SET to turn off the backlight completely. The LCD backlight will be restored by pressing any button.
  • In case of an I²C communication error alert, verify PLL hardware and SDA/SCL connection and press SET to restart. I²C communication will be retried several times
    before alerting an error.
*/

// version & credits
#define description "PLL Control"
#define version "V1.6"
#define credits "(C)2025 Loenie"

// required libraries
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
const uint16_t EEPROM_FREQ_ADDR = 0; // VCO frequency
const uint16_t EEPROM_NAME_ADDR = 10; // station name
const uint16_t EEPROM_DIM_ADDR = 30; // backlight dimmer status

// I²C settings
const long wireTimeout = 5000; // I²C transmission timeout, preventing I²C bus crash in some cases
const long i2cClock = 32000; // low I²C clock frequency, more robust through SDA/SCL RF decoupling circuitry (min. 31,25 kHz for 16 MHz ATmega328P)
const long i2cHealthCheckInterval = 2000; // I²C health check interval
const uint8_t i2cMaxRetries = 10; // maximum number of retries in case of a failed I²C transmission

// PLL settings
const byte PLL_ADDR = 0x30; // 7-bit I²C address
const byte PLL_ADDR_WRITE = (PLL_ADDR << 1); // 8-bit write address
const byte PLL_ADDR_READ = ((PLL_ADDR << 1) | 1); // 8-bit read address
const byte PLL_CP_LOW = 0x8E; // charge pump low
const byte PLL_CP_HIGH = 0xCE; // charge pump high
const byte PLL_ALL_LOW = 0x00; // all outputs (P0-P7) low
const byte PLL_P2_HIGH = 0x04; // P2 high
const byte PLL_P2_P5_HIGH = 0x24; // P2/P5 high
const long PLL_XTAL_FREQ = 3200000; // crystal frequency (Hz)
const uint16_t PLL_XTAL_DIVISOR = 512; // crystal frequency divisor
const uint8_t PLL_PRESCALER_DIVISOR = 8; // prescaler divisor
const long PLL_REF_FREQ = (PLL_XTAL_FREQ / PLL_XTAL_DIVISOR) * PLL_PRESCALER_DIVISOR; // reference frequency (Hz), also equals the minimum VCO frequency and step size
const uint8_t PLL_LOCK_BIT = 6; // lock flag bit

// VCO frequency and step size settings
float freqBand[2] = {80000000, 108000000}; // frequency band edges (Hz)
uint8_t stepSizeMultiplier = 1; // frequency step size multiplier (frequency step size will be 50 kHz at 3,2 MHz crystal frequency)

// VCO frequency and step size validation
long validateFreq(float frequency) {
    frequency = constrain(frequency, PLL_REF_FREQ, 0x7FFF * PLL_REF_FREQ); // minimum frequency ≥ PLL_REF_FREQ, cap divisor at 15 bits (MSB of first byte must be zero)
    return round(frequency / PLL_REF_FREQ) * PLL_REF_FREQ; // ensure that frequency equals or is an exact multiple of PLL_REF_FREQ
}
const long lowerFreq = validateFreq(min(freqBand[0], freqBand[1])); // validated lower band edge frequency
const long upperFreq = validateFreq(max(freqBand[0], freqBand[1])); // validated upper band edge frequency
const long freqStep = constrain(stepSizeMultiplier * PLL_REF_FREQ, PLL_REF_FREQ, upperFreq - lowerFreq); // constrain step size to multiple of PLL_REF_FREQ and within range

// station name settings
const uint8_t maxNameLength = 16; // maximum station name length
const char defaultName[maxNameLength + 1] = "Station Name"; // +1 for null terminator
char stationName[maxNameLength + 1]; // +1 for null terminator

// other definitions
const long splashDelay = 2500; // duration to show splash screen
const long initialPressDelay = 1000; // delay before auto-repeat when holding button
const long initialPressInterval = 80; // auto-repeat interval when holding button
const long charScrollInterval = 300; // interval between character scroll steps
const long freqSetTimeout = 5000; // inactivity timeout in frequency set mode
const long errBlinkRate = 250; // error indicator blink interval
long freq;
long currentFreq;
uint8_t nameEditPos;
bool initialized = false;
bool dimmerSetMode = false;
bool nameEditMode = false;
bool freqSetMode = false;
bool buttonDownPressed = false;
bool buttonSetPressed = false;
bool buttonUpPressed = false;
bool backlightDimActive = false;
bool backlightOff = false;
bool pllLock = false;
bool pllWatchdog = false;

// display modi
enum {
    SPLASH_SCREEN,
    STATION_NAME_EDITOR,
    MAIN_INTERFACE,
    SET_FREQUENCY_INTERFACE,
    PLL_LOCK_STATUS,
    LCD_DIMMER_STATUS,
    LCD_HIBERNATE,
    I2C_ERROR
};

void setup() {
    wdt_disable(); // disable watchdog to prevent reset loop
    pinMode(downButton, INPUT_PULLUP);
    pinMode(setButton, INPUT_PULLUP);
    pinMode(upButton, INPUT_PULLUP);
    pinMode(lockIndicator, OUTPUT);
    pinMode(errIndicator, OUTPUT);
    pinMode(lcdBacklight, OUTPUT);

    Wire.begin();
    Wire.setClock(i2cClock);
    Wire.setWireTimeout(wireTimeout, true);
    lcd.begin(16, 2);
    initialize(true);
}

void loop() {
    bool buttonDownState = !digitalRead(downButton);
    bool buttonSetState = !digitalRead(setButton);
    bool buttonUpState = !digitalRead(upButton);

    handleBacklightControl(buttonDownState, buttonSetState, buttonUpState);
    handleNameEditor(buttonDownState, buttonSetState, buttonUpState);
    handleFrequencyChange(buttonDownState, buttonSetState, buttonUpState);
    checkPLL();
    checkI2C();
}

void initialize(bool fullInit) { // full initialization at startup
    if (fullInit) {
        analogWrite(lcdBacklight, maxBrightness);
        display(SPLASH_SCREEN);
        delay(splashDelay);
        readDimmerStatus();
        readStationName();
        readFrequency();
        if (!digitalRead(setButton)) { // enable station name editor when holding SET during startup
            display(STATION_NAME_EDITOR);
            while (!digitalRead(setButton)); // lock cursor position until SET release
            nameEditMode = true;
        }
    } else { // finalize initialization after returning from station name editor
        configurePLL();
        display(MAIN_INTERFACE);
        display(PLL_LOCK_STATUS);
        initialized = true;
    }
}

void readDimmerStatus() {
    backlightDimActive = EEPROM.read(EEPROM_DIM_ADDR); // no check for invalid stored value required; any non-zero value reads as true
}

void readStationName() {
    EEPROM.get(EEPROM_NAME_ADDR, stationName);
    stationName[maxNameLength] = '\0'; // ensure null terminator

    // set standard station name if string is invalid
    size_t length = strnlen(stationName, maxNameLength);
    for (size_t i = 0; i < length; i++) {
        if (!isprint(stationName[i]) || stationName[i] == 0xFF) { // reject non-printable character, no 0xFF
            strncpy(stationName, defaultName, maxNameLength); // copy default station name to array
            memset(stationName + strlen(defaultName), 32, maxNameLength - strlen(defaultName)); // fill remaining positions with spaces (ASCII 32)
            break;
        }
    }
}

void readFrequency() {
    if (!freqSetMode) {
        // get last stored frequency from EEPROM
        long storedFreq;
        EEPROM.get(EEPROM_FREQ_ADDR, storedFreq);

        // check if storedFreq lies within valid range (lowerFreq to upperFreq)
        if (storedFreq < lowerFreq || storedFreq > upperFreq) {
            freq = lowerFreq; // default initial frequency
        } else {
            freq = round(storedFreq / PLL_REF_FREQ) * PLL_REF_FREQ; // round to closest multiple of PLL_REF_FREQ
        }
    }
}

void handleBacklightControl(bool buttonDownState, bool buttonSetState, bool buttonUpState) {
    static bool backlightControlActive = false;

    // no LCD backlight control in station name edit mode, frequency set mode or if unlocked
    if (nameEditMode || freqSetMode || !pllLock) {
        backlightControlActive = false;
        return;
    }

    static unsigned long dimmerTimer = 0;
    static unsigned long lastDimmerUpdateTime = 0;
    static unsigned long buttonHoldStartTime = 0;
    static unsigned long statusDisplayTime = 0;
    static unsigned long lastSetButtonClickTime = 0;
    unsigned long currentMillis = millis();
    static uint8_t setButtonClickCount = 0;
    static uint8_t currentBrightness = maxBrightness;

    // enable LCD backlight control after SET release
    if (!backlightControlActive) {
        while (!digitalRead(setButton));
        backlightControlActive = true;
        dimmerTimer = currentMillis;
    }

    // turn off background lighting by pressing and holding SET
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
            while (!digitalRead(setButton)); // ensure that SET is released, to prevent backlight from being turned off again
        }
        dimmerTimer = currentMillis;
    }

    // toggle dimmer function, store setting in EEPROM and show dimmer status screen
    if (buttonSetState) {
        if (currentMillis - lastSetButtonClickTime >= 30 && currentMillis - lastSetButtonClickTime < 300) { // detect SET double-click (30 ms debounce period)
            setButtonClickCount++;
        } else {
            setButtonClickCount = 1;
        }
        lastSetButtonClickTime = currentMillis;
    }
    if (setButtonClickCount == 2) {
        dimmerSetMode = true;
        backlightDimActive = !backlightDimActive;
        storeDimmerStatus();
        display(LCD_DIMMER_STATUS);
        statusDisplayTime = currentMillis + dimMessageTime;
        while (!digitalRead(setButton));
        setButtonClickCount = 0;
        dimmerTimer = currentMillis;
    }

    // prevent backlight from being turned off during message display
    if (currentMillis < statusDisplayTime && !buttonDownState && !buttonUpState) { // allow UP/DOWN to bypass return
        buttonHoldStartTime = 0;
        return;
    }

    // end dimmer message display and restore main screen
    if (statusDisplayTime != 0 || buttonDownState || buttonUpState) { // allow UP/DOWN to end message display
        dimmerSetMode = false;
        dimmerTimer = currentMillis;
        display(MAIN_INTERFACE);
        display(PLL_LOCK_STATUS);
        statusDisplayTime = 0;
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

void storeDimmerStatus() {
    EEPROM.write(EEPROM_DIM_ADDR, backlightDimActive);
}

void handleNameEditor(bool buttonDownState, bool buttonSetState, bool buttonUpState) {
    if (nameEditMode) {
        // select character
        auto nameChange = [](int8_t direction) { nameEditorAction(true, direction); };
        handleButtonInput(buttonDownState, buttonDownPressed, -1, nameChange);
        handleButtonInput(buttonUpState, buttonUpPressed, 1, nameChange);
        // confirm selection
        handleButtonInput(buttonSetState, buttonSetPressed, 0, [](int8_t direction) { nameEditorAction(false, direction); });
    } else {
        if (!initialized) { initialize(false); } // finalize initialization
    }
}

void nameEditorAction(bool nameChange, int8_t direction) {
    if (nameChange) {
        // UP/DOWN action
        uint8_t charRange = 127 - 32 + 1; // allowed ASCII character range
        stationName[nameEditPos] = (stationName[nameEditPos] - 32 + direction + charRange) % charRange + 32;
        display(STATION_NAME_EDITOR);
    } else {
        // SET action
        if (nameEditPos >= maxNameLength - 1) { // store station name when last character has been confirmed
            storeStationName();
            nameEditMode = false;
        } else {
            nameEditPos++; // move to next cursor position
            display(STATION_NAME_EDITOR);
            delay(charScrollInterval);
        }
    }
}

void storeStationName() {
    char storedStationName[maxNameLength + 1]; // +1 for null terminator
    EEPROM.get(EEPROM_NAME_ADDR, storedStationName);

    // avoid unnecessary write operations to protect EEPROM
    if (strncmp(stationName, storedStationName, maxNameLength) != 0) {
        EEPROM.put(EEPROM_NAME_ADDR, stationName);
    }
}

void handleFrequencyChange(bool buttonDownState, bool buttonSetState, bool buttonUpState) {
    static bool timedOut = true;
    if (!(buttonDownState || buttonSetState || buttonUpState) && timedOut) return; // avoid redundant processing
    static unsigned long inactivityTimer = 0;
    unsigned long currentMillis = millis();

    if (initialized && !dimmerSetMode && !nameEditMode) {
        // change frequency
        auto freqChange = [](int8_t direction) { frequencyChangeAction(true, &freq, direction); };
        handleButtonInput(buttonDownState, buttonDownPressed, -1, freqChange);
        handleButtonInput(buttonUpState, buttonUpPressed, 1, freqChange);
        if (buttonDownState || buttonUpState) {
            inactivityTimer = currentMillis;
            timedOut = false;
        }
        // confirm frequency
        if (freqSetMode && buttonSetState) {
            frequencyChangeAction(false, &freq, 0);
            timedOut = true;
        } else if (currentMillis - inactivityTimer > freqSetTimeout) { // inactivity timeout
            freqSetMode = false;
            if (!timedOut) { // restore initial status
                freq = currentFreq;
                display(MAIN_INTERFACE);
                timedOut = true;
            }
        }
    }
}

void frequencyChangeAction(bool freqChange, long* newFreq, int8_t direction) {
    if (freqChange) {
        // UP/DOWN action
        if (freqSetMode) { *newFreq += (direction * freqStep); }
        *newFreq = (*newFreq < lowerFreq) ? upperFreq : (*newFreq > upperFreq) ? lowerFreq : *newFreq;
        freqSetMode = true;
        display(SET_FREQUENCY_INTERFACE);
    } else {
        // SET action
        configurePLL();
        freqSetMode = false;
        display(MAIN_INTERFACE);
    }
}

void storeFrequency(long frequency) {
    // avoid unnecessary write operations during startup to protect EEPROM
    if (initialized) { EEPROM.put(EEPROM_FREQ_ADDR, frequency); }
}

void handleButtonInput(bool buttonState, bool& buttonPressed, int8_t direction, void (*action)(int8_t)) {
    static unsigned long pressStartTime = 0, lastPressTime = 0;
    unsigned long currentMillis = millis();
    unsigned long totalPressTime = currentMillis - pressStartTime, fastPressInterval = initialPressInterval;

    // disallow any combination of DOWN/SET/UP
    if (!digitalRead(downButton) + !digitalRead(setButton) + !digitalRead(upButton) > 1) return;

    if (buttonState) {
        // change on first button press, or - when holding button - continuously after initialPressDelay
        if (!buttonPressed) { pressStartTime = currentMillis; }
        if (totalPressTime >= initialPressDelay) {
            if (!nameEditMode) {
                // gradual acceleration
                long postDelayTime = totalPressTime - initialPressDelay;
                fastPressInterval = max(initialPressInterval / (0.7 + (postDelayTime / initialPressDelay)), initialPressInterval / 7);
            }
        }
        if (!buttonPressed || (totalPressTime >= initialPressDelay && currentMillis - lastPressTime >= fastPressInterval)) {
            lastPressTime = currentMillis;
            buttonPressed = true;
            action(direction);
        }
    } else {
        buttonPressed = false;
    }
}

void configurePLL() {
    if (freq == currentFreq) return;
    long divisor = (freq / PLL_REF_FREQ); // calculate divisor
    byte data[4]; // full programming (TSA 5511 datasheet, table 1)
    data[0] = (divisor & 0xFF00) >> 8; // extract high divisor byte
    data[1] = divisor & 0x00FF; // extract low divisor byte
    data[2] = PLL_CP_HIGH; // set charge pump
    data[3] = PLL_ALL_LOW; // set output ports
    if (attemptI2C(false, PLL_ADDR_WRITE, data, 4)) { // I²C transmission with i2cMaxRetries
        storeFrequency(freq);
        currentFreq = freq;
        pllWatchdog = true;
    }
}

void checkPLL() {
    if (!pllWatchdog) return;
    byte readByte;
    if (attemptI2C(true, PLL_ADDR_READ, &readByte, 1)) {
        pllLock = (readByte >> PLL_LOCK_BIT) & 0x01;
        display(PLL_LOCK_STATUS);
        if (pllLock) {
            byte data[2]; // partial programming, starting with byte 4 (TSA 5511 datasheet, table 1)
            data[0] = PLL_CP_HIGH; // set charge pump
            data[1] = PLL_P2_P5_HIGH; // set output ports
            if (attemptI2C(false, PLL_ADDR_WRITE, data, 2)) { // I²C transmission with i2cMaxRetries
                pllWatchdog = false; // PLL watchdog not used after lock, as PLL lock flag may flicker due to FM modulation
            }
            digitalWrite(lockIndicator, HIGH);
        } else {
            digitalWrite(lockIndicator, LOW);
        }
    }
}

void checkI2C() {
    if (nameEditMode) return;
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
        delay(50);
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
    // show and right-align frequency with two decimals on display
    auto printFreq = []() {
        lcd.print(freq < 10000000 ? "   " : (freq < 100000000 ? "  " : (freq < 1000000000 ? " " : "")));
        lcd.print(freq / 1000000.0, 2);
        lcd.print(" MHz");
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

        case STATION_NAME_EDITOR:
            lcd.clear();
            lcd.print("SET Station Name");
            lcd.setCursor(0, 1);
            lcd.print(stationName);
            lcd.setCursor(nameEditPos, 1);
            lcd.cursor();
            break;

        case MAIN_INTERFACE:
            lcd.noCursor(); // required when returning from station name editor
            lcd.setCursor(5, 0);
            printFreq();
            lcd.setCursor(0, 1);
            lcd.print(stationName);
            break;

        case SET_FREQUENCY_INTERFACE:
            lcd.setCursor(0, 1);
            lcd.print("SET  ");
            lcd.setCursor(5, 1);
            printFreq();
            break;

        case PLL_LOCK_STATUS:
            lcd.setCursor(0, 0);
            if (pllLock){
                lcd.print("LOCK  ");
            } else {
                // animation if unlocked
                static unsigned long lastCharScrollTime = 0;
                unsigned long currentMillis = millis();
                static uint8_t charPos = 0;
                static bool movingRight = true;
                if (currentMillis - lastCharScrollTime >= charScrollInterval) {
                    lastCharScrollTime = currentMillis;
                    lcd.print("     ");
                    lcd.setCursor(charPos, 0);
                    lcd.print(movingRight ? ">>" : "<<");
                    charPos += movingRight ? 1 : -1;
                    if (charPos == 3 || charPos == 0) { movingRight = !movingRight; }
                }
            }
            break;

        case LCD_DIMMER_STATUS:
            lcd.clear();
            lcd.print("Backlight Dimmer");
            lcd.setCursor(0, 1);
            lcd.print(backlightDimActive ? "ON" : "OFF");
            break;

        case LCD_HIBERNATE:
            backlightOff ? lcd.clear() : (display(MAIN_INTERFACE), display(PLL_LOCK_STATUS));
            break;

        case I2C_ERROR:
            digitalWrite(lcdBacklight, HIGH);
            lcd.noCursor(); // required when returning from station name editor
            lcd.clear();
            lcd.print("I2C ERROR");
            lcd.setCursor(0, 1);
            lcd.print("SET to restart");
            break;
    }
}
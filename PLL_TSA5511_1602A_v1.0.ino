// DESCRIPTION
//   PLL controller for the TSA5511, intended to replace the proprietary controller for the DRFS06 exciter by Dutch RF Shop.
//   Using a 3,2 MHz crystal, the control ranges from 50 kHz up to 1.638,35 MHz, with a step size of 50 kHz or any multiple thereof. 
//   The practical lower and upper limits will be tighter, as the TSA5511 is rated from 64 MHz up to 1.300 MHz.
//   Using any other crystal frequency (as far as the TSA5511 will support), valid frequency step size and divisor bytes for the TSA5511 are calculated automatically.
//   Charge pump is kept high at all times for the DRFS06 exciter. For other hardware, in checkPll() set "data[0] = PLL_CP_LOW" if required.
//   Button debouncing: 100 nF || switch contact
//
// USE
// - Verify the actual crystal frequency and required band edge frequencies below and change if necessary.
// - Change frequency using UP/DOWN-buttons and confirm with SET-button. The new frequency will be stored in EEPROM. 
//   Changing frequency without confirmation will timeout and return to main screen unchanged.
// - Hold SET-button during startup to enable the station name editor. Select characters using UP/DOWN-buttons and confirm with SET-button.
//   The new station name will be stored in EEPROM after the last character has been confirmed and the main screen will be displayed.
// - In case of an I2C communication error alert, verify PLL hardware and SDA/SCL connection and press SET-button to restart.

// version & credits
const String description = "PLL Control";
const String version = "V1.0";
const String credits = "(C)2024 Loenie";

// included libraries 
#include <EEPROM.h>
#include <Wire.h>
#include <LiquidCrystal.h>

// buttons pin mapping
const int downButton = 2;
const int setButton = 3;
const int upButton = 4;

// PLL lock LED pin mapping
const int pllLockOutput = 7;

// LCD-display pin mapping
LiquidCrystal lcd(8, 9, 10, 11, 12, 13);

// EEPROM storage
const int EEPROM_FREQ_ADDR = 0;
const int EEPROM_NAME_ADDR = 10;

//I2C settings
const long i2cClock = 32000; // low I2C clock frequency, more robust through SDA/SCL RF-decoupling circuitry (min. 31,25 kHz for 16 MHz ATmega328P)
const long wireTimeout = 5000; // I2C transmission timeout, avoiding I2C bus crash in some cases
const int maxRetries = 10; // maximum number of retries in case of a failed I2C transmission

// PLL settings
const byte PLL_ADDR = 0x30; // PLL 7-bit I2C address (left-aligned in 8-bit format)
const byte PLL_ADDR_WRITE = (PLL_ADDR << 1); // 8-bit write address
const byte PLL_ADDR_READ = ((PLL_ADDR << 1) | 1); // 8-bit read address
const byte PLL_CP_LOW = 0x8E; // charge pump low
const byte PLL_CP_HIGH = 0xCE; // charge pump high
const byte PLL_ALL_LOW = 0x00; // all outputs (P0-P7) low
const byte PLL_P2_HIGH = 0x04; // P2 high
const byte PLL_P2_P5_HIGH = 0x24; // P2/P5 high
const long PLL_XTAL_FREQ = 3200000; // PLL crystal frequency (Hz)
const int PLL_XTAL_DIVISOR = 512; // PLL crystal divisor
const int PLL_PRESCALER_DIVISOR = 8; // PLL prescaler divisor
const long PLL_REF_FREQ = PLL_XTAL_FREQ / PLL_XTAL_DIVISOR * PLL_PRESCALER_DIVISOR; // PLL reference frequency (Hz), also equals the minimum possible VCO frequency and step size
const int PLL_LOCK_BIT = 6; // PLL lock flag bit
bool pllLock = false;
bool pllWatchdog = false;

// VCO frequency settings
uint64_t lowerBandEdge = 80000000; // lower band edge frequency (Hz)
uint64_t upperBandEdge = 108000000; // upper band edge frequency (Hz)
unsigned long freqStep = PLL_REF_FREQ * 1; // frequency step size (Hz), must be equal to or an exact multiple of PLL_REF_FREQ
unsigned long freq;
unsigned long currentFreq;

// VCO frequency validation
unsigned long validateFreq(uint64_t frequency) {
    if (frequency < PLL_REF_FREQ) { frequency = PLL_REF_FREQ; } // ensure that minimum frequency is not lower than PLL_REF_FREQ
    if (frequency / PLL_REF_FREQ > 0x7FFF) { frequency = 0x7FFF * PLL_REF_FREQ; } // ensure that PLL divisor does not exceed 15 bits, as 1st bit of first PLL divisor byte must be 0
    return round((double)frequency / PLL_REF_FREQ) * PLL_REF_FREQ; // ensure that frequency equals or is an exact multiple of PLL_REF_FREQ
}
const unsigned long lowerFreq = validateFreq(lowerBandEdge); // set valid lower band edge frequency
const unsigned long upperFreq = validateFreq(upperBandEdge); // set valid upper band edge frequency

// station name settings
const int maxNameLength = 16;
const char defaultName[maxNameLength + 1] = "Station Name"; // +1 for null terminator
char stationName[maxNameLength + 1]; // +1 for null terminator

// general definitions/declarations
const long startupDelay = 2500; // time to show startup message
const long initialPressDelay = 1000; // delay before continuous change when holding button
const long buttonPressInterval = 80; // continuous change speed when holding button
const long charScrollInterval = 300; // display character scrolling speed
const long freqSetTimeout = 5000; // inactivity timeout in frequency set mode
unsigned long currentTime;
int nameEditPos;
bool initialized = false;
bool nameEditMode = false;
bool freqSetMode = false;
bool buttonDownPressed = false;
bool buttonSetPressed = false;
bool buttonUpPressed = false;

// display modi
enum DisplayMode {
    STARTUP = 0,
    STATION_NAME_EDITOR,
    MAIN_INTERFACE,
    SET_FREQUENCY_INTERFACE,
    PLL_LOCK_STATUS,
    I2C_ERROR
};

void setup() {
    pinMode(downButton, INPUT_PULLUP);
    pinMode(setButton, INPUT_PULLUP);
    pinMode(upButton, INPUT_PULLUP);
    pinMode(pllLockOutput, OUTPUT);

    Wire.begin();
    Wire.setClock(i2cClock);
    Wire.setWireTimeout(wireTimeout, true);
    lcd.begin(16, 2);

    display(STARTUP);
    delay(startupDelay);
    readStationName();
    readFrequency();

    if (!digitalRead(setButton)) { // enable station name editor when holding setButton during startup
        display(STATION_NAME_EDITOR);
        while(!digitalRead(setButton)); // lock cursor position until setButton release
        nameEditMode = true;
    } else { // complete initialization
        configurePll();
        display(MAIN_INTERFACE);
        initialized = true;
    }
}

void loop() {
    currentTime = millis();
    bool buttonDownState = !digitalRead(downButton);
    bool buttonSetState = !digitalRead(setButton);
    bool buttonUpState = !digitalRead(upButton);

    handleNameEditMode(buttonDownState, buttonSetState, buttonUpState);
    handleFrequencyChange(buttonDownState, buttonSetState, buttonUpState);
    checkPll();
}

void handleNameEditMode(bool buttonDownState, bool buttonSetState, bool buttonUpState) {
    if (nameEditMode) {
        handleButtonPress(buttonDownState, buttonDownPressed, -1, selectCharacter);
        handleButtonPress(buttonUpState, buttonUpPressed, 1, selectCharacter);
        handleButtonPress(buttonSetState, buttonSetPressed, 1,[](int) {
            if (nameEditPos >= maxNameLength - 1) {
                storeStationName();
                nameEditMode = false;
            } else {
                nameEditPos++; // move to next character position
                display(STATION_NAME_EDITOR);
                delay(charScrollInterval);
            }
        });
    } else {
        if (!initialized) { // complete initialization when returning from station name editor
            configurePll();
            display(MAIN_INTERFACE);
            initialized = true;
        }
    }
}

void selectCharacter(int direction) {
    int charRange = 'z' - ' ' + 1; // allowed range of characters
    stationName[nameEditPos] = (stationName[nameEditPos] - ' ' + direction + charRange) % charRange + ' ';
    display(STATION_NAME_EDITOR);
}

void handleFrequencyChange(bool buttonDownState, bool buttonSetState, bool buttonUpState) {
    static long timedOut = 0, lastButtonPressTime = 0;

    if (initialized && !nameEditMode) {
        auto freqChange = [](int direction) { setFrequency(&freq, 0, direction); };
        handleButtonPress(buttonDownState, buttonDownPressed, -1, freqChange);
        handleButtonPress(buttonUpState, buttonUpPressed, 1, freqChange);

        if (buttonDownState || buttonUpState) {
            lastButtonPressTime = millis();
            timedOut = false;
        }
        if (freqSetMode && buttonSetState) {
            setFrequency(&freq, 1, 0);
        } else if (millis() - lastButtonPressTime > freqSetTimeout) { // inactivity timeout
            freqSetMode = false;
            if (!timedOut) { // restore initial status
                freq = currentFreq;
                display(MAIN_INTERFACE);
                timedOut = true;
            }
        }
    }
}

void handleButtonPress(bool buttonState, bool& buttonPressed, int direction, void (*action)(int)) {
    static long buttonPressStartTime = 0, lastButtonPressTime = 0;

    // no change if DOWN and UP are pressed simultaneously
    if (!digitalRead(downButton) && !digitalRead(upButton)) { return; }    
    
    if (buttonState) {
      // change on first button press, or - when holding button - continuously after initialPressDelay
        if (!buttonPressed || (currentTime - buttonPressStartTime >= initialPressDelay && currentTime - lastButtonPressTime >= buttonPressInterval)) {
            if (!buttonPressed) { buttonPressStartTime = currentTime; }
            lastButtonPressTime = currentTime;
            buttonPressed = true;
            action(direction);
        }
    } else {
        buttonPressed = false;
    }
}

void readStationName() {
    EEPROM.get(EEPROM_NAME_ADDR, stationName);
    stationName[maxNameLength] = '\0'; // null terminator

    // set standard station name if string is invalid
    size_t length = strnlen(stationName, maxNameLength);
    for (size_t i = 0; i < length; i++) {
        if (stationName[i] < 32 || stationName[i] > 126 || stationName[i] == 0xFF) { // no invalid ASCII-character, no 0xFF
            strncpy(stationName, defaultName, maxNameLength); // copy default station name to array
            for (size_t i = strlen(defaultName); i < maxNameLength; i++) { // fill remaining positions with spaces
                stationName[i] = ' ';
            }
            stationName[maxNameLength] = '\0'; // null terminator
            break;
        }
    }
}

void storeStationName() {
    char storedStationName[maxNameLength + 1];
    EEPROM.get(EEPROM_NAME_ADDR, storedStationName);
    storedStationName[maxNameLength] = '\0'; // null terminator

    if (strncmp(stationName, storedStationName, maxNameLength) != 0) { // avoid unnecessary write operations to protect EEPROM
        EEPROM.put(EEPROM_NAME_ADDR, stationName);
    }
}

void setFrequency(long* newFreq, int set, int direction) {
    switch(set){
        case 0: // change frequency
            if (freqSetMode) { *newFreq += (direction * freqStep); }
            *newFreq = (*newFreq < lowerFreq) ? upperFreq : (*newFreq > upperFreq) ? lowerFreq : *newFreq;
            freqSetMode = true;
            display(SET_FREQUENCY_INTERFACE);
        break;
        case 1: // set frequency
            configurePll();
            freqSetMode = false;
            display(MAIN_INTERFACE);
        break;
    }
}

void readFrequency() {
    if (!freqSetMode) {
        // get last stored frequency from EEPROM
        float storedFreq;
        EEPROM.get(EEPROM_FREQ_ADDR, storedFreq);

        // check if storedFreq is a valid float and within the valid range (lowerFreq to upperFreq)
        if (isnan(storedFreq) || storedFreq < (lowerFreq) || storedFreq > (upperFreq)) {
            freq = lowerFreq; // default initial frequency
        } else {
            freq = round((double)storedFreq / PLL_REF_FREQ) * PLL_REF_FREQ; // round to closest multiple of PLL_REF_FREQ
        }
    }
}

void storeFrequency(float frequency) {
    // avoid unnecessary write operations during startup to protect EEPROM
    if (initialized) { EEPROM.put(EEPROM_FREQ_ADDR, frequency); }
}

void configurePll() {
    if (freq != currentFreq) {
        long divisor = (freq / PLL_REF_FREQ); // calculate divisor
        byte data[4];
        data[0] = (divisor & 0xFF00) >> 8; // extract high divisor byte
        data[1] = divisor & 0x00FF; // extract low divisor byte
        data[2] = PLL_CP_HIGH; // set charge pump
        data[3] = PLL_ALL_LOW; // set output ports
        for (int i = maxRetries; i > 0; i--) {
            Wire.beginTransmission(PLL_ADDR_WRITE);
            Wire.write(data, 4);
            if (Wire.endTransmission() == 0) {
                storeFrequency(freq);
                currentFreq = freq;
                pllWatchdog = true;
                break;
            }
            delay(50);
            if (i == 1) { i2cErrHandler(); }
        }
    }
}

void checkPll() {
    if (pllWatchdog && !nameEditMode) {
        for (int i = maxRetries; i > 0; i--) {
            Wire.requestFrom(PLL_ADDR_READ, (byte)1);
            if (Wire.available()) {
                byte readByte = Wire.read();
                pllLock = (readByte >> PLL_LOCK_BIT) & 0x01;
                display(PLL_LOCK_STATUS);
                break;
            }
            delay(50);
            if (i == 1) { i2cErrHandler(); }
        }
        if (pllLock) {
            byte data[2]; // partial programming, starting with byte 4
            data[0] = PLL_CP_HIGH; // set charge pump
            data[1] = PLL_P2_P5_HIGH; // set output ports
            for (int i = maxRetries; i > 0; i--) {
                Wire.beginTransmission(PLL_ADDR_WRITE);
                Wire.write(data, 2);
                if (Wire.endTransmission() == 0) {
                    pllWatchdog = false; // PLL watchdog not used after lock, as PLL lock flag may fluctuate due to FM modulation
                    break;
                }
                delay(50);
                if (i == 1) { i2cErrHandler(); }
            }
        }
    }
}

void i2cErrHandler() {
    display(I2C_ERROR);
    while (!digitalRead(setButton)); // ensure setButton is released, as it may trigger a premature reset
    while (true) {
        if (!digitalRead(setButton)) {
            delay(30); // alleviate processor loading
            if (!digitalRead(setButton)) { asm volatile ("jmp 0"); } // soft reset
        }
    }
}

void display(int mode) {
    // show and right-align frequency on display
    auto printFreq = [&](long freq) {
        lcd.print(freq < 10000000 ? "   " : (freq < 100000000 ? "  " : (freq < 1000000000 ? " " : "")));
        lcd.print(freq / 1000000.0);
        lcd.print(" MHz");
    };

    switch(mode){
        case STARTUP:
            lcd.clear();
            lcd.setCursor(0, 0);
            lcd.print(description + " " + version);
            lcd.setCursor(0, 1);
            lcd.print(credits);
            break;

        case STATION_NAME_EDITOR:
            lcd.clear();
            lcd.setCursor(0, 0);
            lcd.print("SET Station Name");
            lcd.setCursor(0, 1);
            lcd.print(stationName);
            lcd.setCursor(nameEditPos, 1);
            lcd.cursor();
            break;

        case MAIN_INTERFACE:
            lcd.noCursor(); // required when returning from case STATION_NAME_EDITOR
            lcd.setCursor(5, 0);
            printFreq(freq);
            lcd.setCursor(0, 1);
            lcd.print(stationName);
            break;

        case SET_FREQUENCY_INTERFACE:
            lcd.setCursor(0, 1);
            lcd.print("SET  ");
            lcd.setCursor(5, 1);
            printFreq(freq);
            break;

        case PLL_LOCK_STATUS:
            lcd.setCursor(0, 0);
            if (pllLock){
                lcd.print("LOCK  ");
                digitalWrite(pllLockOutput, HIGH);
            } else {
                // animation during lock wait
                static long lastCharScrollTime = 0;
                static int charPos = 0;
                static bool movingRight = true;
                digitalWrite(pllLockOutput, LOW);
                if (currentTime - lastCharScrollTime >= charScrollInterval) {
                    lastCharScrollTime = currentTime;
                    lcd.print("     ");
                    lcd.setCursor(charPos, 0);
                    lcd.print(movingRight ? ">>" : "<<");
                    charPos += movingRight ? 1 : -1;
                    if (charPos == 3 || charPos == 0) { movingRight = !movingRight; }
                }
            }
            break;
        
        case I2C_ERROR:
            lcd.noCursor(); // required when returning from case STATION_NAME_EDITOR
            lcd.clear();
            digitalWrite(pllLockOutput, LOW);
            lcd.setCursor(0, 0);
            lcd.print("I2C ERROR");
            lcd.setCursor(0, 1);
            lcd.print("SET to restart");
            break;
    }
}
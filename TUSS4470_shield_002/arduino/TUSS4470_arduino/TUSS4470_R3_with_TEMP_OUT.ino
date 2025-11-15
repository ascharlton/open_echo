#include <SPI.h>

#define ARDUINO_R4 false

// Pin configuration
const int SPI_CS = 10;
const int IO1 = 8;
const int IO2 = 9;
const int O3 = 3;
const int O4 = 2;
const int analogIn = A0;


// Number of ADC samples to take per measurement cycle
// Each sample takes approximately 13.2 microseconds
// This value must match the number of samples expected by the Python visualization tool
// Max 1800 on R3, ~10000 on R4
#define NUM_SAMPLES 1800

// Number of initial samples to ignore after sending the transducer pulse
// These ignored samples represent the "blind zone" where the transducer is still ringing
#define BLINDZONE_SAMPLE_END 450

// Threshold level for detecting the bottom echo
// The first echo stronger than this value (after the blind zone) is considered the bottom
#define THRESHOLD_VALUE 0x19


// ---------------------- DRIVE FREQUENCY SETTINGS ----------------------
// Sets the output frequency of the ultrasonic transducer
// Uses DRIVE_FREQUENCY directly for R4, uses divider for R3
// #define DRIVE_FREQUENCY 40000
#define DRIVE_FREQUENCY 200000
const int DRIVE_FREQUENCY_TIMER_DIVIDER = (16000000 / (2 * DRIVE_FREQUENCY)) - 1;

// ---------------------- BANDPASS FILTER SETTINGS ----------------------
// Sets the digital band-pass filter frequency on the TUSS4470 driver chip
// This should roughly match the transducer drive frequency
// For additional register values, see TUSS4470 datasheet, Table 7.1 (pages 17–18)
// #define FILTER_FREQUENCY_REGISTER 0x00 // 40 kHz
// #define FILTER_FREQUENCY_REGISTER 0x09 // 68 kHz
// #define FILTER_FREQUENCY_REGISTER 0x10 // 100 kHz
// #define FILTER_FREQUENCY_REGISTER 0x18 // 151 kHz
#define FILTER_FREQUENCY_REGISTER 0x1E // 200 kHz

// ---------------------- TUSS4470 REGISTER DEFS ----------------------
#define TEMP_OUT_REG 0x06 // Temperature output register


byte misoBuf[2];  // SPI receive buffer
byte inByteArr[2];  // SPI transmit buffer

byte analogValues[NUM_SAMPLES];
volatile int pulseCount = 0;
volatile int sampleIndex = 0;

// Global variables for data logging
float temperature = 0.0f;
int vDrv = 0; // VDRV is not used in this function set, but kept for completeness

volatile bool detectedDepth = false;  // Condition flag
volatile int depthDetectSample = 0;

// ---------------------- TUSS4470 SPI FUNCTIONS ----------------------
void startTransducerBurst()
{
  TCCR1A = _BV(COM1A0);  // Toggle OC1A (pin 9) on Compare Match
  TCCR1B = _BV(WGM12) | _BV(CS10);  // CTC mode, no prescaler

  OCR1A = DRIVE_FREQUENCY_TIMER_DIVIDER;

  TIMSK1 = _BV(OCIE1A);  // Enable Timer1 Compare Match A interrupt
}

void stopTransducer()
{
  TCCR1A = 0;
  TCCR1B = 0;  // Stop Timer1 by clearing clock select bits
  TIMSK1 = 0;  // Disable Timer1 interrupt
}

ISR(TIMER1_COMPA_vect)
{
  pulseCount++;
  if (pulseCount >= 32)
  {
    stopTransducer();
    pulseCount = 0;  // Reset counter for next cycle
  }
}

byte tuss4470Read(byte addr) {
  // 0x80 = Read bit (bit 7) set
  // (addr & 0x3F) << 1 = Register address (bits 6-1)
  inByteArr[0] = 0x80 + ((addr & 0x3F) << 1);  // Set read bit and address
  inByteArr[1] = 0x00;  // Empty data byte
  inByteArr[0] |= tuss4470Parity(inByteArr);
  spiTransfer(inByteArr, sizeof(inByteArr));

  return misoBuf[1]; // Return the data received on the MISO line (byte 2)
}

void tuss4470Write(byte addr, byte data) {
  // (addr & 0x3F) << 1 = Register address (bits 6-1)
  inByteArr[0] = (addr & 0x3F) << 1;  // Set write bit and address (Write bit 7 is 0)
  inByteArr[1] = data;
  inByteArr[0] |= tuss4470Parity(inByteArr);
  spiTransfer(inByteArr, sizeof(inByteArr));
}

byte tuss4470Parity(byte* spi16Val) {
  return parity16(BitShiftCombine(spi16Val[0], spi16Val[1]));
}

void spiTransfer(byte* mosi, byte sizeOfArr) {
  memset(misoBuf, 0x00, sizeof(misoBuf));

  digitalWrite(SPI_CS, LOW);
  for (int i = 0; i < sizeOfArr; i++) {
    misoBuf[i] = SPI.transfer(mosi[i]);
  }
  digitalWrite(SPI_CS, HIGH);
}

unsigned int BitShiftCombine(unsigned char x_high, unsigned char x_low) {
  return (x_high << 8) | x_low;  // Combine high and low bytes
}

byte parity16(unsigned int val) {
  byte ones = 0;
  for (int i = 0; i < 16; i++) {
    if ((val >> i) & 1) {
      ones++;
    }
  }
  return (ones + 1) % 2;  // Odd parity calculation (1=odd, 0=even)
}

// ---------------------- TEMPERATURE READING ----------------------

void readAndProcessTemperature() {
  // The TEMP_OUT (0x06) register returns an 8-bit value (R).
  // The conversion formula is typically T(°C) = R * 0.25 (LSB is 0.25°C).
  
  uint8_t rawTemp = tuss4470Read(TEMP_OUT_REG);
  
  // Convert the raw value to degrees Celsius
  temperature = (float)rawTemp * 0.25;
}


// ---------------------- TIMER & INTERRUPT FUNCTIONS ----------------------



void handleInterrupt() {
  // This interrupt is triggered by OUT_4 when the analog input voltage crosses the THRESHOLD_VALUE
  if (!detectedDepth) {
    depthDetectSample = sampleIndex;
    detectedDepth = true;
  }
}

// ---------------------- MAIN ARDUINO FUNCTIONS ----------------------

void setup()
{
  Serial.begin(250000);

  SPI.begin();

  SPI.setBitOrder(MSBFIRST);
  // Set clock divider to 16 (16MHz / 16 = 1MHz SPI clock)
  SPI.setClockDivider(SPI_CLOCK_DIV16); 
  SPI.setDataMode(SPI_MODE1);  // CPOL=0, CPHA=1 (for TUSS4470)

  pinMode(SPI_CS, OUTPUT);
  digitalWrite(SPI_CS, HIGH);

  // Configure GPIOs
  pinMode(IO1, OUTPUT);
  digitalWrite(IO1, HIGH);
  pinMode(IO2, OUTPUT);
  pinMode(O4, INPUT_PULLUP);
  // Attach interrupt to the threshold detection output (O4)
  attachInterrupt(digitalPinToInterrupt(O4), handleInterrupt, RISING);

  // Initialize TUSS4470 with specific configurations
  tuss4470Write(0x10, FILTER_FREQUENCY_REGISTER);  // Set BPF center frequency
  tuss4470Write(0x16, 0xF);  // Enable VDRV (not Hi-Z)
  tuss4470Write(0x1A, 0x0F);  // Set burst pulses to 16 (4 pulses)
  tuss4470Write(0x17, THRESHOLD_VALUE); // enable threshold detection on OUT_4

  // Set up ADC for free-running conversion
  ADCSRA = (1 << ADEN)  |  // Enable ADC
          (1 << ADPS2);   // Set prescaler to 16 (16 MHz / 16 = 1 MHz ADC clock)
  ADMUX = (1 << REFS0);    // Reference voltage: AVcc
  // Input channel: ADC0 (default)
  ADCSRB = 0;              // Free-running mode
  ADCSRA |= (1 << ADATE);  // Enable auto-trigger (free-running)
  ADCSRA |= (1 << ADSC);   // Start conversion

}

void loop()
{
  // 1. Read temperature before the ping/sample cycle
  readAndProcessTemperature();
  
  // 2. Trigger time-of-flight measurement
  tuss4470Write(0x1B, 0x01);

  startTransducerBurst();

  // 3. Read analog values from A0
  for (sampleIndex = 0; sampleIndex < NUM_SAMPLES; sampleIndex++) {

    while (!(ADCSRA & (1 << ADIF))); // Wait for conversion to complete
    ADCSRA |= (1 << ADIF);           // Clear the interrupt flag
    // Read ADC value (10-bit) and convert to 8-bit for simplicity (analogValues is byte array)
    analogValues[sampleIndex] = ADC >> 2; 

    if (sampleIndex == BLINDZONE_SAMPLE_END) {
      detectedDepth = false; // Start monitoring for a valid echo after the blind zone
    }
  }

  // 4. Stop time-of-flight measurement
  tuss4470Write(0x1B, 0x00);
  
  // 5. Send all collected data over Serial
  sendData();

  // 6. Loop Delay
  delay(10);
}

void sendData() {
  Serial.write(0xAA);  // Start byte (Header)

  uint8_t checksum = 0;

  // 1. Depth (2 bytes, sample index)
  uint8_t depthHigh = depthDetectSample >> 8;
  uint8_t depthLow  = depthDetectSample & 0xFF;
  Serial.write(depthHigh);
  Serial.write(depthLow);
  checksum ^= depthHigh ^ depthLow;

  // 2. Temperature × 100 (2 bytes, scaled short integer)
  // Scaling by 100 preserves two decimal places for the float value (e.g., 25.50C -> 2550)
  int16_t temp_scaled = (int16_t)(temperature * 100.0f);
  uint8_t tempHigh = temp_scaled >> 8;
  uint8_t tempLow  = temp_scaled & 0xFF;
  Serial.write(tempHigh);
  Serial.write(tempLow);
  checksum ^= tempHigh ^ tempLow;

  // 3. Drive Voltage × 100 (2 bytes, scaled short integer)
  uint16_t vDrv_scaled = vDrv * 100;
  uint8_t vDrvHigh = vDrv_scaled >> 8;
  uint8_t vDrvLow  = vDrv_scaled & 0xFF;
  Serial.write(vDrvHigh);
  Serial.write(vDrvLow);
  checksum ^= vDrvHigh ^ vDrvLow;

  // 4. Analog samples (NUM_SAMPLES bytes)
  // NOTE: This now sends only the 8-bit analogValues, not 16-bit
  for (int i = 0; i < NUM_SAMPLES; i++) {
    Serial.write(analogValues[i]);
    checksum ^= analogValues[i];
  }

  // Send checksum (1 byte)
  Serial.write(checksum);
}

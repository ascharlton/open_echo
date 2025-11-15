#include <SPI.h>

#define ARDUINO_R4 false

// Pin configuration
const int SPI_CS = 10;
const int IO1 = 8;
const int IO2 = 9;
const int O3 = 3;
const int O4 = 2;
const int analogIn = A0;


// --- WORKING SRAM VALUE ---
#define NUM_SAMPLES 900 
// --------------------------

// Number of initial samples to ignore after sending the transducer pulse
#define BLINDZONE_SAMPLE_END 450

// Threshold level for detecting the bottom echo
#define THRESHOLD_VALUE 0x19


// ---------------------- DRIVE FREQUENCY SETTINGS ----------------------
#define DRIVE_FREQUENCY 200000
const int DRIVE_FREQUENCY_TIMER_DIVIDER = (16000000 / (2 * DRIVE_FREQUENCY)) - 1;

// ---------------------- BANDPASS FILTER SETTINGS ----------------------
#define FILTER_FREQUENCY_REGISTER 0x1E // 200 kHz

byte misoBuf[2];
byte inByteArr[2];

uint16_t analogValues[NUM_SAMPLES];
volatile int pulseCount = 0;
volatile int sampleIndex = 0;

float temperature = 0.0f;
int vDrv = 0;

volatile bool detectedDepth = false;
volatile int depthDetectSample = 0;

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
  inByteArr[0] = 0x80 + ((addr & 0x3F) << 1);
  inByteArr[1] = 0x00;
  inByteArr[0] |= tuss4470Parity(inByteArr);
  spiTransfer(inByteArr, sizeof(inByteArr));

  return misoBuf[1];
}

void tuss4470Write(byte addr, byte data) {
  inByteArr[0] = (addr & 0x3F) << 1;
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
  return (x_high << 8) | x_low;
}

byte parity16(unsigned int val) {
  byte ones = 0;
  for (int i = 0; i < 16; i++) {
    if ((val >> i) & 1) {
      ones++;
    }
  }
  return (ones + 1) % 2;
}

void handleInterrupt() {
  if (!detectedDepth) {
    depthDetectSample = sampleIndex;
    detectedDepth = true;
  }
}

void setup()
{
  Serial.begin(250000);

  SPI.begin();
  SPI.setBitOrder(MSBFIRST);
  SPI.setClockDivider(SPI_CLOCK_DIV16);
  SPI.setDataMode(SPI_MODE1);

  pinMode(SPI_CS, OUTPUT);
  digitalWrite(SPI_CS, HIGH);

  // Configure GPIOs
  pinMode(IO1, OUTPUT);
  digitalWrite(IO1, HIGH);
  pinMode(IO2, OUTPUT);
  pinMode(O4, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(O4), handleInterrupt, RISING);

  // Initialize TUSS4470
  tuss4470Write(0x10, FILTER_FREQUENCY_REGISTER);  // Set BPF center frequency
  
  // *** GAIN RAMP IMPLEMENTATION ***
  // 1. Set the Analog Gain Ramp Slope (0x11)
  // 0x02 = 0.5 dB / 10us ramp slope (slow, gradual increase)
  tuss4470Write(0x11, 0x02); 
  
  // 2. Set Analog Gain Control (0x15) to specify the MAX gain (0x07 = 40dB) 
  // and set the gain control mode to GAIN_RAMP_EN (bit 3 = 1).
  // 0x0F = (max gain 0x07) | (bit 3: Enable Gain Ramp)
  tuss4470Write(0x15, 0x0F); 
  // **********************************
  
  tuss4470Write(0x16, 0xF);  // Enable VDRV (not Hi-Z)
  tuss4470Write(0x1A, 0x0F);  // Set burst pulses to 16
  tuss4470Write(0x17, THRESHOLD_VALUE); // enable threshold detection on OUT_4

  // Set up ADC
  ADCSRA = (1 << ADEN)  |  // Enable ADC
          (1 << ADPS2);   // Set prescaler to 16
  ADMUX = (1 << REFS0);    // Reference voltage: AVcc
  ADCSRB = 0;              // Free-running mode
  ADCSRA |= (1 << ADATE);  // Enable auto-trigger
  ADCSRA |= (1 << ADSC);   // Start conversion

}

void loop()
{
  // Trigger time-of-flight measurement
  // Writing 0x01 to 0x1B starts the ping, and the gain ramp starts immediately after.
  tuss4470Write(0x1B, 0x01);

  startTransducerBurst();

  // Read analog values from A0
  for (sampleIndex = 0; sampleIndex < NUM_SAMPLES; sampleIndex++) {

    while (!(ADCSRA & (1 << ADIF))); // Wait for conversion to complete
    ADCSRA |= (1 << ADIF);           // Clear the interrupt flag
    analogValues[sampleIndex] = ADC; // Read full 10-bit value

    if (sampleIndex == BLINDZONE_SAMPLE_END) {
      detectedDepth = false;
    }
  }

  // Stop time-of-flight measurement
  tuss4470Write(0x1B, 0x00);
  
  sendData();

  delay(10);
}

void sendData() {
  Serial.write(0xAA);  // Start byte

  uint8_t checksum = 0;

  // Metadata (Depth, Temp, VDrv)
  uint8_t depthHigh = depthDetectSample >> 8;
  uint8_t depthLow  = depthDetectSample & 0xFF;
  Serial.write(depthHigh);
  Serial.write(depthLow);
  checksum ^= depthHigh ^ depthLow;

  int16_t temp_scaled = temperature * 100;
  uint8_t tempHigh = temp_scaled >> 8;
  uint8_t tempLow  = temp_scaled & 0xFF;
  Serial.write(tempHigh);
  Serial.write(tempLow);
  checksum ^= tempHigh ^ tempLow;

  uint16_t vDrv_scaled = vDrv * 100;
  uint8_t vDrvHigh = vDrv_scaled >> 8;
  uint8_t vDrvLow  = vDrv_scaled & 0xFF;
  Serial.write(vDrvHigh);
  Serial.write(vDrvLow);
  checksum ^= vDrvHigh ^ vDrvLow;

  // Analog samples (900 samples * 2 bytes each = 1800 bytes)
  for (int i = 0; i < NUM_SAMPLES; i++) {
    uint8_t highByte = analogValues[i] >> 8;
    uint8_t lowByte  = analogValues[i] & 0xFF;
    Serial.write(highByte);
    Serial.write(lowByte);
    checksum ^= highByte ^ lowByte;
  }

  // Send checksum
  Serial.write(checksum);
}

// Pin definitions
#define RED   1  // PA6
#define GREEN 6  // PA7
#define BLUE  7  // PA1
#define ADC_PIN   2  // AIN2 (PA2)
#define NUM_SAMPLES 10 

// Current thresholds in mA
#define LEVEL_A_CURRENT  1.7// Threshold A in mA
#define LEVEL_B_CURRENT  1.9  // Threshold B in mA

void setup() {
  // Configure RGB pins as outputs
  PORTA.DIR |= (1 << 6) | (1 << 7) | (1 << 1);  // PA6, PA7, PA1 as output
  
  // Configure PA2 (AIN2) as ADC input (check datasheet for pin configuration)
  PORTA.DIR &= ~(1 << 2);  // Set PA2 as input
  
  // Configure ADC0 to use PA2 (AIN2) as input
  ADC0.MUXPOS = ADC_MUXPOS_AIN2_gc;   // Select AIN2 (PA2) as the input for ADC0
  
  // Enable ADC, with 10-bit resolution
  ADC0.CTRLA = ADC_ENABLE_bm | ADC_RESSEL_10BIT_gc;  // Enable ADC, 10-bit resolution
  
  // Set reference voltage (optional, VDD by default)
  VREF.CTRLA &= ~VREF_ADC0REFSEL_gm;  // Default reference to VDD
}

void loop() {
  uint32_t sum = 0;  // Accumulate ADC values
  for (int i = 0; i < NUM_SAMPLES; i++) {
    sum += readADC();
    delay(10);  // Small delay for ADC stability between readings
  }

  // Calculate the average ADC value
  uint16_t avgADCValue = sum / NUM_SAMPLES;
  float current_mA = (avgADCValue * 5) / 204.6;  // Convert to mA
//  float current_mA = adcValue;

  // Set LED based on current thresholds
  if (current_mA < LEVEL_A_CURRENT) {
    //  // Turn off all colors
//  PORTA.OUT = ~0;  // Set all to 1 (off)
    PORTA.OUT = ~((1 << RED));  // Active low: Set PA6 to 0, others to 1
  } 
  else if (current_mA >= LEVEL_A_CURRENT && current_mA <= LEVEL_B_CURRENT) {
     // Turn on Green (PA7), off Red and Blue
  PORTA.OUT = ~((1 << GREEN));  // Active low: Set PA7 to 0
  } 
  else {
    //  // Turn off all colors
//  PORTA.OUT = ~0;  // Set all to 1 (off)
      // Turn on Blue (PA1), off Red and Green
  PORTA.OUT = ~((1 << BLUE));  // Active low: Set PA1 to 0
  }

//  // Turn off all colors
//  PORTA.OUT = ~0;  // Set all to 1 (off)
//  delay(1000);

  delay(50);  // Delay for stability
}

uint16_t readADC() {
  uint16_t timeout = 1000;  // Timeout threshold (adjust as needed)
  
  // Start the ADC conversion
  ADC0.COMMAND = ADC_STCONV_bm;  

  // Wait for the result or timeout
  while (!(ADC0.INTFLAGS & ADC_RESRDY_bm) && timeout--) {
    // Optional: You can include a small delay here if desired
  }

  // Check if timeout occurred
  if (timeout == 0) {
    // Handle timeout error (e.g., return a default value, or set an error flag)
    return 0xFFFF;  // Example: Return an error code
  }

  // Return the ADC result
  return ADC0.RES;
}


void setLED(uint8_t pin) {
  PORTA.OUT = ~((1 << pin));  // Active low: Set the pin to 0 to turn on
}

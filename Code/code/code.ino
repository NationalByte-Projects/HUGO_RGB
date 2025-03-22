// Pin definitions
#define RED   1  // PA6
#define GREEN 6  // PA7
#define BLUE  7  // PA1
#define ADC_PIN   2  // AIN2 (PA2)
#define NUM_SAMPLES 30

// Current thresholds in mA
#define LEVEL_A_CURRENT  1.75f // Threshold A in mA
#define LEVEL_B_CURRENT  1.95f // Threshold B in mA

#include <avr/sleep.h>

void setup() {
    // Configure RGB pins as outputs
    PORTA.DIR |= (1 << RED) | (1 << GREEN) | (1 << BLUE);  // Set PA6, PA7, PA1 as outputs

    // Configure ADC0 to use PA2 (AIN2) as input
    ADC0.MUXPOS = ADC_MUXPOS_AIN2_gc;   // Select AIN2 (PA2) as input

    // Set the ADC clock prescaler for the slowest ADC clock
    ADC0.CTRLC = ADC_REFSEL_VDDREF_gc | ADC_PRESC_DIV256_gc;  // VDD as reference, slow clock

    // Enable the ADC with 10-bit resolution
    ADC0.CTRLA = ADC_ENABLE_bm | ADC_RESSEL_10BIT_gc;  // Enable ADC, 10-bit resolution

    // Set sampling length to maximum
    ADC0.SAMPCTRL = 0x1F;  // Maximum sampling length (31 ADC clock cycles)

    // Set sampling delay for noise reduction
    ADC0.CTRLD = 15;  // Maximum sampling delay (15 ADC clock cycles)

    // Disable digital input buffer on PA2 to reduce noise
    PORTA.PIN2CTRL |= PORT_ISC_INPUT_DISABLE_gc;

    // Allow some startup delay
    PORTA.OUT = ~0;  // Set all to 1 (off)
    delay(1000);
}

void loop() {
    // Take a single set of measurements
    uint32_t sum = 0;
    for (int i = 0; i < NUM_SAMPLES; i++) {
        sum += readADC();
        delay(10);  // Small delay for ADC stability between readings
    }

    // Calculate the average ADC value
    uint16_t avgADCValue = sum / NUM_SAMPLES;
    float current_mA = (avgADCValue * 5.13f) / 409.2f;  // Convert to mA

    // Set LED based on current thresholds
    if (current_mA < LEVEL_A_CURRENT) {
        setLED(RED);  // Turn on Red (PA6)
    } else if (current_mA >= LEVEL_A_CURRENT && current_mA <= LEVEL_B_CURRENT) {
        setLED(GREEN);  // Turn on Green (PA7)
    } else {
        setLED(BLUE);  // Turn on Blue (PA1)
    }

    // Enter sleep mode for low power consumption
    enterSleepMode();
}

uint16_t readADC() {
    uint16_t timeout = 1000;  // Timeout threshold (adjust as needed)

    // Start the ADC conversion
    ADC0.COMMAND = ADC_STCONV_bm;

    // Wait for the result or timeout
    while (!(ADC0.INTFLAGS & ADC_RESRDY_bm) && timeout--) {
        // Optional: Include a small delay here if needed
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
    // Set the LED pin (active low) and turn off others
    PORTA.OUT = ~((1 << pin));
}

void enterSleepMode() {
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);  // Set the sleep mode to the lowest power consumption
    sleep_enable();  // Enable sleep mode
    sleep_cpu();     // Enter sleep mode
    // The device will remain in sleep until an external reset or interrupt wakes it up
}

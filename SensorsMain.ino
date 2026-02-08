/*
Vref = 2v
V at 14cm = 1.8V
V at 42cm = 0.7V

ADC = Vin/Vref * 1024
ADC(14cm) = 1.8/2 * 1024 = 921.6 -> 922
ADC(42cm) = 0.7/2 * 1024 = 358.4 -> 358

V = m (1/D) + b
V - b = m/D
D (V - b) = m
D = m/(V-b)

therefore the distance formula is 
Distance = K/(ADC - C)

14 = K/(922-C)
42 = K/(358-C)
Solving...
c = 76
K = 11844
*/

#include <avr/io.h> // control registers
#include <util/delay.h> //delay_ms/delay_us

// CALIBRATION CONSTANTS
#define CALIB_K 11844
#define CALIB_C 76
#define ADC_MAX_READING 1023

// ULTRASONIC SENSOR DEFINITIONS
#define TRIG_PIN_MASK (1 << 0) // Trigger on Pin 8 (PORTB0)
#define ECHO_PIN_MASK (1 << 1) // Echo on Pin 9 (PORTB1)

// UART SETTINGS FOR DEBUGGING
#define BAUD 9600
#define MYUBRR F_CPU/16/BAUD-1

// SENSOR MODE
#define IR_MODE 0
#define US_MODE 1
uint8_t sensor_mode = IR_MODE;  // change to US_MODE when needed

// initialize UART 
void UART_init(unsigned int ubrr) {
  // set baud rate
  UBRR0H = (unsigned char)(ubrr >> 8);
  UBRR0L = (unsigned char)ubrr;

  //enable receiver and transmitter
  UCSR0B = (1 << RXEN0) | (1 << TXEN0);
  //8 data, 2 stop bit
  UCSR0C = (1 << USBS0) | (3 << UCSZ00);
}

// transmit a single character
void UART_transmit(unsigned char data) {
  // wait untill empty
  while(!(UCSR0A & (1 << UDRE0)))
  // send one byte
  UDR0 = data;
}

// transmit a number
void UART_printNum(unsigned int num){
  char buffer[10];
  itoa(num, buffer, 10); // convert int to string
  for (int i = 0; buffer[i] != '\0'; i++){
    UART_transmit(buffer[i]);
  }
}

// transmit a string
void UART_printString(const char* str) {                                                
    while (*str) { // loops untill null terminator
        UART_transmit(*str++); // sends each character
    }
}

// Initialize ADC
void ADC_init() {
    // Set Reference to EXTERNAL (AREF pin). 
    // REFS1=0, REFS0=0.
    // ADLAR=0 (Right adjust for full 10-bit resolution)
    ADMUX = 0x00; 

    // Enable ADC and set Prescaler to 128 (16MHz/128 = 125kHz)
    // ADEN=1 (Enable), ADPS2,1,0 = 111
    ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
}

// read value
unsigned int ADC_read(unsigned char channel) {
    // ensure channel is 0-7
    channel &= 0x07; 
    
    // Clear the bottom 4 bits (channel selection) and set new channel
    // keep the top bits 00 to ensure we stay on External Reference
    ADMUX = (ADMUX & 0xF8) | channel;

    // Start Conversion
    ADCSRA |= (1 << ADSC);

    // Wait for conversion to complete
    while (ADCSRA & (1 << ADSC));

    // Return the result
    return ADC;
}

// Convert Raw ADC to Centimeters
unsigned int get_distance_cm(unsigned int raw_adc) {
    // Safety
    if (raw_adc <= CALIB_C) {
        return 80; 
    }
    
    // If reading is too high, object too close, <10cm, clamp it.
    if (raw_adc > 1000) {
        return 9; 
    }

    // Distance
    unsigned int dist = CALIB_K / (raw_adc - CALIB_C);
    return dist;
}

void Ultrasonic_init() {
    // Configure Pins (DDRB Register)
    DDRB |= TRIG_PIN_MASK;   // Set PB0 to Output
    DDRB &= ~ECHO_PIN_MASK;  // Set PB1 to Input

    // Configure Timer1
    // TCCR1A = 0 (Normal mode)
    // TCCR1B: Set Prescaler to 8. 
    // 16MHz / 8 = 2MHz. Each "tick" is 0.5 microseconds.
    TCCR1A = 0;
    TCCR1B = (1 << CS11); // sets prescaler to 8
}

unsigned int Measure_Ultrasonic_Distance() {
    // SEND TRIGGER PULSE 
    // Set Trigger LOW for 2us to ensure clean start
    PORTB &= ~TRIG_PIN_MASK;
    _delay_us(2);
    
    // Set Trigger HIGH for 10us
    PORTB |= TRIG_PIN_MASK;
    _delay_us(10);
    
    // Set Trigger LOW again
    PORTB &= ~TRIG_PIN_MASK;

    // WAIT FOR ECHO TO START
    // The sensor takes a moment to drive the Echo pin HIGH.
    // We wait while the pin is LOW (checking PINB register).
    // Add a timeout counter so we don't hang forever if sensor is unplugged.
    long timeout = 50000;
    while (! (PINB & ECHO_PIN_MASK)) {
        if (--timeout == 0) return 0; // Error: No response
    }

    // MEASURE PULSE WIDTH
    // Echo is now HIGH. Start the stopwatch!
    TCNT1 = 0; // Reset Timer Counter to 0

    // while Echo is HIGH (checking PINB register)
    while (PINB & ECHO_PIN_MASK) {
        // Check for overflow (object too far, >400cm)
        // 60000 ticks * 0.5us = 30ms ~ 500cm
        if (TCNT1 > 60000) return 400; 
    }

    // CALCULATE DISTANCE
    // Read the final timer value
    unsigned int timer_ticks = TCNT1;

    // 1 tick = 0.5 microseconds (because of Prescaler 8)
    // Time (us) = ticks * 0.5 = ticks / 2
    // Distance (cm) = Time (us) / 58
    // Combined: Distance = (ticks / 2) / 58 = ticks / 116
    unsigned int distance_cm = timer_ticks / 116;
    return distance_cm;
}

void setup() {
    // Initialize our raw C functions
    UART_init(MYUBRR); 
    ADC_init();
    
    UART_printString("System Initialized. Vref=2.0V Mode.\n");
}

void loop() {

    if (sensor_mode == IR_MODE) {
        // read sensor
        unsigned int raw_val = ADC_read(0);          // IR on A0
        
        // convert to distance 
        unsigned int distance = get_distance_cm(raw_val);

        // print to serial monitor
        UART_printString("IR | ADC: ");
        UART_printNum(raw_val);
        UART_printString(" | Dist: ");
        UART_printNum(distance);
        UART_printString(" cm\n");
    }

    else if (sensor_mode == US_MODE) {
        unsigned int distance = Measure_Ultrasonic_Distance();

        UART_printString("US | Dist: ");
        UART_printNum(distance);
        UART_printString(" cm\n");
    }
    // 500 ms delay
    _delay_ms(500);
}


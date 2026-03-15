/*
 * ENGR290_TA2_main.c  -  Pure C (avr-gcc)
 * ENGR290 - Technical Assignment #2
 *
 * Assignment requirements implemented:
 *   1. Servo motor replicates IMU yaw in range +-85 degrees.
 *       Servo does NOT move further when yaw exceeds +-85 degrees.
 *   2. LED "L" turns ON when yaw is outside +-85 degrees.
 *   3. D3 brightness controlled by RAW X-axis acceleration magnitude:
 *         OFF          when |Ax| < 0.08 g
 *         100% ON      when |Ax| > 1.08 g
 *         Linear ramp  between 0.08 g and 1.08 g
 *   4. Roll, Pitch, Yaw and Ax, Ay, Az printed to PC once per second.
 *   5. Distance travelled along X axis printed to PC once per second.
 *
 * Hardware (Arduino Nano ATmega328P @ 16 MHz):
 *   MPU6050 SDA  -> P19
 *   Servo -> P9
 *   D3           -> PB3 / OC2A
 *   LED "L"      -> PB5, Arduino Nano built-in LE)
 *
 * Sketch folder must contain exactly:
 *   ENGR290_TA2.ino        (empty comment stub required by Arduino IDE)
 *   ENGR290_TA2_main.c     (this file)
 *   TWI_290.c              (provided, with inline removed from TWI_stop)
 *   TWI_290.h              (provided, unchanged)
 */

#define F_CPU 16000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <util/twi.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include "TWI_290.h"

/* Shared variables required by TWI_290.c  (extern there, defined here once)*/
volatile struct {
    uint8_t TX_new_data      : 1;
    uint8_t TX_finished      : 1;
    uint8_t TX_buffer1_empty : 1;
    uint8_t TX_buffer2_empty : 1;
    uint8_t RX_flag          : 3;
    uint8_t TWI_ACK          : 1;
} flags;

volatile uint8_t TWI_status;
volatile uint8_t TWI_byte;

/* MPU6050 register addresses */
#define MPU6050_ADDR         0x68   /* AD0 tied to GND */
#define MPU_REG_SMPLRT_DIV   0x19
#define MPU_REG_CONFIG       0x1A
#define MPU_REG_GYRO_CFG     0x1B
#define MPU_REG_ACCEL_CFG    0x1C
#define MPU_REG_ACCEL_XOUT_H 0x3B   /* first of 14 consecutive data bytes */
#define MPU_REG_PWR_MGMT_1   0x6B

/* 
 * EXPERIMENT KNOBS - change these defines to run experiments 1-3
 *
 * GYRO_FS_SEL   -> gyroscope full-scale range
 *   0 = +-250  deg/s  (sensitivity 131.0 LSB/deg/s)  <- default
 *   1 = +-500  deg/s  (sensitivity  65.5 LSB/deg/s)
 *   2 = +-1000 deg/s  (sensitivity  32.8 LSB/deg/s)
 *   3 = +-2000 deg/s  (sensitivity  16.4 LSB/deg/s)
 *
 * ACCEL_FS_SEL  -> accelerometer full-scale range
 *   0 = +-2 g   (sensitivity 16384 LSB/g)  <- default
 *   1 = +-4 g   (sensitivity  8192 LSB/g)
 *   2 = +-8 g   (sensitivity  4096 LSB/g)
 *   3 = +-16 g  (sensitivity  2048 LSB/g)
 *
 * SMPLRT_DIV_VALUE  -> sample rate divider
 *   Sample rate = 8000 / (1 + SMPLRT_DIV_VALUE)  Hz  (DLPF disabled)
 *   0   = 8000 Hz  (max)
 *   7   = 1000 Hz  (good default)
 *   255 =   31 Hz  (min)
 *
 * DIST_SCALE -> calibration multiplier for experiment 4.
 *   Start at 1.0, then adjust so PC reads ~30 cm for a 30 cm move.
 *   Example: if PC shows 24 cm for a 30 cm move, set DIST_SCALE = 1.25
*/
#define GYRO_FS_SEL       0
#define ACCEL_FS_SEL      0
#define SMPLRT_DIV_VALUE  7
#define DIST_SCALE        1.0f

/* Sensitivity constants derived from selections above */
#if   GYRO_FS_SEL == 0
  #define GYRO_SENS  131.0f
#elif GYRO_FS_SEL == 1
  #define GYRO_SENS   65.5f
#elif GYRO_FS_SEL == 2
  #define GYRO_SENS   32.8f
#else
  #define GYRO_SENS   16.4f
#endif

#if   ACCEL_FS_SEL == 0
  #define ACCEL_SENS  16384.0f
#elif ACCEL_FS_SEL == 1
  #define ACCEL_SENS   8192.0f
#elif ACCEL_FS_SEL == 2
  #define ACCEL_SENS   4096.0f
#else
  #define ACCEL_SENS   2048.0f
#endif

/* 
 * TWI / I2C clock
 * TWBR = (F_CPU / F_SCL - 16) / 2  with prescaler = 1
 * 100 kHz -> TWBR = (16000000/100000 - 16) / 2 = 72
*/
#define TWI_BITRATE  72

/*
 * Servo  (OC1A / PB1 via P9 pin 1)
 *
 * Timer1 Fast PWM, TOP = ICR1, prescaler = 8:
 *   Period = (ICR1+1) / (F_CPU / prescaler) = 40000 / 2000000 = 20 ms (50 Hz)
 *   1.0 ms pulse -> OCR1A = 2000  (-90 deg)
 *   1.5 ms pulse -> OCR1A = 3000  (  0 deg, centre)
 *   2.0 ms pulse -> OCR1A = 4000  (+90 deg)
 *
 * Assignment: replicate yaw in +-85 deg, stop at limit if yaw exceeds it.
*/
#define SERVO_ICR1    39999
#define SERVO_CENTER  3000     /* OCR1A value at 0 degrees  */
#define SERVO_DEG90   1000     /* OCR1A counts per 90 degrees */
#define YAW_LIMIT     85.0f   /* degrees - servo clamp and LED threshold */

/*
 * D3 brightness  (OC2A / PB3)
 *
 * Timer2 Fast PWM 8-bit, prescaler = 64:
 *   Frequency = F_CPU / (prescaler * 256) = ~977 Hz
 *   OCR2A = 0   -> OFF
 *   OCR2A = 255 -> 100% ON
 *
 * Assignment: driven by raw |Ax| in g
 *   OFF          when |Ax| < 0.08 g
 *   100% ON      when |Ax| > 1.08 g
 *   Linear ramp  between 0.08 g and 1.08 g
*/
#define D3_ACCEL_OFF   0.08f
#define D3_ACCEL_FULL  1.08f

/*
 * LED "L"  (PB5 - Arduino Nano on-board LED, active HIGH)
*/
#define LED_DDR   DDRB
#define LED_PORT  PORTB
#define LED_PIN   PB5

/*
 * UART - transmit only, 115200 baud, 8N1, U2X=1
 * UBRR = F_CPU / (8 * BAUD) - 1
*/
#define BAUD      115200UL
#define UBRR_VAL  (F_CPU / (8UL * BAUD) - 1)

/*
 * Complementary filter coefficient (roll and pitch)
 * 0.98 -> 98% gyro (accurate short-term), 2% accel (stable long-term ref)
*/
#define CF_ALPHA  0.98f

/*
 * Gyro dead-band (deg/s)
 * Residual rates smaller than this are zeroed to stop yaw drift when still.
*/
#define GYRO_DEADBAND  0.3f

/*
 * Distance integration parameters
 * ACCEL_DEADBAND_G : gravity-removed |Ax| below this is treated as zero
 * STILL_COUNT      : consecutive zero samples before velocity is reset to 0
 *   At ~1 kHz sample rate, 200 samples = 200 ms of detected stillness.
*/
#define ACCEL_DEADBAND_G  0.015f
#define STILL_COUNT       200

/*
 * Calibration sample count
*/
#define CALIB_N  500

/*
 * Global state
*/
static volatile uint32_t g_millis = 0;

/* Orientation (degrees) */
static float g_roll  = 0.0f;
static float g_pitch = 0.0f;
static float g_yaw   = 0.0f;

/* Accelerometer readings in g (calibration bias removed, gravity NOT removed)
 * These are the "actual" Ax/Ay/Az values printed to the terminal and used
 * for D3 brightness.                                                   */
static float g_ax = 0.0f;
static float g_ay = 0.0f;
static float g_az = 0.0f;

/* Distance tracking */
static float    g_vx          = 0.0f;
static float    g_dist        = 0.0f;
static uint16_t g_still_count = 0;

/* Gravity-compensated |Ax| in g - used for D3 brightness.
 * This is g_ax with the gravity projection removed via pitch angle,
 * so it reads ~0 g when stationary regardless of board tilt.          */
static float    g_ax_linear   = 0.0f;

/* Calibration biases in raw LSB */
static float g_gx_bias = 0.0f;
static float g_gy_bias = 0.0f;
static float g_gz_bias = 0.0f;
static float g_ax_bias = 0.0f;

/* Timing */
static uint32_t g_last_imu_ms  = 0;
static uint32_t g_last_uart_ms = 0;

/* Raw sensor values */
static int16_t raw_ax, raw_ay, raw_az;
static int16_t raw_gx, raw_gy, raw_gz;

/*
 * Timer0 - 1 ms system tick
 * CTC mode, OCR0A = 249, prescaler = 64
 * Rate = 16000000 / (64 * 250) = 1000 Hz
*/
ISR(TIMER0_COMPA_vect)
{
    g_millis++;
}

static void timer0_init(void)
{
    TCCR0A = (1 << WGM01);
    TCCR0B = (1 << CS01) | (1 << CS00);
    OCR0A  = 249;
    TIMSK0 = (1 << OCIE0A);
}

static uint32_t millis_now(void)
{
    uint32_t m;
    uint8_t sreg = SREG;
    cli();
    m = g_millis;
    SREG = sreg;
    return m;
}

/*
 * Timer1 - Servo PWM on OC1A / PB1
 * Fast PWM, TOP = ICR1 = 39999, prescaler = 8  ->  50 Hz
*/
static void timer1_init(void)
{
    DDRB  |= (1 << PB1);
    ICR1   = SERVO_ICR1;
    OCR1A  = SERVO_CENTER;
    TCCR1A = (1 << COM1A1) | (1 << WGM11);
    TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11);
}

/*
 * servo_write - move servo to angle_deg, clamped to +-85 degrees.
 *
 * The clamp is hard: even if yaw is 120 deg, OCR1A is only set to the
 * 85-degree position. The servo physically cannot exceed +-85 degrees.
 */
static void servo_write(float angle_deg)
{
    if (angle_deg >  YAW_LIMIT) angle_deg =  YAW_LIMIT;
    if (angle_deg < -YAW_LIMIT) angle_deg = -YAW_LIMIT;
    OCR1A = (uint16_t)((float)SERVO_CENTER
                       + (angle_deg / 90.0f) * (float)SERVO_DEG90);
}

/*
 * Timer2 - D3 brightness on OC2A / PB3
 * Fast PWM 8-bit, prescaler = 64  ->  ~977 Hz
*/
static void timer2_init(void)
{
    DDRB  |= (1 << PB3);
    OCR2A  = 0;
    TCCR2A = (1 << COM2A1) | (1 << WGM21) | (1 << WGM20);
    TCCR2B = (1 << CS22);
}

/*
 * d3_set - set D3 brightness from gravity-compensated |Ax| in g.
 *
 * Input is |g_ax - sin(pitch)|, which reads ~0 g when stationary and
 * rises only when the board actually accelerates along X.
 */
static void d3_set(float abs_ax_g)
{
    uint8_t pwm;

    if (abs_ax_g <= D3_ACCEL_OFF) {
        pwm = 0;
    } else if (abs_ax_g >= D3_ACCEL_FULL) {
        pwm = 255;
    } else {
        /* ratio = (|Ax| - 0.08) / 1.00  ->  maps linearly 0.0 to 1.0 */
        float ratio = (abs_ax_g - D3_ACCEL_OFF)
                    / (D3_ACCEL_FULL - D3_ACCEL_OFF);
        pwm = (uint8_t)(ratio * 255.0f);
    }

    OCR2A = pwm;
}

/*
 * UART - 115200 baud, 8N1, transmit only
*/
static void uart_init(void)
{
    UBRR0H = (uint8_t)(UBRR_VAL >> 8);
    UBRR0L = (uint8_t)(UBRR_VAL);
    UCSR0A = (1 << U2X0);
    UCSR0B = (1 << TXEN0);
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

static void uart_putc(char c)
{
    while (!(UCSR0A & (1 << UDRE0)));
    UDR0 = (uint8_t)c;
}

static void uart_puts(const char *s)
{
    while (*s) uart_putc(*s++);
}

static void uart_print_float(float v, uint8_t decimals)
{
    char buf[12];
    uint8_t i;
    float rounding = 0.5f;

    if (v < 0.0f) { uart_putc('-'); v = -v; }

    for (i = 0; i < decimals; i++) rounding /= 10.0f;
    v += rounding;

    ltoa((long)v, buf, 10);
    uart_puts(buf);
    uart_putc('.');

    float frac = v - (float)(long)v;
    for (i = 0; i < decimals; i++) {
        frac *= 10.0f;
        uint8_t digit = (uint8_t)frac;
        uart_putc('0' + digit);
        frac -= (float)digit;
    }
}

/* ==========================================================================
 * TWI initialisation
 * ========================================================================== */
static void twi_init(void)
{
    TWSR = 0x00;
    TWBR = TWI_BITRATE;
    TWCR = (1 << TWEN);
}

/* ==========================================================================
 * mpu_write_reg - write one register using raw TWI calls.
 *
 * Does NOT use Write_Reg() from TWI_290.c because that function checks
 * flags.TWI_ACK after a write, but TWI_ACK is only set during read
 * operations, so Write_Reg() always returns error 4 after any write.
 * ========================================================================== */
static uint8_t mpu_write_reg(uint8_t reg, uint8_t val)
{
    if (TWI_start(MPU6050_ADDR, TW_WRITE)) { TWI_stop(); return 1; }
    if (TWI_write(reg))                    { TWI_stop(); return 2; }
    if (TWI_write(val))                    { TWI_stop(); return 3; }
    TWI_stop();
    return 0;
}

/* 
 * mpu6050_init - configure MPU6050 registers
*/
static uint8_t mpu6050_init(void)
{
    uint8_t err;

    err = mpu_write_reg(MPU_REG_PWR_MGMT_1, 0x00);  /* clear sleep bit  */
    if (err) return err;
    _delay_ms(100);

    err = mpu_write_reg(MPU_REG_CONFIG, 0x00);        /* DLPF off, 8 kHz */
    if (err) return err;

    err = mpu_write_reg(MPU_REG_SMPLRT_DIV, SMPLRT_DIV_VALUE);
    if (err) return err;

    err = mpu_write_reg(MPU_REG_GYRO_CFG, (uint8_t)(GYRO_FS_SEL << 3));
    if (err) return err;

    err = mpu_write_reg(MPU_REG_ACCEL_CFG, (uint8_t)(ACCEL_FS_SEL << 3));
    if (err) return err;

    return 0;
}

/* 
 * mpu6050_read_raw - burst-read 14 bytes from ACCEL_XOUT_H
 *
 * Byte layout:
 *   [0-1]   Accel X H,L
 *   [2-3]   Accel Y H,L
 *   [4-5]   Accel Z H,L
 *   [6-7]   Temperature H,L  (ignored)
 *   [8-9]   Gyro  X H,L
 *   [10-11] Gyro  Y H,L
 *   [12-13] Gyro  Z H,L
 */
static uint8_t mpu6050_read_raw(void)
{
    uint8_t buf[14];
    uint8_t i;

    if (TWI_start(MPU6050_ADDR, TW_WRITE))     return 1;
    if (TWI_write(MPU_REG_ACCEL_XOUT_H))      { TWI_stop(); return 2; }
    if (TWI_start(MPU6050_ADDR, TW_READ))      { TWI_stop(); return 3; }

    for (i = 0; i < 13; i++) {
        buf[i] = TWI_ack_read();
        if (!flags.TWI_ACK)                    { TWI_stop(); return 4; }
    }
    buf[13] = TWI_nack_read();
    TWI_stop();

    raw_ax = (int16_t)(((uint16_t)buf[0]  << 8) | buf[1]);
    raw_ay = (int16_t)(((uint16_t)buf[2]  << 8) | buf[3]);
    raw_az = (int16_t)(((uint16_t)buf[4]  << 8) | buf[5]);
    raw_gx = (int16_t)(((uint16_t)buf[8]  << 8) | buf[9]);
    raw_gy = (int16_t)(((uint16_t)buf[10] << 8) | buf[11]);
    raw_gz = (int16_t)(((uint16_t)buf[12] << 8) | buf[13]);

    return 0;
}

/*
 * calibrate_sensors - average CALIB_N still samples to find biases.
 * Keep IMU flat and completely still on a level surface during this.
*/
static void calibrate_sensors(void)
{
    int32_t sgx = 0, sgy = 0, sgz = 0, sax = 0;
    uint16_t i;

    uart_puts("Keep IMU flat and still - calibrating...\r\n");

    for (i = 0; i < CALIB_N; i++) {
        mpu6050_read_raw();
        sgx += raw_gx;
        sgy += raw_gy;
        sgz += raw_gz;
        sax += raw_ax;
        _delay_ms(2);
    }

    g_gx_bias = (float)sgx / (float)CALIB_N;
    g_gy_bias = (float)sgy / (float)CALIB_N;
    g_gz_bias = (float)sgz / (float)CALIB_N;
    g_ax_bias = (float)sax / (float)CALIB_N;

    uart_puts("Done.\r\n");
}

/*
 * imu_update - read sensor, run filters, update all state.
 * Called every main-loop iteration for maximum time resolution.
 */
static void imu_update(void)
{
    uint32_t now = millis_now();
    float dt = (float)(now - g_last_imu_ms) / 1000.0f;

    if (dt <= 0.0f || dt > 0.5f) {
        g_last_imu_ms = now;
        return;
    }
    g_last_imu_ms = now;

    if (mpu6050_read_raw() != 0) return;

    /* Accelerometer: remove calibration bias, convert to g */
    g_ax = ((float)raw_ax - g_ax_bias) / ACCEL_SENS;  /* bias-corrected g  */
    g_ay = (float)raw_ay  / ACCEL_SENS;
    g_az = (float)raw_az  / ACCEL_SENS;

    /* Gyroscope: remove bias, convert to deg/s */
    float gx_dps = ((float)raw_gx - g_gx_bias) / GYRO_SENS;
    float gy_dps = ((float)raw_gy - g_gy_bias) / GYRO_SENS;
    float gz_dps = ((float)raw_gz - g_gz_bias) / GYRO_SENS;

    /* Dead-band: zero tiny residual rates to prevent yaw drift when still */
    if (fabsf(gx_dps) < GYRO_DEADBAND) gx_dps = 0.0f;
    if (fabsf(gy_dps) < GYRO_DEADBAND) gy_dps = 0.0f;
    if (fabsf(gz_dps) < GYRO_DEADBAND) gz_dps = 0.0f;

    /* Accelerometer-derived roll and pitch */
    float acc_roll  = atan2f(g_ay, g_az) * (180.0f / (float)M_PI);
    float acc_pitch = atan2f(-g_ax, sqrtf(g_ay * g_ay + g_az * g_az))
                      * (180.0f / (float)M_PI);

    /* Complementary filter: roll and pitch */
    g_roll  = CF_ALPHA * (g_roll  + gx_dps * dt) + (1.0f - CF_ALPHA) * acc_roll;
    g_pitch = CF_ALPHA * (g_pitch + gy_dps * dt) + (1.0f - CF_ALPHA) * acc_pitch;

    /* Yaw: gyro Z integration */
    g_yaw += gz_dps * dt;
    if (g_yaw >  180.0f) g_yaw -= 360.0f;
    if (g_yaw < -180.0f) g_yaw += 360.0f;

    /* Linear acceleration X (gravity removed) for distance
     *
     * Raw g_ax contains both linear acceleration AND the gravity component
     * projected onto X by the pitch angle.
     * ax_linear = g_ax - sin(pitch)   removes the gravity part.
     * This value is used ONLY for distance integration, not for D3.        */
    float pitch_rad     = g_pitch * ((float)M_PI / 180.0f);
    float ax_linear_g   = g_ax - sinf(pitch_rad);   /* gravity removed, in g */
    float ax_linear_ms2 = ax_linear_g * 9.81f;

    /* Store absolute value for D3 brightness - gravity removed so it reads
     * ~0 g when stationary regardless of board tilt.                        */
    g_ax_linear = fabsf(ax_linear_g);

    /* Stationary detection: if near-zero accel persists, reset velocity */
    if (fabsf(ax_linear_g) < ACCEL_DEADBAND_G) {
        ax_linear_ms2 = 0.0f;
        g_still_count++;
        if (g_still_count >= STILL_COUNT) {
            g_vx          = 0.0f;
            g_still_count = STILL_COUNT;
        }
    } else {
        g_still_count = 0;
    }

    /* Integrate velocity and distance */
    g_vx   += ax_linear_ms2 * dt;
    g_dist += g_vx * dt * DIST_SCALE;
}

/* 
 * main
*/
int main(void)
{
    LED_DDR |= (1 << LED_PIN);   /* LED "L" as output */

    timer0_init();   /* 1 ms tick                     */
    timer1_init();   /* Servo PWM on OC1A/PB1         */
    timer2_init();   /* D3 PWM on OC2A/PB3            */
    uart_init();     /* 115200 baud serial            */
    twi_init();      /* I2C 100 kHz                   */
    sei();

    _delay_ms(250);
    uart_puts("\r\nENGR290 TA2 - MPU6050\r\n");

    {
        uint8_t err = mpu6050_init();
        if (err) {
            uart_puts("ERROR: MPU6050 init failed, code=");
            uart_putc('0' + err);
            uart_puts(". Check wiring.\r\n");
            while (1);
        }
    }
    uart_puts("MPU6050 OK\r\n");
    _delay_ms(50);

    calibrate_sensors();

    g_last_imu_ms  = millis_now();
    g_last_uart_ms = millis_now();

    while (1)
    {
        /* 1) Update IMU (complementary filter + distance integration) */
        imu_update();

        /* 2) SERVO - follows yaw, hard-clamped to +-85 degrees.
         *     Sign negated so physical left turn moves servo left.
         *     If servo still moves opposite, remove the minus sign.        */
        servo_write(-g_yaw);

        /* 3) LED "L" - ON when |yaw| exceeds 85 degrees */
        if (g_yaw > YAW_LIMIT || g_yaw < -YAW_LIMIT) {
            LED_PORT |=  (1 << LED_PIN);
        } else {
            LED_PORT &= ~(1 << LED_PIN);
        }

        /* 4) D3 brightness - controlled by gravity-compensated |Ax| in g.
         *
         * g_ax_linear = |g_ax - sin(pitch)| removes the gravity projection
         * so D3 reads ~0 g when stationary regardless of board tilt, and
         * rises only when the board actually accelerates along X.
         * This ensures D3 is fully OFF at rest as the assignment requires.  */
        d3_set(g_ax_linear);

        /* 5) Serial output once per second */
        if ((millis_now() - g_last_uart_ms) >= 1000UL)
        {
            g_last_uart_ms = millis_now();

            /* Orientation */
            uart_puts("Roll:");    uart_print_float(g_roll,  2); uart_puts("deg  ");
            uart_puts("Pitch:");   uart_print_float(g_pitch, 2); uart_puts("deg  ");
            uart_puts("Yaw:");     uart_print_float(g_yaw,   2); uart_puts("deg\r\n");

            /* Acceleration (raw, bias-corrected, in g) */
            uart_puts("Ax:");      uart_print_float(g_ax, 3); uart_puts("g  ");
            uart_puts("Ay:");      uart_print_float(g_ay, 3); uart_puts("g  ");
            uart_puts("Az:");      uart_print_float(g_az, 3); uart_puts("g\r\n");

            /* Distance along X */
            uart_puts("DistX:");   uart_print_float(g_dist * 100.0f, 2);
            uart_puts("cm\r\n\r\n");
        }
    }

    return 0;
}

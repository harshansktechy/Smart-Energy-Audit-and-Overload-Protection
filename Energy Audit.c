

#pragma config FOSC  = HS
#pragma config WDTE  = OFF
#pragma config PWRTE = ON
#pragma config BOREN = ON
#pragma config LVP   = OFF
#pragma config CPD   = OFF
#pragma config WRT   = OFF
#pragma config CP    = OFF

#include <xc.h>
#include <stdint.h>

#define _XTAL_FREQ  20000000UL

/* ?? ACS712 Sensor Constants (millivolt-based) ???????????????????? */
#define ADC_VREF_mV        5000UL
#define ADC_RESOLUTION     1023UL
#define ACS_ZERO_mV        2500
#define ACS_SENS_mVpA      185

/* ?? Load / Threshold Constants ??????????????????????????????????? */
#define SUPPLY_V_x10       120
#define NOISE_FLOOR_mA     20
#define OVERLOAD_LIMIT_mA  320 

/* ?? Relay : RB0, Active LOW ??????????????????????????????????????? */
#define RELAY_ON()    do { PORTBbits.RB0 = 0; } while(0)
#define RELAY_OFF()   do { PORTBbits.RB0 = 1; } while(0)

/* ?? Buzzer : RB2, Active HIGH ????????????????????????????????????? */
#define BUZZER_ON()   do { PORTBbits.RB2 = 1; } while(0)
#define BUZZER_OFF()  do { PORTBbits.RB2 = 0; } while(0)

static void buzzer_beep(uint16_t on_ms, uint16_t off_ms) {
    uint16_t i;
    BUZZER_ON();
    for (i = 0; i < on_ms;  i++) __delay_ms(1);
    BUZZER_OFF();
    for (i = 0; i < off_ms; i++) __delay_ms(1);
}

/* ?? Globals ??????????????????????????????????????????????????????? */
volatile uint8_t g_overload  = 0;
volatile uint8_t g_tick_flag = 0;

int16_t  g_current_mA = 0;
uint16_t g_power_mW   = 0;
uint32_t g_energy_mWs = 0;

static volatile uint8_t tmr0_cnt = 0;
#define TMR0_OVF_PER_SEC  76

/* ============================================================
 * UART DRIVER  (9600 baud, 8N1, TX on RC6)
 * ============================================================
 * Baud rate calculation for 9600 @ 20 MHz, BRGH=1:
 *   SPBRG = (Fosc / (16 x Baud)) - 1
 *         = (20000000 / (16 x 9600)) - 1
 *         = 129
 * ============================================================ */
static void uart_init(void) {
    TRISCbits.TRISC6 = 0;    /* RC6 = TX output                */
    TRISCbits.TRISC7 = 1;    /* RC7 = RX input (unused but set) */
    SPBRG  = 129;             /* 9600 baud @ 20 MHz, BRGH=1     */
    TXSTA  = 0x24;            /* TXEN=1, BRGH=1, async mode     */
    RCSTA  = 0x90;            /* SPEN=1, CREN=1                 */
}

/* Send one character */
static void uart_putch(char c) {
    while (!TXSTAbits.TRMT);  /* wait for transmit shift reg empty */
    TXREG = (uint8_t)c;
}

/* Send a null-terminated string */
static void uart_print(const char *s) {
    while (*s) uart_putch(*s++);
}

/*
 * uart_print_uint()
 * Sends an unsigned 32-bit integer as ASCII digits.
 */
static void uart_print_uint(uint32_t val) {
    char buf[11]; int8_t i = 0;
    if (val == 0) { uart_putch('0'); return; }
    while (val) { buf[i++] = (char)('0' + (val % 10)); val /= 10; }
    while (--i >= 0) uart_putch(buf[i]);
}

/*
 * uart_print_scaled()
 * Sends a fixed-point integer with 'dp' decimal places.
 * Mirrors lcd_print_scaled() exactly so terminal output
 * matches what the LCD shows.
 *   e.g. val=237, dp=3  ?  "0.237"
 *        val=2840, dp=3 ?  "2.840"
 */
static void uart_print_scaled(int16_t val, uint8_t dp) {
    uint8_t  d;
    uint16_t divisor = 1;
    if (val < 0) { uart_putch('-'); val = (int16_t)(-val); }
    for (d = 0; d < dp; d++) divisor *= 10;
    uart_print_uint((uint32_t)((uint16_t)val / divisor));
    if (dp > 0) {
        uart_putch('.');
        uint16_t frac = (uint16_t)val % divisor;
        uint16_t tmp  = divisor / 10;
        while (tmp > 1) {
            uart_putch((char)('0' + (frac / tmp)));
            frac %= tmp;
            tmp  /= 10;
        }
        uart_putch((char)('0' + frac));
    }
}

/*
 * uart_print_values()
 * Called once per second after display_normal().
 * Sends one line to the terminal:
 *   I=0.237A  P=2.840W  E=0.002Wh\r\n
 */
static void uart_print_values(void) {
    uart_print("I=");
    uart_print_scaled(g_current_mA, 3);
    uart_print("A  P=");
    uart_print_scaled((int16_t)g_power_mW, 3);
    uart_print("W  E=");
    uart_print_scaled((int16_t)(g_energy_mWs / 3600UL), 3);
    uart_print("Wh\r\n");
}

/* ============================================================
 * LCD DRIVER (4-bit, PORTD)
 * ============================================================ */
static void lcd_send(uint8_t data, uint8_t rs_flag) {
    PORTDbits.RD2 = rs_flag;
    PORTDbits.RD4 = (data >> 4) & 1;
    PORTDbits.RD5 = (data >> 5) & 1;
    PORTDbits.RD6 = (data >> 6) & 1;
    PORTDbits.RD7 = (data >> 7) & 1;
    PORTDbits.RD3 = 1; __delay_us(50); PORTDbits.RD3 = 0; __delay_us(50);
    PORTDbits.RD4 = (data >> 0) & 1;
    PORTDbits.RD5 = (data >> 1) & 1;
    PORTDbits.RD6 = (data >> 2) & 1;
    PORTDbits.RD7 = (data >> 3) & 1;
    PORTDbits.RD3 = 1; __delay_us(50); PORTDbits.RD3 = 0;
    if (rs_flag == 0) __delay_ms(2); else __delay_us(50);
}

static void lcd_cmd(uint8_t cmd)  { lcd_send(cmd, 0); }
static void lcd_char(char c)      { lcd_send((uint8_t)c, 1); }

static void lcd_init(void) {
    __delay_ms(50);
    PORTDbits.RD2 = 0;
    PORTDbits.RD4 = 1; PORTDbits.RD5 = 1;
    PORTDbits.RD6 = 0; PORTDbits.RD7 = 0;
    PORTDbits.RD3 = 1; __delay_us(50); PORTDbits.RD3 = 0; __delay_ms(5);
    PORTDbits.RD3 = 1; __delay_us(50); PORTDbits.RD3 = 0; __delay_us(200);
    PORTDbits.RD3 = 1; __delay_us(50); PORTDbits.RD3 = 0; __delay_us(200);
    PORTDbits.RD4 = 0; PORTDbits.RD5 = 1;
    PORTDbits.RD6 = 0; PORTDbits.RD7 = 0;
    PORTDbits.RD3 = 1; __delay_us(50); PORTDbits.RD3 = 0; __delay_ms(2);
    lcd_cmd(0x28); lcd_cmd(0x0C); lcd_cmd(0x06); lcd_cmd(0x01);
    __delay_ms(2);
}

static void lcd_set_cursor(uint8_t row, uint8_t col) {
    lcd_cmd((uint8_t)((row == 0 ? 0x80 : 0xC0) + col));
}

static void lcd_print(const char *str) {
    while (*str) lcd_char(*str++);
}

static void lcd_print_uint32(uint32_t val) {
    char buf[11]; int8_t i = 0;
    if (val == 0) { lcd_char('0'); return; }
    while (val) { buf[i++] = (char)('0' + (val % 10)); val /= 10; }
    while (--i >= 0) lcd_char(buf[i]);
}

static void lcd_print_scaled(int16_t val, uint8_t dp) {
    uint8_t  d;
    uint16_t divisor = 1;
    if (val < 0) { lcd_char('-'); val = (int16_t)(-val); }
    for (d = 0; d < dp; d++) divisor *= 10;
    lcd_print_uint32((uint32_t)((uint16_t)val / divisor));
    if (dp > 0) {
        lcd_char('.');
        uint16_t frac = (uint16_t)val % divisor;
        uint16_t tmp  = divisor / 10;
        while (tmp > 1) {
            lcd_char((char)('0' + (frac / tmp)));
            frac %= tmp;
            tmp  /= 10;
        }
        lcd_char((char)('0' + frac));
    }
}

/* ============================================================
 * ADC ? 32x oversampling on AN0 (RA0)
 * ============================================================ */
static void adc_init(void) {
    ADCON1 = 0x8E;
    ADCON0 = 0xC1;
    __delay_us(20);
}

static uint16_t adc_read(void) {
    uint32_t sum = 0;
    uint8_t  i;
    for (i = 0; i < 32; i++) {
        GO_nDONE = 1;
        while (GO_nDONE);
        sum += (((uint16_t)ADRESH << 8) | ADRESL);
        __delay_us(50);
    }
    return (uint16_t)(sum / 32);
}

/* ============================================================
 * TIMER 0 & ISR
 * ============================================================ */
static void timer0_init(void) {
    OPTION_REGbits.T0CS = 0;
    OPTION_REGbits.PSA  = 0;
    OPTION_REGbits.PS   = 0b111;
    TMR0 = 0;
    INTCONbits.T0IF = 0;
    INTCONbits.T0IE = 1;
}

void __interrupt() isr(void) {
    if (PIE2bits.CMIE && PIR2bits.CMIF) {
        if (CMCONbits.C1OUT == 1) {
            RELAY_OFF();
            g_overload = 1;
        }
        PIR2bits.CMIF = 0;
    }
    if (INTCONbits.T0IE && INTCONbits.T0IF) {
        INTCONbits.T0IF = 0;
        if (++tmr0_cnt >= TMR0_OVF_PER_SEC) {
            tmr0_cnt = 0;
            g_tick_flag = 1;
        }
    }
}

/* ============================================================
 * DISPLAY HELPERS
 * ============================================================ */
static void display_normal(void) {
    lcd_set_cursor(0, 0);
    lcd_print("I=");
    lcd_print_scaled(g_current_mA, 3);
    lcd_print("A ");

    lcd_set_cursor(0, 9);
    lcd_print("P=");
    lcd_print_scaled((int16_t)g_power_mW, 3);
    lcd_print("W");

    lcd_set_cursor(1, 0);
    lcd_print("E=");
    lcd_print_scaled((int16_t)(g_energy_mWs / 3600UL), 3);
    lcd_print("Wh      ");
}

static void lock_overload(void) {
    RELAY_OFF();
    BUZZER_OFF();
    INTCONbits.GIE = 0;

    lcd_cmd(0x01); __delay_ms(2);
    lcd_set_cursor(0, 0); lcd_print("** OVERLOAD!! **");
    lcd_set_cursor(1, 0); lcd_print("RELAY TRIPPED   ");

    uart_print("** OVERLOAD - RELAY TRIPPED **\r\n");

    while (1) {
        BUZZER_ON();  __delay_ms(100);
        BUZZER_OFF(); __delay_ms(100);
        BUZZER_ON();  __delay_ms(100);
        BUZZER_OFF(); __delay_ms(300);

        lcd_set_cursor(1, 0); lcd_print("                ");
        __delay_ms(400);
        lcd_set_cursor(1, 0); lcd_print("RELAY TRIPPED   ");
    }
}

/* ============================================================
 * MAIN
 * ============================================================ */
int main(void) {
    TRISA = 0xFF;          /* RA0 = ADC input                         */
    TRISB = 0x00;          /* RB0 = relay output, RB2 = buzzer output */
    TRISD = 0x00;          /* PORTD = LCD outputs                     */

    /*
     * RB0 = 1  relay OFF (active LOW, safe at startup)
     * RB2 = 0  buzzer OFF (active HIGH, kept LOW before lcd_init
     *          so buzzer current does not corrupt LCD init sequence)
     */
    PORTB = 0x01;
    PORTD = 0x00;

    lcd_init();
    adc_init();
    timer0_init();
    uart_init();   /* 9600 8N1 on RC6(TX) ? connect to USB-UART adapter */

    lcd_set_cursor(0, 0); lcd_print("Motor Booting...");
    uart_print("Motor Booting...\r\n");

    /* Single startup beep - confirms buzzer is wired correctly */
    buzzer_beep(200, 0);

    RELAY_ON();
    __delay_ms(500); __delay_ms(500);
    __delay_ms(500); __delay_ms(500);
    lcd_cmd(0x01); __delay_ms(2);

    /* Hardware comparator guard (trips ~5 A shorts) */
    CVRCON = 0b10001111;   /* VR=15: CVREF=3.125V, trips ~3.38A (doubled) */
    CMCON  = 0b00010110;
    PIR2bits.CMIF   = 0;
    PIE2bits.CMIE   = 1;
    INTCONbits.PEIE = 1;
    INTCONbits.GIE  = 1;

    /* ================================================================
     * MAIN LOOP
     * ================================================================ */
    while (1) {

        /* Overload latch (comparator ISR or software guard) */
        if (g_overload) lock_overload();   /* never returns */

        /* 1-second energy tick */
        if (!g_tick_flag) continue;
        g_tick_flag = 0;

        /* STEP 1: ADC ? millivolts */
        uint16_t raw  = adc_read();
        uint16_t v_mV = (uint16_t)((uint32_t)raw * ADC_VREF_mV / ADC_RESOLUTION);

        /* STEP 2: millivolts ? milliamps */
        int16_t diff_mV  = (int16_t)v_mV - ACS_ZERO_mV;
        g_current_mA = (int16_t)((int32_t)diff_mV * 1000 / ACS_SENS_mVpA);
        if (g_current_mA < 0)              g_current_mA = -g_current_mA;
        if (g_current_mA < NOISE_FLOOR_mA) g_current_mA = 0;

        /* STEP 3: Software overload guard */
        if (g_current_mA >= OVERLOAD_LIMIT_mA) {
            g_overload = 1;
            lock_overload();   /* never returns */
        }

        /* STEP 4: Power & energy accumulation */
        g_power_mW    = (uint16_t)((uint32_t)SUPPLY_V_x10 * (uint16_t)g_current_mA / 10UL);
        g_energy_mWs += (uint32_t)g_power_mW;

        /* Update LCD and send values to UART terminal */
        display_normal();
        uart_print_values();   /* ? I=0.237A  P=2.840W  E=0.002Wh  */
    }

    return 0;
}
#include <inttypes.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/pgmspace.h>
#include <util/atomic.h>
#include <stdio.h>
#include <ctype.h>
#include <string.h>

#define F_CPU		12000000
#include <util/delay.h>

/*-------------------------------------------------------------------------*/
#define INACT_TIMEOUT		 20     /* s */

#define TIMER_PRESCALER	64

/* timing formulas */
#define MS(x)		((x) * (F_CPU / 1000) / (TIMER_PRESCALER))
#define US(x)		(MS(x) / 1000)
#define SECS_TO_OVF(x)	((x) * F_CPU / (TIMER_PRESCALER * 65536))

#define VPW_SHORT_PULSE_MIN	 35     /* us */
#define VPW_LONG_PULSE_MIN	 97     /* us */
#define VPW_SOF_MIN		164     /* us */
#define VPW_EOF_MIN		240     /* us */


/*-------------------------------------------------------------------------*/
#define OUTSIZE 96
#define MIDSIZE 80
#define JBSIZE 24
static unsigned char  jmsgbuf[JBSIZE], midbuf[MIDSIZE], outbuf[OUTSIZE], outhead, outtail,  midlen, jmsglen;
#define UPUTC(x) { outbuf[outhead++] = (x); if (outhead >= OUTSIZE) outhead = 0; }
#define UGETC(x) { (x) = outbuf[outtail++]; if (outtail >= OUTSIZE) outtail = 0; }

// time in TOVFs to make the UART RX to TX transparent (dump other traffic).
static unsigned transptime;

// add buffered data from J1850 or PPS or whereever to output buffer
static void sendmid()
{
    if( transptime )
	return;
    unsigned char *mp = midbuf;
    while (midlen) {
	UPUTC(*mp++);
        midlen--;
    }
}

/*-------------------------------------------------------------------------*/
static unsigned lasttcnt, hightime, marktime, marklow;
#define checkhitime(t) { if( t < lasttcnt )hightime++; lasttcnt = t;}

/*-------------------------------------------------------------------------*/
static unsigned char innmea; // middle of NMEA, wait for end to add interleaved
// Received character from GPS - put to transmit buffer, but also handle other cases
ISR(USART_RX_vect)
{
    unsigned char c = UDR;

    if (!c || c == 0xa0)
        transptime = SECS_TO_OVF(10);
    if (c == '$' && !transptime)
	innmea = 1;
    UPUTC(c);
    if (midlen && (!c || c == '\n')) {
	innmea = 0;
	// append any queued lines
        if (transptime)
            midlen = 0;         // dump during config
        else
            sendmid();
    }
    // enable send
    UCSRB |= _BV(UDRIE);
}

// Send next character from transmit buffer
ISR(USART_UDRE_vect)
{
    // disable interrupt when buffer empty
    if (outhead == outtail) {
        UCSRB &= ~_BV(UDRIE);
        return;
    }
    UGETC(UDR);
}

/*-------------------------------------------------------------------------*/
static unsigned char tsout[3];
static void timestamp(unsigned lowtime) {
    unsigned long mark = marktime;
    mark <<= 16;
    mark |= marklow;
    checkhitime(lowtime);
    unsigned long now = hightime;
    now <<= 16;
    now |= lowtime;
    now -= mark;
    if( (now & 0xffff0000UL) == 0xffff0000UL )
	now &= 0xffffUL;
    if( now > (F_CPU/TIMER_PRESCALER) ) {
	now -= (F_CPU/TIMER_PRESCALER);
	if( now > 8*(F_CPU/TIMER_PRESCALER) ) { // reset mark
	    marklow = TCNT1;
	    marktime = hightime;
	    now = 0;
	}
	else {
	    mark += (F_CPU/TIMER_PRESCALER); // move mark 1 second
	    marklow = mark;
	    marktime = mark >> 16;
	}
    }
    while( now >= (F_CPU/TIMER_PRESCALER) )
	now -= (F_CPU/TIMER_PRESCALER);
    unsigned char digit = '0';
    while( now >= (F_CPU/TIMER_PRESCALER/10) )
	digit++, now -= (F_CPU/TIMER_PRESCALER/10);
    tsout[0] = digit;
    digit = '0';
    while( now >= (F_CPU/TIMER_PRESCALER/100) )
	digit++, now -= (F_CPU/TIMER_PRESCALER/100);
    tsout[1] = digit;
    digit = '0';
    while( now > (F_CPU/TIMER_PRESCALER/1000) && digit < '9')
	digit++, now -= (F_CPU/TIMER_PRESCALER/1000);
    tsout[2] = digit;
}
// PPS on the UTC second mark

ISR(INT1_vect)
{
    if( transptime )
	return;
    //    PORTB |= _BV(PB2);
    unsigned t = TCNT1;

    timestamp(t);
    midbuf[midlen++] = tsout[0];
    midbuf[midlen++] = tsout[1];
    midbuf[midlen++] = tsout[2];
    midbuf[midlen++] = '=';
    marktime = hightime;
    marklow = t;
    //    if( TIFR & _BV(TOV1) ) // timer overflowed while in this interrupt
    //	marktime++;
    midbuf[midlen++] = '\r';
    midbuf[midlen++] = '\n';
    if (!innmea) {
	sendmid();
	UCSRB |= _BV(UDRIE);
    }
    //    PORTB &= ~_BV(PB2);

}

/*-------------------------------------------------------------------------*/
// set up UART port and configuration, out of reset and deep sleep
static void uart_init()
{
    outhead = outtail = midlen = innmea = 0;
    /* turn on bluetooth */
    DDRB |= _BV(PB1);
    PORTB |= _BV(PB1);

    DDRB |= _BV(PB2);
    PORTB &= ~_BV(PB2);

    /* setup serial port */
#define BAUD 115200
#include <util/setbaud.h>
    UBRRH = UBRRH_VALUE;
    UBRRL = UBRRL_VALUE;
#if USE_2X
    UCSRA |= (1 << U2X);
#else
    UCSRA &= ~(1 << U2X);
#endif
    UCSRC = _BV(UCSZ0) | _BV(UCSZ1);    /* 8N1 */
    UCSRB = _BV(RXEN) | _BV(TXEN);      /* enable rx/tx */
}

// send string from flash to local buffer and push it out
static void uart_puts_P(const char *p)
{
    char c = pgm_read_byte(p);
    while (c) {
        midbuf[midlen++] = c;
        c = pgm_read_byte(++p);
    }
    sendmid();
    UCSRB |= _BV(UDRIE);
}

/*-------------------------------------------------------------------------*/
static volatile unsigned int lastedge;  /* = 0 */
static unsigned char polarity;  /* = 0 */
static unsigned char jbitaccum, jbitcnt;

// General setup of timing and J1850 - for out of reset and out of deep sleep
void receiver_init(void)
{
    lastedge = 0;
    polarity = 0;
    jmsglen = 0;
    jbitaccum = 0;
    jbitcnt = 0;
    hightime = 0;
    marktime = 0;
    marklow = 0;
    /* j1850 input - PD6 as input without pullup */
    DDRD &= ~_BV(PD6);
    PORTD &= ~_BV(PD6);

    GTCCR = _BV(PSR10);         /* reset prescaler */
    TCNT1 = 0;                  /* reset counter value */
    /* activate noise canceller, */
    /* trigger on rising edge, clk/8 */
    // lower 3 bits is div, off,1,8,64,256,1024,extfall,extris ; CS12,11,10
    TCCR1B = _BV(ICES1) | _BV(ICNC1) | _BV(CS11) | _BV(CS10);

    /* clear and enable Overflow and Input Capture interrupt */
    TIFR |= _BV(TOV1) | _BV(ICF1) | _BV(OCF1B);
    TIMSK |= _BV(TOIE1) | _BV(ICIE1) | _BV(OCIE1B);
    OCR1B = 32768;

    MCUCR |= 0xC;
    GIMSK |= 0x80;
}

/*-------------------------------------------------------------------------*/
// Special timing requires assembly to disable brownout detect.
#define sleep_bod_disable() \
do { \
	uint8_t tempreg; \
	__asm__ __volatile__("in %[tempreg], %[mcucr]" "\n\t" \
			     "ori %[tempreg], %[bods_bodse]" "\n\t" \
			     "out %[mcucr], %[tempreg]" "\n\t" \
			     "andi %[tempreg], %[not_bodse]" "\n\t" \
			     "out %[mcucr], %[tempreg]" \
			     : [tempreg] "=&d" (tempreg) \
			     : [mcucr] "I" _SFR_IO_ADDR(BODCR), \
			       [bods_bodse] "i" (_BV(BPDS) | _BV(BPDSE)), \
			       [not_bodse] "i" (~_BV(BPDSE))); \
} while (0)

// Wake-up on J1850 edge after sleep
ISR(PCINT_D_vect)
{
#define PCIE2 4
    GIMSK &= ~_BV(PCIE2);
}

// Shut things down, deep-sleep until j1850 interrupt, then restart everything
void deepsleep(void)
{
    /* disable timers */
    TIMSK = 0;

    UCSRB &= ~_BV(RXCIE);       // disable receives

    outhead = outtail = midlen = 0;
    uart_puts_P(PSTR("\r\n=Sleep\r\n"));
    sei();
    while (UCSRB & _BV(UDRIE));
    cli();
    _delay_ms(20);

    /* turn off bluetooth */
    PORTB &= ~_BV(PB1);
    //    DDRB &= ~_BV(PB1);

    /* disable UART */
    UCSRB = 0;
    _delay_ms(1);

    /* set RX/TX as inputs in HiZ to prevent leakage */
    DDRD &= ~(_BV(PD0) | _BV(PD1));
    PORTD &= ~(_BV(PD0) | _BV(PD1));

    /* clear and enable bit change interrupt for wakeup */
    GIMSK = _BV(PCIE2);
    PCMSK2 = _BV(PCINT17);
#define PCIF2 4
    EIFR = _BV(PCIF2);

    /* power down with BOD disabled, INT will awake */
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    cli();
    sleep_enable();
    sleep_bod_disable();
    sei();
    sleep_cpu();
    sleep_disable();

    receiver_init();
    /* setup uart */
    uart_init();
    _delay_ms(1);

    uart_puts_P(PSTR("=WakeUp\r\n"));

    UCSRB |= _BV(RXCIE);
}

/*-------------------------------------------------------------------------*/
ISR(TIMER1_COMPB_vect)
{
    checkhitime(TCNT1);
}
volatile unsigned inactime;     /* = 0 */
// overflow - used as a coarse counter for long timeouts
ISR(TIMER1_OVF_vect)
{
    //    hightime++;
    checkhitime(TCNT1);
    // downcount transparency counter if active
    if (transptime) {
        transptime--;
        return;
    }
    // downcount inactivity timer
    if (inactime > SECS_TO_OVF(INACT_TIMEOUT)) {
        if (PIND & _BV(PD4))
            return;             // don't powerdown while BT active
        inactime = 0;
        deepsleep();
    }
    else
        ++inactime;
}

// J1850 EOD/EOF after 300 or so microseconds.
ISR(TIMER1_COMPA_vect)
{
    jbitcnt = jbitaccum = 0;
    /* disable compare match interrupt */
    TIMSK &= ~_BV(OCIE1A);

    lastedge = TCNT1;

    /* timeout - J1850 EOD/EOF */
    jmsgbuf[jmsglen++] = '\r';
    jmsgbuf[jmsglen++] = '\n';
    if( midlen + jmsglen < MIDSIZE ){
	memcpy(&midbuf[midlen], jmsgbuf, jmsglen);
	midlen += jmsglen;
    }
    jmsglen = 0;
    if (!innmea) {
	sendmid();
	UCSRB |= _BV(UDRIE);
    }
}

// Process edge on J1850
ISR(TIMER1_CAPT_vect)
{
    unsigned int now, width;

    /* toggle interrupt on rising/falling edge */
    TCCR1B ^= _BV(ICES1);

    now = ICR1;
    width = now - lastedge;
    lastedge = now;
    polarity ^= 1;

    /* bad width or outside, will happen at EOD - ignore */
    if (width >= US(VPW_EOF_MIN) || width < US(VPW_SHORT_PULSE_MIN))
        return;

    OCR1A = now + US(VPW_EOF_MIN);      /* timeout - EOD */
    TIFR |= _BV(OCF1A);         /* clear compare match interrupt */
    TIMSK |= _BV(OCIE1A);       /* enable compare match interrupt */

    inactime = 0;

    /* doesn't quite do IFRs - normalization bit, crc? */
    if (!polarity && width >= US(VPW_SOF_MIN)) {
#if 1
	jmsglen = 4;
	timestamp(now);
	jmsgbuf[0] = tsout[0];
	jmsgbuf[1] = tsout[1];
	jmsgbuf[2] = tsout[2];
	jmsgbuf[3] = 'J';
#else
	jmsglen = 1;
	jmsgbuf[0] = 'J';
#endif
        jbitcnt = jbitaccum = 0;
        return;
    }

    jbitaccum <<= 1;
    if ((width < US(VPW_LONG_PULSE_MIN)) ^ polarity)
	jbitaccum++;
    if (++jbitcnt < 4)
        return;
    if( jmsglen < 22 )
	jmsgbuf[jmsglen++] = jbitaccum > 9 ? 'A' - 10 + jbitaccum : '0' + jbitaccum;
    jbitcnt = jbitaccum = 0;
}

/*-------------------------------------------------------------------------*/

int main(void)
{
    /* power savings */
    PRR = _BV(PRUSI);           /* shut down USI */
    DIDR = _BV(AIN0D) | _BV(AIN1D);     /* disable digital input on analog */
    ACSR = _BV(ACD);            /* disable analog comparator */

    /* setup uart */
    uart_init();
    receiver_init();
    sei();
    uart_puts_P(PSTR("=Harley J1850-GPS\r\n"));

    UCSRB |= _BV(RXCIE);
    for (;;) {
        /* idle sleep, INT from J1850 or serial will awake */
        set_sleep_mode(SLEEP_MODE_IDLE);
        sleep_mode();
    }
}

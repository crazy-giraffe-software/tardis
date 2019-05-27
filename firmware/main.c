// Tardis simulator
// 
// Copyright (c) 2019 crazy-giraffe-software
//
// MIT License
//
// Permission is hereby granted, free of charge, to any person obtaining a copy of this
// software and associated documentation files (the "Software"), to deal in the Software
// without restriction, including without limitation the rights to use, copy, modify, merge,
// publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons
// to whom the Software is furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all copies or
// substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED *AS IS*, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
// INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
// PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE
// FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
// OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
// DEALINGS IN THE SOFTWARE.
// -----------------------------------------------------------------------------------------------

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <stdint.h>
#include <math.h>
#include <util/delay.h>

//
// Chip:
// ATMega328
//
// PB0:        ISD25xxx CE (Chip Enable)
// PB1:        ISD25xxx PD (Power Down)
// SS/PB2:     ISD25xxx PR (Play/Record)
// MOSI/PB3:   MOSI for ICSP
// MISO/PB4:   MISO for ICSP
// SCLK/PB5:   SCLK for ICSP
// XTAL1/PB6:  ISD25xxx OEM (End of mesaage)
// XTAL2/PB7:  ISD25xxx OVF (Overflow)
// ...
// PC0:        Start Sw
// PC1:        Stop Sw
// PC2:        Record Sw
// PC3:        Led 1
// SDA/PC4:    Led 2
// SCL/PC5:    Led 3
// RST/PC6:    Reset
// ...
// RXD/PD0:    D1 from radio
// TXD/PD1:    D0 from radio
// PD2/INT0:   Acc from vehicle
// PD3/INT1:   VT from radio
// PD4:        D3 from radio
// PD5:        D2 from radio
// PD6:        Lamp
// PD7:        Amp Remote

//
// Configuration
//
// START_FROM_ACC: Trigger sounds/lights when you switch vehicle on
// 0 to disable, 1 to enable.
#define START_FROM_ACC   0

// Timer 1 counter value before starting lamp.  Set to start lamp with 1st sound.
#define LAMP_DELAY_COUNT 60

// Timer 1 counter value after cycle before restart.  Set to start lamp with 2nd,3rd,etc... sound.
#define LAMP_DWELL_COUNT 24

// Timer 1 count
#define LAMP_TIMER_COUNT 0x0A00

// Amount to step for each timer interrupt (count = M_PI/LAMP_STEP_VALUE).  Set to produce smooth lamp output.
#define LAMP_STEP_VALUE 0.05

// In lights only, the number of cycles max
#define LAMP_MAX_CYCLES 9

//
// Constants
//
// Version
#define FW_VERSION  "1.0"

// True/false
#ifndef FALSE
#define FALSE   0
#endif
#ifndef TRUE
#define TRUE    (!FALSE)
#endif

// LED
#define LED_PORT    PORTC
#define LED_PIN     PINC
#define LED1_PORT   PORTC3
#define LED2_PORT   PORTC4
#define LED2_PIN    PINC4
#define LED3_PORT   PORTC5

// Switch
#define SWITCH_PIN PINC
#define SWITCH1_PIN PINC2
#define SWITCH2_PIN PINC1
#define SWITCH3_PIN PINC0

// Radio
#define RADIO_PIN  PIND
#define RADIO0_PIN PIND1
#define RADIO1_PIN PIND0
#define RADIO2_PIN PIND5
#define RADIO3_PIN PIND4

// ISD
#define ISD_PORT PORTB
#define ISD_CE_PORT PORTB0
#define ISD_PD_PORT PORTB1
#define ISD_PR_PORT PORTB2
#define ISD_CE_PIN PINB0
#define ISD_PD_PIN PINB1
#define ISD_PIN PINB
#define ISD_EOM_PIN PINB6
#define ISD_OVF_PIN PINB7

// declare
uint16_t switch1Value = 0;
uint16_t switch2Value = 0;
uint16_t switch3Value = 0;

uint8_t switch1State  = 0;
uint8_t switch2State  = 0;
uint8_t switch3State  = 0;

uint8_t prevSwitch1State  = 0;
uint8_t prevSwitch2State  = 0;
uint8_t prevSwitch3State  = 0;

uint16_t radio0Value = 0;
uint16_t radio1Value = 0;
uint16_t radio2Value = 0;
uint16_t radio3Value = 0;

uint8_t radio0State = 0;
uint8_t radio1State = 0;
uint8_t radio2State = 0;
uint8_t radio3State = 0;

uint8_t prevRadio0State = 0;
uint8_t prevRadio1State = 0;
uint8_t prevRadio2State = 0;
uint8_t prevRadio3State = 0;

uint16_t accValue = 0;
uint8_t accState = 0;
uint8_t prevAccState = 0;

uint8_t isdEOMState = 0;
uint8_t isdOVFState = 0;
uint8_t isdIsPlayingOrRecording = 0 ;

double lampValue = 0;
uint16_t lampCounter = 0;
uint16_t lampStartCounter = 0;
uint16_t lampDwellCounter = 0;
uint16_t lampOnCounter = 0;
uint16_t lampCycleCounter = 0;


// Fwd Decl
void InitPorts(void);
void InitPWMTimer(uint8_t start);
void InitSequenceTimer(uint8_t start);
void InitUserInputTimer(void);

void Sleep(void);

uint8_t DebounceSwitch(uint16_t * pValue, uint8_t bit);
uint8_t DebounceRadio(uint16_t * pValue, uint8_t bit);
uint8_t DebounceAcc(uint16_t * pValue);

void SetLed(uint8_t led, uint8_t state);

void SetAmp(uint8_t state);

void ISDStop(void);
void ISDPlay(void);
void ISDRecord(void);

void StartSequence(uint8_t playSound);
void StopSequence(void);
uint8_t IsSequence(void);

uint8_t IsAcc(void);
uint8_t IsRadio(void);

void ClearUserInput(void);

// main
int main(void)
{
    // disable interrupts
    cli();

    // deterine counter values
    // Dwell and delay period appear at the start of the timer
    // cycle so reset counter to delay if delay is larger than dwell
    // or dwell - delay if dwell is larger than delay.
    if (LAMP_DELAY_COUNT > LAMP_DWELL_COUNT)
    {
        lampStartCounter = 0;
        lampDwellCounter = LAMP_DELAY_COUNT - LAMP_DWELL_COUNT;
        lampOnCounter = LAMP_DELAY_COUNT;
    }
    else
    {
        lampStartCounter = LAMP_DWELL_COUNT - LAMP_DELAY_COUNT;
        lampDwellCounter = 0;
        lampOnCounter = LAMP_DWELL_COUNT;
    }

    // Init all ports
    InitPorts();

    // Stop sequence and clear user input
    StopSequence();
    ClearUserInput();

    // Init timers
    InitPWMTimer(FALSE);
    InitSequenceTimer(FALSE);
    InitUserInputTimer();

    // Flash led for 1 second.  We may sleep soon
    // and we want and indication we powered up.
    SetLed(LED1_PORT, TRUE);
    _delay_ms(200);
    SetLed(LED1_PORT, FALSE);
    _delay_ms(200);
    SetLed(LED1_PORT, TRUE);
    _delay_ms(200);
    SetLed(LED1_PORT, FALSE);
    _delay_ms(200);
    SetLed(LED1_PORT, TRUE);
    _delay_ms(200);

    // enable interrupts
    sei();

    // loop forever
    for(;;)
    {
        // Sleep?
        if ((!IsSequence()) && (!IsAcc()) && (!IsRadio()))
        {
            // Stop sequence and clear user input
            StopSequence();
            ClearUserInput();
        
            // When Acc is off and the sequence is idle, drop
            // into sleep mode to save power.  Either INT0 (Acc)
            // or INT1 (radio) will wake us up.
            Sleep();
        }
    }

    // done
    return 0;
}

//
// Initialize ports
//
void InitPorts(void)
{
    // Disable pullups
    MCUCR |= _BV(PUD);

    // Configure port B
    DDRB =  _BV(DDB0) | // ISD CE
            _BV(DDB1) | // ISD PD
            _BV(DDB2) | // ISD PR
            _BV(DDB3) | // ICSP
            _BV(DDB4) | // ICSP
            _BV(DDB5);  // ICSP
    PORTB = 0x00; // pullup disable on all pins

    // Configure port C
    DDRC =  _BV(DDC3) | // LED 1
            _BV(DDC4) | // LED 2
            _BV(DDC5);  // LED 3
    PORTC = _BV(PORTC0) | // Switch 1
            _BV(PORTC1) | // Switch 2
            _BV(PORTC2);  // Switch 3

    // Configure port D
    DDRD =  _BV(DDD6) | // Lamp
            _BV(DDD7);  // Amp Remote
    PORTD = 0x00; // pullup disable on all pins

    // Configure external interrupts for any level change
    // to allow wakeup from sleep.
    EICRA = _BV(ISC10) |
            _BV(ISC00);
    EIMSK = _BV(INT1) |
            _BV(INT0);

    // Re-enable pullups
    MCUCR &= ~_BV(PUD);
}

//
// Init Timer 0: PWM
//
void InitPWMTimer(uint8_t start)
{
    if (start)
    {
        // Set OC0A non-inverting mode,
        // PWM, Phase Correct
        TCCR0A = _BV(COM0A1) |
                 _BV(WGM00);

        // Clock / 64
        TCCR0B = _BV(CS01) + _BV(CS00);
    }
    else
    {
        // Set OC0A disconnect,
        // Normal mode
        TCCR0A = 0;

        // Clock stopped
        TCCR0B = 0;
    }

    // No pulse
    // Port On (high, lamp off)
    OCR0A = 0x00;
    PORTD |= PORTD6;

    // Disable interrupts
    TIMSK0 = 0;
}

//
// Init Timer 1: Play sequence
//
void InitSequenceTimer(uint8_t start)
{
    if (start)
    {
        // PWM generation: disable. CTC
        // Clock / 8
        TCCR1A = 0;
        TCCR1B = _BV(WGM12) +
                 _BV(CS11);
    }
    else
    {
        // PWM generation: disable. CTC
        // Clock stopped;
        TCCR1A = 0;
        TCCR1B = 0;
    }

    // Set CTC value
    // To do a 16-bit write, the high byte must be written before the low byte.
    //uint32_t timer1Max = 0;
    OCR1AH = ((LAMP_TIMER_COUNT >> 8) & 0Xff);
    OCR1AL = (LAMP_TIMER_COUNT & 0Xff);

    // Enable comp A interrupt
    TIMSK1 = _BV(OCIE1A);
}

//
// Init Timer 2: IO check
//
void InitUserInputTimer(void)
{
    // PWM generation: disable
    // CTC
    // Clock / 64
    TCCR2A = _BV(WGM21);
    TCCR2B = _BV(CS22);

    // Fire every 10ms (1 / 0.010 == 100)
    uint32_t timer2Max = (F_CPU / 64) / 100;
    OCR2A = timer2Max;

    // Enable comp A interrupt
    TIMSK2 = _BV(OCIE2A);
}

//
// Sleep
//
void Sleep(void)
{
    // Disable ADC, analog comp, internal voltage ref, watchdog
    ADCSRA &= ~_BV(ADEN); // disable ADC
    ACSR |= _BV(ACD); // disable analong comp
    ACSR &= ~_BV(ACBG); //  disable analog comp bandgap select
    WDTCSR &= ~_BV(WDIE); // disable watchdog interrupt
    WDTCSR &= ~_BV(WDE); // disable waitchdog reset
    
    // We are going to use Idle mode since the external interrupts
    // are active high so we need a level change interrupt, which
    // only works in idle mode.  However, Idle mode wakes up for
    // Timer2 match and we use Timer2 to check for user input.
    // Therefore, we'll shutdown Timer2 before going to sleep
    // and re-enable when we wake up.
    TCCR2B = 0;
    
    // Re-configure port D to reduce power consumption.
    // Transition these pins to high-z.  First, change to
    // output low then to high-z.
    PORTD &= ~_BV(PORTD6); // Lamp
    PORTD &= ~_BV(PORTD7); // Amp Remote
    DDRD &= ~_BV(DDD6); // Lamp
    DDRD &= ~_BV(DDD7); // Amp Remote

    // Led off
    SetLed(LED1_PORT, FALSE);

    // enable interrupts so we can wake-up again
    sei();

    // Switch to power-down mode
    // Or, I think we need Idle mode so that the external
    // interrupts wake on rising edge because both our interrupts
    // are high instead of low.
    SMCR = _BV(SE);

    // Sleep.  Execution will stop and resume here when we wake.
    sleep_cpu();

    // Disable sleep
    SMCR &= ~_BV(SE);
    
    // Configure port D
    DDRD =  _BV(DDD6) | // Lamp
            _BV(DDD7);  // Amp Remote
    PORTD = 0x00; // pullup disable on all pins

    // Led on
    SetLed(LED1_PORT, TRUE);
    
    // Re-enable Timer2
    InitUserInputTimer();    
}

//
// Debounce logic
//
uint8_t DebounceSwitch(uint16_t * pValue, uint8_t bit)
{
    // Set state: require 12 bits, all 0's
    // All switches are on SWITCH_PIN.
    *pValue = 0xF000 |
              (*pValue << 0x01) |
              ((SWITCH_PIN >> bit) & 0x01);

    // If we got all 0's, the switch has debounced
    // and is being pressed
    if(0xF000 == *pValue)
    {
        return 0x01;
    }

    // We've still got some 1's, button is released
    return 0x00;
}

uint8_t DebounceRadio(uint16_t * pValue, uint8_t bit)
{
    // Set state: require 4 bits, all 1's
    // All radio inputs are on RADIO_PIN.
    *pValue = 0xFFF0 |
              (*pValue << 0x01) |
              ((RADIO_PIN >> bit) & 0x01);

    // If we got all 1's, the radio has debounced
    // and is being pressed
    if(0xFFFF == *pValue)
    {
        return 0x01;
    }

    // We've still got some 0's, button is released
    return 0x00;
}

uint8_t DebounceAcc(uint16_t * pValue)
{
    // Set state: require 12 bits, all 1's
    *pValue = 0xF000 |
              (*pValue << 0x01) |
              IsAcc();

    // If we got all 1's, acc has debounced
    // and is on
    if(0xFFFF == *pValue)
    {
        return 0x01;
    }

    // We've still got some 0's, acc is off
    return 0x00;
}

//
// Leds
//
void SetLed(uint8_t led, uint8_t state)
{
    if (state)
    {
        LED_PORT &= ~_BV(led);
    }
    else
    {
        LED_PORT |= _BV(led);
    }
}

//
// Amp routines
//
void SetAmp(uint8_t state)
{
    if (state)
    {
        PORTD |= _BV(PORTD7);
    }
    else
    {
        PORTD &= ~_BV(PORTD7);
    }
}

//
// ISD routines
//
void ISDStop(void)
{
    // Led off
    SetLed(LED3_PORT, FALSE);

    // CE is high to stop
    ISD_PORT |= _BV(ISD_CE_PORT);

    // set PR to high to minimize power
    ISD_PORT |= _BV(ISD_PR_PORT);

    // set PD to high to power down
    ISD_PORT |= _BV(ISD_PD_PORT);
}

void ISDPlay(void)
{
    // Led off
    SetLed(LED3_PORT, FALSE);

    // PD is low for play
    ISD_PORT &= ~_BV(ISD_PD_PORT);

    // PR is high for play
    ISD_PORT |= _BV(ISD_PR_PORT);

    // CE is low to start
    ISD_PORT &= ~_BV(ISD_CE_PORT);
}

void ISDRecord(void)
{
    // Led on
    SetLed(LED3_PORT, TRUE);

    // PD is low for play
    ISD_PORT &= ~_BV(ISD_PD_PORT);

    // PR is low for record
    ISD_PORT &= ~_BV(ISD_PR_PORT);

    // CE is low to start
    ISD_PORT &= ~_BV(ISD_CE_PORT);
}

//
// Play sequence
//
void StartSequence(uint8_t playSound)
{
    // Amp On
    if (playSound)
    {
      SetAmp(TRUE);
    }
    
    // Reset counter and value
    lampValue = 0;
    lampCounter = lampStartCounter;
    lampCycleCounter = 0;

    // Start PWM and sequence timers
    InitPWMTimer(TRUE);
    InitSequenceTimer(TRUE);

    // Start sound
    if (playSound)
    {
        ISDPlay();
    }

    // Led on
    SetLed(LED2_PORT, TRUE);

}

void StopSequence(void)
{
    // Amp Off
    SetAmp(FALSE);

    // Reset counter and value
    lampValue = 0;
    lampCounter = 0;

    // Stop sound, lamp off
    OCR0A = 0xFF; // Lamp off
    ISDStop();

    // Stop PWM and sequence timers
    InitPWMTimer(FALSE);
    InitSequenceTimer(FALSE);

    // Led off
    SetLed(LED2_PORT, FALSE);
}

uint8_t IsSequence(void)
{
    return ((LED_PIN & _BV(LED2_PIN)) == 0);
}

//
// Acc
//
uint8_t IsAcc(void)
{
    // Acc is connected to D2.
    return PIND & _BV(PIND2);
}

//
// Radio
//
uint8_t IsRadio(void)
{
    // Radio VT is connected to D3.
    return PIND & _BV(PIND3);
}

//
// User Input
//
void ClearUserInput(void)
{
    switch1Value = 0;
    switch2Value = 0;
    switch3Value = 0;
    
    switch1State  = 0;
    switch2State  = 0;
    switch3State  = 0;
    
    prevSwitch1State  = 0;
    prevSwitch2State  = 0;
    prevSwitch3State  = 0;
    
    radio0Value = 0;
    radio1Value = 0;
    radio2Value = 0;
    radio3Value = 0;
    
    radio0State = 0;
    radio1State = 0;
    radio2State = 0;
    radio3State = 0;
    
    prevRadio0State = 0;
    prevRadio1State = 0;
    prevRadio2State = 0;
    prevRadio3State = 0;
    
    accValue = 0;
    accState = 0;
    prevAccState = 0;
}

//
// Timer 1 interrupt
// Purpose: play sequence
//
ISR(TIMER1_COMPA_vect)
{
    ++lampCounter;
    if (lampCounter >= lampOnCounter)
    {
        // Increment lamp step and set PWM
        lampValue += LAMP_STEP_VALUE;
        OCR0A = sin(lampValue) * 255;

        // If lamp value completes a cycle, enter dwell period.
        if (lampValue >= M_PI)
        {
            lampValue = 0;
            OCR0A = 0;
            lampCounter = lampDwellCounter;
            lampCycleCounter++;
            if (lampCycleCounter >= LAMP_MAX_CYCLES)
            {
                StopSequence();
            }
        }
    }
}

//
// Timer 2 interrupt
// IO check
//
ISR(TIMER2_COMPA_vect)
{
    // Determine Acc
    if (START_FROM_ACC)
    {
        accState = DebounceAcc(&accValue);
    
        if (prevAccState != accState)
        {
            if (accState)
            {
                if (!IsSequence())
                {
                    uint8_t playSound = TRUE;
                    StartSequence(playSound);
                }
            }
            prevAccState = accState;
        }
    }

    // Determine radio button
    radio0State = DebounceRadio(&radio0Value, RADIO0_PIN);
    radio1State = DebounceRadio(&radio1Value, RADIO1_PIN);
    radio2State = DebounceRadio(&radio2Value, RADIO2_PIN);
    radio3State = DebounceRadio(&radio3Value, RADIO3_PIN);

    // Wake-up and play sequence.
    // We respond to buttons:
    //  3 (A) (lights only)
    //  2 (B) (sound + lights)
    if (prevRadio2State != radio2State)
    {
        if (radio2State)
        {
            if (IsSequence())
            {
                StopSequence();
            }
            else
            {
                uint8_t playSound = TRUE;
                StartSequence(playSound);
            }
        }
        prevRadio2State = radio2State;
    }

    if (prevRadio3State != radio3State)
    {
        if (radio3State)
        {
            if (IsSequence())
            {
                StopSequence();
            }
            else
            {
                uint8_t playSound = FALSE;
                StartSequence(playSound);
            }
        }
        prevRadio3State = radio3State;
    }
    
    // check the switches & inputs
    switch1State = DebounceSwitch(&switch1Value, SWITCH1_PIN);
    switch2State = DebounceSwitch(&switch2Value, SWITCH2_PIN);
    switch3State = DebounceSwitch(&switch3Value, SWITCH3_PIN);

    // Check switches
    if (prevSwitch1State != switch1State)
    {
        if (switch1State)
        {
            StopSequence();
        }
        prevSwitch1State = switch1State;
    }

    if (prevSwitch2State != switch2State)
    {
        if (switch2State && switch3State)
        {
            ISDRecord();
        }
        else if (switch2State)
        {
            if (IsSequence())
            {
                // Since this is also an external switch, allow
                // it to start/stop.
                StopSequence();
            }
            else
            {
                uint8_t playSound = TRUE;
                StartSequence(playSound);
            }
        }
        prevSwitch2State = switch2State;
    }

    // Check ISD inputs
    isdEOMState = ((ISD_PIN & _BV(ISD_EOM_PIN)) == 0);
    isdOVFState = ((ISD_PIN & _BV(ISD_OVF_PIN)) == 0);
    isdIsPlayingOrRecording = ((ISD_PIN & _BV(ISD_CE_PIN)) == 0);
    if ((isdEOMState || isdOVFState) && isdIsPlayingOrRecording)
    {
        // This is a little hack.  The ISD seems to produce an EOM
        // when the message starts.  This ignores that EOM while not ignoring
        // the one at the end of the message.
        if (lampCounter > 40)
        {
          StopSequence();
        }
    }
}


//
// External 0 interrupt
// Purpose: ACC
//
ISR(INT0_vect)
{
}


//
// External 1 interrupt
// Purpose: Radio
//
ISR(INT1_vect)
{
}

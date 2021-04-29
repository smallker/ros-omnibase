#include "Motor.h"
void Motor::setPwmFrequency(byte divisor){
    this->divisor = divisor;
    setPwmFrequency();
}
void Motor::setPwmFrequency()
{
    byte mode;
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega128RFA1__) || defined(__AVR_ATmega256RFR2__) || \
    defined(__AVR_AT90USB82__) || defined(__AVR_AT90USB162__) || defined(__AVR_ATmega32U2__) || defined(__AVR_ATmega16U2__) || defined(__AVR_ATmega8U2__)
      switch(divisor) {
      case 1: mode = 0x01; break;
      case 2: mode = 0x02; break;
      case 3: mode = 0x03; break;
      case 4: mode = 0x04; break;
      case 5: mode = 0x05; break;
      case 6: mode = 0x06; break;
      case 7: mode = 0x07; break;
      default: return;
      }
      
        switch(pin) {	  
      case 2:  TCCR3B = TCCR3B  & 0b11111000 | mode; break;
      case 3:  TCCR3B = TCCR3B  & 0b11111000 | mode; break;
      case 4:  TCCR0B = TCCR0B  & 0b11111000 | mode; break;
      case 5:  TCCR3B = TCCR3B  & 0b11111000 | mode; break;
      case 6:  TCCR4B = TCCR4B  & 0b11111000 | mode; break;
      case 7:  TCCR4B = TCCR4B  & 0b11111000 | mode; break;
      case 8:  TCCR4B = TCCR4B  & 0b11111000 | mode; break;
      case 9:  TCCR2B = TCCR0B  & 0b11111000 | mode; break;
      case 10: TCCR2B = TCCR2B  & 0b11111000 | mode; break;
      case 11: TCCR1B = TCCR1B  & 0b11111000 | mode; break;  
      case 12: TCCR1B = TCCR1B  & 0b11111000 | mode; break;  
      case 13: TCCR0B = TCCR0B  & 0b11111000 | mode; break;
      default: return;
    }
#elif defined(__AVR_ATmega1284__) || defined(__AVR_ATmega1284P__) || defined(__AVR_ATmega644__) || defined(__AVR_ATmega644A__) || defined(__AVR_ATmega644P__) || defined(__AVR_ATmega644PA__)
#define EXTERNAL_NUM_INTERRUPTS 3
#elif defined(__AVR_ATmega32U4__)
#define EXTERNAL_NUM_INTERRUPTS 5
#elif defined(__AVR_ATmega328P__)
    if (pwm_pin == 5 || pwm_pin == 6 || pwm_pin == 9 || pwm_pin == 10)
    {
        switch (divisor)
        {
        case 1:
            mode = 0x01;
            break;
        case 2:
            mode = 0x02;
            break;
        case 3:
            mode = 0x03;
            break;
        case 4:
            mode = 0x04;
            break;
        case 5:
            mode = 0x05;
            break;
        default:
            return;
        }
        if (pwm_pin == 5 || pwm_pin == 6)
        {
            TCCR0B = TCCR0B & 0b11111000 | mode;
        }
        else
        {
            TCCR1B = TCCR1B & 0b11111000 | mode;
        }
    }
    else if (pwm_pin == 3 || pwm_pin == 11)
    {
        switch (divisor)
        {
        case 1:
            mode = 0x01;
            break;
        case 2:
            mode = 0x02;
            break;
        case 3:
            mode = 0x03;
            break;
        case 4:
            mode = 0x04;
            break;
        case 5:
            mode = 0x05;
            break;
        case 6:
            mode = 0x06;
            break;
        case 7:
            mode = 0x07;
            break;
        default:
            return;
        }
        TCCR2B = TCCR2B & 0b11111000 | mode;
    }
#else
#endif
}
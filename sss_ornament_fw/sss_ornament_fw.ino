#include <avr/sleep.h>

#define LED0_OUT PIN_PA6
#define LED1_OUT PIN_PA7
#define PIR_IN PIN_PA3

#define SLEEP_TIMEOUT_MIN (30)
#define SLEEP_TIMEOUT_MS (1000UL * 60 * SLEEP_TIMEOUT_MIN)

#define LAST_DISPLAYED_MODE_EEPROM_ADDR (0x00)

enum led_mode_t {
  LED_MODE_ALTERNATE_FLASH = 0x00,
  LED_MODE_ALTERNATE_FADE,
  LED_MODE_TRIPLE_FLASH,
  LED_MODE_COUNT,
};

bool success;
volatile uint32_t start_ms;
led_mode_t mode;
uint8_t triple_flash_pin;

#if F_CPU != 1000000
#error Must burn bootloader for 1MHz operation!
#endif

uint16_t normalized_fade_values[101];

ISR(PORTA_PORT_vect) {
  PORTA.INTFLAGS = 0xFF;

  // Refresh start time every time PIR triggers. millis doesn't increment in an ISR but we don't
  // care, just refresh to the most recent time. It's also fast enough and ISR safe to do here.
  start_ms = millis();
}

//EMPTY_INTERRUPT(PORTA_PORT_vect);

void gpio_set(uint8_t pin) {
  digitalWrite(pin, HIGH);
//  if (pin == LED0_OUT) {
//    TCD0.CTRLD &= ~TCD_CMPAVAL_gm;
//  } else if (pin == LED1_OUT) {
//    TCD0.CTRLD &= ~TCD_CMPBVAL_gm;
//  }  
}

void gpio_clr(uint8_t pin) {
  digitalWrite(pin, LOW);
//if (pin == LED0_OUT) {
//    TCD0.CTRLD |= TCD_CMPAVAL_gm;
//  } else if (pin == LED1_OUT) {
//    TCD0.CTRLD |= TCD_CMPBVAL_gm;
//  }
}

void gpio_toggle(uint8_t pin) {
  digitalWrite(pin, !digitalRead(pin));
//  if (pin == LED0_OUT) {
//    TCD0.CTRLD ^= TCD_CMPAVAL_gm;
//  } else if (pin == LED1_OUT) {
//    TCD0.CTRLD ^= TCD_CMPBVAL_gm;
//  }
}

void pwm_timer_init(void) {
    // Enable-protected reg must have 0 written to enable bit before any other bits can be changed, and it defaults to
    // enabled on startup
    TCD0.CTRLA &= ~TCD_ENABLE_bm;

    // Don't need overlapping PWM signals so just do oneramp
    TCD0.CTRLB = TCD_WGMODE_ONERAMP_gc;

    // Disable all input control
    TCD0.INPUTCTRLA = TCD_INPUTMODE_NONE_gc;
    TCD0.INPUTCTRLB = TCD_INPUTMODE_NONE_gc;

    // We will always start high with A output and end high with B output. The clear for A and
    // set for B values are what are adjusted based on desired duty cycle. EVERYTHING is cleared at CMPBCLR, therefore
    // you can't have both pwm signals start at 0 and end at arbitrary times. You have to offset B so it's pwm
    // cycle always ends at 0xFFF, then adjust the start time. That way you can overlap outputs all the way to
    // CMPASET=0,CMPACLR=0xFFE and CMPBSET=0x1,CMPBCLR=0xFFF.
    TCD0.CMPASET = 0x000;
    TCD0.CMPBCLR = 0xFFF;

    TCD0.CMPACLR = 0x000;
    TCD0.CMPBSET = 0x801;
    // System 20mhz clock w/ largest prescaler possible of 32 but no synchronization prescaler. Must be done as last
    // operation before starting timer with ENABLE bit
    // NOTE :: DEFAULT TIMER FOR MILLIS/MICROS FUNCTIONS ON 1-SERIES ATTINIES IS TCD. YOU MUST CHANGE THIS TO TCB IN
    // ARDUINO PORT SETTINGS
    TCD0.CTRLA = TCD_CLKSEL_SYSCLK_gc | TCD_CNTPRES_DIV4_gc | TCD_SYNCPRES_DIV1_gc;
}

void pwm_timer_start(void) {
  TCD0.CTRLC &= ~TCD_CMPOVR_bm;
    
  // Enable WOA and WOB (don't need C and D). Since FAULTCTRL reg is write protected we need to make it editable in
  // CCP reg. Must be done in start func because we can't take over output pins in stop func unless we disable them in FAULTCTRL.
  CPU_CCP        = CCP_IOREG_gc;
  TCD0.FAULTCTRL = TCD_CMPAEN_bm | TCD_CMPBEN_bm;
  
  while (!(TCD0.STATUS & TCD_ENRDY_bm)) {
      ;
  }

  TCD0.CTRLA |= TCD_ENABLE_bm;
}

void pwm_timer_stop(void) {
//  // Enable-protected reg must have 0 written to enable bit before any other bits can be changed,
//  // apparently for the FULL TCD0 reg set (including FAULTCTRL below);
//  TCD0.CTRLA &= ~TCD_ENABLE_bm;
//    
//  // Must disable the WOA/B outputs as well, otherwise we can't write to them still
//  CPU_CCP        = CCP_IOREG_gc;
//  TCD0.FAULTCTRL &= ~TCD_CMPAEN_bm;
////  CPU_CCP        = CCP_IOREG_gc;
////  TCD0.FAULTCTRL &= ~TCD_CMPBEN_bm;

//  TCD0.CTRLA &= ~TCD_ENABLE_bm;
  
  TCD0.CTRLC |= TCD_CMPOVR_bm;
}

void pwm_timer_sync(void) {
    TCD0.CTRLE = TCD_SYNCEOC_bm;
}

bool set_pwm_duty_cycle(uint8_t led_group, uint8_t duty_cycle) {
  if (led_group != LED0_OUT && led_group != LED1_OUT) {
    return false;
  }
  
  if (duty_cycle > 100) {
    return false;
  }

  if (led_group == LED0_OUT) {
    TCD0.CMPACLR = normalized_fade_values[duty_cycle];
  } else if (led_group == LED1_OUT) {
    TCD0.CMPBSET = 0xFFF - normalized_fade_values[duty_cycle];
  }

  return true;
}

void deep_sleep() {
  // Turn off all LEDs before sleeping
  gpio_set(LED0_OUT);
  gpio_set(LED1_OUT);
  
  sleep_enable();
  set_sleep_mode(SLEEP_MODE_STANDBY);
  sleep_cpu();

  // Disable once (if) we wake back up and refresh sleep timer
  start_ms = millis();
  sleep_disable();
}

void eeprom_get_byte(uint8_t eeprom_byte_offset, uint8_t *val) {
  memcpy(val, (uint8_t *)(EEPROM_START + eeprom_byte_offset), 1);
}

void eeprom_save_byte(uint8_t eeprom_byte_offset, uint8_t val) {
  while (NVMCTRL.STATUS & NVMCTRL_EEBUSY_bm);
    
  NVMCTRL.ADDR = EEPROM_START;
    
  *(uint8_t *)(EEPROM_START + eeprom_byte_offset) = val;
  
  CPU_CCP = CCP_SPM_gc;
  NVMCTRL.CTRLA = NVMCTRL_CMD_PAGEERASEWRITE_gc;
}

void setup(){
  success = false;
  
  pinMode(LED0_OUT, OUTPUT);
  pinMode(LED1_OUT, OUTPUT);

  uint8_t last_displayed_mode = 0x00;
  eeprom_get_byte(LAST_DISPLAYED_MODE_EEPROM_ADDR, &last_displayed_mode);

  // Increment last displayed and set mode, resetting to zero if we were on last mode. Default eeprom state of 0xFF also
  // handled by increment automatically rolling over to 0x00. Save for next boot, then continue normal operation
  last_displayed_mode++;
  if (last_displayed_mode >= LED_MODE_COUNT) {
    mode = (led_mode_t)0x00;
  } else {
    mode = (led_mode_t)last_displayed_mode;
  }
  eeprom_save_byte(LAST_DISPLAYED_MODE_EEPROM_ADDR, mode);

  // Startup sequence to differentiate reboot
  for (int i = 0; i < 8; i++) {
      digitalWrite(LED0_OUT, HIGH);
      digitalWrite(LED1_OUT, HIGH);
      delay(100);
      digitalWrite(LED0_OUT, LOW);
      digitalWrite(LED1_OUT, LOW);
      delay(100);
  }
  delay(500);

  if (mode == LED_MODE_ALTERNATE_FADE) {
    // Mapping 1-99 directly to 1-4094 for PWM duty value is very skewed w/ human eye towards brighter, so fade
    // is quick drop to zero then seemingly a long full bright time. The mapping ratio of 1 count on former scale
    // to 10-bit scale is ~41, so slowly scale the incremement value each time by the index / 2. Once incremement
    // value reaches standard mapped step of 41, just keep it there for the remaining normalized values.
    float mult = 0.5;
    int normalized_val;
    int factor;
    for (int i = 1; i < 99; i++) {
      factor = (int)(i * mult);
      if (factor < 41) {
        normalized_val = i * factor;
      } else {
        normalized_val = i * 41;
      }
  
      // Store 0xFFF - val because we need to invert pwm logic for low side driver
      normalized_fade_values[i] = 0xFFF - normalized_val;
    }
  
    normalized_fade_values[0] = 0xFFF;
    normalized_fade_values[100] = 0x000;
    
    pwm_timer_init();  
    pwm_timer_start();
  } else if (mode == LED_MODE_TRIPLE_FLASH) {
    triple_flash_pin = LED0_OUT;
  }
  
  // Set up interrupt on PIR pin. Must be BOTHEDGES or LEVEL since we're using a partially-async pin. If using PA2
  // or PA6 (fully async), can wake from deep sleep with any trigger.
  PORTA.PIN3CTRL = (PORTA.PIN3CTRL & ~PORT_ISC_gm) | PORT_ISC_BOTHEDGES_gc | PORT_PULLUPEN_bm;
  
  sei();
  
  start_ms = millis();
}

void loop() {
  // If we've reached timeout with nothing, that means either no movement or constant movement for the timeout period.
  // If the PIR pin is still high, that means it's continuously reported movement, so just refresh the timeout time. If
  // it's low, then the timeout has actually elapsed with no trigger, so sleep.
  if ((millis() - start_ms) > SLEEP_TIMEOUT_MS) {
    if (digitalRead(PIR_IN) == LOW) {
      deep_sleep();
    } else {
      start_ms = millis();
    }
  }
  
  switch (mode) {
    case LED_MODE_ALTERNATE_FLASH:
      gpio_toggle(LED0_OUT);
      gpio_toggle(LED1_OUT);
      delay(1000);
      break;
    case LED_MODE_ALTERNATE_FADE:
      pwm_timer_start();
      for (int i = 0; i <= 100; i += 1) {
        success = set_pwm_duty_cycle(LED0_OUT, i);
        (void)success;
        success = set_pwm_duty_cycle(LED1_OUT, 100 - i);
        (void)success;
      
        pwm_timer_sync();
        delay(10);
      }
      
      for (int i = 100; i >= 0; i-=1) {
        success = set_pwm_duty_cycle(LED0_OUT, i);
        (void)success;
        success = set_pwm_duty_cycle(LED1_OUT, 100 - i);
        (void)success;
      
        pwm_timer_sync();
        delay(10);
      }
      break;
    case LED_MODE_TRIPLE_FLASH:
      for (int i = 0; i < 6; i++) {
        gpio_toggle(triple_flash_pin);
        delay(300);
      }
      
      gpio_set(triple_flash_pin);
      delay(300);

      triple_flash_pin = triple_flash_pin == LED0_OUT ? LED1_OUT : LED0_OUT;
      break;
    default:
      mode = LED_MODE_ALTERNATE_FADE;
      break;
  }
}

#include <assert.h>
#include <string.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/dma.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/timer.h>

static void tim7_enqueue(uint16_t delay, uint8_t operation);
static void writer_start();

volatile uint32_t breakpoint = 0;

#define CPU_FREQUENCY (64*1000*1000)
#define CARRIER_FREQUENCY (125000)
#define SAMPLE_FREQUENCY (2*1000*1000) 
#define CARRIER_PAIRS_PER_INTERRUPT (4)
#define CYCLES_PER_SAMPLE (CPU_FREQUENCY/SAMPLE_FREQUENCY)
#define CYCLES_PER_CARRIER (CPU_FREQUENCY/CARRIER_FREQUENCY)
#define SAMPLES_PER_CARRIER (SAMPLE_FREQUENCY/CARRIER_FREQUENCY)
#define SAMPLES_PER_CARRIER_PAIR (2 * SAMPLES_PER_CARRIER)
#define CARRIERS_PER_INTERRUPT (2 * CARRIER_PAIRS_PER_INTERRUPT)
#define SAMPLES_PER_INTERRUPT (CARRIERS_PER_INTERRUPT * SAMPLES_PER_CARRIER)
#define ADC_DMA_BUF_CAP (2 * SAMPLES_PER_INTERRUPT)

#define DIFFERENCES_CAP (6*1024)
typedef uint16_t sample_t;
typedef int8_t delta_t;

static volatile sample_t adc_dma_buf[ADC_DMA_BUF_CAP];

static int adc_dma_stop() {
    return ADC_CR(ADC2) |= ADC_CR_ADSTP;
}

#define MAJOR_STATE_READ 0
#define MAJOR_STATE_ACQUIRED 1
#define MAJOR_STATE_WRITE 2
#define MAJOR_STATE_DONE 3
static uint8_t major_state = MAJOR_STATE_READ;

static bool button_enabled = true;

#define WRITE_DONE_POLL_WINDOW (1<<15) // roughly 4 full packets, 0.25 seconds

// writer state
// TODO rename delay -> writer
typedef struct {
    uint16_t length:15;
    uint16_t enable:1;
} delay_entry;
#define DELAY_SEQUENCE_CAP 649
delay_entry delay_sequence[DELAY_SEQUENCE_CAP] = {
};
static uint16_t delay_sequence_length = 0;
static uint16_t delay_index = 0;
static delay_entry *generateProgrammingDelaySequence(delay_entry *delays, const bool *bigData);

// done state
static bool writer_readback_since_last_poll = false;

int main(void) {
  rcc_clock_setup_hsi(&rcc_hsi_8mhz[RCC_CLOCK_64MHZ]);
  assert(CPU_FREQUENCY == rcc_apb2_frequency);

  rcc_periph_clock_enable(RCC_GPIOA);
  rcc_periph_clock_enable(RCC_GPIOB);

  // ARM->PB3: green LED on the dev board
  gpio_mode_setup(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO3);

  // TIM16OC1->PB4: PWM 125kHz square wave for carrier generation
  rcc_periph_clock_enable(RCC_TIM16);
  timer_set_mode(TIM16, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
  timer_continuous_mode(TIM16);
  timer_set_prescaler(TIM16, 0);
  timer_set_period(TIM16, CYCLES_PER_CARRIER - 1);
  // magic. <https://libopencm3.github.io/docs/latest/stm32f4/html/group__timer__file.html> says we need this line:
  timer_enable_break_main_output(TIM16);
  timer_disable_oc_preload(TIM16, TIM_OC1);
  timer_set_oc_mode(TIM16, TIM_OC1, TIM_OCM_PWM1);
  timer_set_oc_value(TIM16, TIM_OC1, CYCLES_PER_CARRIER/2); // change this to disable
  timer_set_oc_polarity_low(TIM16, TIM_OC1);
  timer_enable_oc_output(TIM16, TIM_OC1);
  gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO4);
  gpio_set_af(GPIOB, GPIO_AF1, GPIO4);
  // enabled later next to ADC to match phase

  // PB4->PA7: analog circuit involving resonant coil and RFID tag

  // PA7->OPAMP2: we have one opamp, it's called OPAMP2, and it's on the SYSCFG clock
  rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_SYSCFGEN);
  gpio_mode_setup(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO7); // opamp in
  gpio_mode_setup(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO6); // opamp out
  #define OPAMP2_CSR (MMIO32(OPAMP_BASE + 0x3C))
  // bits 6:5      11 -> negative input: follower mode
  // bits 3:2      11 -> positive input: PA7, or pin A6 on nucleo32 dev board
  // bit    0       1 -> enable
  OPAMP2_CSR |= (0b11 << 5) | (0b11 << 2) | (0b1 << 0);

  // OPAMP2(PA6)->ADC2CH3: ADC sampling at 64MHz/(19.5+12.5) = 2MHz
  rcc_periph_clock_enable(RCC_ADC12);
  rcc_adc_prescale(RCC_CFGR2_ADCxPRES_PLL_CLK_DIV_1, 0);
  adc_calibrate(ADC2);
  adc_set_sample_time_on_all_channels(ADC2, ADC_SMPR_SMP_19DOT5CYC);
  adc_set_resolution(ADC2, ADC_CFGR1_RES_12_BIT);
  uint8_t adc2_regular_channels[8] = {3}; // 3=OPAMP2; 4=PA7
  adc_set_regular_sequence(ADC2, 1, adc2_regular_channels);
  adc_set_continuous_conversion_mode(ADC2);
  adc_set_right_aligned(ADC2);
  adc_enable_dma(ADC2);
  ADC_CFGR1(ADC2) |= ADC_CFGR1_DMACFG; // circular mode
  adc_power_on(ADC2);
  timer_enable_counter(TIM16);

  // ADC2CH3->DMA1CH2 moves the samples into adc_dma_buf
  rcc_periph_clock_enable(RCC_DMA1);
  dma_set_read_from_peripheral(DMA1, DMA_CHANNEL2);
  dma_set_peripheral_address(DMA1, DMA_CHANNEL2, (uint32_t)&ADC_DR(ADC2));
  dma_set_peripheral_size(DMA1, DMA_CHANNEL2, DMA_CCR_PSIZE_16BIT);
  dma_set_memory_address(DMA1, DMA_CHANNEL2, (uint32_t)adc_dma_buf);
  dma_set_memory_size(DMA1, DMA_CHANNEL2, DMA_CCR_MSIZE_16BIT);
  dma_enable_memory_increment_mode(DMA1, DMA_CHANNEL2);
  dma_set_number_of_data(DMA1, DMA_CHANNEL2, ADC_DMA_BUF_CAP);
  dma_enable_memory_increment_mode(DMA1, DMA_CHANNEL2);
  dma_enable_circular_mode(DMA1, DMA_CHANNEL2);
  dma_set_priority(DMA1, DMA_CHANNEL2, DMA_CCR_PL_MEDIUM);
  dma_enable_half_transfer_interrupt(DMA1, DMA_CHANNEL2);
  dma_enable_transfer_complete_interrupt(DMA1, DMA_CHANNEL2);
  dma_enable_channel(DMA1, DMA_CHANNEL2);

  // DMA1CH2->ARM dma1_channel2_isr: when DMA buffer is half full or full
  nvic_set_priority(NVIC_DMA1_CHANNEL2_IRQ, 0xFF); // least urgent
  nvic_enable_irq(NVIC_DMA1_CHANNEL2_IRQ);

  ADC_CR(ADC2) |= ADC_CR_ADSTART;

  // button PA0 pulldown, 3v3 if pressed -> exti0_isr
  gpio_mode_setup(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_PULLDOWN, GPIO0);
  exti_select_source(EXTI0, GPIOA);
  exti_set_trigger(EXTI0, EXTI_TRIGGER_BOTH);
  nvic_set_priority(NVIC_EXTI0_IRQ, 0xFF); // least urgent, same as adc dma
  nvic_enable_irq(NVIC_EXTI0_IRQ);
  exti_enable_request(EXTI0);

  // TIM7->ARM generates timer interrupts
  assert(CPU_FREQUENCY/2 == rcc_apb1_frequency);
  rcc_periph_clock_enable(RCC_TIM7);
  timer_set_mode(TIM7, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
  timer_continuous_mode(TIM7);
  timer_disable_preload(TIM7);
  timer_set_prescaler(TIM7, CYCLES_PER_CARRIER - 1);
  nvic_enable_irq(NVIC_TIM7_IRQ);
  timer_enable_irq(TIM7, TIM_DIER_UIE);
  // timer_disable_counter(TIM7);
  timer_enable_counter(TIM7);
  timer_set_period(TIM7, 0); // blocked

  while (1) { __asm__("wfi"); }
}

static bool need_carrier_pwm_calibration = 1;
static void calibrate_carrier_phase(sample_t* buf) {
  uint32_t mean = 0;
  for (uint32_t i = 0; i < SAMPLES_PER_INTERRUPT; i++) {
    mean += buf[i];
  }
  mean /= SAMPLES_PER_INTERRUPT;

  uint32_t sum_of_phase_offsets_sampels = 0;
  uint32_t carrier_periods = 0;
  for (uint32_t i = 0+1; i < SAMPLES_PER_INTERRUPT; i++) {
    if ((buf[i-1] < mean) && (buf[i] >= mean)) {
      sum_of_phase_offsets_sampels += i - carrier_periods * SAMPLES_PER_CARRIER;
      carrier_periods++;
    }
  }
  if (carrier_periods != CARRIERS_PER_INTERRUPT) {
    return;
  }
  
  uint32_t carrier_lags_dma_cycles = (CYCLES_PER_SAMPLE * sum_of_phase_offsets_sampels) / carrier_periods;
  TIM_CNT(TIM16) = (TIM_CNT(TIM16) + carrier_lags_dma_cycles)%CYCLES_PER_CARRIER;
  need_carrier_pwm_calibration = 0;
}

// reader state
#define PSK_BITS_PER_READING (224)
#define PSK_CARRIER_PAIRS_PER_BIT (16)
#define PSK_CARRIERS_PER_BIT (2 * PSK_CARRIER_PAIRS_PER_BIT)
#define PSK_LEADER_BITS (30)
#define PSK_CARRIER_PAIRS_PER_LEADER ((int)((PSK_LEADER_BITS - .5f) * PSK_CARRIER_PAIRS_PER_BIT))
#define PSK_BITS_PER_PACKET (PSK_BITS_PER_READING - PSK_LEADER_BITS)
static uint16_t psk_leader_carrier_pairs = 0;
static delta_t psk_leader_isnegative = 1;
static uint8_t psk_bits_remaining = 0xff;
static uint8_t psk_carrier_pairs_until_bit = 0;

#ifdef ARPHID_DEBUG_DIFFERENCES
static volatile delta_t differences[DIFFERENCES_CAP];
#else
static delta_t differences[DIFFERENCES_CAP];
#endif

static uint16_t differences_index = 0;
static bool bigData[PSK_BITS_PER_READING];
static bool refData[PSK_BITS_PER_READING];

static void successful_reading() {
  if (major_state == MAJOR_STATE_READ) {
    delay_sequence_length = generateProgrammingDelaySequence(delay_sequence, bigData) - delay_sequence;
    memcpy(refData, bigData, PSK_BITS_PER_READING * sizeof(bool));
    major_state = MAJOR_STATE_ACQUIRED;
    gpio_set(GPIOB, GPIO3); 
  } else if (major_state == MAJOR_STATE_ACQUIRED) {
    if (gpio_get(GPIOB, GPIO3)) {
      tim7_enqueue(CARRIER_FREQUENCY/40, 2/*enable_led*/); // NOTE: /40 is eyeball, we actually want "1/5 of a card full reading period"
      gpio_clear(GPIOB, GPIO3); 
    }
  } else if (major_state == MAJOR_STATE_WRITE || major_state == MAJOR_STATE_DONE) {
    if (memcmp(bigData, refData, PSK_BITS_PER_READING * sizeof(bool)) == 0) {
      writer_readback_since_last_poll = true;
    }
  }
}

static void process(sample_t* buf) {
  if (need_carrier_pwm_calibration) {
    calibrate_carrier_phase(buf);
    return;
  }

  for (uint32_t carrier_pair = 0; carrier_pair < CARRIER_PAIRS_PER_INTERRUPT; carrier_pair++) {
    // Find even-carrier min
    sample_t even_min = (sample_t) -1;
    for (uint32_t sample = 0; sample < SAMPLES_PER_CARRIER; sample++) {
      sample_t val = buf[carrier_pair * SAMPLES_PER_CARRIER_PAIR + sample];
      if (val < even_min) {
        even_min = val;
      }
    }
    
    // Find odd-carrier min
    sample_t odd_min = (sample_t) -1;
    for (uint32_t sample = 0; sample < SAMPLES_PER_CARRIER; sample++) {
      sample_t val = buf[carrier_pair * SAMPLES_PER_CARRIER_PAIR + SAMPLES_PER_CARRIER + sample];
      if (val < odd_min) {
        odd_min = val;
      }
    }

    // Save result for the carrier pair
    delta_t d = odd_min - even_min;
    differences[differences_index] = d;
    bool d_isnegative = d < 0;

    if (psk_bits_remaining == 0xff) { // waiting for sync
      if (d_isnegative == psk_leader_isnegative) {
        psk_leader_carrier_pairs++;
      } else {
        if (psk_leader_carrier_pairs >= PSK_CARRIER_PAIRS_PER_LEADER) {
          psk_bits_remaining = PSK_BITS_PER_PACKET;
          psk_carrier_pairs_until_bit = PSK_CARRIER_PAIRS_PER_BIT/2;

          differences[differences_index-1] = INT8_MAX;
          differences[differences_index] = INT8_MIN;
        } else {
          psk_leader_isnegative = d_isnegative;
        }
        psk_leader_carrier_pairs = 0;
      }
    } else {
      psk_carrier_pairs_until_bit--;
      if (psk_carrier_pairs_until_bit == 0) {
        psk_carrier_pairs_until_bit = PSK_CARRIER_PAIRS_PER_BIT;
        bool bit = d_isnegative != psk_leader_isnegative;
        bigData[PSK_BITS_PER_READING - psk_bits_remaining] = bit;
        differences[differences_index] = bit ? INT8_MAX : INT8_MIN;
        psk_bits_remaining--;
        if (psk_bits_remaining == 0) {
          psk_bits_remaining = 0xff;
          successful_reading();
        }
      }
    }

    if (++differences_index >= DIFFERENCES_CAP) {
#ifdef ARPHID_DEBUG_DIFFERENCES
        adc_dma_stop();
        breakpoint = 1;
        return;
#else
        (void)adc_dma_stop;
        differences_index = 0;
        continue;
#endif
    }
  }
}

// writer

// these constants are measured in units of 1/125kHz
// rounded to even because tim7 queue does not support odd delays
#define CARD_STARTUP_DELAY (374) // 375 = 3000/8; // 3ms
static const uint16_t PROGRAMMING_START_GAP_LENGTH = 30;
static const uint16_t PROGRAMMING_WRITE_GAP_LENGTH = 16; // 15
static const uint16_t PROGRAMMING_WRITE_ZERO_DELAY = 24;
static const uint16_t PROGRAMMING_WRITE_ONE_DELAY  = 56;
static const uint16_t PROGRAMMING_COMMIT_DELAY     = 64;
static const uint16_t PROGRAMMING_WAIT_FOR_FINISH_DELAY = 6000/8; // 6ms, required >5.4ms by spec
static const uint16_t PROGRAMMING_POWER_OFF_AFTER_END_DELAY = 624; // 625 = 5000/8; // 5ms power off to reset
static const uint16_t PROGRAMMING_READBACK_DELAY = CARD_STARTUP_DELAY + PSK_BITS_PER_READING * PSK_CARRIERS_PER_BIT; // turn the card on and read it back once

static delay_entry *_generateWriteBit(delay_entry *delayPtr, bool bit) {
    *(delayPtr++) = (delay_entry){(((bit) == 0) ? PROGRAMMING_WRITE_ZERO_DELAY : PROGRAMMING_WRITE_ONE_DELAY), 1};
    *(delayPtr++) = (delay_entry){PROGRAMMING_WRITE_GAP_LENGTH, 0};
    return delayPtr;
}

static delay_entry *generateBlockProgrammingDelaySequence(delay_entry *delayPtr, uint8_t page, uint8_t block, const bool *data) {
    uint8_t opcode1 = 1;
    uint8_t opcode0 = page;
    uint8_t lock = 0;

    *(delayPtr++) = (delay_entry){CARD_STARTUP_DELAY, 1};
    *(delayPtr++) = (delay_entry){PROGRAMMING_START_GAP_LENGTH, 0};
    delayPtr = _generateWriteBit(delayPtr, opcode1);
    delayPtr = _generateWriteBit(delayPtr, opcode0);
    delayPtr = _generateWriteBit(delayPtr, lock);
    for (uint8_t i=0; i<32; i++) {
        delayPtr = _generateWriteBit(delayPtr, data[i]);
    }
    delayPtr = _generateWriteBit(delayPtr, block & (1 << 2));
    delayPtr = _generateWriteBit(delayPtr, block & (1 << 1));
    delayPtr = _generateWriteBit(delayPtr, block & (1 << 0));
    *(delayPtr++) = (delay_entry){PROGRAMMING_COMMIT_DELAY, 1};
    *(delayPtr++) = (delay_entry){PROGRAMMING_WAIT_FOR_FINISH_DELAY, 1};
    *(delayPtr++) = (delay_entry){PROGRAMMING_POWER_OFF_AFTER_END_DELAY , 0};

    return delayPtr;
}

// Precondition: bigData length = 32 * 7 = 224 = PSK_BITS_PER_READING
static delay_entry *generateProgrammingDelaySequence(delay_entry *delays, const bool *bigData) {
    delay_entry *delayPtr = delays;

    static const bool block0data[32] = {
        0, 0, 0, 0, // master key, 4 bits
        0, 0, 0, 0, 0, 0, 0, // spec requires 0
        0, 1, 0, // Data Bit Rate RF/32
        0, // spec requires 0
        0, 0, 0, 0, 1, // Modulation PSK1
        0, 0, // PSK frequency RF/2
        0, // answer on request = false
        0, // spec requires 0
        1, 1, 1, // max block 7
        0, // no PWD
        0, // sequence start marker
        0, // no fast downlink
        0, // no inverse data
        0, // no init delay
    };

    delayPtr = generateBlockProgrammingDelaySequence(delayPtr, 0, 0, block0data);

    for (uint16_t block = 1; block <= 7; block++) {
        delayPtr = generateBlockProgrammingDelaySequence(delayPtr, 0, block, &bigData[(block - 1) * 32]);
    }

    *(delayPtr++) = (delay_entry){PROGRAMMING_READBACK_DELAY, 1};

    return delayPtr;
}

// top-level

void dma1_channel2_isr() {
  __asm__("dmb");
  if (dma_get_interrupt_flag(DMA1, DMA_CHANNEL2, DMA_HTIF)) {
    process((sample_t *)&adc_dma_buf[0]);
    dma_clear_interrupt_flags(DMA1, DMA_CHANNEL2, DMA_HTIF);
  } else if (dma_get_interrupt_flag(DMA1, DMA_CHANNEL2, DMA_TCIF)) {
    process((sample_t *)&adc_dma_buf[SAMPLES_PER_INTERRUPT]);
    dma_clear_interrupt_flags(DMA1, DMA_CHANNEL2, DMA_TCIF);
  } else {
    assert(0);
  }
  __asm__("dsb st");
}

static void disable_carrier() {
  timer_set_oc_value(TIM16, TIM_OC1, CYCLES_PER_CARRIER);
}

static void enable_carrier() {
  timer_set_oc_value(TIM16, TIM_OC1, CYCLES_PER_CARRIER/2);
}

static void writer_step() {
  if (delay_sequence_length < DELAY_SEQUENCE_CAP) {
    gpio_clear(GPIOB, GPIO3);
    assert(0);
  }
  if (delay_index < delay_sequence_length) {
    // if (delay_index == 1) { gpio_set(GPIOB, GPIO3); }
    delay_entry e = delay_sequence[delay_index];
    delay_index++;

    if (e.enable) {
      enable_carrier();
    } else {
      disable_carrier();
    }

    tim7_enqueue(e.length, 1/*writer_step*/);
  } else if (delay_index == delay_sequence_length) {
    if (major_state == MAJOR_STATE_WRITE) {
      if (writer_readback_since_last_poll) {
        writer_readback_since_last_poll = false;
        major_state = MAJOR_STATE_DONE;
        tim7_enqueue(WRITE_DONE_POLL_WINDOW, 4/*writer_done_poll*/);
        gpio_set(GPIOB, GPIO3);
      } else {
        gpio_toggle(GPIOB, GPIO3);
        writer_start();
      }
    } else if (major_state == MAJOR_STATE_READ) {
      memset(delay_sequence, 0, DELAY_SEQUENCE_CAP * sizeof(delay_entry));
    }
  }
}

static void writer_start() {
  delay_index = 0;
  writer_step();
}

// TIM7 time is always measured relative to the LAST TIM7 INTERRUPT, there is NO NOTION of "current time"
typedef struct {
  uint16_t delay;
  uint8_t operation; // 1:writer_step, 2:led_on, 3:enable_button, 4:done->write polling
} tim7_task;

#define TIM7_QUEUE_CAP 4
static tim7_task tim7_queue[TIM7_QUEUE_CAP];
static uint8_t tim7_queue_len = 0;

static void tim7_queue_head_to_timer() {
  if (tim7_queue_len == 0) {
    timer_set_period(TIM7, 0); // blocked
  } else {
    assert(1 < tim7_queue[0].delay);
    // assert(TIM_CNT(TIM7) < tim7_queue[0].delay);
    timer_set_period(TIM7, tim7_queue[0].delay-1);
  }
}

static void tim7_enqueue(uint16_t delay, uint8_t operation) {
  assert (tim7_queue_len < TIM7_QUEUE_CAP);

  int i = 0;
  while (i < tim7_queue_len && tim7_queue[i].delay <= delay) {
      delay -= tim7_queue[i].delay;
      i++;
  }

  tim7_queue_len++;
  for (int j = tim7_queue_len-1; i < j; --j) {
    tim7_queue[j] = tim7_queue[j-1];
  }
  tim7_queue[i].delay = delay;
  tim7_queue[i].operation = operation;

  if (i+1 < tim7_queue_len) {
    tim7_queue[i+1].delay -= delay;
  }
  if (i == 0) {
    tim7_queue_head_to_timer();
  }
}

void tim7_isr() {
  uint8_t todo[TIM7_QUEUE_CAP];
  int32_t elapsed = (TIM_ARR(TIM7)&0xffff)+1;

  int i = 0;
  while (i < tim7_queue_len && tim7_queue[i].delay <= elapsed) {
      elapsed -= tim7_queue[i].delay;
      todo[i] = tim7_queue[i].operation;
      i++;
  }
  assert(i != 0); // no needless interrupts

  for (int j = 0; j < tim7_queue_len - i; j++) {
    tim7_queue[j] = tim7_queue[i+j];
  }
  tim7_queue_len -= i;
  tim7_queue_head_to_timer();

  for (int j = 0; j < i; j++) {
    uint8_t operation = todo[j];
    if (operation == 1) {
      writer_step();
    } else if (operation == 2) {
      gpio_set(GPIOB, GPIO3); 
    } else if (operation == 3) {
      button_enabled = true;
    } else if (operation == 4) {
      if (major_state == MAJOR_STATE_DONE) {
        if (writer_readback_since_last_poll) {
          writer_readback_since_last_poll = false;
          tim7_enqueue(WRITE_DONE_POLL_WINDOW, 4/*writer_done_poll*/);
        } else {
          major_state = MAJOR_STATE_WRITE;
          gpio_toggle(GPIOB, GPIO3);
          writer_start();
        }
      }
    } else {
      assert(0);
    }
  }

  timer_clear_flag(TIM7, TIM_SR_UIF);
  __asm__("dsb st");
}


void exti0_isr() {
  if (button_enabled) {
    if (gpio_get(GPIOA, GPIO0)) {
      if (major_state == MAJOR_STATE_ACQUIRED) {
        major_state = MAJOR_STATE_WRITE;
        writer_start();
      } else if (major_state == MAJOR_STATE_WRITE || major_state == MAJOR_STATE_DONE) {
        memset(refData, 0, PSK_BITS_PER_READING * sizeof(bool));
        if (major_state != MAJOR_STATE_WRITE) {
          memset(delay_sequence, 0, DELAY_SEQUENCE_CAP * sizeof(delay_entry));
        } // else clearing delay_sequence is handled in writer_step when the write finishes
        major_state = MAJOR_STATE_READ;
        gpio_clear(GPIOB, GPIO3);
      }
    } else { // debounce
      button_enabled = false;
      tim7_enqueue(1 << 14, 3/*reenable_button*/);
    }
  }

  exti_reset_request(EXTI0);
  __asm__("dsb st");
}


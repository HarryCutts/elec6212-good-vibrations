#include <math.h>

#undef SHOW_TIMINGS

const unsigned long SAMPLE_RATE =    48000UL;
const unsigned long CLOCK_MAIN  = 84000000UL;
#define TMR_CNTR (CLOCK_MAIN / (2 * SAMPLE_RATE))

const uint16_t NUM_INPUTS = 4;
const uint16_t microphone_averages[NUM_INPUTS]   = {1800, 1700, 1600, 1500}; //{1500, 1600, 1700, 1800};
const uint16_t microphone_thresholds[NUM_INPUTS] = {1900, 1800, 1700, 1600}; //{1600, 1700, 1800, 1900};
  // Also update adc_setup method

const uint16_t NUM_MIC_ID_AVERAGE_POINTS = 10;
  // The number of data points to average when identifying microphones

const uint8_t NUM_BUFFERS = 2;

const uint16_t MEASUREMENTS_PER_BUFF = 2048;
  // The number of measurements *from each sensor* to store in the buffer
const uint16_t INP_BUFF = MEASUREMENTS_PER_BUFF * NUM_INPUTS;

uint8_t mic_to_input_number[NUM_INPUTS];
uint8_t input_to_mic_number[NUM_INPUTS];
bool have_identified_microphones = false;

typedef struct {
  unsigned long start_time, end_time;
    // The times at which the first and last measurements in the buffer
    // were taken, in microseconds.

  uint16_t data[INP_BUFF] = {0};     // DMA likes ping-pongs buffer
} InputBuffer;

void identify_microphones(InputBuffer &buff);
void dump_buffer(InputBuffer &buff);
void process_data(InputBuffer &buff);

volatile bool data_to_process = false;
volatile uint8_t write_buffer_index = 0, read_buffer_index = 0;

InputBuffer buffers[NUM_BUFFERS];
unsigned long data_start_time, data_end_time;
  // The times at which the first and last measurements in the last buffer
  // that was filled, in microseconds.

void setup() {
  Serial.begin(115200);
  adc_setup();
  tmr_setup();
  pio_TIOA0();  // drive Arduino pin 2 at SMPL_RATE to bring clock out
  data_end_time = micros();
}

int mic_id_counter = 0;

void loop() {
  if (data_to_process) {
    InputBuffer &buff = buffers[read_buffer_index];
    if (!have_identified_microphones) {
      // The analogue inputs take a while to "warm up"
      mic_id_counter++;
      if (mic_id_counter == 5) identify_microphones(buff);
    }

    #ifdef SHOW_TIMINGS
      unsigned long timing_proc_start_time = micros();
    #endif

    process_data(buff);

    #ifdef SHOW_TIMINGS
      Serial.println(micros() - timing_proc_start_time);
    #endif

    data_to_process = false;
  }
}

void identify_microphones(InputBuffer &buff) {
  for (uint8_t input = 0; input < NUM_INPUTS; input++) {
    uint16_t sum = 0;
    for (size_t i = input; i < NUM_MIC_ID_AVERAGE_POINTS * NUM_INPUTS; i += NUM_INPUTS) {
      sum += buff.data[i];
    }
    uint16_t average = sum / NUM_MIC_ID_AVERAGE_POINTS;
    
    int mic_no = -1;
    for (int i = 0; i < NUM_INPUTS; i++) {
      uint16_t difference = abs(microphone_averages[i] - average);
      if (difference < 50) {
        mic_no = i;
        break;
      }
    }
    
    mic_to_input_number[mic_no] = input;
    input_to_mic_number[input] = mic_no;
  }
  
  have_identified_microphones = true;
}

bool all_non_zero(unsigned long (&values)[NUM_INPUTS]) {
  for (size_t i = 0; i < NUM_INPUTS; i++) {
    if (values[i] == 0) return false;
  }
  return true;
}

void process_tap(unsigned long (&times)[NUM_INPUTS]) {
  double angle;
  signed long t1 = times[0], t2 = times[1], t3 = times[2], t4 = times[3];
  if (abs(t4 - t2) >= abs(t3 - t1)) {
    angle = atan2(t4 - t2, t3 - t1);
  } else {
    angle = atan2(t3 - t1, t2 - t4) - M_PI_2;
  }
  
  Serial.print("Tap: ");
  for (size_t i = 0; i < NUM_INPUTS; i++) {
      Serial.print(times[i]); Serial.print("\t");
  }
  Serial.print("\t");
  Serial.println(angle);
}

void dump_buffer(InputBuffer &buff) {
  for (size_t row = 0; row < INP_BUFF; row += NUM_INPUTS) {
    for (size_t column = 0; column < NUM_INPUTS; column++) {
      Serial.print(buff.data[row + column]);
      Serial.print(',');
    }
    Serial.println();
  }
}

void process_data(InputBuffer &buff) {
  unsigned long buffer_duration = buff.end_time - buff.start_time;
  #ifdef SHOW_TIMINGS
    Serial.print(buffer_duration); Serial.print(",");
  #endif
  unsigned long trigger_times[NUM_INPUTS] = {0};
  for (uint8_t mic_no = 0; mic_no < NUM_INPUTS; mic_no++) {
    uint16_t threshold = microphone_thresholds[mic_no];
    size_t trigger_index;
    uint16_t max_so_far = 0;

    for (size_t i = mic_to_input_number[mic_no]; i < INP_BUFF; i += NUM_INPUTS) {
      if (buff.data[i] >= max_so_far) {
        max_so_far = buff.data[i];
        trigger_index = (i - mic_to_input_number[mic_no]) / NUM_INPUTS;
      }
    }

    bool crossed_threshold = (max_so_far >= threshold);
    if (crossed_threshold) {
      float fraction_of_buffer = (float)crossing_index / (float)MEASUREMENTS_PER_BUFF;
      trigger_times[mic_no] = buff.start_time + (long)(buffer_duration * fraction_of_buffer);
    }
  }

  if (all_non_zero(trigger_times)) {
    dump_buffer(buff);
    process_tap(trigger_times);
  }
}

void pio_TIOA0() { // Configure Ard pin 2 as output from TC0 channel A (copy of trigger event)
  PIOB->PIO_PDR = PIO_PB25B_TIOA0 ;  // disable PIO control
  PIOB->PIO_IDR = PIO_PB25B_TIOA0 ;   // disable PIO interrupts
  PIOB->PIO_ABSR |= PIO_PB25B_TIOA0 ;  // switch to B peripheral
}

void tmr_setup() {
  pmc_enable_periph_clk(TC_INTERFACE_ID + 0 * 3 + 0); // clock the TC0 channel 0

  TcChannel * t = &(TC0->TC_CHANNEL)[0] ;            // pointer to TC0 registers for its channel 0
  t->TC_CCR = TC_CCR_CLKDIS ;                        // disable internal clocking while setup regs
  t->TC_IDR = 0xFFFFFFFF ;                           // disable interrupts
  t->TC_SR ;                                         // read int status reg to clear pending
  t->TC_CMR = TC_CMR_TCCLKS_TIMER_CLOCK1 |           // use TCLK1 (prescale by 2, = 42MHz)
              TC_CMR_WAVE |                          // waveform mode
              TC_CMR_WAVSEL_UP_RC |                  // count-up PWM using RC as threshold
              TC_CMR_EEVT_XC0 |     // Set external events from XC0 (this setup TIOB as output)
              TC_CMR_ACPA_CLEAR | TC_CMR_ACPC_CLEAR |
              TC_CMR_BCPB_CLEAR | TC_CMR_BCPC_CLEAR ;

  t->TC_RC = TMR_CNTR;              // counter resets on RC, so sets period in terms of 42MHz clock
  t->TC_RA = TMR_CNTR / 2;          // roughly square wave
  t->TC_CMR = (t->TC_CMR & 0xFFF0FFFF) | TC_CMR_ACPA_CLEAR | TC_CMR_ACPC_SET ;  // set clear and set from RA and RC compares
  t->TC_CCR = TC_CCR_CLKEN | TC_CCR_SWTRG ;  // re-enable local clocking and switch to hardware trigger source.
}

void adc_setup() {
  pmc_enable_periph_clk(ID_ADC);
  adc_init(ADC, SystemCoreClock, ADC_FREQ_MAX, ADC_STARTUP_FAST);
  NVIC_EnableIRQ(ADC_IRQn);               // enable ADC interrupt vector

  adc_disable_all_channel(ADC);
  adc_enable_interrupt(ADC, ADC_IER_RXBUFF);

  ADC->ADC_RPR  = (uint32_t) buffers[0].data;      // DMA buffer
  ADC->ADC_RCR  = INP_BUFF;
  ADC->ADC_RNPR = (uint32_t) buffers[1].data;      // next DMA buffer
  ADC->ADC_RNCR = INP_BUFF;
  ADC->ADC_PTCR = 1;

  adc_set_bias_current(ADC, 0x01);
  //  adc_enable_tag(ADC);
  adc_enable_channel(ADC, ADC_CHANNEL_7);  // AN0
  adc_enable_channel(ADC, ADC_CHANNEL_6);  // AN1
  adc_enable_channel(ADC, ADC_CHANNEL_5);  // AN2
  adc_enable_channel(ADC, ADC_CHANNEL_4);  // AN3
  adc_configure_trigger(ADC, ADC_TRIG_TIO_CH_0, 0);
  adc_start(ADC);
}

void ADC_Handler(void) {
  if ((adc_get_status(ADC) & ADC_ISR_RXBUFF) ==	ADC_ISR_RXBUFF) { // If the buffer is full
    data_start_time = data_end_time;
    data_end_time = micros();

    InputBuffer &buff = buffers[write_buffer_index];
    buff.start_time = data_start_time;
    buff.end_time   = data_end_time;

    // Instruct the DMA to copy into `inp`
    ADC->ADC_RNPR = (uint32_t)buff.data;
    ADC->ADC_RNCR = INP_BUFF;

    read_buffer_index = write_buffer_index;
    write_buffer_index = (write_buffer_index + 1) % NUM_BUFFERS;
    data_to_process = true;
  }
}

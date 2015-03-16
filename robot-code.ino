#define SMP_RATE          48000UL
#define CLK_MAIN       84000000UL
#define TMR_CNTR       CLK_MAIN / (2 *SMP_RATE)

#define NUM_INPUTS     3

#define MEASUREMENTS_PER_BUFF 1024
#define INP_BUFF       (MEASUREMENTS_PER_BUFF * NUM_INPUTS)

const uint8_t input_buffer_offsets[] = {1, 2, 0};
const uint8_t input_thresholds[] = {3072, 3072, 3072};

volatile int16_t flag = 0 ;

uint16_t inp[INP_BUFF] = {0};     // DMA likes ping-pongs buffer
unsigned long buffer_start_time, buffer_end_time;
  // The times at which the first and last measurements in the buffer
  // were taken, in microseconds.

void setup() {
  Serial.begin(115200);
  adc_setup();
  tmr_setup();
  pio_TIOA0();  // drive Arduino pin 2 at SMPL_RATE to bring clock out
  buffer_end_time = micros();
}

void loop() {
  if (flag) {
    unsigned long buffer_duration = buffer_end_time - buffer_start_time;
    for (uint8_t input = 0; input < NUM_INPUTS; input++) {
      bool crossed_threshold = false;
      size_t crossing_index;
      
      for (size_t i = input_buffer_offsets[input]; i < INP_BUFF; i += NUM_INPUTS) {
        if (inp[i] >= input_thresholds[input]) {
          crossed_threshold = true;
          crossing_index = (i - input_buffer_offsets[input]) / NUM_INPUTS;
          break;
        }
      }
      
      if (crossed_threshold) {
        float fraction_of_buffer = (float)crossing_index / (float)MEASUREMENTS_PER_BUFF;
        unsigned long crossing_time = buffer_start_time + (long)(buffer_duration * fraction_of_buffer);
        Serial.print("Input "); Serial.print(input); Serial.print(" crossed at "); Serial.println(crossing_time);
      }
    }

    // A2 A0 A1

    flag = 0;
  }
}

void print_data() {
  Serial.print("DATA: ");
  for (uint32_t i = 0; i < 12; i++) {
    Serial.print(inp[i]);
    Serial.print(" ");
  }
  Serial.println();
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

  ADC->ADC_RPR  = (uint32_t) inp;      // DMA buffer
  ADC->ADC_RCR  = INP_BUFF;
  ADC->ADC_RNPR = (uint32_t) inp;      // next DMA buffer
  ADC->ADC_RNCR = INP_BUFF;
  ADC->ADC_PTCR = 1;

  adc_set_bias_current(ADC, 0x01);
  //  adc_enable_tag(ADC);
  adc_enable_channel(ADC, ADC_CHANNEL_7);  // AN0
  adc_enable_channel(ADC, ADC_CHANNEL_6);  // AN1
  adc_enable_channel(ADC, ADC_CHANNEL_5);  // AN2
  adc_configure_trigger(ADC, ADC_TRIG_TIO_CH_0, 0);
  adc_start(ADC);
}

void ADC_Handler(void) {
  if ((adc_get_status(ADC) & ADC_ISR_RXBUFF) ==	ADC_ISR_RXBUFF) {
    buffer_start_time = buffer_end_time;
    buffer_end_time = micros();
    flag = 1;
    ADC->ADC_RNPR = (uint32_t) inp;
    ADC->ADC_RNCR = INP_BUFF;
  }
}

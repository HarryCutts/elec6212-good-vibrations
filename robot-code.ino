const unsigned long SAMPLE_RATE =    48000UL;
const unsigned long CLOCK_MAIN  = 84000000UL;
#define TMR_CNTR (CLOCK_MAIN / (2 * SAMPLE_RATE))

const uint16_t NUM_INPUTS = 3;

const uint16_t NUM_MIC_ID_AVERAGE_POINTS = 10;
  // The number of data points to average when identifying microphones

const uint16_t MEASUREMENTS_PER_BUFF = 1024;
  // The number of measurements *from each sensor* to store in the buffer
const uint16_t INP_BUFF = MEASUREMENTS_PER_BUFF * NUM_INPUTS;

const uint16_t microphone_averages[NUM_INPUTS] = {1400, 1500, 1600};
const uint16_t microphone_thresholds[NUM_INPUTS] = {1500, 1600, 1700};

uint8_t mic_to_input_number[NUM_INPUTS];
uint8_t input_to_mic_number[NUM_INPUTS];
bool have_identified_microphones = false;

volatile bool data_to_process = false;

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

int mic_id_counter = 0;

void loop() {
  if (data_to_process) {
    if (!have_identified_microphones) {
      // The analogue inputs take a while to "warm up"
      mic_id_counter++;
      if (mic_id_counter == 5) identify_microphones();
    } 
    
    unsigned long buffer_duration = buffer_end_time - buffer_start_time;
    Serial.print(buffer_duration); Serial.print(",");
    for (uint8_t mic_no = 0; mic_no < NUM_INPUTS; mic_no++) {
      uint16_t threshold = microphone_thresholds[mic_no];
      bool crossed_threshold = false;
      size_t crossing_index;
      
      for (size_t i = mic_to_input_number[mic_no]; i < INP_BUFF; i += NUM_INPUTS) {
        if (inp[i] >= threshold) {
          crossed_threshold = true;
          crossing_index = (i - mic_to_input_number[mic_no]) / NUM_INPUTS;
          break;
        }
      }
      
      if (crossed_threshold) {
        float fraction_of_buffer = (float)crossing_index / (float)MEASUREMENTS_PER_BUFF;
        unsigned long crossing_time = buffer_start_time + (long)(buffer_duration * fraction_of_buffer);
        Serial.print("m"); Serial.print(mic_no); Serial.print(" crossed at "); Serial.println(crossing_time);
      }
    }

    // A2 A0 A1

    unsigned long processing_time = micros() - buffer_start_time;
    Serial.println(processing_time);
    data_to_process = false;
  }
}

void identify_microphones() {
  for (uint8_t input = 0; input < NUM_INPUTS; input++) {
    uint16_t sum = 0;
    for (size_t i = input; i < NUM_MIC_ID_AVERAGE_POINTS * NUM_INPUTS; i += NUM_INPUTS) {
      sum += inp[i];
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
    Serial.print("i"); Serial.print(input);
    Serial.print(" (average "); Serial.print(average);
    Serial.print(") -> m"); Serial.println(mic_no);
  }
  
  Serial.println("Microphones identified.");
  have_identified_microphones = true;
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
  if ((adc_get_status(ADC) & ADC_ISR_RXBUFF) ==	ADC_ISR_RXBUFF) { // If the buffer is full
    buffer_start_time = buffer_end_time;
    buffer_end_time = micros();
    data_to_process = true;

    // Instruct the DMA to copy into `inp`
    ADC->ADC_RNPR = (uint32_t)inp;
    ADC->ADC_RNCR = INP_BUFF;
  }
}

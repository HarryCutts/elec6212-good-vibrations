#undef TIMING

const int ADC_MAX = 4096;

const unsigned long SAMPLE_RATE =    150000UL;
const unsigned long CLOCK_MAIN  = 84000000UL;
#define TMR_CNTR (CLOCK_MAIN / (2 * SAMPLE_RATE))

const uint16_t NUM_INPUTS = 4;

const uint16_t MEASUREMENTS_PER_BUFFER = 2048*4;
  // The number of measurements *from each sensor* to store in the buffer
const uint16_t INP_BUFF = MEASUREMENTS_PER_BUFFER * NUM_INPUTS;

uint16_t data[INP_BUFF] = {0};     // DMA likes ping-pongs buffer

void setup() {
  SerialUSB.begin(10000000);
  Serial.begin(115200);
  adc_setup();
  tmr_setup();
  pio_TIOA0();  // drive Arduino pin 2 at SMPL_RATE to bring clock out
}

void loop() {
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

void adc_enable_freerunning(Adc *p_adc, const enum adc_channel_num_t adc_ch) {
  p_adc->ADC_MR |= 1 << adc_ch;
}

void adc_setup() {
  pmc_enable_periph_clk(ID_ADC);
  adc_init(ADC, SystemCoreClock, ADC_FREQ_MAX, ADC_STARTUP_FAST);
  // adc_enable_freerunning(ADC, ADC_CHANNEL_7);  // AN0
  // adc_enable_freerunning(ADC, ADC_CHANNEL_6);  // AN1
  // adc_enable_freerunning(ADC, ADC_CHANNEL_5);  // AN2
  // adc_enable_freerunning(ADC, ADC_CHANNEL_4);  // AN3
  NVIC_EnableIRQ(ADC_IRQn);               // enable ADC interrupt vector

  adc_disable_all_channel(ADC);
  adc_enable_interrupt(ADC, ADC_IER_RXBUFF);

  ADC->ADC_RPR  = (uint32_t) data;      // DMA buffer
  ADC->ADC_RCR  = INP_BUFF;
  ADC->ADC_RNPR = (uint32_t) data;      // next DMA buffer
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

void write_uint16_t(uint16_t value) {
  // TODO: check the endianness
  for (int i = 0; i < 2; i++) {
    SerialUSB.write((value >> (8 * i)) & 0xff);
  }
}

#ifdef TIMING
unsigned long sampling_period_start;
#endif

void ADC_Handler(void) {
  if ((adc_get_status(ADC) & ADC_ISR_RXBUFF) ==	ADC_ISR_RXBUFF) { // If the buffer is full
    #ifdef TIMING
	Serial.println(micros() - sampling_period_start);
	#endif

    int maxs[NUM_INPUTS] = {0,0,0,0};
    int mins[NUM_INPUTS] = {ADC_MAX,ADC_MAX,ADC_MAX,ADC_MAX};

    for (int i = 0; i < INP_BUFF; i++) {
      if (data[i] > maxs[i%NUM_INPUTS]){
        maxs[i%NUM_INPUTS] = data[i];
      }
      if (data[i] < mins[i%NUM_INPUTS]){
        mins[i%NUM_INPUTS] = data[i];
      }
    }

    boolean rangeFlag = false;

    for (int m = 0; m < NUM_INPUTS; m++){
      if(maxs[m] - mins[m] > 100 ){
        rangeFlag = true;
      }
    }

    if (rangeFlag == true){
      for (int i = 0; i < INP_BUFF; i++) {
        write_uint16_t(data[i]);
      }
    }

    //set the next write
    ADC->ADC_RNPR = (uint32_t)data;
    ADC->ADC_RNCR = INP_BUFF;

    #ifdef TIMING
    sampling_period_start = micros();
    #endif
  }
}

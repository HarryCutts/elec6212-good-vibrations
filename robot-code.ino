#include <math.h>

#define SHOW_TIMINGS

const unsigned long SAMPLE_RATE =    10000UL;
const unsigned long CLOCK_MAIN  = 84000000UL;
#define TMR_CNTR (CLOCK_MAIN / (2 * SAMPLE_RATE))

const uint16_t NUM_INPUTS = 4;
const uint16_t microphone_averages[NUM_INPUTS]   = {1619, 1477, 1375, 1262}; //{1500, 1600, 1700, 1800};
uint16_t microphone_averages_measured[NUM_INPUTS];
const uint16_t microphone_thresholds[NUM_INPUTS] = {1900, 1800, 1700, 1600}; //{1600, 1700, 1800, 1900};
const uint8_t THRESHOLD = 100;

const uint16_t NUM_MIC_ID_AVERAGE_POINTS = 10;
  // The number of data points to average when identifying microphones

const uint8_t NUM_BUFFERS = 2;

const uint16_t MEASUREMENTS_PER_BUFFER = 2048*2;
  // The number of measurements *from each sensor* to store in the buffer
const uint16_t INP_BUFF = MEASUREMENTS_PER_BUFFER * NUM_INPUTS;

uint8_t mic_to_input_number[NUM_INPUTS];
uint8_t input_to_mic_number[NUM_INPUTS];
bool have_identified_microphones = false;

typedef enum {
  WAITING_FOR_CONNECTION,
  SENDING_DATA
} State;

State state;

typedef struct {
  unsigned long start_time, end_time;
    // The times at which the first and last measurements in the buffer
    // were taken, in microseconds.

  uint16_t data[INP_BUFF] = {0};     // DMA likes ping-pongs buffer
} InputBuffer;

void identify_microphones(InputBuffer &buff);
void dump_buffer(InputBuffer &buff);
void process_data(InputBuffer &buff);
void serial_dump_buffer(InputBuffer &buff);
bool check_buffer_for_tap(InputBuffer &buff);

volatile bool data_to_process = false;
volatile uint8_t write_buffer_index = 0, read_buffer_index = 0;

InputBuffer buffers[NUM_BUFFERS];
unsigned long data_start_time, data_end_time;
  // The times at which the first and last measurements in the last buffer
  // that was filled, in microseconds.

void setup() {
  serial_init();
  data_end_time = micros();
  state = WAITING_FOR_CONNECTION;
}

int mic_id_counter = 0;

void loop() {
  switch (state) {
    case WAITING_FOR_CONNECTION:
      if (serial_connection_ready()) {
        adc_setup();
        tmr_setup();
        pio_TIOA0();  // drive Arduino pin 2 at SMPL_RATE to bring clock out
        serial_send_size_header();
        state = SENDING_DATA;
      }
      break;
    case SENDING_DATA:
      break;
  }
  if (data_to_process) {
    unsigned long buffer_duration = data_end_time - data_start_time;
    #ifdef SHOW_TIMINGS
      Serial.println(buffer_duration);
    #endif

    InputBuffer &buff = buffers[read_buffer_index];
    if (!have_identified_microphones) {
      // The analogue inputs take a while to "warm up"
      mic_id_counter++;
      if (mic_id_counter >= 5){
        identify_microphones(buff);
        for (uint8_t mic_no = 0; mic_no < NUM_INPUTS; mic_no++) {
          Serial.println(microphone_averages_measured[mic_no]);
        }
      }
    }
    else{
      if (check_buffer_for_tap(buff)){
        serial_dump_buffer(buff);
      }
    }

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
    microphone_averages_measured[mic_no] = average;
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
    if (data_to_process==true){
      Serial.println("Congestion. Last data has not been sent before next buffer was filled.");
    }

    data_start_time = data_end_time;
    data_end_time = micros();

    read_buffer_index = write_buffer_index;
    write_buffer_index = (write_buffer_index + 1) % NUM_BUFFERS;

    InputBuffer &buff = buffers[write_buffer_index];
    buff.start_time = data_start_time;
    buff.end_time   = data_end_time;

    //set the next write
    ADC->ADC_RNPR = (uint32_t)buff.data;
    ADC->ADC_RNCR = INP_BUFF;

    data_to_process = true;
  }
}

////////////
// Serial //
////////////

const unsigned long USB_BAUD_RATE = 1000000;
const unsigned long BAUD_RATE = 115200;

const uint8_t HOST_READY = 'r';
const uint8_t HEADER_DATA_SIZE = 's';
  // Sent at the start of the connection. Followed by:
  // * the number of measurements per buffer (16 bits)
  // * the number of inputs in the data (8 bits)

const uint8_t BUFFER_START = 'b';
  // Sent at the start of a buffer. Followed by:
  // * the start time of the buffer (32 bits)
  // * the end time of the buffer (32 bits)

void write_uint16_t(uint16_t value) {
  // TODO: check the endianness
  for (int i = 0; i < 2; i++) {
    SerialUSB.write((value >> (8 * i)) & 0xff);
  }
}

void write_unsigned_long(unsigned long value) {
  for (int i = 0; i < 4; i++) {
    SerialUSB.write((value >> (8 * i)) & 0xff);
  }
}

void serial_init() {
  SerialUSB.begin(USB_BAUD_RATE);
  Serial.begin(BAUD_RATE);
}

bool serial_connection_ready() {
  if (SerialUSB.available() > 0) {
    return SerialUSB.read() == HOST_READY;
  } else {
    return false;
  }
}

void serial_send_size_header() {
  // Tell the host what buffer sizes to expect
  SerialUSB.write(HEADER_DATA_SIZE);
  write_uint16_t(MEASUREMENTS_PER_BUFFER);
  SerialUSB.write(NUM_INPUTS);
}

void serial_dump_buffer(InputBuffer &buff) {
  SerialUSB.write(BUFFER_START);
  write_unsigned_long(buff.start_time);
  write_unsigned_long(buff.end_time);
    for (int i = 0; i < NUM_INPUTS; i++){
    SerialUSB.write(input_to_mic_number[i]);
    Serial.print("MICS: ");
    Serial.print(input_to_mic_number[i]);
    Serial.print("\n");
  }
  for (int i = 0; i < INP_BUFF; i++) {
    write_uint16_t(buff.data[i]);
  }
}

bool check_buffer_for_tap(InputBuffer &buff) {
  for (size_t i = 0; i < INP_BUFF; i += 1) {
    int mic_no = input_to_mic_number[i%NUM_INPUTS];
    if(abs(buff.data[i] - microphone_averages_measured[mic_no]) > THRESHOLD){
      Serial.println(buff.data[i]);
      have_identified_microphones=false;
      return true;
    }
  }
  return false;
}

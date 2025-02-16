#include <driver/i2s.h>
#include <Adafruit_NeoPixel.h>

#define NUMPIXELS        1
Adafruit_NeoPixel pixels(NUMPIXELS, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);

// Ring
#define LED_PIN 7
#define LED_COUNT 12
Adafruit_NeoPixel ring(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800) ;
int ring_yellow_flag = 1; // flag to allow switch on ring in Yellow ONE time (avoid mutlitple call to HalfRingYellow)
int cnt_ring = 0;

// you shouldn't need to change these settings
#define SAMPLE_BUFFER_SIZE 512
#define SAMPLE_RATE 8000
// most microphones will probably default to left channel but you may need to tie the L/R pin low
#define I2S_MIC_CHANNEL I2S_CHANNEL_FMT_ONLY_LEFT
// either wire your microphone to the same pins or change these to match your wiring
#define I2S_MIC_SERIAL_CLOCK GPIO_NUM_3
#define I2S_MIC_LEFT_RIGHT_CLOCK GPIO_NUM_4
#define I2S_MIC_SERIAL_DATA GPIO_NUM_1

#define ONE_Q46 70368744177664

#define YELLOW_LED_PIN 0 // GPIO 0 
#define RED_LED_PIN 5 // GPIO 5

// state machine
#define NOMINAL 0
#define ALERTE1 1
#define ALERTE2 2
int state = NOMINAL;
uint64_t cnt = 0; // compter temps au dessus du bruit 
uint64_t cnt_debounce = 0;
bool flag_debounce = false;

double th_braille_db = 80;//60;
uint64_t noise_time = 30;
uint64_t debounce_time = 15;

typedef struct {
  float alpha; 
  float step;
  float state;
  float last_state;
  int fixed_state_Q8;
} led_state; 

#ifdef EXP_FILT
void switch_led(led_state *led, float target) {
  led->state = led->alpha *target + (1-led->alpha)*led->last_state;
  led->fixed_state_Q8 = (int)(led->state*255);
  led->last_state = led->state;
}
#else
void switch_led(led_state *led, float target) {
  //float step = 0.01; 
  float step = led->step; 
  if ((led->state + step)<= target)
  {
    led->state = led->last_state + step;
  } else if((led->state - step) >= target) {
    led->state = led->last_state - step;
  }
  
  led->fixed_state_Q8 = (int)(led->state*255);
  led->last_state = led->state;
}
#endif

void switch_led_on(led_state *led) {
  switch_led(led, 1);
}
void switch_led_off(led_state *led) {
  switch_led(led, 0);
}

led_state red_led_state = {.alpha = 0.005,
                           .step = 0.01,
                           .state = 0,
                           .last_state = 0,
                           .fixed_state_Q8 = 0};

led_state yellow_led_state = {.alpha = 0.05,
                             .step = 0.05,
                             .state = 1,
                             .last_state = 1,
                             .fixed_state_Q8 = 255};

void reset_state_red_led(led_state *led) {
    led->state = 0;
    led->last_state = 0;
    led->fixed_state_Q8 = 0;
}

void reset_state_yellow_led(led_state * led) {
    led->state = 1;
    led->last_state = 1;
    led->fixed_state_Q8 = 255;
  
}

// don't mess around with this
i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 4,
    .dma_buf_len = 1024,
    .use_apll = false,
    .tx_desc_auto_clear = false,
    .fixed_mclk = 0};

// and don't mess around with this
i2s_pin_config_t i2s_mic_pins = {
    .bck_io_num = I2S_MIC_SERIAL_CLOCK,
    .ws_io_num = I2S_MIC_LEFT_RIGHT_CLOCK,
    .data_out_num = I2S_PIN_NO_CHANGE,
    .data_in_num = I2S_MIC_SERIAL_DATA};

char bytes_buff[3];

void setup()
{
  state = NOMINAL;
  // we need serial output for the plotter
  Serial.begin(115200);
  // start up the I2S peripheral
  i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_NUM_0, &i2s_mic_pins);
  Serial.printf("START PRGM\n");
  
  // PIN GPIO2 - NeoPixel => TO DELETE
  /*pixels.begin(); // INITIALIZE NeoPixel strip object (REQUIRED)
  pixels.setBrightness(20); // not so bright*/

  // Ring 
  ring.begin() ;
  ring.show() ;
  ring.setBrightness(100) ;

  // LED 
  pinMode(YELLOW_LED_PIN, OUTPUT);
  pinMode(RED_LED_PIN, OUTPUT);
  
  // Init : nominal
  analogWrite(YELLOW_LED_PIN, 255);
  analogWrite(RED_LED_PIN, 0);
}

int32_t raw_samples[SAMPLE_BUFFER_SIZE];
uint64_t temp;
double rms_level = 0;
double rms_level_dB;
int i = 0;
void loop()
{
    if (Serial.available()) {
    Serial.readBytes(bytes_buff, 3*sizeof(uint8_t));
    Serial.printf("RKEOKOZE = %d,%d,%d\n", atoi(&bytes_buff[0]), atoi(&bytes_buff[1]), atoi(&bytes_buff[2]));
  }
    uint32_t color;

  // read from the I2S device
  size_t bytes_read = 0;
  
  i2s_read(I2S_NUM_0, raw_samples, sizeof(int32_t) * SAMPLE_BUFFER_SIZE, &bytes_read, portMAX_DELAY);
  int samples_read = bytes_read / sizeof(int32_t);
  // dump the samples out to the serial channel.
  rms_level = 0;
  for (int i = 0; i < samples_read; i++)
  {
    temp = (uint64_t)((int64_t)raw_samples[i]*(int64_t)raw_samples[i]);
    rms_level += (double)temp/ONE_Q46 ;
  }
  rms_level = rms_level / ((float)samples_read);
  rms_level_dB = 20*log10(rms_level)+120 ; // + 120 for dBSPL convertion
  /*Serial.printf("rms_level_dB=%f\r\n", rms_level_dB);
  Serial.printf("state= %d\n",state);*/
  state = get_state();

  switch (state)
  {
    case NOMINAL :
      cnt_debounce = 0;      
      switch_led_on(&yellow_led_state);
      switch_led_off(&red_led_state);
      analogWrite(YELLOW_LED_PIN, yellow_led_state.fixed_state_Q8);      
      analogWrite(RED_LED_PIN, red_led_state.fixed_state_Q8);      
      if (cnt_ring <= 1)
      {
        Serial.printf("cnt_ring %d\r\n",cnt_ring);
        cnt_ring ++;
        for(int i=0 ; i<LED_COUNT; i++) { 
            //Serial.printf("lighton: %i",i);
            color = ring.Color(200>>3, 60>>3, 0); //
            ring.setPixelColor(i, color);     
            
            //ring.setBrightness(10) ;
        }
        ring.show();           

      }
        //ring.fill(0xFFD700, 0, LED_COUNT)
      break;

    case ALERTE1:
      // Inversion progressive JAUNE / ROUGE
      switch_led_on(&red_led_state);
      switch_led_off(&yellow_led_state);
      Serial.printf("cnt=%d, %f %d | %f %d\r\n",cnt, red_led_state.state, red_led_state.fixed_state_Q8, yellow_led_state.state, yellow_led_state.fixed_state_Q8);
      analogWrite(YELLOW_LED_PIN, yellow_led_state.fixed_state_Q8);      
      analogWrite(RED_LED_PIN, red_led_state.fixed_state_Q8);      

      if (red_led_state.fixed_state_Q8 >= 254){
        state = ALERTE2;
        reset_state_red_led(&red_led_state);
        reset_state_yellow_led(&yellow_led_state);
      }      
      break;

      case ALERTE2:                                                 
        flag_debounce = true; 
        analogWrite(YELLOW_LED_PIN, 0);
        analogWrite(RED_LED_PIN, 0);
        HalfRingRed(0);
        delay(200);
        analogWrite(RED_LED_PIN, 255);
        HalfRingRed(1);
        delay(200);
        break;
  }
  delay(10);
}

int get_state()
{
  //int state;
    if ((state == ALERTE2) & (flag_debounce == true))
    {
      Serial.printf("cnt_debounce=%d\r\n", cnt_debounce);
        cnt_debounce ++;
        if (cnt_debounce < debounce_time)
        {
          return ALERTE2;
        } else {
          cnt_ring = 0;
          flag_debounce = false;
        }
    }    
    if (rms_level_dB > th_braille_db)
    {
      cnt ++;
      if   (cnt > noise_time)
      { 
        cnt = 0;
        if ((state != ALERTE1) &&  (state != ALERTE2)) {         
            state = ALERTE1;
          }
      }

    } else {
        cnt = 0;
        state = NOMINAL;
  }
  return state;
}

void HalfRingRed(int mod) 
{
    uint32_t color;
    for(int i=0 ; i<LED_COUNT; i++) { 
        //Serial.printf("lighton: %i",i);
        if (i%2 == mod)
        {
          color = ring.Color(255, 0, 0); // red          
        } else {
          color = ring.Color(0, 0, 0); // black          
        }
        ring.setPixelColor(i, color);        
    }
    ring.show();

}
/*
void vuMeter(float rms_db, int state)
{
    // mappgig rms <-> 12 led    
    uint32_t color;
    float rms_max = 160;
    float rms_min = 40;
    float dynamic = rms_max-rms_min;
    int sensitivity_in_led = (int)(rms_max-rms_min)/12;
    int nb_led_on = 0;
    nb_led_on = (rms_db - rms_min) * 1/sensitivity_in_led;

    if (state != ALERTE2){
      Serial.printf("%d, %d, %f\r\n", nb_led_on, sensitivity_in_led, rms_db);
      if (nb_led_on == 0) {my_clear(); delay(10); return;}
      for(int i=0 ; i<nb_led_on; i++) {
        if ((i+1) <= 4) 
          color = ring.Color(0, 255, 0); // green
        else if (((i+1) > 4) & ((i+1) <= 8))
          color = ring.Color(255, 128, 0); // orange
        else if ((i+1)>8)
          color = ring.Color(255, 0, 0); // red

        //Serial.printf("lighton: %i",i);
        ring.setPixelColor(i, color);
      }
      ring.show();
      //ring.clear();
    }
}
*/

void my_clear() {
  for(int i=0 ; i<LED_COUNT; i++) {
    //Serial.printf("clear %d\n", i);
    ring.setPixelColor(i, ring.Color(0,0,0));
  }
  ring.show();
}


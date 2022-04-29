#include "driver/i2s.h"
#include <arduinoFFT.h>
#include <math.h>

const i2s_port_t I2S_PORT = I2S_NUM_0;
const int BLOCK_SIZE = 1024;
const double samplingFrequency = 8000;
double vReal[BLOCK_SIZE];
double vImag[BLOCK_SIZE];
int32_t samples[BLOCK_SIZE];
double X[BLOCK_SIZE];
double Y[BLOCK_SIZE];
arduinoFFT FFT = arduinoFFT();
const int N = 81;
float Cof[81] = {
   -0.01480451588141, -0.01069529966469,4.464568485722e-17,   0.0107136861949,
    0.01483114229013,  0.01009383334006,-3.940174545072e-17,-0.008786048967062,
    -0.0111234655039,-0.006765865255715,6.632226657758e-18, 0.004036432778346,
   0.003414781918883,0.0006297614487458,6.689604225833e-18, 0.003393286719965,
   0.007932798288516, 0.007944290372283,-2.358644403188e-17,  -0.0129098235611,
   -0.02192971814126, -0.01815518804624,4.407178429298e-17,  0.02352918076801,
    0.03707221606579,  0.02886970216429, -5.8765567894e-17,  -0.0340099677099,
   -0.05155262647519, -0.03878505272618, 1.70957414817e-17,  0.04303848088918,
    0.06353291382628,  0.04662856107621,-4.112952345533e-17, -0.04943418565441,
   -0.07143455583997,   -0.051359825729,2.745296114216e-17,  0.05233949421756,
    0.07419353184649,  0.05233949421756,2.745296114216e-17,   -0.051359825729,
   -0.07143455583997, -0.04943418565441,-4.112952345533e-17,  0.04662856107621,
    0.06353291382628,  0.04303848088918, 1.70957414817e-17, -0.03878505272618,
   -0.05155262647519,  -0.0340099677099, -5.8765567894e-17,  0.02886970216429,
    0.03707221606579,  0.02352918076801,4.407178429298e-17, -0.01815518804624,
   -0.02192971814126,  -0.0129098235611,-2.358644403188e-17, 0.007944290372283,
   0.007932798288516, 0.003393286719965,6.689604225833e-18,0.0006297614487458,
   0.003414781918883, 0.004036432778346,6.632226657758e-18,-0.006765865255715,
    -0.0111234655039,-0.008786048967062,-3.940174545072e-17,  0.01009383334006,
    0.01483114229013,   0.0107136861949,4.464568485722e-17, -0.01069529966469,
   -0.01480451588141
};
float p = 0.0;
double Ex, Ey = 0;
double y[BLOCK_SIZE];

#define LED 5

void setup() {
  // put your setup code here, to run once:
  pinMode(LED, OUTPUT);
  Serial.begin(115200);
  esp_err_t err;

  //Thw I2S config as per the example
  const i2s_config_t i2s_config = { 
      .mode = i2s_mode_t(I2S_MODE_MASTER | I2S_MODE_RX),//Receive, not transfer
      .sample_rate = samplingFrequency,                 //16KHz
      .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,     //could only get it to work with 32bits
      .channel_format = I2S_CHANNEL_FMT_ONLY_RIGHT,     //use right channel
      .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB),
      .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,         //Interrupt level 1
      .dma_buf_count = 3,                               //NUMBER OF BUFFERS(4)
      .dma_buf_len = BLOCK_SIZE                         //8 samples per buffer (minimum)
  };
  const i2s_pin_config_t pin_config = {
      .bck_io_num = 26,                                 //Serial Clock (SCK)
      .ws_io_num = 25,                                  //Word Select (WS)
      .data_out_num = I2S_PIN_NO_CHANGE,                //not used (only for speakers)
      .data_in_num = 33                                 //Serial Data (SD)
    };
  //Configuring the I2S driver and pins
  //This function must be called before any I2S driver read/write operations.
  err = i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
  if(err != ESP_OK){
    Serial.printf("Failed installing driver: %d\n",err);
    while(true);
    }
  err = i2s_set_pin(I2S_PORT, &pin_config);
  if(err != ESP_OK){
    Serial.printf("Failed setting pin: %d\n",err);
    while(true);
    }
  Serial.println("I2S driver installed.");
}

void loop() {
  // put your main code here, to run repeatedly:
  //Read a single sample and log it for the Serial Plotter
  Ex = 0;
  Ey = 0;
  for(int i=0; i<BLOCK_SIZE; i++){
    int32_t sample = 0;
    int bytes_read = i2s_pop_sample(I2S_PORT, (char *)&sample, portMAX_DELAY); // no timeout
    X[i]=sample/5000000;  
    Ex = X[i]*X[i]+Ex;
  }
  for(int n=N-1; n< BLOCK_SIZE; n++){
    //ESPACIO PARA APLICAR EL FILTRO
    y[n] = 0;
    for(int j=0; j< N; j++){
      p = Cof[j]*X[n-j];
      y[n] = p + y[n];
    }
    //CALCULO DE ENERGIA
    Ey = y[n]*y[n]+Ey;
    //Serial.print(X[n]); Serial.print(" , "); Serial.print(y[n]); Serial.println();
  }
  //Condicion
  if((Ey/Ex) > 0.78){
    digitalWrite(LED, HIGH);
  }else{
    digitalWrite(LED, LOW);  
  }
  //ESPACIO PARA PLOTEAR Ex/Ey
  Serial.println((Ey/Ex));
  delay(5);
}

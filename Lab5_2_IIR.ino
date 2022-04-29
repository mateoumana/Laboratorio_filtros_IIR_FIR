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
arduinoFFT FFT = arduinoFFT();
double y11[BLOCK_SIZE], y12[BLOCK_SIZE], y21[BLOCK_SIZE], y22[BLOCK_SIZE] ,y23[BLOCK_SIZE], y24[BLOCK_SIZE];
float r = 0.97;
/*
//Frecuencias del espectro en frecuencias de los audios
float Fr1_si = 205, Fr2_si = 105; //208
float Fr1_no = 100, Fr2_no = 200, Fr3_no = 303, Fr4_no = 402;
//Filtro resonador para cada frecuencia
float w01_si = 2*PI*Fr1_si/8000, w02_si = 2*PI*Fr2_si/8000; //normalizacion
float w01_no = 2*PI*Fr1_no/8000, w02_no = 2*PI*Fr2_no/8000, w03_no = 2*PI*Fr2_no/8000, w04_no = 2*PI*Fr2_no/8000; //normalizacion
//Coeficientes filtros para SI
float b01_si1, b11_si1, b01_si2, b11_si2;
//Coeficientes filtros para NO
float b02_no1, b12_no1, b02_no2, b12_no2, b02_no3, b12_no3, b02_no4, b12_no4;
//Coeficientes filtros para SI
b01_si1 = ((1-r)*sqrt(1+r*r-2*r*cos(2*w01_si))); b11_si1 = 2*r*cos(w01_si);
b01_si2 = ((1-r)*sqrt(1+r*r-2*r*cos(2*w02_si))); b11_si2 = 2*r*cos(w02_si);
//Coeficientes filtros para NO
b02_no1 = ((1-r)*sqrt(1+r*r-2*r*cos(2*w01_no))); b12_no1 = 2*r*cos(w01_no);
b02_no2 = ((1-r)*sqrt(1+r*r-2*r*cos(2*w02_no))); b12_no2 = 2*r*cos(w02_no);
b02_no3 = ((1-r)*sqrt(1+r*r-2*r*cos(2*w03_no))); b12_no3 = 2*r*cos(w03_no);
b02_no4 = ((1-r)*sqrt(1+r*r-2*r*cos(2*w04_no))); b12_no4 = 2*r*cos(w04_no);
*/
double Coef_11[3] = {0.00951598971121077,1.91490883497003,r*r}, Coef_12[3] = {0.00495020373013465, 1.93340698668302,r*r};
//double Coef_21[3] = {0.00472293947885103,1.93401962744227,r*r}, Coef_22[3] = {0.00928791271215778, 1.91611538075457,r*r};
//double Coef_23[3] = {0.013959413507478, 1.88532532744983,r*r}, Coef_24[3] = {0.0183711029884377, 1.84410568440778,r*r};
//double Coef_11[3] = {0.00933353545196035, 1.91587643491838,r*r}, Coef_12[3] = {0.01832703292276120, 1.84457823192445,r*r};
double Coef_21[3] = {0.01010852288726950, 1.91163366204767,r*r}, Coef_22[3] = {0.02095068992276270, 1.81422228027067,r*r};
double Coef_23[3] = {0.029734198307489, 1.67677826914214,r*r};
double Ex, Ey11, Ey12, Ey21, Ey22, Ey23, Ey24, Ey1t, Ey2t = 0;

#define LED_GREEN 5
#define LED_RED 18

void setup() {
  // put your setup code here, to run once:
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_RED, OUTPUT);
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
  Ex = 0; Ey11 = 0; Ey12 = 0; Ey21 = 0; Ey22 = 0; Ey23 = 0; Ey24 = 0; Ey1t = 0; Ey2t = 0;
  for(int i=0; i<BLOCK_SIZE; i++){
    int32_t sample = 0;
    int bytes_read = i2s_pop_sample(I2S_PORT, (char *)&sample, portMAX_DELAY); // no timeout
    X[i]=sample/100000;  
    Ex = X[i]*X[i]+Ex;
  }
  for(int n=2; n< BLOCK_SIZE; n++){
    //ESPACIO PARA APLICAR EL FILTRO1 y FILTRO2
    //y(n) = b0*x(n) + 2*r*cos(w0)*y(n-1) - r*r*y(n-2);
    y11[n] = Coef_11[0]*X[n] + Coef_11[1]*y11[n-1] - Coef_11[2]*y11[n-2];
    y12[n] = Coef_12[0]*X[n] + Coef_12[1]*y12[n-1] - Coef_12[2]*y12[n-2];
    //ESPACIO PARA APLICAR EL FILTRO2
    y21[n] = Coef_21[0]*X[n] + Coef_21[1]*y21[n-1] - Coef_21[2]*y21[n-2];
    y22[n] = Coef_22[0]*X[n] + Coef_22[1]*y22[n-1] - Coef_22[2]*y22[n-2];
    y23[n] = Coef_23[0]*X[n] + Coef_23[1]*y23[n-1] - Coef_23[2]*y23[n-2];
    //y24[n] = Coef_24[0]*X[n] + Coef_24[1]*y24[n-1] - Coef_24[2]*y24[n-2];
    //ESPACIO PARA PLOTEAR Y[N] Y X[N]
    //Serial.print(X[n]); Serial.print(" "); Serial.print(y24[n]);
    //Serial.println();
    //CALCULO DE ENERGIA
    Ey11 = y11[n]*y11[n] + Ey11;
    Ey12 = y12[n]*y12[n] + Ey12;
    Ey1t = Ey11 + Ey12;
    Ey21 = y21[n]*y21[n] + Ey21;
    Ey22 = y22[n]*y22[n] + Ey22;
    Ey23 = y23[n]*y23[n] + Ey23;
    //Ey24 = y24[n]*y24[n] + Ey24;
    Ey2t = Ey21 + Ey22 + Ey23 + Ey24;
  }
  //Condicion
  if(((Ey1t/Ex)) > 0.85){
    digitalWrite(LED_GREEN, HIGH);
    digitalWrite(LED_RED, LOW);
  }else if(((Ey2t/Ex)) > 0.85){
    digitalWrite(LED_GREEN, LOW);
    digitalWrite(LED_RED, HIGH);
  }else{
    digitalWrite(LED_GREEN, LOW); 
    digitalWrite(LED_RED, LOW); 
  }
  //ESPACIO PARA PLOTEAR Ex/Ey
  Serial.print(Ey1t/Ex); Serial.print(" , "); Serial.print((Ey2t/Ex));
  Serial.println();
}

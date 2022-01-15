#include <Arduino.h>
#include <M5Stack.h>
#include <driver/i2s.h>

#define PIN_CLK  22
#define PIN_DATA 21

#define SAMPLE_RATE 44000
#define N_VALUES_TO_READ 4096

#define HORIZONTAL_RESOLUTION 320
#define VERTICAL_RESOLUTION   240
#define POSITION_OFFSET_Y      20 // Keicia grafika

double sigPeakDetect = 16000;
int peakLenght = 1000;
int claps = 0;
int clapType = 0;
int clapLenght = 1000;
int timeStart = 0;
int maxPossiblePauseBetweenClaps = 4000;
int oneClapType = 0;
int doubleClapType = 0;
int tripleClapType = 0;




//------------------------------------------------------------------------------------------
// PDM mikrofonui reikalingos I2S konfiguracijos strukturos 

i2s_config_t i2s_config = {
  .mode = (i2s_mode_t)(i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_PDM),
  .sample_rate = SAMPLE_RATE,
  .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT, // is fixed at 12bit, stereo, MSB
  .channel_format = I2S_CHANNEL_FMT_ONLY_RIGHT,
  .communication_format = I2S_COMM_FORMAT_I2S,
  .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
  .dma_buf_count = 2,
  .dma_buf_len = 128,
};

i2s_pin_config_t pin_config = {
  .bck_io_num = I2S_PIN_NO_CHANGE,
  .ws_io_num = PIN_CLK,
  .data_out_num = I2S_PIN_NO_CHANGE,
  .data_in_num = PIN_DATA
};

// Pagalbine struktura signalui
struct rawSignalStruct
{
  int16_t micRawData[N_VALUES_TO_READ]; // Nuskaitytos signalo reiksmes (taskai)
  size_t bytesRead;                     // Realiai nuskaitytu tasku kiekis
};

QueueHandle_t rawSignalQueue = nullptr;

//------------------------------------------------------------------------------------------
// Pagalbiniu f-ju deklaracijos
void showSignal(int16_t * sig, uint16_t length, uint16_t amplify);
void showSignalUnsigned(uint16_t * sig, uint16_t length, uint16_t amplify);
//------------------------------------------------------------------------------------------

static void i2cMicReadtask(void *arg)
{
  static struct rawSignalStruct rawSignal;

  while(1)
  {
    i2s_read(I2S_NUM_0, rawSignal.micRawData, N_VALUES_TO_READ*2, &rawSignal.bytesRead, portMAX_DELAY);
    rawSignal.bytesRead /= 2;
    if (xQueueSendToBack(rawSignalQueue, (void *)&rawSignal, pdMS_TO_TICKS(1)) != pdPASS)
    {
      Serial.println("ERR: signal processing is not done - samples will be lost");
    }
 }
}

static void i2cMicProcesstask(void *arg)
{
  static struct rawSignalStruct rawSignalCopy;
  static long tic, toc;
  static bool signalOrFFT = false;
  while (1)
  {
    if (xQueueReceive(rawSignalQueue, (void *)&rawSignalCopy, pdMS_TO_TICKS(5000)) == pdPASS)
    {
      M5.update();
      if (M5.BtnA.wasReleased() || M5.BtnA.pressedFor(1000, 200)) {
        if (clapLenght < maxPossiblePauseBetweenClaps) { //didziauasias laiko tarpas tarp suplojimu 
          clapLenght += 200;
          M5.Lcd.fillRect(43,36,62,21, BLACK); // Istrina(Pakeicia juodais pixeliais) sena teksta 21 turbut eilutes aukstis pixeliais
          M5.Lcd.setCursor(43,36);
          M5.Lcd.printf("Tarpas:");
          M5.Lcd.print(clapLenght);
        }
      }
      if (M5.BtnB.wasReleased() || M5.BtnB.pressedFor(1000, 200)) {
        if (clapLenght > 500) {
          clapLenght -= 200;
          M5.Lcd.fillRect(43,36,62,21, BLACK); // Istrina(Pakeicia juodais pixeliais) sena teksta 21 turbut eilutes aukstis pixeliais
          M5.Lcd.setCursor(43,36);
          M5.Lcd.printf("Tarpas:");
          M5.Lcd.print(clapLenght);
        }
      }
      if (M5.BtnC.wasReleased() || M5.BtnC.pressedFor(1000, 200)) {
        signalOrFFT = !signalOrFFT;
        //M5.Lcd.clear();
      }
      if (signalOrFFT) {
        showSignal(rawSignalCopy.micRawData, rawSignalCopy.bytesRead, 10);
      }

    //Signalo spausdinimas i Serial terminala - imame kas 2 reiksme
    for (int i = 0; i < rawSignalCopy.bytesRead; i+=2){  
      //Serial.println(rawSignalCopy.micRawData[i]);
    // if(maxSignalas < rawSignalCopy.micRawData[i])
    // {
    //   maxSignalas = rawSignalCopy.micRawData[i];
    // }
    if(rawSignalCopy.micRawData[i] > sigPeakDetect || rawSignalCopy.micRawData[i] < -sigPeakDetect) // Aptinkam suplojimus
    {
      int a = i;
      int count = 0;
      bool noise = false;
      while(rawSignalCopy.micRawData[a] > sigPeakDetect || rawSignalCopy.micRawData[a] < -sigPeakDetect) { // Einame per garso signala tol kol nukris zemiau aptikimo ribos
        count++;
        a++;
        if(count > 2000) { // Tikriname ar signalas buvo per aukstas per ilga laiko tarpa 2000 yra tarpas, galimai reiks koreguot
          noise = true;
          break; // Iseinam is ciklo
        }
      }
      if (!noise) {
        i += peakLenght;
        claps += 1;
        M5.Lcd.fillRect(0,0,42,21, BLACK); //ekrano pikseliu koordinates
        M5.Lcd.setCursor(0,0);
        M5.Lcd.println(claps);

        clapType = clapType + 1;
        timeStart = millis();
        if(clapType > 3){
          clapType = 3;
        }
      }
    }
    //tipu atskyrimas ir atvaizdavimas
   if(millis()-timeStart>clapLenght && timeStart!=0) {
      M5.Lcd.fillRect(43,0,62,21, BLACK); //ekrano pikseliu koordinates
      M5.Lcd.setCursor(43,0);
      M5.Lcd.printf("Viengubas:");
      M5.Lcd.print(oneClapType);
      M5.Lcd.setCursor(43,12);
      M5.Lcd.printf("Dvigubas:");
      M5.Lcd.print(doubleClapType);
      M5.Lcd.setCursor(43,24);
      M5.Lcd.printf("Trigubas:");
      M5.Lcd.print(tripleClapType);
      clapType=0;
      timeStart=0;
      }
    }

  }  
    else
    {
      Serial.println("ERR: raw Signal buffer empty");
    }
  }
}

void setup() {
 
  M5.begin();

  Serial.begin(2000000);
  M5.Lcd.setTextColor(WHITE);
  M5.Lcd.setTextSize(2); // Teksto dydis
  M5.Lcd.clear();
  M5.Lcd.setCursor(0,0); //kiekio spausdinimui
  //M5.Lcd.print("0");
  M5.Lcd.fillRect(43,36,62,21, BLACK); // Istrina(Pakeicia juodais pixeliais) sena teksta 21 turbut eilutes aukstis pixeliais
  M5.Lcd.setCursor(43,36);
  M5.Lcd.printf("Tarpas:");
  M5.Lcd.print(clapLenght);
  i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_NUM_0, &pin_config);
  
  rawSignalQueue = xQueueCreate(1, sizeof(rawSignalStruct));
  xTaskCreatePinnedToCore(i2cMicReadtask, "i2cMicReadtask", 4096, NULL, 3, NULL, 1);
  xTaskCreatePinnedToCore(i2cMicProcesstask, "i2cMicProcesstask", 4096, NULL, 3, NULL, 0);
}
void loop() 
{
 
}

//------------------------------------------------------------------------------------------------------------
// Pagalbines f-jos
//--- Rodyti signala LCD ekrane
// sig - signalas, reiksmes su zenklu
// length - signalo ilgis taskais
// amplify - signalo stiprinimas kartais
void showSignal(int16_t * sig, uint16_t length, uint16_t amplify)
{
  static int oldSignal[HORIZONTAL_RESOLUTION];

  int n, m;

  int oldx;
  int oldy;
  int oldSig;
  int x, y;
  int resample_factor = 1;

  if(length > HORIZONTAL_RESOLUTION)
  {
    resample_factor = length/HORIZONTAL_RESOLUTION;
  }

  for (n = 0, m = 0; n < length && m < HORIZONTAL_RESOLUTION; n+=resample_factor, m++)
  {
    x = m;
    y = map(sig[n], SHRT_MIN/amplify, SHRT_MAX/amplify, VERTICAL_RESOLUTION, POSITION_OFFSET_Y);

    if (n > 0)
    {
      // delete old line element
      M5.Lcd.drawLine(oldx , oldSig, x, oldSignal[m], BLACK );

      // draw new line element
      if (m < HORIZONTAL_RESOLUTION - 1) // don't draw last element because it would generate artifacts
      {
        M5.Lcd.drawLine(oldx,    oldy, x,            y, GREEN );
      }
    }
    oldx = x;
    oldy = y;
    oldSig = oldSignal[m];
    oldSignal[m] = y;
  }
}

//--- Rodyti signala LCD ekrane
// sig - signalas, reiksmes be zenklo
// length - signalo ilgis taskais
// amplify - signalo stiprinimas kartais
void showSignalUnsigned(uint16_t * sig, uint16_t length, uint16_t amplify)
{
   static int oldSignal[HORIZONTAL_RESOLUTION];

  int n, m;

  int oldx;
  int oldy;
  int oldSig;
  int x, y;
  int resample_factor = 1;

  if(length > HORIZONTAL_RESOLUTION)
  {
    resample_factor = length/HORIZONTAL_RESOLUTION;
  }

  for (n = 0, m = 0; n < length && m < HORIZONTAL_RESOLUTION; n+=resample_factor, m++)
  {
    x = m;
    y = map(sig[n], 0, USHRT_MAX/amplify, VERTICAL_RESOLUTION, POSITION_OFFSET_Y);

    if (n > 0)
    {
      // delete old line element
      M5.Lcd.drawLine(oldx , oldSig, x, oldSignal[m], BLACK );

      // draw new line element
      if (m < HORIZONTAL_RESOLUTION - 1) // don't draw last element because it would generate artifacts
      {
        M5.Lcd.drawLine(oldx,    oldy, x,            y, RED );
      }
    }
    oldx = x;
    oldy = y;
    oldSig = oldSignal[m];
    oldSignal[m] = y;
  }
}


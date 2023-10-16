#include "BluetoothA2DPSink.h"
#include <arduinoFFT.h>
#include <Wire.h>

#define NUM_BANDS 64
#define SAMPLES 1024
#define SAMPLING_FREQUENCY 44100

#define SDA_0 21
#define SCL_0 22
#define I2C_Freq 150000

BluetoothA2DPSink a2dp_sink;
bool devicePlayedAudio = false;
esp_a2d_audio_state_t currentAudioState = ESP_A2D_AUDIO_STATE_REMOTE_SUSPEND;
bool bleDeviceConnected = false;
int iii = 0;
int32_t peak[NUM_BANDS];
int32_t dotpeak[NUM_BANDS];
int smoothed_peak[NUM_BANDS];
double vReal[SAMPLES];
double vImag[SAMPLES];

QueueHandle_t queue;
int16_t sample_l_int;
int16_t sample_r_int;
int visualizationCounter = 0;
int32_t lastVisualizationUpdate = 0;
float amplitude = 600.0;

int pausewaitn = 0;

String singer;
String songname;

String strofstst = "";

arduinoFFT FFT = arduinoFFT();

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define OLED_RESET -1       // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(128, 64, &Wire, OLED_RESET);
int textx, textminX;
String oledtext = "Playing";
unsigned long frameStartTime;
unsigned long frameEndTime;
int frameCount = 0;
unsigned long lastFrameRateTime = 0;
int frameRate = 0;

int lasttouchbutton = 0;

void createBands(int i, int dsize)
{
  uint8_t band = 0;
  // if (i <= 2) {
  //   band = 0; // 125Hz
  // } else if (i <= 5) {
  //   band = 1; // 250Hz
  // } else if (i <= 7) {
  //   band = 2; // 500Hz
  // } else if (i <= 15) {
  //   band = 3; // 1000Hz
  // } else if (i <= 30) {
  //   band = 4; // 2000Hz
  // } else if (i <= 53) {
  //   band = 5; // 4000Hz
  // } else if (i <= 106) {
  //   band = 6; // 8000Hz
  // } else {
  //   band = 7;
  // }

  // if (i <= 3)
  // {
  //   band = 0; // 125Hz
  // }
  // else if (i <= 6)
  // {
  //   band = 1; // 250Hz
  // }

  // else if (i <= 9)
  // {
  //   band = 2; // 500Hz
  // }
  // else if (i <= 12)
  // {
  //   band = 3; // 250Hz
  // }
  // else if (i <= 16)
  // {
  //   band = 4; // 500Hz
  // }

  // else if (i <= 24)
  // {           // 8
  //   band = 5; // 1000Hz
  // }
  // else if (i <= 28)
  // {
  //   band = 6; // 2000Hz
  // }
  // else if (i <= 32)
  // {
  //   band = 7; // 4000Hz
  // }
  // else if (i <= 36)
  // {
  //   band = 8; // 8000Hz
  // }
  // else if (i <= 40)
  // {           //
  //   band = 9; // 250Hz
  // }
  // else if (i <= 44)
  // {
  //   band = 10; // 500Hz
  // }
  // else if (i <= 48)
  // {
  //   band = 11; // 1000Hz
  // }
  // else if (i <= 56)
  // {            // 8 160
  //   band = 12; // 2000Hz
  // }
  // else if (i <= 64)
  // {
  //   band = 13; // 4000Hz
  // }
  // else if (i <= 76)
  // {
  //   band = 14; // 8000Hz
  // }
  // else if (i <= 89)
  // {
  //   band = 15; // 8000Hz
  // }
  // else if (i <= 101)
  // {
  //   band = 16; // 8000Hz
  // }
  // else if (i <= 109)
  // {
  //   band = 17; // 8000Hz
  // }
  // else if (i <= 116)
  // {
  //   band = 18; // 8000Hz
  // }
  // else
  // {
  //   band = 19; // 8000Hz //265
  // }
  band = map(min(i, 160), 0, 160, 0, NUM_BANDS - 1);
  int dmax = amplitude;
  if (dsize > dmax)
    dsize = dmax;
  if (dsize > peak[band])
  {
    peak[band] = dsize;
  }
}

void connection_state_changed(esp_a2d_connection_state_t state, void *)
{
  log_i("Connection state changed, new state: %d", state);
  if (ESP_A2D_CONNECTION_STATE_CONNECTED == state)
  {
    bleDeviceConnected = true;
  }
  else
  {
    bleDeviceConnected = false;
  }
}

uint8_t getLedIndex(uint8_t x, uint8_t y)
{
  // x = 7 - x;
  if (y % 2 == 0)
  {
    return y * 8 + x;
  }
  else
  {
    return y * 8 + (7 - x);
  }
}

void renderFFT(void *parameter)
{
  int item = 0;
  for (;;)
  {
    if (uxQueueMessagesWaiting(queue) > 0)
    {

      FFT.Windowing(vReal, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
      FFT.Compute(vReal, vImag, SAMPLES, FFT_FORWARD);
      FFT.ComplexToMagnitude(vReal, vImag, SAMPLES);

      for (uint8_t band = 0; band < NUM_BANDS; band++)
      {
        peak[band] = 0;
      }

      // Don't use sample 0 and only first SAMPLES/2 are usable. Each array eleement represents a frequency and its value the amplitude.
      for (int i = 2; i < (SAMPLES / 2); i++)
      {
        if (vReal[i] > 60000)
        { // Add a crude noise filter, 10 x amplitude or more
          createBands(i, (int)vReal[i] / amplitude);
          // Serial.println(i);
          // Serial.print(" ");
        }
      }

      // Release handle
      xQueueReceive(queue, &item, 0);

      uint8_t intensity;
      if ((millis() - lastVisualizationUpdate) > 1000)
      {

        visualizationCounter = 0;
        lastVisualizationUpdate = millis();
      }
      visualizationCounter++;
    }
  }
}

void avrc_metadata_callback(uint8_t id, const uint8_t *text)
{
  // Serial.printf("\d : \s\n",id,text);
  int ischangeinf = 0;
  if (id == ESP_AVRC_MD_ATTR_ARTIST)
  {
    singer = (char *)text;
    ischangeinf = 1;
    // Serial.printf("==> AVRC metadata rsp: attribute id 0x%x, %s\n", id, text);
  }
  else if (id == ESP_AVRC_MD_ATTR_TITLE)
  {
    songname = (char *)text;
    ischangeinf = 1;
  }

  if (ischangeinf)
  {
    if (singer != "" && songname != "")
    {
      oledtext = singer + " / " + songname;
      textx = display.width();
      textminX = -12 * oledtext.length();
    }
    else if (singer != "" || songname != "")
    {
      oledtext = singer + songname;
      textx = display.width();
      textminX = -12 * oledtext.length();
    }
    else
    {
      oledtext = "No Title Name";
      textx = display.width();
      textminX = -12 * oledtext.length();
    }
  }
}

void audio_data_callback(const uint8_t *data, uint32_t len)
{
  int item = 0;
  // Only prepare new samples if the queue is empty
  if (uxQueueMessagesWaiting(queue) == 0)
  {
    // log_e("Queue is empty, adding new item");
    int byteOffset = 0;
    float currentVolume = (float)a2dp_sink.get_volume(); // 當前手機音量級別
    for (int i = 0; i < SAMPLES; i++)
    {
      sample_l_int = (int16_t)(((*(data + byteOffset + 1) << 8) | *(data + byteOffset)));
      sample_r_int = (int16_t)(((*(data + byteOffset + 3) << 8) | *(data + byteOffset + 2)));
      vReal[i] = (sample_l_int + sample_r_int) / 2.0f;
      float gain = powf(2.0f, (120 - currentVolume) / 20.0f); // 計算所需增益
      vReal[i] = vReal[i] * gain;                             // 增益
      vImag[i] = 0;

      byteOffset = byteOffset + 4;
    }

    // Tell the task in core 1 that the processing can start
    xQueueSend(queue, &item, portMAX_DELAY);
  }
}

void setup()
{
  Serial.begin(115200);
  setCpuFrequencyMhz(240);
  Wire.begin(SDA_0, SCL_0, 800000UL);
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);

  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS))
  {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;)
      ; // Don't proceed, loop forever
  }
  display.display();
  delay(500); // Pause for 2 seconds

  // Clear the buffer
  display.clearDisplay();
  display.setTextWrap(false);
  textx = display.width();
  textminX = -12 * oledtext.length();
  display.display();
  display.setTextSize(1);
  display.setTextColor(1);

  // static const i2s_config_t i2s_config = {
  //     .mode = (i2s_mode_t) (I2S_MODE_MASTER | I2S_MODE_TX | I2S_MODE_DAC_BUILT_IN),
  //     .sample_rate = 44100, // corrected by info from bluetooth
  //     .bits_per_sample = (i2s_bits_per_sample_t) 16, /* the DAC module will only take the 8bits from MSB */
  //     .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
  //     .communication_format = (i2s_comm_format_t)I2S_COMM_FORMAT_STAND_MSB,
  //     .intr_alloc_flags = 0, // default interrupt priority
  //     .dma_buf_count = 8,
  //     .dma_buf_len = 64,
  //     .use_apll = false
  // };
  // static const i2s_config_t i2s_config = {
  //     .mode = (i2s_mode_t) (I2S_MODE_MASTER | I2S_MODE_TX),                                  // Only TX
  //     .sample_rate = 44100,
  //     .bits_per_sample = (i2s_bits_per_sample_t)16,
  //     .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,                           //2-channels
  //     .communication_format = I2S_COMM_FORMAT_I2S_MSB,
  //     .dma_buf_count = 6,
  //     .dma_buf_len = 60,
  //     .intr_alloc_flags = 0,                                                  //Default interrupt priority
  //     .tx_desc_auto_clear = true                                              //Auto clear tx descriptor on underflow
  // };
  static const i2s_config_t i2s_config = {
      .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
      .sample_rate = 44100,                         // corrected by info from bluetooth
      .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT, /* the DAC module will only take the 8bits from MSB */
      .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
      .communication_format = (i2s_comm_format_t)I2S_COMM_FORMAT_STAND_MSB,
      .intr_alloc_flags = 0, // default interrupt priority
      .dma_buf_count = 8,
      .dma_buf_len = 64,
      .use_apll = true,
  };

  i2s_pin_config_t my_pin_config = {
      .bck_io_num = 26,
      .ws_io_num = 25,
      .data_out_num = 17,
      .data_in_num = I2S_PIN_NO_CHANGE};

  queue = xQueueCreate(1, sizeof(int));
  if (queue == NULL)
  {
    Serial.println("Error creating the A2DP->FFT queue");
  }

  // This task will process the data acquired by the Bluetooth audio stream
  xTaskCreatePinnedToCore(renderFFT,      // Function that should be called
                          "FFT Renderer", // Name of the task (for debugging)
                          10000,          // Stack size (bytes)
                          NULL,           // Parameter to pass
                          1,              // Task priority
                          NULL,           // Task handle
                          1               // Core you want to run the task on (0 or 1)
  );
  a2dp_sink.set_i2s_config(i2s_config);
  a2dp_sink.set_pin_config(my_pin_config);
  a2dp_sink.set_auto_reconnect(false);
  a2dp_sink.start("MyMusic");
  a2dp_sink.set_stream_reader(audio_data_callback);
  a2dp_sink.set_avrc_metadata_callback(avrc_metadata_callback);
  a2dp_sink.set_on_connection_state_changed(connection_state_changed);
}

void smoothData()
{
  int WINDOW_SIZE = 5;
  float sum = 0;
  int count = 0;
  for (int i = 0; i < NUM_BANDS - 1; i++)
  {
    sum += peak[i];
    count++;
    if (i < (WINDOW_SIZE - 1) / 2)
    {
      peak[i] = sum / count;
    }
    else if (i >= NUM_BANDS - (WINDOW_SIZE - 1) / 2)
    {
      peak[i] = sum / count;
      sum -= peak[i - (WINDOW_SIZE - 1) / 2];
      count--;
    }
    else if (count == WINDOW_SIZE)
    {
      peak[i - (WINDOW_SIZE - 1) / 2] = sum / WINDOW_SIZE; // 將平均值存儲在中間的數據點中
      sum -= peak[i - (WINDOW_SIZE - 1) / 2];
      count--;
    }
  }
}

void loop()
{
  esp_a2d_audio_state_t state = a2dp_sink.get_audio_state();
  if (currentAudioState != state)
  {
    Serial.print("Audio state changed; new state: %d");
    Serial.println(state);
    currentAudioState = state;
  }
  switch (state)
  {
  // Unclear how stopped and remote suspend really differ from one another. In
  // ESP32-A2DP >= v1.6 we seem to be getting the later when the client stops
  // audio playback.
  case ESP_A2D_AUDIO_STATE_REMOTE_SUSPEND:
  case ESP_A2D_AUDIO_STATE_STOPPED:
    if (bleDeviceConnected)
    {
      if (strofstst != "pause")
      {
        strofstst = "pause";
      }

      // pause
    }
    else
    {
      if (strofstst != "wait for connect")
      {

        strofstst = "wait for connect";
      }

      // playing
    }
    break;
  case ESP_A2D_AUDIO_STATE_STARTED:
    if (strofstst != "playing")
    {
      strofstst = "playing";
    }

    devicePlayedAudio = true;

    // lcd.clear();

    // lcd.setCursor(0, 0);

    // Serial.println(ESP.getMaxAllocHeap() );

    // for (int linei=1;linei>=0;linei--){
    //   lcd.setCursor(0, (1-linei));
    //   for (int iofp=0;iofp<NUM_BANDS;iofp++){
    //     int tmpc = map(peak[iofp],0,amplitude,1,16) - linei*8;

    //     if (tmpc >= 8){
    //       lcd.write(7);
    //     }
    //     else if (tmpc < 1){
    //       lcd.print(" ");
    //     }
    //     else{
    //       lcd.write(tmpc);
    //     }
    //   }
    // }

    // vga.clear(0);

    // vga.show();

    break;
  }
  // vga.line((iofp+5)*5,150,(iofp+5)*5,150-map(peak[iofp],0,amplitude,1,100), vga.RGB(255, 255, 255));
  // vga.line((iofp+5)*5,150-map(peak[iofp],0,amplitude,1,100),(iofp+5)*5,0, vga.RGB(0, 0, 0));
  // vga.dot((iofp+5)*5,150-dotpeak[iofp]-1, vga.RGB(255,0,255));

  // if (iii >= 1){
  //   Serial.printf("%d %d %d\n",a2dp_sink.get_volume(),touchRead(15),touchRead(4)); //15 4

  //   //set_volume

  //   iii = 0;
  // }
  // iii++;
  // Serial.printf("%d %d %d\n",a2dp_sink.get_volume(),touchRead(15),touchRead(4));
  // if (touchRead(15) < 10 | touchRead(4) < 10){
  //   if (millis() - lasttouchbutton >= 100){
  //     if (touchRead(15) < 10){
  //       Serial.println("up");
  //       a2dp_sink.set_volume(a2dp_sink.get_volume()+1);
  //     }
  //     else {
  //       Serial.println("down");
  //       a2dp_sink.set_volume(a2dp_sink.get_volume()-1);
  //     }
  //     lasttouchbutton = millis();
  //   }
  // }
  frameStartTime = millis();
  smoothData();
  for (int iofp = 0; iofp < NUM_BANDS; iofp++)
  {
    int calcpeak = map(peak[iofp], 0, amplitude, 1, 48);

    display.drawLine((iofp)*2, 64, (iofp)*2, 64 - calcpeak, SSD1306_WHITE);
    display.drawLine((iofp)*2, 64 - calcpeak, (iofp)*2, 0, SSD1306_BLACK);

    if (dotpeak[iofp] < calcpeak)
    {
      dotpeak[iofp] = calcpeak;
    }
    else
    {
      if (dotpeak[iofp] >= 0)
      {
        dotpeak[iofp] -= 1;
      }
    }

    display.drawPixel((iofp)*2, 64 - dotpeak[iofp] - 1, SSD1306_WHITE);
  }

  if (strofstst == "wait for connect")
  {
    display.fillRect(0, 0, 128, 16, SSD1306_BLACK);
    display.setCursor(0, 0);
    display.print("Waiting For Connect");
    display.display();
    // Serial.println("p w");
  }
  else
  { //(strofstst == "playing" || strofstst == "pause")
    display.fillRect(0, 0, 128, 16, SSD1306_BLACK);
    display.setCursor(textx, 0);
    display.print(oledtext);
    if (strofstst == "playing")
    {
      display.fillRect(0, 0, 50, 16, SSD1306_BLACK);
      display.setCursor(1, 0);
      display.print("Playing:");
    }
    else if (strofstst == "pause")
    {
      display.fillRect(0, 0, 38, 16, SSD1306_BLACK);
      display.setCursor(1, 0);
      display.print("Pause:");
    }

    if (--textx < textminX)
      textx = display.width();
  }

  if (strofstst != "playing")
  {
    for (int nnfp = 0; nnfp <= NUM_BANDS - 1; nnfp++)
    {
      if (nnfp == pausewaitn % NUM_BANDS)
      {
        peak[nnfp] = map(NUM_BANDS, 1, 48, 0, amplitude);
      }
      else if (nnfp == (pausewaitn + 1) % NUM_BANDS)
      {
        peak[nnfp] = map(30, 1, 48, 0, amplitude);
      }
      else if (nnfp == (pausewaitn + 2) % NUM_BANDS)
      {
        peak[nnfp] = map(NUM_BANDS, 1, 48, 0, amplitude);
      }
      else
      {
        peak[nnfp] = map(1, 1, 48, 0, amplitude);
      }
    }
    pausewaitn += 1;
    if (pausewaitn >= NUM_BANDS)
    {
      pausewaitn = 0;
    }
  }
  frameEndTime = millis();
  frameCount++;

  // 如果已經過了一秒鐘，計算當前幀率並重置計數器
  if (frameEndTime - lastFrameRateTime >= 1000)
  {
    frameRate = frameCount;
    frameCount = 0;
    lastFrameRateTime = frameEndTime;
  }

  // 計算每幀的渲染時間和當前幀率
  int frameRenderTime = frameEndTime - frameStartTime;
  int fps = frameRate;
  // display.fillRect(96, 20, 32, 16, SSD1306_BLACK);
  // display.setCursor(96, 20);

  // display.print(fps);

  display.display();
  // Serial.println(strofstst);

  // print volume
  Serial.printf("%d\n", a2dp_sink.get_volume());
}

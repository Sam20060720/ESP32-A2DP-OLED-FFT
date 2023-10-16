#include <Arduino.h>
#include "BluetoothA2DPSink.h"
#include <arduinoFFT.h>
#include <Wire.h>

#define NUM_BANDS 20
#define SAMPLES 1024
#define SAMPLING_FREQUENCY 44100

#define SDA_0 21
#define SCL_0 22

BluetoothA2DPSink a2dp_sink;
bool devicePlayedAudio = false;
esp_a2d_audio_state_t currentAudioState = ESP_A2D_AUDIO_STATE_REMOTE_SUSPEND;
bool bleDeviceConnected = false;
int iii = 0;
int32_t peak[NUM_BANDS];
int32_t dotpeak[NUM_BANDS] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
int32_t linepeak[NUM_BANDS] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

double vReal[SAMPLES];
double vImag[SAMPLES];
QueueHandle_t queue;
int16_t sample_l_int;
int16_t sample_r_int;
int visualizationCounter = 0;
int32_t lastVisualizationUpdate = 0;
float amplitude = 200.0;

String singer;
String songname;

String strofstst = "";

arduinoFFT FFT = arduinoFFT();

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h> //SSD1306

#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C
Adafruit_SSD1306 display(128, 64, &Wire, OLED_RESET); // pin 21,22
int textx, textminX;
String oledtext = "Playing";

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
  if (i <= 5) //< 180hz
  {
    band = 0; //
  }
  else if (i <= 10) // 180 ~ 300
  {
    band = 1; //
  }

  else if (i <= 15) //
  {
    band = 2; // 500Hz
  }
  else if (i <= 20)
  {
    band = 3; // 250Hz
  }
  else if (i <= 25)
  {
    band = 4; // 500Hz
  }

  else if (i <= 30)
  {           // 8
    band = 5; // 1000Hz
  }
  else if (i <= 35)
  {
    band = 6; // 2000Hz
  }
  else if (i <= 40) //
  {
    band = 7; // 1200Hz
  }
  else if (i <= 45)
  {
    band = 8; // 8000Hz
  }
  else if (i <= 50)
  {           //
    band = 9; // 250Hz
  }
  else if (i <= 55) // peo high
  {
    band = 10; // 500Hz
  }
  else if (i <= 60) // peo high
  {
    band = 11; // 1000Hz
  }
  else if (i <= 65)
  {            // 8 160
    band = 12; // 2000Hz
  }
  else if (i <= 70)
  {
    band = 13; // 4000Hz
  }
  else if (i <= 75)
  {
    band = 14; // 8000Hz //high
  }
  else if (i <= 80)
  {
    band = 15; // 8000Hz
  }
  else if (i <= 85)
  {
    band = 16; // 8000Hz
  }
  else if (i <= 90)
  {
    band = 17; // 8000Hz
  }
  else if (i <= 95)
  {
    band = 18; // 8000Hz
  }
  else
  {
    band = 19; // 8000Hz //265
  }
  //(float)1 - (float)a2dp_sink.get_volume() / (float)100)

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
        if (vReal[i] > 50000)
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
  if (id == ESP_AVRC_MD_ATTR_ARTIST)
  {
    singer = (char *)text;
    // Serial.printf("==> AVRC metadata rsp: attribute id 0x%x, %s\n", id, text);

    oledtext = "Playing : " + singer + " / " + songname;
    textx = display.width();
    textminX = -12 * oledtext.length();
  }
  else if (id == ESP_AVRC_MD_ATTR_TITLE)
  {
    songname = (char *)text;
    oledtext = "Playing : " + singer + " / " + songname;
    textx = display.width();
    textminX = -12 * oledtext.length();
  }
  else
  {
    Serial.printf("%d : %s\n", id, text);
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
    for (int i = 0; i < SAMPLES; i++)
    {
      sample_l_int = (int16_t)(((*(data + byteOffset + 1) << 8) | *(data + byteOffset)));
      sample_r_int = (int16_t)(((*(data + byteOffset + 3) << 8) | *(data + byteOffset + 2)));
      vReal[i] = (sample_l_int + sample_r_int) / 2.0f;
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

  pinMode(23, OUTPUT);
  // digitalWrite(2, HIGH);

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
  a2dp_sink.start("MyMusic");
  a2dp_sink.set_stream_reader(audio_data_callback);
  a2dp_sink.set_avrc_metadata_callback(avrc_metadata_callback);
  a2dp_sink.set_on_connection_state_changed(connection_state_changed);
  a2dp_sink.set_volume(100);

  // vga.setFrameBufferCount(1);
  // vga.init(vga.MODE200x150, pinConfig);
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
      //播放過
      if (devicePlayedAudio)
      {
        if (strofstst != "pause")
        {
          display.setTextSize(1);  // 設定文字大小
          display.setTextColor(1); // 1:OLED預設的顏色(這個會依該OLED的顏色來決定)
          display.setCursor(1, 0); // 設定起始座標
          display.fillRect(0, 0, 128, 16, SSD1306_BLACK);
          display.print("Pause");
          display.display();
          strofstst = "pause";
          digitalWrite(23, LOW);
        }

        // pause
      }
      else
      {
        if (strofstst != "ble")
        {
          display.setTextSize(1);  // 設定文字大小
          display.setTextColor(1); // 1:OLED預設的顏色(這個會依該OLED的顏色來決定)
          display.setCursor(1, 0); // 設定起始座標
          display.fillRect(0, 0, 128, 16, SSD1306_BLACK);
          display.print("Waiting For Playing");
          display.display();
          strofstst = "ble";
          digitalWrite(23, LOW);
        }
        // wait for connect
      }
    }
    else
    {

      if (strofstst != "wait for connect")
      {
        display.setTextSize(1);  // 設定文字大小
        display.setTextColor(1); // 1:OLED預設的顏色(這個會依該OLED的顏色來決定)
        display.setCursor(1, 0); // 設定起始座標
        display.fillRect(0, 0, 128, 16, SSD1306_BLACK);
        display.print("Waiting For Connect");
        display.display();
        strofstst = "wait for connect";
        digitalWrite(23, LOW);
      }

      // playing
    }
    break;
  case ESP_A2D_AUDIO_STATE_STARTED:
    if (strofstst != "playing")
    {
      display.setTextSize(1);  // 設定文字大小
      display.setTextColor(1); // 1:OLED預設的顏色(這個會依該OLED的顏色來決定)
      display.setCursor(1, 0); // 設定起始座標
      display.fillRect(0, 0, 128, 16, SSD1306_BLACK);
      // display.print("Playing");
      display.display();
      strofstst = "playing";
      digitalWrite(23, HIGH);
    }

    devicePlayedAudio = true;

    for (int iofp = 0; iofp < NUM_BANDS; iofp++)
    {

      if (dotpeak[iofp] < map(peak[iofp], 0, amplitude, 1, 48))
      {
        dotpeak[iofp] = map(peak[iofp], 0, amplitude, 1, 48);
      }
      else
      {
        if (dotpeak[iofp] >= 0)
        {
          dotpeak[iofp] -= 1;
        }
      }
      if (linepeak[iofp] < map(peak[iofp], 0, amplitude, 1, 48))
      {
        linepeak[iofp] = map(peak[iofp], 0, amplitude, 1, 48);
      }
      else
      {
        if (linepeak[iofp] >= 0)
        {
          linepeak[iofp] -= 3;
        }
      }
      //下落效果

      display.drawLine((iofp)*5, 64, (iofp)*5, 64 - linepeak[iofp], SSD1306_WHITE);
      display.drawLine((iofp)*5, 64 - linepeak[iofp], (iofp)*5, 0, SSD1306_BLACK);

      display.drawPixel((iofp)*5, 64 - dotpeak[iofp] - 1, SSD1306_WHITE);
    }

    display.fillRect(0, 0, 128, 16, SSD1306_BLACK);
    display.setCursor(textx, 0);
    display.print(oledtext);
    display.display();

    if (--textx < textminX)
      textx = display.width();

    display.display();

    break;
  }

  // if (iii >= 1){
  //   Serial.printf("%d %d %d\n",a2dp_sink.get_volume(),touchRead(15),touchRead(4)); //15 4

  //   //set_volume

  //   iii = 0;
  // }
  // iii++;

  // 0 ~ 120 //100
  // Serial.printf("%d %f\n", a2dp_sink.get_volume(), (float)1 - ((float)1 - (float)a2dp_sink.get_volume() / (float)10000));
  // if (touchRead(15) < 10 | touchRead(4) < 10)
  // {
  //   if (millis() - lasttouchbutton >= 100)
  //   {
  //     if (touchRead(15) < 10)
  //     {
  //       Serial.println("up");
  //       a2dp_sink.set_volume(a2dp_sink.get_volume() + 1);
  //     }
  //     else
  //     {
  //       Serial.println("down");
  //       a2dp_sink.set_volume(a2dp_sink.get_volume() - 1);
  //     }
  //     lasttouchbutton = millis();
  //   }
  // }
}
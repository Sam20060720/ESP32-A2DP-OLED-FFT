#include "BluetoothA2DPSink.h"
#include <arduinoFFT.h>
#include <Wire.h>
#include <Arduino.h>
#define NUM_BANDS 20
#define SAMPLES 2048
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
int32_t dotpeak[NUM_BANDS] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
double vReal[SAMPLES];
double vImag[SAMPLES];
QueueHandle_t queue;
int16_t sample_l_int;
int16_t sample_r_int;
int visualizationCounter = 0;
int32_t lastVisualizationUpdate = 0;
float amplitude = 500.0;

int pausewaitn = 0;

String singer;
String songname;

String strofstst = "";

arduinoFFT FFT = arduinoFFT();

#include <LiquidCrystal_I2C.h>
int lcdColumns = 20;
int lcdRows = 4;
LiquidCrystal_I2C lcd(0x27, lcdColumns, lcdRows);
byte customChar1[] = {
    B00000,
    B00000,
    B00000,
    B00000,
    B00000,
    B00000,
    B00000,
    B11111};
byte customChar2[] = {
    B00000,
    B00000,
    B00000,
    B00000,
    B00000,
    B00000,
    B11111,
    B11111};
byte customChar3[] = {
    B00000,
    B00000,
    B00000,
    B00000,
    B00000,
    B11111,
    B11111,
    B11111};
byte customChar4[] = {
    B00000,
    B00000,
    B00000,
    B00000,
    B11111,
    B11111,
    B11111,
    B11111};
byte customChar5[] = {
    B00000,
    B00000,
    B00000,
    B11111,
    B11111,
    B11111,
    B11111,
    B11111};
byte customChar6[] = {
    B00000,
    B00000,
    B11111,
    B11111,
    B11111,
    B11111,
    B11111,
    B11111};
byte customChar7[] = {
    B00000,
    B11111,
    B11111,
    B11111,
    B11111,
    B11111,
    B11111,
    B11111};
byte customChar8[] = {
    B11111,
    B11111,
    B11111,
    B11111,
    B11111,
    B11111,
    B11111,
    B11111};

// #include <ESP32Lib.h>
// // #include <Ressources/Font6x8.h>
// //VGA Device
// VGA3Bit vga;
// //Pin presets are avaialable for: VGAv01, VGABlackEdition, VGAWhiteEdition, PicoVGA
// const PinConfig &pinConfig = VGA3Bit::VGABlackEdition;

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define OLED_RESET -1       // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(128, 64, &Wire, OLED_RESET);
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
  if (i <= 6)
  {
    band = 0; // 125Hz
  }
  else if (i <= 12)
  {
    band = 1; // 250Hz
  }

  else if (i <= 19)
  {
    band = 2; // 500Hz
  }
  else if (i <= 25)
  {
    band = 3; // 250Hz
  }
  else if (i <= 32)
  {
    band = 4; // 500Hz
  }

  else if (i <= 48)
  {           // 8
    band = 5; // 1000Hz
  }
  else if (i <= 56)
  {
    band = 6; // 2000Hz
  }
  else if (i <= 64)
  {
    band = 7; // 4000Hz
  }
  else if (i <= 72)
  {
    band = 8; // 8000Hz
  }
  else if (i <= 80)
  {           //
    band = 9; // 250Hz
  }
  else if (i <= 88)
  {
    band = 10; // 500Hz
  }
  else if (i <= 96)
  {
    band = 11; // 1000Hz
  }
  else if (i <= 112)
  {            // 8 160
    band = 12; // 2000Hz
  }
  else if (i <= 128)
  {
    band = 13; // 4000Hz
  }
  else if (i <= 153)
  {
    band = 14; // 8000Hz
  }
  else if (i <= 178)
  {
    band = 15; // 8000Hz
  }
  else if (i <= 203)
  {
    band = 16; // 8000Hz
  }
  else if (i <= 218)
  {
    band = 17; // 8000Hz
  }
  else if (i <= 233)
  {
    band = 18; // 8000Hz
  }
  else
  {
    band = 19; // 8000Hz //265
  }
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
  int ischangeinf = 0;
  if (id == ESP_AVRC_MD_ATTR_ARTIST)
  {
    singer = (char *)text;
    ischangeinf = 1;
    // Serial.printf("==> AVRC metadata rsp: attribute id 0x%x, %s\n", id, text);

    // lcd.setCursor(0, 0);
    // lcd.print("                   ");
    // lcd.setCursor(0, 0);
    // lcd.print(singer);
    // oledtext = singer + " / " + songname;
    // textx = display.width();
    // textminX = -12 * oledtext.length();
  }
  else if (id == ESP_AVRC_MD_ATTR_TITLE)
  {
    songname = (char *)text;
    ischangeinf = 1;
    // lcd.setCursor(0, 1);
    // lcd.print("                   ");
    // lcd.setCursor(0, 1);
    // lcd.print(songname);
    // oledtext = singer + " / " + songname;
    // textx = display.width();
    // textminX = -12 * oledtext.length();
  }
  // else {
  //   Serial.printf("%d : %s\n", id, text);
  // }

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

  // lcd.setCursor(0, 3);
  // lcd.print("Playing");
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
  Wire.begin(SDA_0, SCL_0, 150000UL);
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

  // lcd.init();
  // // turn on LCD backlight
  // lcd.backlight();
  // lcd.createChar(0, customChar1);
  // lcd.createChar(1, customChar2);
  // lcd.createChar(2, customChar3);
  // lcd.createChar(3, customChar4);
  // lcd.createChar(4, customChar5);
  // lcd.createChar(5, customChar6);
  // lcd.createChar(6, customChar7);
  // lcd.createChar(7, customChar8);
  // lcd.clear();
  // lcd.home();
  // lcd.write(0);
  // lcd.write(1);
  // lcd.write(2);
  // lcd.write(3);
  // lcd.write(4);
  // lcd.write(5);
  // lcd.write(6);
  // lcd.write(7);

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
      if (strofstst != "pause")
      {
        // lcd.clear();
        // lcd.setCursor(0, 0);
        // lcd.print(singer);
        // lcd.setCursor(0, 1);
        // lcd.print(songname);
        // lcd.setCursor(0, 3);
        // lcd.print("PAUSE");

        // display.setTextSize(1);   // 設定文字大小
        // display.setTextColor(1);  // 1:OLED預設的顏色(這個會依該OLED的顏色來決定)
        // display.setCursor(1, 0);  // 設定起始座標
        // display.fillRect(0, 0, 128, 16, SSD1306_BLACK);
        // display.print("Pause");
        // display.display();
        strofstst = "pause";
      }

      // pause
    }
    else
    {
      if (strofstst != "wait for connect")
      {
        // lcd.setCursor(0, 3);
        // lcd.clear();
        // lcd.print("Waiting For Connect");
        // display.setTextSize(1);   // 設定文字大小
        // display.setTextColor(1);  // 1:OLED預設的顏色(這個會依該OLED的顏色來決定)
        // display.setCursor(1, 0);  // 設定起始座標
        // display.fillRect(0, 0, 128, 16, SSD1306_BLACK);
        // display.print("Waiting For Connect");
        // display.display();

        strofstst = "wait for connect";
      }

      // playing
    }
    break;
  case ESP_A2D_AUDIO_STATE_STARTED:
    if (strofstst != "playing")
    {
      // lcd.clear();
      // lcd.setCursor(0, 0);
      // lcd.print(singer);
      // lcd.setCursor(0, 1);
      // lcd.print(songname);
      // lcd.setCursor(0, 3);
      // lcd.print("Playing");
      // display.setTextSize(1);   // 設定文字大小
      // display.setTextColor(1);  // 1:OLED預設的顏色(這個會依該OLED的顏色來決定)
      // display.setCursor(1, 0);  // 設定起始座標
      // display.fillRect(0, 0, 128, 16, SSD1306_BLACK);
      // // display.print("Playing");
      // display.display();
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
  for (int iofp = 0; iofp < NUM_BANDS; iofp++)
  {
    display.drawLine((iofp)*5, 64, (iofp)*5, 64 - map(peak[iofp], 0, amplitude, 1, 48), SSD1306_WHITE);
    display.drawLine((iofp)*5, 64 - map(peak[iofp], 0, amplitude, 1, 48), (iofp)*5, 0, SSD1306_BLACK);

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
    display.drawPixel((iofp)*5, 64 - dotpeak[iofp] - 1, SSD1306_WHITE);
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
        peak[nnfp] = map(20, 1, 48, 0, amplitude);
      }
      else if (nnfp == (pausewaitn + 1) % NUM_BANDS)
      {
        peak[nnfp] = map(30, 1, 48, 0, amplitude);
      }
      else if (nnfp == (pausewaitn + 2) % NUM_BANDS)
      {
        peak[nnfp] = map(20, 1, 48, 0, amplitude);
      }
      else
      {
        peak[nnfp] = map(1, 1, 48, 0, amplitude);
      }
    }
    pausewaitn += 1;
    if (pausewaitn >= 20)
    {
      pausewaitn = 0;
    }
  }
  display.display();
  Serial.println(strofstst);
}
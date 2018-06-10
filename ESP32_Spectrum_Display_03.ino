/* ESP8266/32 Audio Spectrum Analyser on an SSD1306/SH1106 Display
 * The MIT License (MIT) Copyright (c) 2017 by David Bird. 
 * The formulation and display of an AUdio Spectrum using an ESp8266 or ESP32 and SSD1306 or SH1106 OLED Display using a Fast Fourier Transform
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files 
 * (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, 
 * publish, distribute, but not to use it commercially for profit making or to sub-license and/or to sell copies of the Software or to 
 * permit persons to whom the Software is furnished to do so, subject to the following conditions:  
 * The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software. 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES 
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE 
 * LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN 
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE. 
 * See more at http://dsbird.org.uk 
*/

#include <Wire.h>
#include <WiFi.h> // needed to disable the WiFi
#include "arduinoFFT.h" // Standard Arduino FFT library: in IDE, Manage Library, then search for FFT 
arduinoFFT FFT = arduinoFFT(); // or manually install from https://github.com/kosme/arduinoFFT

#include <driver/adc.h> // needed to get to adc1_get_raw()


// comment this out to use regular ILI9341
#define WROVER_KIT

#ifdef WROVER_KIT
  #define TFT_DC 21
  #define TFT_CS 0
  #define TFT_RST 18
  #define SPI_MISO 25
  #define SPI_MOSI 23
  #define SPI_CLK 19
  #define LCD_BL_CTR 5
  #include "WROVER_KIT_LCD.h" // https://github.com/espressif/WROVER_KIT_LCD
  #include <Adafruit_GFX.h>
  // Some code uses the Adafruit_ILI9341 interface for tft, fix this here
  #define Adafruit_ILI9341 WROVER_KIT_LCD
  #define min(X, Y) (((X) < (Y)) ? (X) : (Y))
  #define max(X, Y) (((X) > (Y)) ? (X) : (Y))
  Adafruit_ILI9341 tft;
#else
  #define TFT_RST 18
  #define SPI_MISO 25
  #define SPI_MOSI 23
  #define SPI_CLK 19
  #define LCD_BL_CTR 5
  #define TFT_CS 10
  #define TFT_DC 9
  #include <Adafruit_ILI9341.h>
  #include <Adafruit_GFX.h>
  Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC);
#endif

/////////////////////////////////////////////////////////////////////////
int SAMPLES = 256; // Must be a power of 2
#define SAMPLING_FREQUENCY 40000 // Hz, must be 40000 or less due to ADC conversion time. Determines maximum frequency that can be analysed by the FFT Fmax=sampleF/2.

struct eqBand {
  const char *freqname;
  uint16_t amplitude;
  byte bandWidth;
  int peak;
  int lastpeak;
  uint16_t curval;
  uint16_t lastval;
  unsigned long lastmeasured;
};

eqBand audiospectrum[8] = {
  /* 
   * Adjust the amplitude/bandWidth values 
   * to fit your microphone
   */
  { "125Hz", 500, 2,   0, 0, 0, 0, 0},
  { "250Hz", 500, 2,   0, 0, 0, 0, 0},
  { "500Hz", 200, 3,   0, 0, 0, 0, 0},
  { "1KHz",  200, 7,   0, 0, 0, 0, 0},
  { "2KHz",  200, 14,  0, 0, 0, 0, 0},
  { "4KHz",  100, 24,  0, 0, 0, 0, 0},
  { "8KHz",  50,  48,  0, 0, 0, 0, 0},
  { "16KHz", 50,  155, 0, 0, 0, 0, 0}
};

/* store bandwidth variations when sample rate changes */
int bandWidth[8] = {
  0,0,0,0,0,0,0,0
};


/* waveform cache */
int16_t eQGraph[320];
float wmultiplier = 320 / SAMPLES;

unsigned int sampling_period_us;
unsigned long microseconds;

double vReal[4096];
double vImag[4096];
unsigned long newTime, oldTime;
/////////////////////////////////////////////////////////////////////////

uint16_t tft_width = 320; // ILI9341_TFTWIDTH;
uint16_t tft_height = 240; // ILI9341_TFTHEIGHT;

uint8_t bands = 8;
uint8_t bands_width = floor( tft_width / bands );
uint8_t bands_pad = bands_width - 10;
uint16_t colormap[255]; // color palette for the band meter (pre-fill in setup)


/* booleans to manage the display state */
bool displayvolume = true;
bool displaywaveform = true;
bool displayspectrometer = true;





void drawAudioSpectrumGrid() {
  tft.setTextColor(ILI9341_YELLOW);
  for(uint8_t i=0;i<tft_height;i++) {
    colormap[i] = tft.color565(tft_height-i*.5, i*1.1, 0);
  }
  for (byte band = 0; band <= 7; band++) {
    tft.setCursor(bands_width*band + 2, 0);
    tft.print(audiospectrum[band].freqname);
  }
  
}


void displayBand(int band, int dsize){
  uint16_t hpos = bands_width*band;
  int dmax = 200;
  if(dsize>tft_height-10) {
    dsize = tft_height-10; // leave some hspace for text
  }
  if(dsize < audiospectrum[band].lastval) {
    // lower value, delete some lines
    tft.fillRect(hpos, tft_height-audiospectrum[band].lastval, bands_pad, audiospectrum[band].lastval - dsize, ILI9341_BLACK);
  }
  if (dsize > dmax) dsize = dmax;
  for (int s = 0; s <= dsize; s=s+4){
    tft.drawFastHLine(hpos, tft_height-s, bands_pad, colormap[tft_height-s]);
  }
  if (dsize > audiospectrum[band].peak) {
    audiospectrum[band].peak = dsize;
  }
  audiospectrum[band].lastval = dsize;
  audiospectrum[band].lastmeasured = millis();
}


void setBandwidth() {
  byte multiplier = SAMPLES / 256;
  bandWidth[0] = audiospectrum[0].bandWidth * multiplier;
  for(byte j=1;j<8;j++) {
     bandWidth[j] = audiospectrum[j].bandWidth * multiplier + bandWidth[j-1];
  }
  wmultiplier = ((float)320 / (float)SAMPLES)*2;
}


byte getBand(int i) {
  for(byte j=0;j<8;j++) {
    if(i<=bandWidth[j]) return j;
  }
  return 8;
}


void peakWaveForm() {
  for(uint16_t i=0;i<SAMPLES/2;i++) {
    if(eQGraph[i]!=0) {
      eQGraph[i]/=(2+getBand(i));
    }
  }
}


void displayWaveForm(uint16_t color) {
  uint16_t lastx = 1;
  uint16_t lasty = 120;
  for(uint16_t i=1;i<SAMPLES/2;i++) {
    if(eQGraph[i]!=0) {
      uint tmpy;
      uint nextx = i*wmultiplier;
      if(i%2==1) {
        tmpy = 120+eQGraph[i]/4;
      } else {
        tmpy = 120-eQGraph[i]/4;
      }
      if(lasty!=0) {
        tft.drawLine(lastx, lasty, nextx, tmpy, color);
      }
      lastx = nextx;
      lasty = tmpy;
    }
  }
  tft.drawLine(lastx, lasty, 320, 120, color);
}


void handleSerial() {
  if(Serial.available()) {
    // toggle display modes
    char c = Serial.read();
    if(displaywaveform) {
      displayWaveForm(ILI9341_BLACK);
      memset(eQGraph, 0, 320);
    }
    switch(c) {
      case 'v':
        displayvolume = !displayvolume;
        Serial.println("Volume " + String(displayvolume));
        tft.fillScreen(ILI9341_BLACK);
      break;
      case 'w':
        displaywaveform = !displaywaveform;
        Serial.println("Waveform " + String(displaywaveform));
        tft.fillScreen(ILI9341_BLACK);
      break;
      case 's':
        displayspectrometer = !displayspectrometer;
        Serial.println("Spectro " + String(displayspectrometer));
        tft.fillScreen(ILI9341_BLACK);
      break;
      case '+':
        SAMPLES*=2;
        Serial.println("Sampling buffer " + String(SAMPLES));
      break;
      case '-':
        if(SAMPLES/2>32) {
          SAMPLES/=2;
        }
        Serial.println("Sampling buffer " + String(SAMPLES));
      break;
      default:
        //Serial.println("wut?");
      break;
    }
    if(displayspectrometer) {
      drawAudioSpectrumGrid(); 
    }
    setBandwidth();
  }
}


void setup() {
  WiFi.mode(WIFI_MODE_NULL);
  Serial.begin(115200);
  
  adc1_config_width(ADC_WIDTH_12Bit);   //Range 0-1023 
  adc1_config_channel_atten(ADC1_CHANNEL_0,ADC_ATTEN_11db);  //ADC_ATTEN_DB_11 = 0-3,6V
  
  tft.begin();
  tft.setRotation( 3 );
  tft.fillScreen(ILI9341_BLACK);
  tft.setTextColor(ILI9341_YELLOW);
  tft.setCursor(98, 42);
  tft.print("loading");
  
  sampling_period_us = round(1000000 * (1.0 / SAMPLING_FREQUENCY));
  
  delay(1000);
  
  tft.fillScreen(ILI9341_BLACK);
  
  drawAudioSpectrumGrid();
  setBandwidth();
  memset(eQGraph, 0, 320);
}


void loop() {

  handleSerial();

  long signalAvg = 0, signalMax = 0, signalMin = 4096;
  
  for (int i = 0; i < SAMPLES; i++) {
    newTime = micros();
    vReal[i] = adc1_get_raw( ADC1_CHANNEL_0 ); //analogRead(A0); // A conversion takes about 1uS on an ESP32
    //vReal[i] = analogRead(A0); // A conversion takes about 1uS on an ESP32
    delayMicroseconds(20);
    vImag[i] = 0;
    if(displayvolume) {
      signalMin = min(signalMin, vReal[i]);
      signalMax = max(signalMax, vReal[i]);
      signalAvg += vReal[i];
    }
    while ((micros() - newTime) < sampling_period_us) { 
      // do nothing to wait
    }
  }
  
  if(displayvolume) {
    signalAvg /= SAMPLES;
    uint16_t vol = (signalMax - signalMin);
    uint16_t volWidth = map(vol, 0, 4096, 1, tft_width);
    tft.drawFastHLine(0, 20, volWidth, ILI9341_GREEN);
    tft.drawFastHLine(volWidth, 20, tft_width-volWidth, ILI9341_BLACK);
  }
  
  FFT.Windowing(vReal, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.Compute(vReal, vImag, SAMPLES, FFT_FORWARD);
  FFT.ComplexToMagnitude(vReal, vImag, SAMPLES);

  if(displaywaveform) {
    displayWaveForm(ILI9341_BLACK);
    peakWaveForm();
  }

  for (int i = 2; i < (SAMPLES/2); i++){ // Don't use sample 0 and only first SAMPLES/2 are usable. Each array eleement represents a frequency and its value the amplitude.
    if (vReal[i] > 1500) { // Add a crude noise filter, 10 x amplitude or more
      byte bandNum = getBand(i);
      if(bandNum!=8) {
        audiospectrum[bandNum].curval = (int)vReal[i]/audiospectrum[bandNum].amplitude;
        if(displayspectrometer) {
          displayBand(bandNum, audiospectrum[bandNum].curval);
        }
        if(displaywaveform && i<=160) {
          eQGraph[i] = audiospectrum[bandNum].curval;
        }
      }
    }
  }

  if(displaywaveform) {
    displayWaveForm(ILI9341_GREEN);
  }
  
  if(displayspectrometer) {
    long vnow = millis();
    bool peakchanged = false;
    for (byte band = 0; band <= 7; band++) {
      // auto decay every 50ms on low activity bands
      if(vnow - audiospectrum[band].lastmeasured > 50) {
        displayBand(band, audiospectrum[band].lastval>4 ? audiospectrum[band].lastval-4 : 0);
      }
      if (audiospectrum[band].peak > 0) {
        audiospectrum[band].peak -= (band/3)+2;
        if(audiospectrum[band].peak<=0) {
          audiospectrum[band].peak = 0;
        }
      }
      // only redraw peak if changed
      if(audiospectrum[band].lastpeak != audiospectrum[band].peak) {
        peakchanged = true;
        // delete last peak
        tft.drawFastHLine(bands_width*band, tft_height-audiospectrum[band].lastpeak, bands_pad, ILI9341_BLACK);
        audiospectrum[band].lastpeak = audiospectrum[band].peak;
        tft.drawFastHLine(bands_width*band, tft_height-audiospectrum[band].peak, bands_pad, colormap[tft_height-audiospectrum[band].peak]);
      }
    }
  }
  
}

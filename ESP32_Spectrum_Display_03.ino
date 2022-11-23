/* ESP8266/32 Audio Spectrum Analyser on an SSD1306/SH1106 Display
   The MIT License (MIT) Copyright (c) 2017 by David Bird.
   The formulation and display of an AUdio Spectrum using an ESp8266 or ESP32 and SSD1306 or SH1106 OLED Display using a Fast Fourier Transform
   Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files
   (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge,
   publish, distribute, but not to use it commercially for profit making or to sub-license and/or to sell copies of the Software or to
   permit persons to whom the Software is furnished to do so, subject to the following conditions:
   The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
   OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
   LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
   CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
   See more at http://dsbird.org.uk
*/

#include <driver/adc.h> // needed to get to adc1_get_raw()
#include <Wire.h>
#include <WiFi.h> // needed to disable the WiFi
#include "arduinoFFT.h" // Standard Arduino FFT library: in IDE, Manage Library, then search for FFT 
arduinoFFT FFT = arduinoFFT(); // or manually install from https://github.com/kosme/arduinoFFT


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

#ifdef ILI9341_TFTWIDTH
#define TFT_WIDTH ILI9341_TFTWIDTH
#else
#define TFT_WIDTH 320
#endif
#ifdef ILI9341_TFTHEIGHT
#define TFT_HEIGHT ILI9341_TFTHEIGHT
#else
#define TFT_HEIGHT 240
#endif

#define WAVEFORM_YPOS (TFT_HEIGHT/2) - 50
#define WAVEFORM_SQUEEZE 5 // e.g. 5 => 1/5th of the screen


int SAMPLES = 512; // Must be a power of 2
#define SAMPLING_FREQUENCY 40000 // Hz, must be 40000 or less due to ADC conversion time. Determines maximum frequency that can be analysed by the FFT Fmax=sampleF/2.
#define EQBANDS 8

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

eqBand audiospectrum[EQBANDS] = {
  /*
     Adjust the amplitude/bandWidth values
     to fit your microphone
  */
  { "125Hz", 1000, 2,   0, 0, 0, 0, 0},
  { "250Hz", 500,  2,   0, 0, 0, 0, 0},
  { "500Hz", 300,  3,   0, 0, 0, 0, 0},
  { "1KHz",  250,  7,   0, 0, 0, 0, 0},
  { "2KHz",  200,  14,  0, 0, 0, 0, 0},
  { "4KHz",  100,  24,  0, 0, 0, 0, 0},
  { "8KHz",  50,   48,  0, 0, 0, 0, 0},
  { "16KHz", 25,   155, 0, 0, 0, 0, 0}
};

/* store bandwidth variations when sample rate changes */
int bandWidth[EQBANDS] = {
  0, 0, 0, 0, 0, 0, 0, 0
};

// sample helpers, buffer and data
unsigned int sampling_period_us;
unsigned long microseconds;
double vReal[1024];
double vImag[1024];
unsigned long newTime;
bool adcread = false; // use adc raw or analogread

// waveform cache and settings
float eQGraph[TFT_WIDTH];
float wmultiplier = TFT_WIDTH / SAMPLES;
bool wafeformdirtoggler = false;

// EQ Bands settings
uint16_t bands = EQBANDS;
uint16_t bands_width = floor( TFT_WIDTH / bands );
uint16_t bands_pad = bands_width - (TFT_WIDTH / 16);
uint16_t colormap[255]; // color palette for the band meter (pre-fill in setup)
uint16_t bands_line_vspacing = 4;
uint16_t bandslabelheight = 10;
uint16_t bandslabelpos = TFT_HEIGHT - bandslabelheight;
uint16_t bandslabelmargin = bandslabelheight * 2;
float bands_vratio = 2; // all collected values will be divided by this, this affects the height of the bands
uint16_t audospectrumheight = TFT_HEIGHT / bands_vratio;
uint16_t asvstart = TFT_HEIGHT - bandslabelmargin;
uint16_t asvend = asvstart - audospectrumheight;

// booleans to manage the display state
bool displayvolume = true;
bool displaywaveform = true;
bool displayspectrometer = true;

// volume level
long signalAvg = 0, signalMax = 0, signalMin = 4096;
long max1 = 0, max2 = 0, then, now, nowthen;
bool isSaturating = false;
bool wasSaturating = false;
uint16_t vol = 0; // stores the volume value (0-4096)
uint16_t volWidth = 0; // stores the display-translated volume bar width
float volMod = 1; // fps maintainer: multiplier for waveform, more volume = smaller lines
float lastVolMod = 1;


void drawAudioSpectrumGrid() {
  tft.setTextColor(ILI9341_YELLOW);

  audospectrumheight = TFT_HEIGHT / bands_vratio;
  asvstart = TFT_HEIGHT - bandslabelmargin;
  asvend = asvstart - audospectrumheight - bandslabelmargin;

  Serial.println( "Audio Spectrum Height(px): " + String(audospectrumheight) + " Start at:" + String(asvstart) + " End at:" + String(asvend));

  /*
    for(uint8_t i=0;i<TFT_HEIGHT;i++) {
      // debug: prefill with blue
      colormap[i] = tft.color565(0, 0, 255);
    }
  */
  for (uint16_t i = asvstart; i >= asvend; i--) {
    uint16_t projected = map(i, asvend - 1, asvstart + 1, 0, 127);
    //Serial.println("pixel[" + String(i) + "] + map(" + String(projected) + ") = rgb(" + String(128 + projected) + ", " + String(255 - projected) + ", 0)");
    colormap[i] = tft.color565(255 - projected / 2, 128 + projected, 0);
  }

  for (byte band = 0; band < bands; band++) {
    tft.setCursor(bands_width * band + 2, bandslabelpos);
    tft.print(audiospectrum[band].freqname);
  }
}






void displayBand(int band, int dsize) {
  uint16_t hpos = bands_width * band + (bands_pad / 2);
  int dmax = (TFT_HEIGHT / bands_vratio) - 20; // roof value
  dsize /= bands_vratio;
  if (dsize > audospectrumheight) {
    dsize = audospectrumheight; // leave some hspace for text
  }
  if (dsize < audiospectrum[band].lastval) {
    // lower value, delete some lines
    uint8_t bardelta = dsize % bands_line_vspacing;
    for (int s = dsize - bardelta; s <= audiospectrum[band].lastval; s = s + bands_line_vspacing) {
      tft.drawFastHLine(hpos, TFT_HEIGHT - (s + 20), bands_pad, ILI9341_BLACK);
    }
  }
  if (dsize > dmax) dsize = dmax;
  for (int s = 0; s <= dsize; s = s + bands_line_vspacing) {
    uint8_t vpos = TFT_HEIGHT - (s + 20);
    tft.drawFastHLine(hpos, vpos, bands_pad, colormap[vpos]);
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
  for (byte j = 1; j < bands; j++) {
    bandWidth[j] = audiospectrum[j].bandWidth * multiplier + bandWidth[j - 1];
  }
  wmultiplier = ((float)TFT_WIDTH / (float)SAMPLES) * 2;
}


byte getBand(int i) {
  for (byte j = 0; j < bands; j++) {
    if (i <= bandWidth[j]) return j;
  }
  return bands;
}


void peakWaveForm() {
  for (uint16_t i = 0; i < SAMPLES / 2; i++) {
    if (eQGraph[i] >= 0.00005) {
      eQGraph[i] /= 2; //(2+getBand(i));
    }
  }
}



void displayWaveForm(uint16_t color) {
  uint16_t lastx = 1;
  uint16_t lasty = WAVEFORM_YPOS;
  float wSqueeze = WAVEFORM_SQUEEZE;
  uint8_t maxWaveFormHeight = WAVEFORM_YPOS;

  if (color == ILI9341_BLACK) {
    //isSaturating = wasSaturating;
    //volMod = lastVolMod;
  } else {
    wasSaturating = isSaturating;
    lastVolMod = volMod;
    wafeformdirtoggler = !wafeformdirtoggler;
    uint red = vol / 16;
    color = tft.color565(red, 255-red, 0);
  }

  float toLog = 1.5-lastVolMod*1.5;
  if(toLog!=0.00) {
    // https://www.google.com/search?q=y%3D(-log(1.5-x*1.5))*8
    float volSqueezer = (-log(toLog))*8;
    if(volSqueezer > 0.00) {
      wSqueeze += volSqueezer/*+WAVEFORM_SQUEEZE*/;
    } else {
      wSqueeze /= -volSqueezer;
    }
  }

  byte wafeformdirection = wafeformdirtoggler ? 1 : 0;
  for (uint16_t i = 1; i < SAMPLES / 2; i++) {

    if (eQGraph[i] >= 0.00005) {
      uint tmpy;
      uint nextx = i * wmultiplier;
      uint eQGraphPos = eQGraph[i] / wSqueeze;

      if (eQGraphPos > maxWaveFormHeight) { // roof values
        eQGraphPos = maxWaveFormHeight;
      }

      if (i % 2 == wafeformdirection) {
        tmpy = WAVEFORM_YPOS + eQGraphPos;
      } else {
        tmpy = WAVEFORM_YPOS - eQGraphPos;
      }

      if (lasty != 0) {
        //color = color==ILI9341_BLACK ? color : colormap[(byte)TFT_HEIGHT-(eQGraph[i])];
        tft.drawLine(lastx, lasty, nextx, tmpy, color);
      }
      lastx = nextx;
      lasty = tmpy;
    }
  }
  tft.drawLine(lastx, lasty, lastx + 1, WAVEFORM_YPOS, color);
  if (lastx < TFT_WIDTH && color != ILI9341_BLACK) {
    tft.drawLine(lastx + 1, WAVEFORM_YPOS, TFT_WIDTH, WAVEFORM_YPOS, color);
  }
}


void captureSoundSample() {
  signalAvg = 0;
  signalMax = 0;
  signalMin = 4096;

  for (int i = 0; i < SAMPLES; i++) {
    newTime = micros();
    if ( adcread ) {
      vReal[i] = adc1_get_raw( ADC1_CHANNEL_0 ); // A raw conversion takes about 20uS on an ESP32
      delayMicroseconds(20);
    } else {
      vReal[i] = analogRead(A0); // A conversion takes about 1uS on an ESP32
    }

    vImag[i] = 0;
    if (displayvolume) {
      signalMin = min(signalMin, vReal[i]);
      signalMax = max(signalMax, vReal[i]);
      signalAvg += vReal[i];
    }

    while ((micros() - newTime) < sampling_period_us) {
      // do nothing to wait
      yield();
    }
  }

  FFT.Windowing(vReal, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.Compute(vReal, vImag, SAMPLES, FFT_FORWARD);
  FFT.ComplexToMagnitude(vReal, vImag, SAMPLES);
}





void renderSpectrometer() {
  if (displayvolume) {
    signalAvg /= SAMPLES;
    vol = (signalMax - signalMin);
    volWidth = map(vol, 0, 4096, 1, TFT_WIDTH);
    volMod = (float) map(vol, 0, 4096, 1, 1000) / 1000;
    tft.drawFastHLine(0, 20, volWidth, ILI9341_GREEN);
    tft.drawFastHLine(volWidth, 20, TFT_WIDTH - volWidth, ILI9341_BLACK);
    if (volMod >= .25) isSaturating = true;
    else isSaturating = false;
  }

  if (displaywaveform) {
    displayWaveForm(ILI9341_BLACK);
    peakWaveForm();
  }

  for (int i = 2; i < (SAMPLES / 2); i++) { // Don't use sample 0 and only first SAMPLES/2 are usable. Each array element represents a frequency and its value the amplitude.
    if (vReal[i] > 512) { // Add a crude noise filter, 10 x amplitude or more
      byte bandNum = getBand(i);
      if (bandNum != bands) {
        audiospectrum[bandNum].curval = (int)vReal[i] / audiospectrum[bandNum].amplitude;
        if (displayspectrometer) {
          displayBand(bandNum, audiospectrum[bandNum].curval);
        }
        if (displaywaveform) {
          eQGraph[i] += audiospectrum[bandNum].curval;
        }
      }
    }
  }

  if (displaywaveform) {
    displayWaveForm(ILI9341_GREEN);
  }

  if (displayspectrometer) {
    long vnow = millis();
    bool peakchanged = false;
    for (byte band = 0; band < bands; band++) {
      // auto decay every 50ms on low activity bands
      if (vnow - audiospectrum[band].lastmeasured > 50) {
        displayBand(band, audiospectrum[band].lastval > bands_line_vspacing ? audiospectrum[band].lastval - bands_line_vspacing : 0);
      }
      if (audiospectrum[band].peak > 0) {
        audiospectrum[band].peak -= bands_line_vspacing;//(band/3)+2;
        if (audiospectrum[band].peak <= 0) {
          audiospectrum[band].peak = 0;
        }
      }
      // only redraw peak if changed
      if (audiospectrum[band].lastpeak != audiospectrum[band].peak) {
        peakchanged = true;
        // delete last peak
        tft.drawFastHLine(bands_width * band + (bands_pad / 2), TFT_HEIGHT - audiospectrum[band].lastpeak - 20, bands_pad, ILI9341_BLACK);
        audiospectrum[band].lastpeak = audiospectrum[band].peak;
        tft.drawFastHLine(bands_width * band + (bands_pad / 2), TFT_HEIGHT - audiospectrum[band].peak - 20, bands_pad, colormap[TFT_HEIGHT - audiospectrum[band].peak - 20]);
      }
    }
  }
}


void handleSerial() {
  if (Serial.available()) {
    // toggle display modes
    char c = Serial.read();
    if (displaywaveform) {
      displayWaveForm(ILI9341_BLACK);
      memset(eQGraph, 0, TFT_WIDTH);
    }
    switch (c) {
      case 'a':
        adcread = !adcread;
        Serial.println("Use adc RAW " + String(adcread));
        tft.fillScreen(ILI9341_BLACK);
        break;
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
        SAMPLES *= 2;
        Serial.println("Sampling buffer " + String(SAMPLES));
        break;
      case '-':
        if (SAMPLES / 2 > 32) {
          SAMPLES /= 2;
        }
        Serial.println("Sampling buffer " + String(SAMPLES));
        break;
      case '*':
        if ( bands + 1 <= 8 ) bands++;
        bands_width = floor( TFT_WIDTH / bands );
        bands_pad = bands_width - (TFT_WIDTH / 16);
        tft.fillScreen(ILI9341_BLACK);
        Serial.println("EQ Bands " + String(bands));
        break;
      case '/':
        if ( bands - 1 > 0 ) bands--;
        bands_width = floor( TFT_WIDTH / bands );
        bands_pad = bands_width - (TFT_WIDTH / 16);
        tft.fillScreen(ILI9341_BLACK);
        Serial.println("EQ Bands " + String(bands));
        break;
      case '@':
        int SAMPLESIZE = SAMPLES / 2;
        int i = 0;

        for (int mag = 1; mag < SAMPLESIZE; mag = mag * 2) {
          byte magmapped = map(mag, 1, SAMPLESIZE, 1, TFT_WIDTH);
          Serial.println("#" + String(i) + " " + String(magmapped) + " " + String(mag)  );
          i++;
        }
        /*
          for (int i = 2; i < (SAMPLES/2); i++){ // Don't use sample 0 and only first SAMPLES/2 are usable. Each array element represents a frequency and its value the amplitude.
          if (vReal[i] > 512) { // Add a crude noise filter, 10 x amplitude or more
            //byte bandNum = getBand(i);

          }
          }*/

        break;

    }
    if (displayspectrometer) {
      drawAudioSpectrumGrid();
    }
    setBandwidth();
    max1 = 0;
    max2 = 0;
  }
}


void setup() {
  WiFi.mode(WIFI_MODE_NULL);
  Serial.begin(115200);

  adc1_config_width(ADC_WIDTH_12Bit);   //Range 0-1023
  adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_11db); //ADC_ATTEN_DB_11 = 0-3,6V
  sampling_period_us = round(1000000 * (1.0 / SAMPLING_FREQUENCY));

  tft.begin();
  tft.setRotation( 3 );
  tft.fillScreen(ILI9341_BLACK);
  tft.setTextColor(ILI9341_YELLOW);
  tft.setCursor(98, 42);
  tft.print("Sampling at: " + String(sampling_period_us) + "uS");

  delay(1000);

  tft.fillScreen(ILI9341_BLACK);

  drawAudioSpectrumGrid();
  setBandwidth();
  memset(eQGraph, 0, TFT_WIDTH);
}




void loop() {

  handleSerial();
  captureSoundSample();
  renderSpectrometer();

}

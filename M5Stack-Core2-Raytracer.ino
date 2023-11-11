// Current prototype config
// M5Stack Core2 AWS

#include <M5Core2.h>
#include <SD.h>
#include <TJpg_Decoder.h>
#include "AXP192.h"
#include <float.h>
#include <cmath>

#define RGBTO565(_r, _g, _b) ((((_r)&B11111000) << 8) | (((_g)&B11111100) << 3) | ((_b) >> 3))
#include "raytracer.h"

volatile int currentQualityLevel = 24;  // Raytracer quality level. 1-24 rays.
volatile int rowSkip = 1;

void setup(void) {
  M5.begin();  //Initialize M5Stack

  Serial.begin(115200);  // For debug

  sdInit();
  displayInit();

  M5.Axp.SetLed(false);
}

void loop() {
  // Must update button state in loop
  M5.update();

  M5.Lcd.setTextColor(TFT_WHITE, TFT_BLACK);
  M5.Lcd.fillScreen(TFT_BLACK);
  M5.Lcd.setSwapBytes(true);
  M5.Lcd.setTextFont(2);

  // start time for rendering
  unsigned long startRenderTime = millis();
  doRaytrace(currentQualityLevel, 320, 240, rowSkip);
  // end time for rendering
  unsigned long endRenderTime = millis();
  // Calculate the elapsed time for rendering
  unsigned long renderElapsedTime = endRenderTime - startRenderTime;
  Serial.print("Frame rendered in ");
  Serial.print(renderElapsedTime);
  Serial.println(" ms");

  // start time for saving
  unsigned long startSaveTime = millis();
  saveTFTToBMP24();
  // end time for saving
  unsigned long endSaveTime = millis();
  // Calculate the elapsed time for saving
  unsigned long saveElapsedTime = endSaveTime - startSaveTime;
  Serial.print(saveElapsedTime);
  Serial.println(" ms");
}

// #########################################################################
// Initialize SD card
// #########################################################################
void sdInit() {
  if (!SD.begin()) {
    Serial.println("SD Card Mount Failed");
    while (1) yield();  // Stay here forever if SD initialization fails
  }
  Serial.println("SD Card initialized.");

  uint8_t cardType = SD.cardType();

  if (cardType == CARD_NONE) {
    Serial.println("No SD card attached");
    return;
  }

  Serial.print("SD Card Type: ");
  if (cardType == CARD_MMC) {
    Serial.println("MMC");
  } else if (cardType == CARD_SD) {
    Serial.println("SDSC");
  } else if (cardType == CARD_SDHC) {
    Serial.println("SDHC");
  } else {
    Serial.println("UNKNOWN");
  }

  uint64_t cardSize = SD.cardSize() / (1024 * 1024);
  Serial.printf("SD Card Size: %lluMB\n", cardSize);

  size_t numSectors = SD.numSectors();
  Serial.printf("Number of Sectors: %u\n", numSectors);

  size_t sectorSize = SD.sectorSize();
  Serial.printf("Sector Size: %u bytes\n", sectorSize);

  uint64_t totalBytes = SD.totalBytes();
  Serial.printf("Total Bytes: %llu\n", totalBytes);

  uint64_t usedBytes = SD.usedBytes();
  Serial.printf("Used Bytes: %llu\n", usedBytes);
}

// #########################################################################
//  Initialize display
// #########################################################################
void displayInit() {
  //  tft.init();
  M5.Lcd.begin();
  M5.Lcd.setRotation(1);
  M5.Lcd.setTextColor(TFT_WHITE, TFT_BLACK);
  M5.Lcd.fillScreen(TFT_BLACK);
  M5.Lcd.setSwapBytes(true);
  M5.Lcd.setTextFont(2);
}

// Save screen **********************************************

// Function to write a 16-bit integer in little-endian format
void writeTwo(File &file, uint16_t value) {
  file.write(value & 0xFF);
  file.write((value >> 8) & 0xFF);
}

// Function to write a 32-bit integer in little-endian format
void writeFour(File &file, uint32_t value) {
  writeTwo(file, value & 0xFFFF);
  writeTwo(file, (value >> 16) & 0xFFFF);
}

// Function to save the TFT screen to a 24-bit BMP file on the SD card
void saveTFTToBMP24() {
  char filename[16];  // Buffer for filename

  // Find an available filename
  for (int i = 1; i <= 9999; ++i) {
    snprintf(filename, sizeof(filename), "/image%04d.bmp", i);
    if (!SD.exists(filename)) {
      break;
    }
    if (i == 9999) {
      Serial.println("No available filename to write BMP");
      return;
    }
  }

  M5.Axp.SetLed(true);  // Signal save start

  File file = SD.open(filename, FILE_WRITE);
  if (!file) {
    Serial.println("Failed to create file on SD");
    M5.Axp.SetLed(true);  // Signal save end with error
    return;
  }

  uint16_t width = M5.Lcd.width(), height = M5.Lcd.height();
  uint32_t rowSize = (3 * width + 3) & ~3;
  uint32_t imageSize = rowSize * height;
  uint32_t fileSize = 54 + imageSize;

  // BMP File Header (14 bytes)
  file.write('B');
  file.write('M');
  writeFour(file, fileSize);  // File size
  writeFour(file, 0);         // Reserved
  writeFour(file, 54);        // Pixel data offset (54 byte header)

  // BMP Image Header (40 bytes)
  writeFour(file, 40);         // Header size
  writeFour(file, width);      // Image width
  writeFour(file, height);     // Image height
  writeTwo(file, 1);           // Number of color planes
  writeTwo(file, 24);          // Bits per pixel
  writeFour(file, 0);          // Compression (none)
  writeFour(file, imageSize);  // Image size
  writeFour(file, 0);          // X pixels per meter
  writeFour(file, 0);          // Y pixels per meter
  writeFour(file, 0);          // Total colors
  writeFour(file, 0);          // Important colors

  // Allocate buffer for pixel data
  uint8_t *buffer = new uint8_t[3 * width * height];
  if (!buffer) {
    Serial.println("Memory allocation failed!");
    file.close();
    M5.Axp.SetLed(true);  // Signal save end with error
    return;
  }

  // Read entire screen data
  M5.Lcd.readRectRGB(0, 0, width, height, buffer);

  // Correct the color order and Write BMP Pixel Data
  for (int row = height - 1; row >= 0; row--) {
    uint8_t *rowPtr = buffer + row * width * 3;
    for (int col = 0; col < width; col++) {
      // Swap red and blue channels
      std::swap(rowPtr[col * 3], rowPtr[col * 3 + 2]);
    }
    file.write(rowPtr, width * 3);  // Write the row
    if (rowSize > width * 3) {
      // Write padding bytes at the end of the row if necessary
      file.write(0, rowSize - width * 3);
    }
  }

  delete[] buffer;  // Free the buffer memory
  file.close();
  Serial.println("File saved: " + String(filename));
  M5.Axp.SetLed(false);  // Signal save end
}
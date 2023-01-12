#include <Arduino.h>
#include <TFT_eSPI.h>
#include <SD.h>
#include <SPI.h>
#include <Wire.h>
#include "MLX90640_API.h"
#include "MLX9064X_I2C_Driver.h"
#include <PNGenc.h>
#include <Notecard.h>
#include <Adafruit_GFX.h>
#include "Adafruit_LEDBackpack.h"

#define IS_DEBUG

// ##########################################
// defines and global vars for PNGenc library
// ##########################################

#define C_BLUE Display.color565(0, 0, 255)
#define C_RED Display.color565(255, 0, 0)
#define C_GREEN Display.color565(0, 255, 0)
#define C_WHITE Display.color565(255, 255, 255)
#define C_BLACK Display.color565(0, 0, 0)
#define C_LTGREY Display.color565(200, 200, 200)
#define C_DKGREY Display.color565(80, 80, 80)
#define C_GREY Display.color565(127, 127, 127)
#define min(a, b) ((a) < (b) ? (a) : (b))
#define max(a, b) ((a) > (b) ? (a) : (b))

PNG png;
File myPNGFile;
int imgCount = 0;
unsigned long startMillis;
unsigned long currentMillis;

// how often do you want to save a PNG image to SD?
#ifdef IS_DEBUG
const unsigned long period = 1000 * 60 * 1;
#else
const unsigned long period = 1000 * 60 * 15;
#endif

// ################################################
// defines and global vars for MLX90640 thermal cam
// ################################################

#define TA_SHIFT 8
#define EMMISIVITY 0.95
// define output size
#define O_WIDTH 224
#define O_HEIGHT 168
#define O_RATIO O_WIDTH / 32

const byte MLX90640_address = 0x33;
paramsMLX90640 mlx90640;
bool measure = true;
float centerTemp;
float minTemp = 20.0;
float maxTemp = 40.0;

// variables for interpolated colors
byte red, green, blue;

// variables for row/column interpolation
float intPoint, val, a, b, c, d, ii;
int x, y, i, j;

// array for the 32 x 24 measured tempValues
static float tempValues[32 * 24];

// ##############################################################
// defines and global vars for Blues Wireless Notecard (blues.io)
// ##############################################################

#ifndef PRODUCT_UID
#define PRODUCT_UID "com.my-company.my-name:my-project"
#pragma message "PRODUCT_UID is not defined in this example. Please ensure your Notecard has a product identifier set before running this example or define it in code here. More details at https://dev.blues.io/notehub/notehub-walkthrough/#finding-a-productuid"
#endif

Notecard notecard;

// ###############################################
// defines and global vars for ILI9341 TFT display
// ###############################################

float **interpolated = NULL;
uint16_t *imageData = NULL;
TFT_eSPI Display = TFT_eSPI();

// ##########################################
// defines and global vars for SD card module
// ##########################################

Sd2Card card;
SdVolume volume;
SdFile root;
const int chipSelect = A3;

// ########################################
// defines and global vars for 7seg display
// ########################################

Adafruit_7segment matrix = Adafruit_7segment();

// ###############################################
// methods for PNG encoding and writing to SD card
// ###############################################

void *myPNGOpen(const char *filename)
{
  Serial.printf("Attempting to open %s\n", filename);
  myPNGFile = SD.open(filename, O_READ | O_WRITE | O_CREAT);
  return &myPNGFile;
}
void myPNGClose(PNGFILE *handle)
{
  File *f = (File *)handle->fHandle;
  f->close();
}
int32_t myPNGRead(PNGFILE *handle, uint8_t *buffer, int32_t length)
{
  File *f = (File *)handle->fHandle;
  return f->read(buffer, length);
}
int32_t myPNGWrite(PNGFILE *handle, uint8_t *buffer, int32_t length)
{
  File *f = (File *)handle->fHandle;
  return f->write(buffer, length);
}
int32_t myPNGSeek(PNGFILE *handle, int32_t position)
{
  File *f = (File *)handle->fHandle;
  return f->seek(position);
}

// ####################################################################
// methods for reading from MLX90640 and writing to ILI9341 TFT display
// ####################################################################

// linear interpolation
float lerp(float v0, float v1, float t)
{
  return v0 + t * (v1 - v0);
}

// get color for temperature value from MLX90640
uint16_t getColor(float val)
{
  red = constrain(255.0 / (c - b) * val - ((b * 255.0) / (c - b)), 0, 255);

  if ((val > minTemp) & (val < a))
  {
    green = constrain(255.0 / (a - minTemp) * val - (255.0 * minTemp) / (a - minTemp), 0, 255);
  }
  else if ((val >= a) & (val <= c))
  {
    green = 255;
  }
  else if (val > c)
  {
    green = constrain(255.0 / (c - d) * val - (d * 255.0) / (c - d), 0, 255);
  }
  else if ((val > d) | (val < a))
  {
    green = 0;
  }

  if (val <= b)
  {
    blue = constrain(255.0 / (a - b) * val - (255.0 * b) / (a - b), 0, 255);
  }
  else if ((val > b) & (val <= d))
  {
    blue = 0;
  }
  else if (val > d)
  {
    blue = constrain(240.0 / (maxTemp - d) * val - (d * 240.0) / (maxTemp - d), 0, 240);
  }

  // use the displays color mapping function to get 5-6-5 color palette (R=5 bits, G=6 bits, B-5 bits)
  return Display.color565(red, green, blue);
}

// read pixel data from MLX90640
void readTempValues()
{
  for (byte x = 0; x < 2; x++)
  {
    uint16_t mlx90640Frame[834];
    int status = MLX90640_GetFrameData(MLX90640_address, mlx90640Frame);
    if (status < 0)
    {
      Serial.print("GetFrame Error: ");
      Serial.println(status);
      while (1)
        ;
    }

    float vdd = MLX90640_GetVdd(mlx90640Frame, &mlx90640);
    float Ta = MLX90640_GetTa(mlx90640Frame, &mlx90640);

    float tr = Ta - TA_SHIFT; // Reflected temperature based on the sensor ambient temperature

    MLX90640_CalculateTo(mlx90640Frame, &mlx90640, EMMISIVITY, tr, tempValues);
  }
}

void interpolate()
{
  int row;
  float temp, temp2;
  for (row = 0; row < 24; row++)
  {
    for (x = 0; x < O_WIDTH; x++)
    {
      temp = tempValues[(31 - (x / 7)) + (row * 32) + 1];
      temp2 = tempValues[(31 - (x / 7)) + (row * 32)];
      interpolated[row * 7][x] = lerp(temp, temp2, x % 7 / 7.0);
    }
  }
  for (x = 0; x < O_WIDTH; x++)
  {
    for (y = 0; y < O_HEIGHT; y++)
    {
      temp = interpolated[y - y % 7][x];
      temp2 = interpolated[min((y - y % 7) + 7, O_HEIGHT - 7)][x];
      interpolated[y][x] = lerp(temp, temp2, 1); // y%7/7.0);
    }
  }
}

void drawPicture(bool forPNG)
{
  if (forPNG) // interpolate image if creating png file
  {
    interpolate();
    for (y = 0; y < O_HEIGHT; y++)
    {
      for (x = 0; x < O_WIDTH; x++)
      {
        imageData[(y * O_WIDTH) + x] = getColor(interpolated[y][x]);
      }
    }
    Display.pushImage(8, 8, O_WIDTH, O_HEIGHT, imageData);
  }
  else
  {
    for (y = 0; y < 24; y++)
    {
      for (x = 0; x < 32; x++)
      {
        Display.fillRect(8 + x * 7, 8 + y * 7, 7, 7, getColor(tempValues[(31 - x) + (y * 32)]));
      }
    }
  }
}

// get the cutoff points in the temp vs RGB graph.
void setAbcd()
{
  a = minTemp + (maxTemp - minTemp) * 0.2121;
  b = minTemp + (maxTemp - minTemp) * 0.3182;
  c = minTemp + (maxTemp - minTemp) * 0.4242;
  d = minTemp + (maxTemp - minTemp) * 0.8182;
}

// draw a legend on display
void drawLegend()
{
  float inc = (maxTemp - minTemp) / 224.0;
  j = 0;
  for (ii = minTemp; ii < maxTemp; ii += inc)
  {
    Display.drawFastVLine(8 + +j++, 292, 20, getColor(ii));
  }

  Display.setTextFont(2);
  Display.setTextSize(1);
  Display.setCursor(8, 272);
  Display.setTextColor(TFT_WHITE, TFT_BLACK);
  Display.print(String(minTemp).substring(0, 5));

  Display.setCursor(192, 272);
  Display.setTextColor(TFT_WHITE, TFT_BLACK);
  Display.print(String(maxTemp).substring(0, 5));

  Display.setTextFont(NULL);
}

void setTempScale()
{
  minTemp = 255;
  maxTemp = 0;

  for (i = 0; i < 768; i++)
  {
    minTemp = min(minTemp, tempValues[i]);
    maxTemp = max(maxTemp, tempValues[i]);
  }

  setAbcd();
  drawLegend();
}

// draw a circle + measured value on display
void drawMeasurement()
{
  // mark center measurement
  Display.drawCircle(120, 8 + 84, 3, TFT_WHITE);

  // measure and print center temperature
  centerTemp = (tempValues[383 - 16] + tempValues[383 - 15] + tempValues[384 + 15] + tempValues[384 + 16]) / 4;
  Display.setCursor(86, 214);
  Display.setTextColor(TFT_WHITE, TFT_BLACK);
  Display.setTextFont(2);
  Display.setTextSize(2);
  Display.print(String(centerTemp).substring(0, 5) + " °C");
}

void generatePNG()
{
  digitalWrite(LED_BUILTIN, HIGH);

  uint16_t WIDTH = 240;
  uint16_t HEIGHT = 184;

  char filename[13] = "image000.png";
  imgCount++;
  sprintf(filename, "image%03d.png", imgCount);

  SD.begin(chipSelect);

  while (SD.exists(filename))
  {
    //  avoid conflicting filenames
    imgCount++;
    sprintf(filename, "image%03d.png", imgCount);
  }

  int rc, iDataSize, x, y;
  uint8_t ucLine[WIDTH * 3];
  uint16_t awColors[WIDTH];
  long l;
  uint8_t r, g, b;
  Serial.printf("WIDTH = %d, HEIGHT = %d\n", WIDTH, HEIGHT);
  l = micros();
  rc = png.open(filename, myPNGOpen, myPNGClose, myPNGRead, myPNGWrite, myPNGSeek);
  if (rc == PNG_SUCCESS)
  {
    rc = png.encodeBegin(WIDTH, HEIGHT, PNG_PIXEL_TRUECOLOR, 24, NULL, 9);
    if (rc == PNG_SUCCESS)
    {
      for (uint16_t y = 0; y < HEIGHT && rc == PNG_SUCCESS; y++)
      {
        Display.readRect(0, y, WIDTH, 1, awColors);
        rc = png.addRGB565Line(awColors, ucLine);
        if (rc != PNG_SUCCESS)
          Serial.printf("Faile to write Line %d\n", y);
      } // for y
      iDataSize = png.close();
      l = micros() - l;
      Serial.printf("%d bytes of data written to file in %d us\n", iDataSize, (int)l);
    }
    else
    {
      Serial.println("Failed to begin encoder!");
    }
  }
  else
  {
    Serial.println("Failed to create the file on the SD card!");
  }

  // update 7 segment display to reflect the number of images
  matrix.print(imgCount);
  matrix.writeDisplay();

  // add a Note to the Notecard, just to let us know an image was captured
  // with the current count of images
  J *req = notecard.newRequest("note.add");
  if (req)
  {
    JAddStringToObject(req, "file", "egg-image.qo");

// if in debug mode, sync it immediately with Notehub
#ifdef IS_DEBUG
    JAddBoolToObject(req, "sync", true);
#endif

    J *body = JCreateObject();
    if (body)
    {
      JAddNumberToObject(body, "count", imgCount);
      JAddNumberToObject(body, "temp", centerTemp);
      JAddItemToObject(req, "body", body);
    }

    notecard.sendRequest(req);
  }

  digitalWrite(LED_BUILTIN, LOW);
}

void setup()
{
  Serial.begin(115200);
#ifdef IS_DEBUG
  while (!Serial)
    ; // Wait for user to open terminal
#endif

  // ################################
  // 7 segment display initialization
  // ################################
  matrix.begin(0x70);
  matrix.setBrightness(1);
  matrix.print(imgCount);
  matrix.writeDisplay();

  // ######################################
  // Blues Wireless Notecard initialization
  // ######################################

#ifdef IS_DEBUG
  notecard.setDebugOutputStream(Serial);
#endif
  notecard.begin();

  // Connect Notecard to your Notehub project
  J *req = notecard.newRequest("hub.set");
  if (req)
  {
    JAddStringToObject(req, "product", PRODUCT_UID);

// if in debug mode, maintain a "continuous" cellular connection
#ifdef IS_DEBUG
    JAddStringToObject(req, "mode", "continuous");
#else
    JAddStringToObject(req, "mode", "periodic");
    JAddNumberToObject(req, "outbound", 60);
#endif
    notecard.sendRequest(req);
  }

  // sync the Notecard with Notehub
  req = notecard.newRequest("hub.sync");
  if (req)
  {
    notecard.sendRequest(req);
  }

  // ########################################
  // MLX90640 initialization and verification
  // ########################################

  Wire.begin();
  Wire.setClock(400000);

  Wire.beginTransmission((uint8_t)MLX90640_address);
  if (Wire.endTransmission() != 0)
  {
    Serial.println("MLX90640 not detected at default I2C address. Please check wiring.");
    while (1)
      ;
  }

  // get MLX90640 device parameters
  int status;
  uint16_t eeMLX90640[832];

  status = MLX90640_DumpEE(MLX90640_address, eeMLX90640);
  if (status != 0)
  {
    Serial.println("Failed to load system parameters");
    while (1)
      ;
  }

  status = MLX90640_ExtractParameters(eeMLX90640, &mlx90640);
  if (status != 0)
  {
    Serial.println("Parameter extraction failed");
    while (1)
      ;
  }

  // set MLX90640 refresh rate
  MLX90640_SetRefreshRate(MLX90640_address, 0x05);

  // Once EEPROM has been read at 400kHz we can increase
  // Wire.setClock(800000);

  // ###########################################
  // TFT display initialization and verification
  // ###########################################

  SPI.begin();
  Display.begin();
  Display.setRotation(2);
  Display.fillScreen(C_BLACK);

  // prepare interpolated array
  interpolated = (float **)malloc(O_HEIGHT * sizeof(float *));
  for (int i = 0; i < O_HEIGHT; i++)
  {
    interpolated[i] = (float *)malloc(O_WIDTH * sizeof(float));
  }

  // prepare imageData array
  imageData = (uint16_t *)malloc(O_WIDTH * O_HEIGHT * sizeof(uint16_t));

  // get the cutoff points for the color interpolation routines
  // note this function called when the temp scale is changed
  setAbcd();
  drawLegend();

  // #########################################
  // SD module initialization and verification
  // #########################################

  Serial.println("Initializing SD card...");

  if (!card.init(SPI_HALF_SPEED, chipSelect))
  {
    Serial.println("SD card initialization failed!");
    while (1)
      ;
  }

  // try to open the 'volume'/'partition' - it should be FAT16 or FAT32
  if (!volume.init(card))
  {
    Serial.println("Could not find FAT16/FAT32 partition.\nMake sure you've formatted the card");
    while (1)
      ;
  }

#ifdef IS_DEBUG
  Serial.println("\nFiles found on the card (name, date and size in bytes): ");
  root.openRoot(volume);
  root.ls(LS_R | LS_DATE | LS_SIZE);
  root.close();
#endif

  startMillis = millis(); // set a start timestamp, so we only create PNGs periodically
}

void loop()
{
  currentMillis = millis();

  // read data from MLX90640
  readTempValues();
  setTempScale();

  // draw image to TFT display
  drawPicture(false);
  drawMeasurement();

  if (currentMillis - startMillis >= period)
  {
    // draw image to TFT display, but use interpolation to get PNG colors right
    drawPicture(true);
    // write compressed PNG image to SD card
    generatePNG();

    startMillis = currentMillis;

#ifdef IS_DEBUG
    root.openRoot(volume);
    root.ls(LS_R | LS_DATE | LS_SIZE);
    root.close();
#endif
  }
}
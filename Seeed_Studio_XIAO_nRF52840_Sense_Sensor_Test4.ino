/*
  Seeed Studio XIAO nRF52840 Sense Sensor Test V3.

  Copyright® Samuel M. Goldwasser, 1994-2025, all rights reserved.  Permission is granted for public use or modification as
  long as the Copyright notice is included.

  This a simple utility to exercise most of the Seeed Studio XIAO nRF52840 Sense sensors using the on-board LEDs and serial
  port.  The required Nano BLE 33 libraries are all either built into the Arduino IDE or Arduino Cloud Editor, or readily
  found via a Web search.  The primary difference between the Rev1 and Rev2 sketches are the libraries for the IMU and T/H.

  Accelerometer (Gs) X, Y, Z; Gyroscope (Degs/s) Roll, Pitch, Yaw; and peak Mic values are all optionally sent via the serial
  port as data-only, or with labels.

  In addition, the on-board USER LED (the CHARGE led) and RGB LEDs provide visual output:

  1. Gyroscope: Displays the values for roll, pitch, and yaw as the brightness of each if the RGB leds as roll (+red/-cyan),
     pitch (+green/-magenta), and yaw (+blue/-yellow).  Optional gyro calibration to compensate for board-specific roll, pitch,
     and yaw offsets.  If enabled, the board must remain stationary at startup while the RGB LEDs are blinking.  The default
     duration is ~12 blinks.
  2. Static tilt (accelerometer Z value): Turns on the USER LED if more than approximately 45 degrees.
  4. Microphone: Displays the peak intensity of the audio on a color scale using the RGB leds only when Gyro is not active.
  5. Heartbeat: The USER LED flashes at an approximately 1 Hz rate if there is no display activity.

  Suggestions for (modest!) improvements welcome.

  To install the necessary board and libraries in the Arduino IDE:

  1. Go to File > Preferences, and fill "Additional Boards Manager URLs" with:
     https://files.seeedstudio.com/arduino/package_seeeduino_boards_index.json (on a separate line if there are others
     there already).
  2. Go to Tools > Board > Boards Manager..., type the keyword "seeed nrf52" in the search box, select the latest version
     of the board you want, and install it.  These will appear: "Seeed nRF52 Boards" and "Seeed nRF52 mbed-enabled Boards".
     You can install both, though using only "Seeed nRF52 mbed-enabled Boards" seems to compile with fewer warnings.
  3. Download the Seeed Arduino LSM6DS3 library from: https://github.com/Seeed-Studio/Seeed_Arduino_LSM6DS3/tree/master.
     Go to Sketch > Include Library > Add Zip Library, and point to the file downloaded above.
  4. Go to Tools > Board, and select "Seeed XIAO nRF52840 Sense".
  5. Go to Tools > Port, and select the correct port.
  6. Also select the data formatting and Gyro AutoCal options in the #defines, below.

  This sketch should then compile without errors.

  Reference: https://github.com/Seeed-Studio/wiki-documents/blob/docusaurus-version/docs/Sensor/SeeedStudio_XIAO/SeeedStudio_XIAO_nRF52840-Sense/XIAO_BLE.md
*/

// User parameters
#define data1 1           // Sends data to serial port if 1, LEDs-only on Nano if 0
#define verbose1 1        // Display labels if 1, data-only if 0
#define senddiag1 0       // Include diagnostic information iff 1.  TBD, currently one integer (diag) is coded.
#define GyroAutoCal 1     // Perform automatic Gyro offset compensation at startup: The board must be stationary
                          // while the LEDs are blinking.  If not enabled, use #define GR/GP/GY_COR values.
                          //  If enabled, at start

// Sketch version number for banner. ;-)
#define Version 3

// Gyro offset parameters and variables
#define CalValues 50      // Number of Gyro samples to average for calibration

float RollOffsetSum = 0;  // Temporary variables for Gyro AutoCal sums
float PitchOffsetSum = 0;
float YawOffsetSum = 0;

float GR_COR = 0;         // Gyro offset correction values if not using GyroAutoCal
float GP_COR = 0;
float GY_COR = 0;

int Ver = Version;

int CalCount = CalValues;
float pgr, pgp, pgy;

// Color palette for audio in RGB_LEDs
#define BLACK 0, 0, 0
#define GRAY 7, 7, 7
#define MAGENTA 25, 0, 25
#define BLUE 0, 0, 75
#define CYAN 0, 50, 50
#define GREEN 0, 192, 0
#define YELLOW 128, 92, 0
#define ORANGE 200, 40, 0
#define RED 255, 0, 0
#define WHITE 255, 255, 255

#define timeoutvalue  30

#define LED_USER 17 // Defined as CHARGE LED on nRF board

#include <Arduino.h>
#include <LSM6DS3.h>  // IMU

//Create a instance of class LSM6DS3
LSM6DS3 myIMU(I2C_MODE, 0x6A);    //I2C device address 0x6A

#include <PDM.h>      // Microphone

int count = 0;
int i = 0;
int timeout = 0;
int sum = 0;

short sampleBuffer[1024];       // buffer to read audio samples into, each sample is 16-bits
volatile int samplesRead = 0;   // number of samples read

float ax, ay, az, gr, gp, gy, grcor, gpcor, gycor;
static int led, ledr, ledg, ledb;
void setup() {

  // Fixed calibration values may be needed if Gyro AutoCal is not enabled
  if (GyroAutoCal == 0) {
    GR_COR = 0; // Sample #1
    GP_COR = 0;
    GY_COR = 0;
  }

  // Initialize LEDs.  Note polarity of all LEDs is HIGH = OFF.
  nrf_gpio_cfg_output(LED_USER);
  pinMode(LEDR, OUTPUT);
  pinMode(LEDG, OUTPUT);
  pinMode(LEDB, OUTPUT);

  // Turn the LED_USER on and set the RGB LEDs at low brightness.
  nrf_gpio_pin_write(LED_USER,0);
  RGB_LED_Color(GRAY);

  // Serial port
  if (data1 == 1) {
    Serial.begin(9600);
    while (!Serial);
    Serial.println();
    Serial.println();
    Serial.println("Seeed Studio XIAO nRF52840 Sense Sensor Exerciser.");
  }
 
  // Configure the IMU
  if (myIMU.begin() != 0) {
    Serial.println("Failed to initialize IMU!");
  }

  // configure the data receive callback
  PDM.onReceive(onPDMdata);

  // Microphone (one channel, mono mode. The only sample rates that work so far are 16.000 kHz and 41.667 kHz.  Go figure. ;-)
  if (!PDM.begin(1, 41667)) {
    Serial.println("Failed to start PDM!");
    while (1);
  }

  PDM.setBufferSize(1024);  // 512 is default; 1024 works but 2048 hangs
  PDM.setGain(25);          // Optionally set gain, defaults to 20

  // Banner blurb
  
  if ((verbose1 == 1) && (data1 == 1)) {
    Serial.println();
    Serial.print("**** Seeed Studio XIAO nRF52840 Sense Rev2 Sensor Test Version ");
    Serial.print(Ver);
    Serial.println(" ****");
    Serial.println();
    Serial.println("Functions:");
    Serial.println();
    Serial.println("  - Acceleration in Gs.");
    Serial.println("  - Gyro angle in degrees/second. LED threshold 25.");
    Serial.println("  - Peak soundlevel in arbitrary units.");
    Serial.println();
    Serial.println("Data:");
    Serial.println("");
    if (GyroAutoCal == 0) delay(1000);
  }

  // Gyro AutoCal
  if (GyroAutoCal == 1) Do_GyroAutoCal(25); // Argument is the delay in ms inside GyroAutoCal loop
}

void loop() {

  char buffer[40];
  int led, ledr, ledg, ledb;
  int diag = 0;

  // read the acceleration data
  ax = myIMU.readFloatAccelX();
  ay = myIMU.readFloatAccelY();
  az = myIMU.readFloatAccelZ();
 
  if (data1 == 1) {
    if (verbose1 == 1) Serial.print("  Axl (Gs) X: ");
    sprintf(buffer, "%5.2f", ax);
    Serial.print(buffer);
    if (verbose1 == 1) Serial.print(" Y: ");
    sprintf(buffer, "%5.2f", ay);
    Serial.print(buffer);
    if (verbose1 == 1) Serial.print(" Z: ");
    sprintf(buffer, "%5.2f", az);
    Serial.print(buffer);
    if (verbose1 == 1) Serial.print(" | ");
  }
 
  led = (az * 255);
  if (led < 180) nrf_gpio_pin_write(LED_USER, LOW);  // Turn on LED_PWR if tilt is more than ~45 degrees
  else nrf_gpio_pin_write(LED_USER, HIGH);

  // Gyroscope

  gr = myIMU.readFloatGyroX();
  gp = myIMU.readFloatGyroY();
  gy = myIMU.readFloatGyroZ();

  grcor = (gr - GR_COR);
  gpcor = (gp - GP_COR);
  gycor = (gy - GY_COR);

  if (data1 == 1) {
    if (verbose1 == 1) Serial.print("Gyro (Degs/s) R: ");
    sprintf(buffer, "%8.2f", grcor);
    Serial.print(buffer);
    if (verbose1 == 1) Serial.print("  P: ");
    sprintf(buffer, "%8.2f", gpcor);
    Serial.print(buffer);
    if (verbose1 == 1) Serial.print("  Y: ");
    sprintf(buffer, "%8.2f", gycor);
    Serial.print(buffer);
    if (verbose1 == 1) Serial.print(" ");
  }

  RGB_Gyro_Colors(grcor, gpcor, gycor);

  // Microphone

  // wait for samples to be read
  if (samplesRead) {
    int i = 0;
    sum = 0;

    for (i = 0; i < samplesRead; i++)
      if (fabs(sampleBuffer[i]) > sum) sum = fabs(sampleBuffer[i]);  // Peak detect

    // Display the peak sound value in RGB_LED
    if (((fabs(gr - GR_COR) < 1) && (fabs(gp - GP_COR) < 1) && (fabs(gy - GY_COR)) < 1)) {  // Only if no Gyro activity
      if (sum >= 1000) RGB_LED_Color(WHITE);
      else if (sum >= 600) RGB_LED_Color(RED);
      else if (sum >= 400) RGB_LED_Color(ORANGE);
      else if (sum >= 325) RGB_LED_Color(YELLOW);
      else if (sum >= 250) RGB_LED_Color(GREEN);
      else if (sum >= 175) RGB_LED_Color(CYAN);
      else if (sum >= 100) RGB_LED_Color(BLUE);
      else if (sum >= 50) RGB_LED_Color(MAGENTA);
      else if (sum >= 25) RGB_LED_Color(GRAY);
      else if (sum >= 0) RGB_LED_Color(BLACK);
    }
    if (sum >= 25) timeout = timeoutvalue * 2;
  }

  if (data1 == 1) {
    if (verbose1 == 1) Serial.print(" | Mic: ");
    sprintf(buffer, "%4d", sum);
    Serial.print(buffer);
    if (senddiag1 == 0) Serial.println("");
  }
 
  // Optional diagnostic field
  if (senddiag1 == 1) {
    if (data1 == 1) {
      if (verbose1 == 1) Serial.print(" | Diag: ");
      sprintf(buffer, "%4d", diag);
      Serial.print(buffer);
      Serial.println("");
    }
  }   

// Heartbeat
  count++;
  if (count >= timeoutvalue) {
    if ((ledr <= 8) && (ledg <= 8) && (ledb <= 8) && (sum < 25) && (timeout == 0)) {
      nrf_gpio_pin_write(LED_USER,0);  // Pulse USER led
      count = 0;
    }
  }
  samplesRead = 0;  // Clear microphone sample buffer
  delay(timeoutvalue);
}

void RGB_LED_Color(int r, int g, int b) {
    analogWrite(LEDR, 255 - r);
    analogWrite(LEDG, 255 - g);
    analogWrite(LEDB, 255 - b);
  }

void onPDMdata() {
  // query the number of bytes available
  int bytesAvailable = PDM.available();

  // read into the sample buffer
  PDM.read(sampleBuffer, bytesAvailable);

  // 16-bit, 2 bytes per sample
  samplesRead = bytesAvailable / 2;
}

void Do_GyroAutoCal (int Delay) {
  RGB_LED_Color(BLACK);
  while (CalCount > 0) {
    pgr = gr; pgp = gp; pgy = gy;
    gr = myIMU.readFloatGyroX();
    gp = myIMU.readFloatGyroY();
    gy = myIMU.readFloatGyroZ();
    if (CalCount == CalValues) {
       CalCount--;          // Skip corrupted first value
    }
    else if (CalCount > 1) {
      delay(Delay);
      if (((fabs(gr - pgr) > 8)) || ((fabs(gp - pgp) > 8)) || ((fabs(gr - pgr) > 8))) { // Start over if too much gyro activity
        CalCount = CalValues;
        RollOffsetSum = 0;
        PitchOffsetSum = 0;
        YawOffsetSum = 0;
      } 
      else {
        RollOffsetSum += gr; // Update sums
        PitchOffsetSum += gp;
        YawOffsetSum += gy;
        if ((CalCount & 3) == 2) RGB_LED_Color(GRAY); // Heartbeat while AutoCal in progress
        else RGB_LED_Color(BLACK);
        CalCount--;
      }
    }
    else if (CalCount == 1) { // Compute average offsets
      GR_COR = RollOffsetSum / CalValues;
      GP_COR = PitchOffsetSum / CalValues;
      GY_COR = YawOffsetSum / CalValues;
      CalCount = 0;
      RGB_LED_Color(BLACK);
    }
  }
}

void RGB_Gyro_Colors(float roll, float pitch, float yaw) {
  // Generate colors for Gyro activity.  Note this color scale differs subtly from the one used for the microphone.
  if ((fabs(roll) / 2 > 8) || ((fabs(pitch) / 2) > 8) || ((fabs(yaw) / 2) > 8)) { // Update if above threahold
    if (roll > 0) {                // Add in RED
      ledr =  ((roll * 255)/255);
      ledg =   (roll * 0); 
      ledb =   (roll * 0);
    }
    else {                         // Add in CYAN
      ledr =  -(roll * 0);
      ledg = -((roll * 180)/255);
      ledb = -((roll * 75)/255);
    }
    if (pitch > 0) {               // Add in GREEN
      ledr +=  (pitch * 0);
      ledg += ((pitch * 255)/255);
      ledb +=  (pitch * 0);
    }
    else {                         // Add in MAGENTA
      ledr -= ((pitch * 127)/255);
      ledg -=  (pitch * 0);
      ledb -= ((pitch * 127)/255);
    }
    if (yaw > 0) {               // Add in BLUE
      ledr +=  (yaw * 0);
      ledg +=  (yaw * 0);
      ledb += ((yaw * 255)/255);
      }
    else {                         // Add in YELLOW
      ledr -= ((yaw * 127)/255);
      ledg -= ((yaw * 127)/255);
      ledb -=  (yaw * 0);
    }

    RGB_LED_Color(ledr, ledg, ledb);
    timeout = 16;
  }
  else if (timeout > 0) timeout--;
}

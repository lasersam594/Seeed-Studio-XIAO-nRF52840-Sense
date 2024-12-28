/*
Seeed Studio XIAO nRF52840 Sense RGB Cycle Colors 1.

Copyright® Samuel M. Goldwasser, 1994-2025, all rights reserved.

This is sort of the fancy equivalent of Blink. ;-)  Cycles through a rainbow of colors in the RGB leds and toggles LED_BUILTIN,
as well as LED_PWR, which many don't even realize can be controlled.

If your first time using a Seed Studio XIAO nRF52840 Sense, install the necessary board in the Arduino IDE:

1. Go to Tools > Board > Boards Manager or click the Boards icon, type the keyword "nrf" in the search box, install "Seeed
   nRF52 mbed-enabled Boards"
2. Go to Tools > Board, and select "Seeed nRF mbed-enabled Boards > Seeed XIAO BLE Sense - nRF52840.
3. Go to Tools > Port, and select the correct port.

The sketch should then compile without errors (though there may be warnings that can be ignored).
*/

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

#define LED_USER 17 // Defined as CHARGE LED on nRF board

bool state; // LED_USER on/off

void setup() {
  // initialize LEDs.
  nrf_gpio_cfg_output(LED_USER);
  pinMode(LEDR, OUTPUT);
  pinMode(LEDG, OUTPUT);
  pinMode(LEDB, OUTPUT);

  // Turn LED_USER off and set the RGB LEDs at low brightness before cycling colors
  nrf_gpio_pin_write(LED_USER,0);
  RGB_LED_Color(GRAY);
  delay(250); // Pause before cycling
}

// Cycle through a rainbow (more or less) of colors in the RGB_LEDs and toggle LED_USER
void loop() {
  nrf_gpio_pin_write(LED_USER, state);
  state = !state;
  RGB_LED_Cycle_Colors();
  delay(50);
}

void RGB_LED_Color(int r, int g, int b) {
  analogWrite(LEDR, 255 - r);
  analogWrite(LEDG, 255 - g);
  analogWrite(LEDB, 255 - b);
}

void RGB_LED_Cycle_Colors() {
  RGB_LED_Color(BLACK); delay(50);
  RGB_LED_Color(GRAY); delay(50);
  RGB_LED_Color(MAGENTA); delay(50);
  RGB_LED_Color(BLUE); delay(50);
  RGB_LED_Color(CYAN); delay(50);
  RGB_LED_Color(GREEN); delay(50);
  RGB_LED_Color(YELLOW); delay(50);
  RGB_LED_Color(ORANGE); delay(50);
  RGB_LED_Color(RED); delay(50);
  RGB_LED_Color(ORANGE); delay(50);
  RGB_LED_Color(YELLOW); delay(50);
  RGB_LED_Color(GREEN); delay(50);
  RGB_LED_Color(CYAN); delay(50);
  RGB_LED_Color(BLUE); delay(50);
  RGB_LED_Color(MAGENTA); delay(50);
  RGB_LED_Color(GRAY); delay(50);
}
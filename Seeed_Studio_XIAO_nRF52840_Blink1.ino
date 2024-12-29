/*
  Seeed Studio XIAO nRF52840 Blink1

  Turns an LED on for one second, then off for one second, repeatedly. Rather
  boring after a short time (like 3 seconds). ;-)

  Most Arduinos have an on-board LED you can control. On the UNO, MEGA and ZERO
  it is attached to digital pin 13, on MKR1000 on pin 6. LED_BUILTIN is set to
  the correct LED pin independent of which board is used.

  However, for the Seeed_Studio_XIAO_nRF52840 boards, this LED is not on a
  normal digital pin but onlythe CPU pin P0.17, so a non-standard method of accessing
  it must be used.  In fact, these boards appear to not even be pre-programmed
  with the BLINK sketch like most others I've seen.

  If you want to know what pin the on-board LED is connected to on most Arduino
  models, check the Technical Specs of your board at:
  https://www.arduino.cc/en/Main/Products.  However, these Seeed Studio boards
  are probably not covered.

  modified 8 May 2014 by Scott Fitzgerald
  modified 2 Sep 2016 by Arturo Guadalupi
  modified 8 Sep 2016 by Colby Newman
  modified 28 Dec 2024 for the Seeed Studeio XIAO nRF52840 by Sam Goldwasser

  This example code is in the public domain.
*/

#define LED_CHARGE 17 // Ugly green LED normally used for battery charge status on CPU PIO pin 0.17

// the setup function runs once after uploading the sketch, when the  Reset button is press (if you
// can find it!, or when power is applied to the board.
void setup() {
  // initialize LED_CHARGE on CPU pin P0.17 as an output
  nrf_gpio_cfg_output(LED_CHARGE);
}

// the loop function runs over and over again forever
void loop() {
  nrf_gpio_pin_write(LED_CHARGE,LOW);  // Turn the LED on (LOW is the voltage level, the LED anode goes to +V)
  delay(1000);                         // Wait for a second
  nrf_gpio_pin_write(LED_CHARGE,HIGH); // Turn the LED off
  delay(1000);                         // Wait for a second
}

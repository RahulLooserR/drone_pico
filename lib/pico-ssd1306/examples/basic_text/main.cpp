#include "pico/stdlib.h"
#include "pico-ssd1306/ssd1306.h"
#include "pico-ssd1306/textRenderer/TextRenderer.h"
#include "hardware/i2c.h"
#include <stdio.h>
#include <tusb.h>

#define I2C_PIN_SDA 20
#define I2C_PIN_SCL 21
#define I2C_PORT i2c0
// Use the namespace for convenience
using namespace pico_ssd1306;

int main()
{
  stdio_init_all();
  // while (!tud_cdc_connected())
  // {
  //   sleep_ms(10);
  // }
  i2c_init(I2C_PORT, 1000000); // Use i2c port with baud rate of 1Mhz
  // Set pins for I2C operation
  gpio_set_function(I2C_PIN_SDA, GPIO_FUNC_I2C);
  gpio_set_function(I2C_PIN_SCL, GPIO_FUNC_I2C);
  gpio_pull_up(I2C_PIN_SDA);
  gpio_pull_up(I2C_PIN_SCL);

  // If you don't do anything before initializing a display pi pico is too fast and starts sending
  // commands before the screen controller had time to set itself up, so we add an artificial delay for
  // ssd1306 to set itself up
  sleep_ms(250);

  // Create a new display object at address 0x3D and size of 128x64
  SSD1306 display = SSD1306(I2C_PORT, 0x3C, Size::W128xH64);

  // Here we rotate the display by 180 degrees, so that it's not upside down from my perspective
  // If your screen is upside down try setting it to 1 or 0
  display.setOrientation(1);
  display.setContrast(255);

  // Draw text on display
  // After passing a pointer to display, we need to tell the function what font and text to use
  // Available fonts are listed in textRenderer's readme
  // Last we tell this function where to anchor the text
  // Anchor means top left of what we draw
  drawText(&display, font_16x32, "TEST text", 0, 0);

  // Send buffer to the display
  display.sendBuffer();

  drawText(&display, font_8x8, "TEST text 1", 0, 9);
  display.sendBuffer();

  // display.clear();
  // drawText(&display, font_5x8, "TEST te", 0, 9);
  // display.sendBuffer();

  while (true)
  {
    sleep_ms(1000);
    printf("TEST text\n");
  }
}
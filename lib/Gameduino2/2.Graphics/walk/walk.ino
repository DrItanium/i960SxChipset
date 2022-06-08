#include <EEPROM.h>
#include <SPI.h>
#include <GD2.h>

static int a[256];

#include "walk_assets.h"

void setup()
{
  Serial.begin(1000000);
  delay(5000);
  Serial.println("HELLO");

  GD.begin(~GD_STORAGE);
  Serial.println(__LINE__);
  LOAD_ASSETS();
  for (int i = 0; i < 256; i++)
    a[i] = GD.random(GD.w);
}

void loop()
{
  GD.ClearColorRGB(0x000050);
  GD.VertexFormat(0);
  GD.Clear();
  GD.Begin(BITMAPS);
  GD.BitmapHandle(WALK_HANDLE);
  for (int i = 0; i < 256; i++) {
    GD.ColorRGB(i, i, i);
    GD.Cell((a[i] >> 1) & 7);
    GD.Vertex2f(a[i], map(i, 0, 255, 0, GD.h - 20));
    a[i] = (a[i] + 1) % GD.w;
  }
  GD.swap();
}

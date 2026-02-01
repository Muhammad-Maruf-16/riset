/*
26 januari 2026
tambahan untuk tombol cross dan circle 
*/

#include <PS2X_lib.h>  //for v1.6



PS2X ps2x;

void setup() {
  Serial.begin(115200);
  ps2x.config_gamepad(13, 11, 10, 12, true, true);  //setup pins and settings:  GamePad(clock, command, attention, data, Pressures?, Rumble?) check for error
}

void loop() {
  ps2x.read_gamepad(false, false);  //read controller and set large motor to spin at 'vibrate' speed

  if (ps2x.NewButtonState())  //will be TRUE if any button changes state (on to off, or off to on)
  {

    if (ps2x.Button(PSB_R1))
      Serial.println("R");
    if (ps2x.Button(PSB_L1))
      Serial.println("L");
    if (ps2x.Button(PSB_PAD_UP))
      Serial.println("W");
    if (ps2x.Button(PSB_PAD_DOWN))
      Serial.println("S");
    if (ps2x.Button(PSB_PAD_RIGHT))
      Serial.println("D");
    if (ps2x.Button(PSB_PAD_LEFT))
      Serial.println("A");
    if (!ps2x.Button(PSB_PAD_UP) && !ps2x.Button(PSB_PAD_DOWN) && !ps2x.Button(PSB_PAD_RIGHT) && !ps2x.Button(PSB_PAD_LEFT) && !ps2x.Button(PSB_R1) && !ps2x.Button(PSB_L1)) {  //will be TRUE as long as button is pressed
      Serial.println("X");
    }


    if (ps2x.Button(PSB_CROSS))
      Serial.println("B");
    if (ps2x.Button(PSB_CIRCLE))
      Serial.println("C");
    if (ps2x.Button(PSB_TRIANGLE))
      Serial.println("E");
    if (ps2x.Button(PSB_SQUARE))
      Serial.println("F");
    if (ps2x.Button(PSB_R2))
      Serial.println("G");
    if (ps2x.Button(PSB_L2))
      Serial.println("H");
    if (ps2x.Button(PSB_R3))
      Serial.println("I");
    if (ps2x.Button(PSB_L3))
      Serial.println("J");
    if (ps2x.Button(PSB_START))
      Serial.println("K");
    if (ps2x.Button(PSB_SELECT))
      Serial.println("M");

  }

  // if (!ps2x.Button(PSB_PAD_UP) && !ps2x.Button(PSB_PAD_DOWN) && !ps2x.Button(PSB_PAD_RIGHT) && !ps2x.Button(PSB_PAD_LEFT)) {  //will be TRUE as long as button is pressed
  //   Serial.println("X");
  // }

  delay(10);
}

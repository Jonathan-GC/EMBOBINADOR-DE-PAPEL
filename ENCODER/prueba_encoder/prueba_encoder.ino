/** \file
 *  \brief     This is the first official release of the phi_interfaces library.
 *  \details   This library unites buttons, rotary encoders and several types of keypads libraries under one library, the phi_interfaces library, for easy of use. This is the first official release. All currently supported input devices are buttons, matrix keypads, rotary encoders, analog buttons, and liudr pads. User is encouraged to obtain compatible hardware from liudr or is solely responsible for converting it to work on other shields or configurations.
 *  \author    Dr. John Liu
 *  \version   1.0
 *  \date      01/24/2012
 *  \copyright Dr. John Liu. Free software for educational and personal uses. Commercial use without authorization is prohibited.
*/

#include <phi_interfaces.h>

#define Encoder1ChnA 12
#define Encoder1ChnB 13
#define EncoderDetent A0

char mapping1[]={'U','D'}; // This is a rotary encoder so it returns U for up and D for down on the dial.
phi_rotary_encoders my_encoder1(mapping1, Encoder1ChnA, Encoder1ChnB, EncoderDetent);
multiple_button_input* dial1=&my_encoder1;

void setup()
{
  Serial.begin(9600);
  Serial.println("Phi_interfaces library rotary encoder test code");
}

void loop()
{
  char temp;
//Rotary encoder 1:  
  temp=my_encoder1.getKey(); // Use phi_keypads object to access the keypad
  temp=dial1->getKey(); // Use the phi_interfaces to access the same keypad
  if (temp!=NO_KEY) Serial.println(temp);
}

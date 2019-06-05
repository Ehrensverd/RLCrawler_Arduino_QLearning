/* Encoder Library - Basic Example
 * http://www.pjrc.com/teensy/td_libs_Encoder.html
 *
 * This example code is in the public domain.
 */

#include <Encoder.h>
#include <SoftwareSerial.h> // import the serial library
#include <Servo.h>
Servo S1, S2;


SoftwareSerial hc05(10, 11); // RX, TX
// Change these two numbers to the pins connected to your encoder.
//   Best Performance: both pins have interrupt capability
//   Good Performance: only the first pin has interrupt capability
//   Low Performance:  neither pin has interrupt capability
Encoder myEnc(7, 8);
//   avoid using pins with LEDs attached
int oldPosition  = -999;
int newPosition = 0;
int btdata= 0;
void setup() {  
  S1.attach(6); //range 20 to 180
  S2.attach(5); //range 10 to 150; 90 is 45degree up-forward; 150 is forward-down; 10 is backwards-up 
  hc05.begin(115200);
  hc05.println("Basic Encoder Test:");
  
}


void loop() {
  newPosition = myEnc.read();
  if (newPosition != oldPosition) {
    oldPosition = newPosition;
    //hc05.print("Current Position: ");
    //hc05.println(newPosition);
    //if(hc05.available())
    //{
    btdata = hc05.read();
   // } 
    S1.write(140); //range 20 to 180 
    S2.write(150); //range 10 to 150;
   
   }
   
}

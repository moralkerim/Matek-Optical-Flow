#include <SoftwareSerial.h>

#define rxPin 10
#define txPin 11

char rng_msg[14];
uint8_t distance_buf[4];
char ck_in;

SoftwareSerial mySerial =  SoftwareSerial(rxPin, txPin);

void setup() {
  // put your setup code here, to run once:
  pinMode(rxPin, INPUT);
  pinMode(txPin, OUTPUT);

  mySerial.begin(115200);
  Serial.begin(115200);

}

void loop() {
  // put your main code here, to run repeatedly:
  serialEvent();


}


void serialEvent() {



  while (mySerial.available())
  {
    char inChar = (char)mySerial.read();
    //Serial.print(inChar);
    //Serial.println(inChar);
    if (inChar == 0x24) {  //Start char received
      rng_msg[0] = 0x24;
      //Serial.print(inChar);
      char X    = (char)mySerial.read();
      rng_msg[1] = X;
      char type = (char)mySerial.read();
      rng_msg[2] = type;
      char flag = (char)mySerial.read();
      rng_msg[3] = flag;
      // Serial.print(X);
      // Serial.print(type);
      //Serial.print(flag);
      char msg_type[2];

      //MSG TYPE
      msg_type[1] = (char)mySerial.read();
      msg_type[0] = (char)mySerial.read();
      // Serial.print(msg_type);
      rng_msg[4] = msg_type[1];
      rng_msg[5] = msg_type[0];

      //RANGEFINDER MESSAGE
      if (msg_type[0] == 0x1f && msg_type[1] == 0x01) {
        uint8_t payload_size_buf[2];
        payload_size_buf[0] = (uint8_t)mySerial.read(); //5
        payload_size_buf[1] = (uint8_t)mySerial.read(); //0

        rng_msg[6] = (char)payload_size_buf[0];
        rng_msg[7] = (char)payload_size_buf[1];
        uint16_t payload_size = payload_size_buf[1] << 8 | payload_size_buf[0];
        //        char payload_size = (char)mySerial.read();
        //        Serial.print( payload_size);
        //        payload_size = (char)mySerial.read();
        //Serial.println( payload_size);
        char FF = mySerial.read(); //FF
        rng_msg[8] = FF;
        //Serial.print(FF);

        for (int i = 0; i < 4; i++) {
          distance_buf[i] = (uint8_t)mySerial.read();
          rng_msg[9 + i] = (char)distance_buf[i];
        }
        ck_in = (char)mySerial.read();


      }

      rng_msg[13] = (char)mySerial.read();


      //CRC Check
      uint8_t ck2 = 0; // initialise CRC
      for (int i = 3; i < 13; i++)
        ck2 = crc8_dvb_s2(ck2, rng_msg[i]); // loop over summable data

      if (rng_msg[13] = ck2) {

        uint16_t distance = distance_buf[0] | distance_buf[1] << 8 | distance_buf[2] << 16 | distance_buf[3] << 24 ;

        Serial.println(distance);
      }

    }



    //Serial.print(ck_in);
  }

}



uint8_t crc8_dvb_s2(uint8_t crc, unsigned char a)
{
  crc ^= a;
  for (int ii = 0; ii < 8; ++ii) {
    if (crc & 0x80) {
      crc = (crc << 1) ^ 0xD5;
    } else {
      crc = crc << 1;
    }
  }
  return crc;
}

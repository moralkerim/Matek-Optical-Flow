#include <SoftwareSerial.h>

#define rxPin 10
#define txPin 11

#define RNG_MSG_L 0x01
#define RNG_MSG_U 0x1f

#define OF_MSG_L 0x02
#define OF_MSG_U 0x1f

#define START_MSG 0x24

typedef enum
{
  RANGE_MSG,
  OF_MSG
} matek_msg_type;

uint8_t distance_buf[4];
char ck_in;
char msg[18];

struct OF_Msg {
  int32_t motion_x;
  int32_t motion_y;
};

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
    MatekRead();
  }
}

void MatekRead() {
  char inChar = (char)mySerial.read();
  //Serial.print(inChar);
  //Serial.println(inChar);
  if (inChar == START_MSG) {  //Start char received

    msg[0] = START_MSG;
    //Serial.print(inChar);
    char X    = (char)mySerial.read();
    msg[1] = X;
    char type = (char)mySerial.read();
    msg[2] = type;
    char flag = (char)mySerial.read();
    msg[3] = flag;
    // Serial.print(X);
    // Serial.print(type);
    //Serial.print(flag);
    char msg_type[2];

    //MSG TYPE
    msg_type[1] = (char)mySerial.read();
    msg_type[0] = (char)mySerial.read();
    // Serial.print(msg_type);
    msg[4] = msg_type[1];
    msg[5] = msg_type[0];

    matek_msg_type matek_type;
    //RANGEFINDER MESSAGE
    if (msg_type[0] == RNG_MSG_U && msg_type[1] == RNG_MSG_L) {
      matek_type = RANGE_MSG;
    }

    //OF MESSAGE
    else if (msg_type[0] == OF_MSG_U && msg_type[1] == OF_MSG_L) {
      matek_type = OF_MSG;
    }


    uint8_t payload_size_buf[2];
    payload_size_buf[0] = (uint8_t)mySerial.read(); //5
    payload_size_buf[1] = (uint8_t)mySerial.read(); //0

    msg[6] = (char)payload_size_buf[0];
    msg[7] = (char)payload_size_buf[1];
    uint16_t payload_size = payload_size_buf[1] << 8 | payload_size_buf[0];
    payload_size = payload_size - 1;
    //        char payload_size = (char)mySerial.read();
    //        Serial.print( payload_size);
    //        payload_size = (char)mySerial.read();
    //Serial.println( payload_size);
    uint8_t quality = mySerial.read(); //FF
    msg[8] = (char)quality;
    //Serial.println(quality);
    for (int i = 0; i < payload_size - 1; i++) {
      msg[9 + i] = (uint8_t)mySerial.read();
    }
    //ck_in = (char)mySerial.read();
    msg[9 + payload_size] = (char)mySerial.read();
    if (matek_type == OF_MSG) {
      OF_Msg of_msg = MatekDecodeOF(msg);
      Serial.print("motion_x: "); Serial.println(of_msg.motion_x);
      Serial.print("motion_y: "); Serial.println(of_msg.motion_y);
    }

    else if (matek_type == RANGE_MSG) {
      uint16_t distance = MatekDecodeRange(msg);
      Serial.print("range: "); Serial.println(distance);
    }


  }
}



uint16_t MatekDecodeRange(char* msg) {
  uint16_t distance = msg[9] | msg[10] << 8 | msg[11] << 16 | msg[12] << 24 ;
  return distance;
}

OF_Msg MatekDecodeOF(char* msg) {
  OF_Msg of_msg;
  of_msg.motion_x = msg[9] | msg[10] << 8 | msg[11] << 16 | msg[12] << 24 ;
  of_msg.motion_y = msg[13] | msg[14] << 8 | msg[15] << 16 | msg[16] << 24 ;
  return of_msg;
}

void CrcCheck(char* msg, matek_msg_type matek_type) {
  uint16_t payload_size;
  switch (matek_type) {
    case RANGE_MSG:
      payload_size = 4;
      break;

    case OF_MSG:
      payload_size = 8;
      break;
  }
  //Serial.println(payload_size);
  //CRC Check
  uint8_t ck2 = 0; // initialise CRC
  for (int i = 3; i < 9 + payload_size; i++)
    ck2 = crc8_dvb_s2(ck2, msg[i]); // loop over summable data
  Serial.print("msg_type: "); Serial.println(matek_type);
  if (msg[9 + payload_size] == ck2) {
    switch (matek_type) {
      case RANGE_MSG:
        uint16_t distance = MatekDecodeRange(msg);
        Serial.print("range: "); Serial.println(distance);
        break;

      case OF_MSG:
        Serial.println("test");
        OF_Msg of_msg = MatekDecodeOF(msg);
        Serial.print("motion_x: "); Serial.println(of_msg.motion_x);
        Serial.print("motion_y: "); Serial.println(of_msg.motion_y);
        break;
    }

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

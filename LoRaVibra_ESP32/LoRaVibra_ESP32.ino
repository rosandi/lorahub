#include <SPI.h>
#include <LoRa.h>
#include "smallFFT.c"

#define BATSTAT A7
#define SCK     15
#define MISO    14
#define MOSI    16
#define SS      8
#define RST     4
#define DI0     7
#define BAND    921000000
#define PABOOST true 
#define CHUNK 0
#define OVERSMP 200
#define RECOK     0x01
#define RECFAILED 0x00
#define RECRETRY  0x0F
#define RECIGNORE 0xFF

#define ADC0 34
#define ADC1 35
#define LED_BUILTIN 2

#define LOGN 7
#define NDATA 128
int16_t val[NDATA], ival[NDATA];
uint8_t *buff=(uint8_t*)ival;
uint8_t *bptr=buff;
uint16_t valmax=-1;

uint16_t DLY=10;
bool data_avail=false;
int sendcnt=0;

uint16_t gwid=0;

int code;
unsigned long ts;

// UBAH DEVICE ID!!!
uint16_t device_id=101;
String devid="<101>";
uint16_t counter=0;
uint16_t imul;
int i,j,k;

void lifesign(int n, int spd=1000) {
  for (int i=0;i<n;i++) {
    digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
    delay(spd);                       // wait for a second
    digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
    delay(spd);                       // wait for a second
  }
}


void acquireData() {
  int ovr;

  analogSetAttenuation(ADC_0db);
  
#ifdef DEBUG
  Serial.println("data acquisition...");
#endif

  ts=millis();
  
  for(i=0;i<NDATA;i++) {
    long d=0;
    
    for(ovr=0;ovr<OVERSMP;ovr++) 
      d+=(analogRead(ADC1)-analogRead(ADC0));
      
    val[i]=(int16_t)(d/OVERSMP);
  }

  fft(val,ival,LOGN);
  
  ts=millis()-ts;
  data_avail=true;
  sendcnt=0;
}

uint8_t checkReceived() {
  uint16_t cnt=0;

  while (LoRa.parsePacket() == 0) {
    delay(random(200));
    if (cnt++ > 10) break;
  } 

  if (cnt>10) return RECIGNORE;

  String sid;
  String stat;
  String spar;
  char ch;
  
/* 
 * reply format
 * 12345 ok parameter
 *
 * if no response --> ignore
 * if not my ID --> ignore
 * if I understand the message --> OK
 * anything else --> failed
 * 
 */

  if(not LoRa.available()) return RECIGNORE;
  
  while(LoRa.available()) {
    ch=(char)LoRa.read();
    if(ch==' ') break;
    sid+=ch;
  }

  if (sid.toInt() != device_id) return RECIGNORE;  
  
  while(LoRa.available()) {
    ch=(char)LoRa.read();
    if(ch==' ') break;
    stat+=ch;
  }

  while(LoRa.available()) {
    ch=(char)LoRa.read();
    spar+=ch;
  }
 
  if (stat=="ok") {
    lifesign(2,200);
    return RECOK;
  }
  if (stat=="sleep") {
    DLY=spar.toInt();
    if(DLY==0) DLY=60;
    return RECOK;
  }
  if (stat=="ping") {
    lifesign(20,500);
    return RECOK;
  }
  
  return RECFAILED;
}

void sendData() {    
  // [bin][len][maxlo][maxhi][chk][rsv*4][data...]

#ifdef DEBUG
Serial.println("Sending data");
#endif

  buff[0]=0x00;          // bitstream mark
  buff[1]=NDATA/2;       // length;
  ival[1]=valmax;        // max value: buff[2],buff[3]
  ival[2]=(uint16_t)ts;  // tsample:   buff[4],buff[5]
  buff[6]=0x00;          // reserved
  buff[7]=0x00;

  for (int i=0; i<NDATA/2; i++) {
    bptr[i]=(uint8_t)(255.0*(float)val[i]/(float)valmax);
  }
  
  LoRa.beginPacket();
  LoRa.print(devid);
  LoRa.write(buff,8+NDATA/2);
  LoRa.endPacket();
  Serial.println("Sent: vmax="+String(valmax)+" tsample="+ts);
  for (int i=0;i<NDATA/2; i++) {
    Serial.println(bptr[i]);
  }
}

// ---- ARDUINO STANDARD ----

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  
  lifesign(5,500);

#ifdef DEBUG
  Serial.begin(115200);
  int scnt=0;
  while(!Serial) {
    scnt++;  
    if (scnt>1000) 
      while(1) {
        lifesign(5,100);
        delay(1000);
      }
  }
  Serial.println("Starting LoRa Device...");
  
#endif
  
  LoRa.setPins(SS,RST,DI0);
  if (!LoRa.begin(BAND)) {
    
#ifdef DEBUG
    Serial.println("Starting LoRa failed!");
#endif

    while (1) lifesign(10,100);
  } else {
    LoRa.setTxPower(17,PABOOST);
    LoRa.setSyncWord(0x12);
  }
  lifesign(2,1000);
}

/*
Strategy:
 - send data
 - check if any gateway receive it
 - if handshake received but failed: resend
 - else ignore
*/

void loop() {
  int recstat;

  digitalWrite(LED_BUILTIN,HIGH);
  acquireData();
  digitalWrite(LED_BUILTIN,LOW);

  for (imul=0;imul<DLY;imul++) {
      
    if(data_avail) {
      sendcnt++;
      sendData();
      recstat=checkReceived();
      if(recstat==RECOK || recstat==RECIGNORE) data_avail=false;
      else if(sendcnt>5) {
        data_avail=false;
      }
      Serial.print("retry num: ");
      Serial.println(sendcnt);
    }    
    delay(8000);
  }
  
  counter++;
}

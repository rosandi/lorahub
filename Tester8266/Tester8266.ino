#include <SPI.h>
#include <LittleFS.h>
#include <LoRa.h>

#define SS      15
#define RST     16
#define DI0     4
#define BAND    921000000
#define PABOOST true 
#define SW    5

LittleFSConfig fscfg = LittleFSConfig();
FSInfo fsinfo;

String recvbuf;
int id=2202;
uint16_t counter=0;
bool fsok=false;

void lifesign(int n, int spd=1000) {
  for (int i=0;i<n;i++) {
    digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
    delay(spd);                       // wait for a second
    digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
    delay(spd);                       // wait for a second
  }  
}

void setupLoRa() {
  LoRa.setPins(SS,RST,DI0);
  if (!LoRa.begin(BAND)) {
    Serial.println("Starting LoRa failed!");
    while (true);
  }
  else LoRa.setTxPower(17,PABOOST);
  Serial.println("LoRa initiated. Waiting for message...");
}

void sendLoRa(String msg) {
  String ss="<"+String(id)+">n="+String(counter)+"&"+msg;
  LoRa.beginPacket();
  LoRa.print(ss);
  LoRa.endPacket();
}

void recvLoRa(int numtry) {
  String sid, stat, spar;

  int ntry=0;
  while (ntry<numtry) {
    int packetSize=LoRa.parsePacket();
    if (not packetSize) {
      delay(100);
    } else break;
    
    if(++ntry > numtry) return;   
  }
  
  char ch;
  recvbuf="";
  
  while(LoRa.available()) {
    ch=(char)LoRa.read();
    recvbuf+=ch;
    if(ch==' ') break;
    sid+=ch;
  }

  if (sid.toInt() != id) return;
  
  while(LoRa.available()) {
    ch=(char)LoRa.read();
    recvbuf+=ch;
    if(ch==' ') break;
    stat+=ch;
  }

  while(LoRa.available()) {
    ch=(char)LoRa.read();
    recvbuf+=ch;
    spar+=ch;
  }
 
  if (stat=="ok") lifesign(2,400);
  else if (stat=="ping") lifesign(10,200);

}

void setup() {
  Serial.begin(115200);
  while(not Serial);
  pinMode(LED_BUILTIN,OUTPUT);
  setupLoRa();
  
  fscfg.setAutoFormat(false);
  LittleFS.setConfig(fscfg);
  fsok = LittleFS.begin();

  if(fsok) {
    if(LittleFS.exists("/count")) {
      File file=LittleFS.open("/count","r");
      counter=file.parseInt();
      file.close();
      Serial.println("reading counter state: "+String(counter));
    }
  } else {
    Serial.println("failed to initialize filesystem");
  }
}

void loop() {
  Serial.println("Sending packet...");
  sendLoRa("msg=unpad");
  recvLoRa(10);
  
  if(recvbuf=="") {
    Serial.println("no packet received");
  } else {
    Serial.println("Received:");
    Serial.println(recvbuf);
  }
      
  if (fsok) {
    counter+=1;
    File file=LittleFS.open("/count","w");
    file.println(counter);
    file.close();
    Serial.println("saving counter state: "+String(counter));
    LittleFS.end();
  }
  
  Serial.println("sleep.");
  LoRa.end();
  ESP.deepSleep(0,RF_DISABLED);
}

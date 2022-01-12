
#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESP8266WebServer.h>
#include <ESP8266HTTPClient.h>
#include <ESP8266mDNS.h>
#include <SPI.h>
#include <LittleFS.h>
#include <LoRa.h>

#define SS      15
#define RST     16
#define DI0     4
#define BAND    921000000
#define PABOOST true 
#define SW    5
#define LED   LED_BUILTIN
#define DEVSSID "geohub"
#define DEVPWD "p4ssw0rd"

unsigned long tref=0; // millisecs after the last report
unsigned long timeref=0; // timestamp updated by server
int lightCnt=0;
int sigcnt=0;
long timethreshold=60000;

FS* FileSystem = &LittleFS;
LittleFSConfig fileSystemConfig = LittleFSConfig();
FSInfo fsinfo;

bool bitstream;
String ssid="";
String password="";
float lat=-6.93;
float lon=107.77;
String dataserv="https://dock.unpad.ac.id/api/commit.php";
String host = "geohub";
String apikey = "aggr-91f5b491/70571dece572c1f7";
String devid = "ZQAMNR3Q384XYLNW";
//String devid = "48ACKHE37UL6KKYO";
String data="empty";

uint16_t nodeid=0;
uint16_t nodecm=0;
String   cmdstr;
uint8_t  runcmd=0;

ESP8266WebServer server(80);
WiFiClientSecure client;

static bool fsOK;
static bool onConfig=true;
bool data_avail=false;
bool append_data=false;

void handleConfig() {
  String uri = ESP8266WebServer::urlDecode(server.uri());
  Serial.println("configuration request");
  
  if (not server.args()) {
    File file=FileSystem->open("/config.htm","r");
    server.streamFile(file, "text/html");
    file.close();
    return;
  }
  
  int rst=0;
  
  for (uint8_t i=0; i<server.args();i++) {
    Serial.print(server.argName(i)+"=");
    Serial.println(server.arg(i));
    
    if(server.argName(i)=="name") host=server.arg(i);
    else if(server.argName(i)=="ssid") ssid=server.arg(i);
    else if(server.argName(i)=="devid") devid=server.arg(i);
    else if(server.argName(i)=="password") password=server.arg(i);
    else if(server.argName(i)=="server") dataserv=server.arg(i);
    else if(server.argName(i)=="key") apikey=server.arg(i);
    else if(server.argName(i)=="lat") lat=server.arg(i).toFloat();
    else if(server.argName(i)=="lon") lon=server.arg(i).toFloat();
    else if(server.argName(i)=="rst") rst=server.arg(i).toInt();
  }
  
  File file=FileSystem->open("/config.txt","w");
  file.print("name=");file.println(host);
  file.print("devid=");file.println(devid);
  file.print("ssid=");file.println(ssid);
  file.print("password=");file.println(password);
  file.print("server=");file.println(dataserv);
  file.print("key=");file.println(apikey);
  file.print("lat=");file.println(lat);
  file.print("lon=");file.println(lon);
  file.close();
  
  String json="{\"status\":true,\"msg\":\"configuration\"}";
  server.send(200, "application/json", json);

  if(rst) {
    Serial.println("configuration file created\nRestarting");
    delay(5000);
    ESP.restart();
  }
}

void handleList() {

  if(server.args()) {
    for (uint8_t i=0; i<server.args();i++) {
      if(server.argName(i)=="rm") {
        String fname=server.arg(i);
        if(not fname.startsWith("/")) fname="/"+fname;
        FileSystem->remove(server.arg(i));
      }
    }
  }
  
  Dir dir=FileSystem->openDir("/");
  String flist="[";
  String fsize="[";

  while (dir.next()) { // first file only
    if(dir.fileName().endsWith(".dat")) {
      flist+=dir.fileName();
      fsize+=String(dir.fileSize());
      break;
    }
  }
  
  while (dir.next()) {
    if(dir.fileName().endsWith(".dat")) {
      flist+=","+dir.fileName();
      fsize+=","+String(dir.fileSize());
    }
  }

  String resp="{\"file\":"+flist+"],\"size\":"+fsize+"]}";
  server.send(200,"application/json", resp);
}

void handleDump() {
  Dir dir=FileSystem->openDir("/");
  String txt="-- data dump --\n\n";
  
  while (dir.next()) {
    if(dir.fileName().endsWith(".dat")) {
      String chn=dir.fileName();
      chn.replace(".dat","");
      txt+="channel: "+chn+"\n";
      File file=FileSystem->open(dir.fileName(),"r");
      while(file.available()) txt+=(char) file.read();
      txt+="\n";
      file.close();
    }
  }

  server.send(200,"text/plain", txt);  
}

/*
 * request format:
 * set?node=12345&cmd=command&par=params
 */

void handleSet() {
  for (uint8_t i=0; i<server.args();i++) {
    if(server.argName(i)=="node") nodecm=server.arg(i).toInt();
    else if(server.argName(i)=="cmd") cmdstr+=server.arg(i);
    else if(server.argName(i)=="par") cmdstr+=" "+server.arg(i);
    else if(server.argName(i)=="run") runcmd=server.arg(i).toInt();
    else if(server.argName(i)=="append") {
      if((server.arg(i)=="yes") or (server.arg(i)=="1")) append_data=true;
      else append_data=false;
    }
  }
  
  String resp="set command for ";
  resp+=String(nodecm)+": "+cmdstr;
  server.send(200,"text/plain", resp); 
}

void handleNotFound() {
  String uri = ESP8266WebServer::urlDecode(server.uri());
  Serial.println(uri);
  if (uri.endsWith("/")) uri+="index.htm";
  
  if (FileSystem->exists(uri)) {
    File file=FileSystem->open(uri,"r");
    String contentType = mime::getContentType(uri);
    if (server.streamFile(file, contentType) != file.size()) {
      Serial.println("Transfer problem");
    } else {
      Serial.println(uri+" sent");
    }
  } else {
    String json="{\"status\":false,\"msg\":\"file not found\"}";
    server.send(200, "application/json", json);
  }
}

void setupFS() {
  fileSystemConfig.setAutoFormat(false);
  FileSystem->setConfig(fileSystemConfig);
  fsOK = FileSystem->begin();
  FileSystem->info(fsinfo);
  
  if (not fsOK) {
    Serial.println("Filesystem init failed!");
    while (true);
  }
  
  Dir dir=FileSystem->openDir("/");
  String out="";
  while (dir.next()) {
    out+=dir.fileName()+" : ";
    out+=dir.fileSize();
    out+="\n";
  }
  
  Serial.println(out);
  Serial.print("Total bytes: ");
  Serial.println(fsinfo.totalBytes);
  Serial.print("Used: ");
  Serial.println(fsinfo.usedBytes);
  Serial.println("filesystem OK");

  if (FileSystem->exists("/config.txt")) {
    Serial.println("Configuration exists. Reading...");
    File file=FileSystem->open("/config.txt", "r");
    
    String token="";
    char ch;
    
    while (file.available()) {
      String item=file.readStringUntil('=');
      String value=file.readStringUntil('\n');
      item.trim();
      value.trim();
      if (item == "ssid") ssid=value;
      else if (item == "password") password=value;
      else if (item == "server") dataserv=value;
      else if (item == "name") host=value;
      else if (item == "devid") devid=value;
      else if (item == "key") apikey=value;
      else if (item == "lat") lat=value.toFloat();
      else if (item == "lon") lon=value.toFloat();
      else {
        Serial.print("invalid token: ");
        Serial.println(item+"="+value);
      }
    }

    file.close();
  }

  if (ssid == "" or password == "") onConfig=true;
  else onConfig = false;
  
}

void setupWiFi() {
  if ( onConfig ) {
    // act as AP
    Serial.println("Configuration access point...");
    ssid=DEVSSID;
    password=DEVPWD;
    WiFi.softAP(ssid,password);
    IPAddress myIP = WiFi.softAPIP();
    Serial.print("AP IP address: ");
    Serial.println(myIP);
    Serial.println("Waiting for config request");
    
    if (MDNS.begin(host)) {
      MDNS.addService("http", "tcp", 80);
      Serial.print(F("url http://"));
      Serial.print(host);
      Serial.println(F(".local"));
    }
    
    server.begin();
    server.on("/config",handleConfig);
    server.on("/list",handleList);
    server.on("/dump",handleDump);
    server.on("/set",handleSet);
    server.onNotFound(handleNotFound); // fetch data by default
     
  } else {
    
    // serving http client while acting as hub is complicated in 8266
    // just do it simple: just client no web request accepted here!
    
    Serial.printf("Connecting to %s\n", ssid);
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    
    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
    }
    
    Serial.println("");
    Serial.print(F("Connected! IP address: "));
    Serial.println(WiFi.localIP());
    
    client.setInsecure();
  }
}

void checkFactoryReset()  { 
  // reset settings by pressing button at boot until light's ON
  if (digitalRead(SW)) return;
   
  digitalWrite(LED,HIGH);
  Serial.println("device reset request detected");
  int icnt=0;
  for (icnt=0;icnt<50;icnt++) {
    delay(100);
    if(digitalRead(SW)) break;
  }
  if(icnt==50) {
    Serial.println("configuration reset");
    Serial.println("please release button");
    while(digitalRead(SW)==0) digitalWrite(LED,LOW);
    
    if(FileSystem->exists("/config.txt"))
      FileSystem->remove("/config.txt");
    
    delay(2000);
    ESP.restart();
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

void sendLoRa(uint16_t id, String msg) {
  LoRa.beginPacket();
  String ss=String(id)+" "+msg;
  LoRa.print(ss);
  LoRa.endPacket();
  Serial.println("sending msg: "+ss);
}

String checkChar(int d) {
  char ch=(char) d;
  if(ch=='\n') return "\\n";
  if(ch=='\t') return "\\t";
  if(ch=='=') return ":";
  if(ch=='&') return ",";
  if(ch<' ') return String("{")+String(d,HEX)+"}";
  if(ch>'~') return String("{")+String(d,HEX)+"}";
  return String(ch);
}

void getData() {
  data_avail=false;
  nodeid=0;
  data="";
  char ch=0x0;
  int dd=0;
  bool firstchar=true;
  
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    
    while (LoRa.available()) {
      ch=(char)LoRa.read();
      if (firstchar) {
        if(ch!='<') {
          Serial.println("Bad packet received");
          while (LoRa.available()) LoRa.read();
          return;
        }
        firstchar=false;
        continue;
      }

      if(ch=='>') break;
      data+=ch;
    }

    nodeid=data.toInt();
    data="";
    
    if (not nodeid) {
      Serial.println("Bad packet received");
      // empty buffer
      while (LoRa.available()) LoRa.read();
      return;
    }
    
    firstchar=true;
    while (LoRa.available()) {
      dd=LoRa.read();
      if(firstchar && dd==0x00) { // bitstream mark
        bitstream=true;
        firstchar=false;
        continue;
      }
      
      if(bitstream) {
        data+=String(dd)+",";
      } else {
        data+=checkChar(dd);
      }
      
      firstchar=false;
    }

    if(bitstream) data+="rssi:";
    else data+=",rssi:";
    data+=String(LoRa.packetRssi());

    if(nodeid==nodecm) {
      sendLoRa(nodeid, cmdstr);
      runcmd--;
      if (runcmd<1) nodecm=0;
    } else {
      sendLoRa(nodeid,"ok");
    }
    
    char fmode[]="w";
    if (append_data) fmode[0]='a';
    
    File file=FileSystem->open(String(nodeid)+".dat", fmode);
    file.println(data);
    file.close();
    
    Serial.print("Received from: ");
    Serial.println(nodeid);
    Serial.println(data);
    data_avail=true;
  }
}

void sendData() {
  String srvreq="key="+apikey+"&dev="+devid+"&node="+String(nodeid)+"&data="+data;
  Serial.println(srvreq);
  HTTPClient http;
  if(http.begin(client, dataserv)) {
    http.addHeader("Content-Type", "application/x-www-form-urlencoded");
    int respnum = http.POST(srvreq);
    
    if (respnum==HTTP_CODE_OK || respnum==HTTP_CODE_MOVED_PERMANENTLY) {
      Serial.println(http.getString());
    } else {
      Serial.println("request failure");
    }
  }
  http.end();
}

void setup(void) {
  pinMode(SW, INPUT_PULLUP);
  pinMode(LED, OUTPUT);

  Serial.begin(115200);
  while(not Serial);
  Serial.println("BOOTING...");
  setupFS();
  checkFactoryReset();
  setupWiFi();
  setupLoRa();
  tref=millis();
}

void loop(void) {
  if (onConfig) {
    server.handleClient();
    MDNS.update();
  } else {
    if (data_avail) {
      lightCnt=500;
      sendData();
    }
  }
  getData();
  if (lightCnt>0) {lightCnt--; digitalWrite(LED, LOW);}
  else digitalWrite(LED,HIGH);
  if (not digitalRead(SW)) lightCnt=2000;
}

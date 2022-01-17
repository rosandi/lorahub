#include <SPI.h>
#include <LoRa.h>
#include <LowPower.h>

#include "DifferentialADC.c"
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

#define DEBUG

#ifdef DEBUG
uint16_t DLY=10;
#else
uint16_t DLY=480;
#endif

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

lifesign(int n, int spd=1000) {
  for (int i=0;i<n;i++) {
    digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
    delay(spd);                       // wait for a second
    digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
    delay(spd);                       // wait for a second
  }
}

// ---- FFT Part ----

#define NTAB 50
#define PI 3.141592653589793
#define PP 6.283185307179586
#define PH 1.570796326794897

#define log2N 8
#define NDATA 128
#define log2N 7
#define AMAX 255

int16_t val[NDATA];
int16_t ival[NDATA];
// warning! reuse buffer
int8_t *buff=(int8_t*)ival;
uint8_t *bptr=buff+8;

int16_t valmax=-1;
int i,j,k,s,m,m2;
struct {float re; float im;} w, wm, t, u;  

unsigned char sintab[NTAB] = 
{0, 16, 31, 47, 63, 78,
 93, 108, 122, 136, 149, 162,
 174, 185, 196, 206, 215, 223,
 230, 237, 242, 246, 250, 252,
 254, 255, 254, 252, 250, 246,
 242, 237, 230, 223, 215, 206,
 196, 185, 174, 162, 149, 136,
 122, 108, 93, 78, 63, 47, 31, 16};

float rsin(float x) {
    int qu=1;
    while (x>PP) x-=PP;
    while (x<0) x+=PP;
    if(x>=PI) {x-=PI;qu=-1;}

    int xx=(int)(x/PP);
    x-=(PP*(float)xx);
    x*=100.0/PP;
    xx=(int)x;
    float ofs=(x-(float)xx);
    float b=xx<(NTAB-1)?(float)sintab[xx+1]:(float)sintab[0];
    b=qu*(sintab[xx]+(b-sintab[xx])*ofs);
    return b/AMAX;
}

float rcos(float x) {
    return rsin(x+PH);
}

void fft() {
    // bit reversal       
    for (i = 0; i < NDATA; ++i) {
        k=0;
        s=i;       
        for (j = 0; j < log2N; j++) {
            k <<= 1;
            k |= (s & 1);
            s >>= 1;
        }        
        ival[i]=val[k];
    }
    
    for (i=0;i<NDATA;i++) {
         val[i]=ival[i];
        ival[i]=0;
    }
    
    for (s = 1; s <= log2N; ++s) {
        m = 1 << s; 
        m2 = m >> 1;
        w.re=1;
        w.im=0;
        
        wm.re=rcos(PI/m2);
        wm.im=-rsin(PI/m2);

        for (j = 0; j < m2; ++j) {
            for (k = j; k < NDATA; k += m) {
                t.re=(w.re*val[k+m2] - w.im*ival[k+m2]);
                t.im=(w.im*val[k+m2] + w.re*ival[k+m2]);
                u.re=val[k];
                u.im=ival[k];
                 val[k] = (int16_t)(u.re + t.re);
                ival[k] = (int16_t)(u.im + t.im);
                 val[k+m2] = (int16_t)(u.re - t.re);
                ival[k+m2] = (int16_t)(u.im - t.im);
            }
            t.re=(w.re*wm.re-w.im*wm.im);
            t.im=(w.im*wm.re+w.re*wm.im);
            w.re=t.re;
            w.im=t.im;
        }
    }
    
    valmax=-1;
    val[0]=0; // remove dc offset
    for (i=0;i<NDATA;i++) {
        if(val[i]<0) val[i]=-val[i];
        if(valmax<val[i]) valmax=val[i];
    }
}

// ----------

int checkVoltage() {
  analogReference(DEFAULT);
  int vbat=analogRead(BATSTAT);
  return(vbat);
}

void acquireData() {
  int ovr;

  analogReferenceDiff(INTERNAL);
  code=analogGetCode(A4,A5,200);
  analogSetDiffCode(code);

#ifdef DEBUG
  Serial.println("data acquisition...");
#endif

  // throw out first junk data
  for(ovr=0;ovr<OVERSMP;ovr++) analogReadDiff();

  ts=millis();
  
  for(i=0;i<NDATA;i++) {
    long d=0;
    for(ovr=0;ovr<OVERSMP;ovr++) 
      d+=signValue(analogReadDiff());
    val[i]=(int16_t)(d/OVERSMP);
  }

  fft();
  
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

#ifdef DEBUG
  Serial.println("Sent: vmax="+String(valmax)+" tsample="+ts);
  for (int i=0;i<NDATA/2; i++) {
    Serial.println(bptr[i]);
  }
#endif;
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

#ifdef DEBUG
  digitalWrite(LED_BUILTIN,HIGH);
  acquireData();
  digitalWrite(LED_BUILTIN,LOW);
#else
  acquireData();
#endif

  for (imul=0;imul<DLY;imul++) {
      
    if(data_avail) {
      sendcnt++;
      sendData();
      recstat=checkReceived();
      if(recstat==RECOK || recstat==RECIGNORE) data_avail=false;
      else if(sendcnt>5) {
        data_avail=false;
      }
#ifdef DEBUG      
      Serial.print("retry num: ");
      Serial.println(sendcnt);
#endif    
    }
    
#ifdef DEBUG
    delay(8000);
#else
    LowPower.idle(SLEEP_8S, ADC_OFF, 
                  TIMER4_OFF, TIMER3_OFF, 
                  TIMER1_OFF, TIMER0_OFF, 
                  SPI_OFF, USART1_OFF, 
                  TWI_OFF, USB_OFF); 
#endif

  }
  
  counter++;
}

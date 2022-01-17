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
#define NDATA 128
#define log2N 7

int16_t val[NDATA];
int16_t valmax=-1;
uint16_t gwid=0;

int code;
unsigned long ts;

// UBAH DEVICE ID!!!
uint16_t device_id=101;

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

#define q 8
#define N (1<<q)

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
    float qu=1;
    while (x>PP) x-=PP;
    while (x<0) x+=PP;
    
    if(x>=PI) {x-=PI;qu=-1;}

    int xx=(int)(x/PP);
    x-=(PP*(float)xx);
    x*=100.0/PP;
    xx=(int)x;
    float ofs=(x-(float)xx);
    float a=(float)sintab[xx];
    float b=xx<(NTAB-1)?(float)sintab[xx+1]:(float)sintab[0];
    a*=qu/255.0;
    b*=qu/255.0;
    a=a+(b-a)*ofs;
    return a;
}

float rcos(float x) {
    return rsin(x+PH);
}

float rsqrt(const float x) {
  union
  {
    int32_t i;
    float x;
  } u;
  u.x = x;
  u.i = (1<<29) + (u.i >> 1) - (1<<22); 
  u.x =       u.x + x/u.x;
  u.x = 0.25f*u.x + x/u.x;
  return u.x;
}

unsigned int bitReverse(unsigned int x, int log2n) {
    int n = 0;
    for (int i = 0; i < log2n; i++) {
        n <<= 1;
        n |= (x & 1);
        x >>= 1;
    }
    return n;
}

void fft(int16_t a[], int log2n) {
    int n = (1<<log2n); 
    struct {float re; float im;} A[N], w, wm, t, u;

    for (unsigned int i = 0; i < n; ++i) {
        int rev=bitReverse(i, log2n);
        A[i].re=(float) a[rev];
        A[i].im=0;
    }
    
    for (int s = 1; s <= log2n; ++s) {
        int m = 1 << s; 
        int m2 = m >> 1;
        
        w.re=1;
        w.im=0;
        wm.re=rcos(PI/m2);
        wm.im=-rsin(PI/m2);

        for (int j = 0; j < m2; ++j) {
            for (int k = j; k < n; k += m) {
                t.re=w.re*A[k+m2].re - w.im*A[k+m2].im;
                t.im=w.im*A[k+m2].re + w.re*A[k+m2].im;
                u.re=A[k].re;
                u.im=A[k].im;
                A[k].re = u.re + t.re;
                A[k].im = u.im + t.im;
                A[k+m2].re = u.re - t.re;
                A[k+m2].im = u.im - t.im;
            }
            t.re=w.re*wm.re-w.im*wm.im;
            t.im=w.im*wm.re+w.re*wm.im;
            w.re=t.re;
            w.im=t.im;
        }
    }
    valmax=-1;
    for (int i=0;i<n;i++) {
        a[i]=(int16_t) rsqrt(A[i].re*A[i].re + A[i].im*A[i].im);
        if(valmax<a[i]) valmax=a[i];
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
  
  for(int i=0;i<NDATA;i++) {
    long d=0;
    for(ovr=0;ovr<OVERSMP;ovr++) 
      d+=signValue(analogReadDiff());
    val[i]=(int16_t)(d/OVERSMP);
  }

  fft(val, log2N);
  
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
  String msg="<"+String(device_id)+">";
  
  // [bin][len][maxlo][maxhi][chk][rsv*4][data...]
  
  uint8_t buff[8+NDATA/2];
  uint8_t *bptr=buff+8;
  
  buff[0]=0x00;        // bitstream mark
  buff[1]=NDATA/2;     // length;
  buff[2]=(uint8_t)(valmax&0x00FF); // max value
  buff[3]=(uint8_t)(valmax>>8);
  buff[4]=0x00;  // reserved for data integrity checking
  buff[5]=0x00;
  buff[6]=0x00;
  buff[7]=0x00;

  for (int i=0; i<NDATA/2; i++) {
    int32_t vv=255*(int32_t)val[i];
    bptr[i]=(uint8_t)vv/valmax;
  }
  
  LoRa.beginPacket();
  LoRa.print(msg);
  LoRa.write(buff,8+NDATA/2);
  LoRa.endPacket();

#ifdef DEBUG
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

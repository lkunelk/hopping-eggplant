#include <Servo.h>
#include <util/atomic.h>

Servo S;
// deadband: 494-530 (482us-517us)
#define AMIN 104
#define AMASK 0xFFF8
#define ASTRIDE 8
#define LG_ASTRIDE 3
#define ANUM 55

// conversion
#define DCOEFFSNUM 7
#define DCOEFFSDELAY 3
const float DCOEFFS[DCOEFFSNUM] = { -0.01666667f,  0.15f      , -0.75f      ,  0.0f        ,  0.75f      , -0.15f      ,  0.01666667f };

#define HANNINGNUM 7
#define HANNINGDELAY 3
const float HANNING[HANNINGNUM] = { 0.0f        , 0.08333333f, 0.25f      , 0.33333333f, 0.25f      , 0.08333333f, 0.0f };

float ALOOKUP[ANUM] = {
  3.04687500e-01f,  2.90039062e-01f,  2.68798828e-01f,  2.39990234e-01f,
  2.16796875e-01f,  1.92382812e-01f,  1.62841797e-01f,  1.40869141e-01f,
  1.19628906e-01f,  9.83886719e-02f,  7.39746094e-02f,  5.68847656e-02f,
  4.68750000e-02f,  3.19824219e-02f,  2.34375000e-02f,  1.70898438e-02f,
  9.27734375e-03f,  3.17382812e-03f, -2.44140625e-04f, -3.17382812e-03f,
  -6.59179688e-03f, -1.07421875e-02f, -1.46484375e-02f, -1.83105469e-02f,
  -2.19726562e-02f, -2.58789062e-02f, -3.07617188e-02f, -3.66210938e-02f,
  -4.24804688e-02f, -4.63867188e-02f, -5.00488281e-02f, -5.41992188e-02f,
  -5.81054688e-02f, -6.17675781e-02f, -6.54296875e-02f, -7.00683594e-02f,
  -7.54394531e-02f, -8.03222656e-02f, -8.39843750e-02f, -8.76464844e-02f,
  -9.25292969e-02f, -9.83886719e-02f, -1.03515625e-01f, -1.07666016e-01f,
  -1.11328125e-01f, -1.15966797e-01f, -1.21826172e-01f, -1.26953125e-01f,
  -1.30859375e-01f, -1.35498047e-01f, -1.41113281e-01f, -1.46728516e-01f,
  -1.52343750e-01f, -1.57714844e-01f, -1.62841797e-01f
};

#define LOOP_FREQ 32 // Hz
#define LOOP_PERIOD 31250 // us
uint16_t aint = 0;

#define ABUFNUM 32
#define ABUFMASK 31
volatile int16_t abuf[ABUFNUM] = { 0 };
volatile uint8_t abuf_idx = 0xFF;
int16_t abuf_stash[ABUFNUM] = { 0 };
uint8_t abuf_idx_stash = 0, last_abuf_idx_stash = 0; // hope it doesn't lap itself
float abuf_rad[ABUFNUM] = { 0 };

#define DEADBAND_PLUS 20
#define DEADBAND_MINUS 20

// 256-units
#define PID_I 0.07071068f
#define PID_P 1.03875926f
#define PID_D 1.43485934f

#define UMIN -30
#define UMAX 110
volatile int16_t u = 0; // ESC input
int16_t vfly = 0; // flywheel speed in 256-units of max-speed
float ci = 0.0f;
#define T0  // units of acceleration in max-speeds per s
// timing + -> flywheel CCW accel -> arm CW torque
// arm CCW -> analog - -> angle +

ISR(ADC_vect) {
  abuf[(++abuf_idx) & ABUFMASK] = ADC;
  // (((uint16_t)ADCH) << 8) | ADCL; // <- surprisingly this didn't work
  // ^ maybe those aren't volatile, or the cast made it non-volatile
  // casting as volatile uint16_t didn't work either
  digitalWrite(13, abuf_idx & 1);
  ADCSRA |= 1 << 4;
}

ISR(TIMER2_COMPA_vect) {
  ADCSRA |= 1 << 6; // start ADC conversion
}

void setup() {
  // put your setup code here, to run once:
  S.attach(2);
  Serial.begin(9600);
  pinMode(13, OUTPUT);
  
  // configure ADC
  ADMUX |= (0b01 << 6) | 0b0011; // A3 input
  ADCSRA |= (0b11 << 6) | (1 << 3 | 0b110); // 64 prescaler (conversion takes 104us)

  // note: MUST USE TIMER2. Arduino millis uses timer0, servo uses timer1.
  // also can't auto-trigger: ADC doesn't accept triggers from timer2. Of course not.
  TCCR2A |= 0b010; // CTC OCR2A top
  TCCR2B |= 0b111; // 1024 prescaling
  OCR2A = 8; // trigger every 1 ms
  TIMSK2 |= 0b010;
  
//  pinMode(11, OUTPUT);
//  analogWrite(11, 50);
//  S.writeMicroseconds(1100);
}

// estimating velocity based on a model of torque (from current) and saturation velocity
// (which is non-linear with velocity possibly due to resistive torque that is proportional to velocity)
// 1520 -> 0Hz, 1607ms -> 122Hz, 1740ms -> 183Hz, saturates just above this
void loop() {
//  return; 
//  uint16_t b = map(a_raw, 0, 1023, 1200, 1900);
//  S.writeMicroseconds(b);
//  Serial.println(b);
//  digitalWrite(13, HIGH);
//  delay(10);
//  return;
  // estimate pid
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    memcpy(abuf_stash, abuf, sizeof(abuf[0]) * ABUFNUM);
    abuf_idx_stash = abuf_idx;
  }
  
  // I, analog -> rad conversion
  {
    uint8_t j = 0;
    float cibuf = 0.0f;
    for(uint8_t i = last_abuf_idx_stash; (i & ABUFMASK) != (abuf_idx_stash & ABUFMASK); i++, j++) {
      uint16_t a_raw = constrain((int16_t)abuf_stash[i & ABUFMASK] - AMIN, 0, ASTRIDE * (ANUM - 2));
      
      uint16_t aidx = a_raw >> LG_ASTRIDE;
      int8_t ares = a_raw - (aidx << LG_ASTRIDE);
      float a = ALOOKUP[aidx];
      abuf_rad[i & ABUFMASK] = ((ares * (ALOOKUP[aidx + 1] - ALOOKUP[aidx])) / ASTRIDE) + ALOOKUP[aidx];
      cibuf += a;
    }
    if(j > 0) {
      cibuf /= j;
      ci += cibuf;
      last_abuf_idx_stash = abuf_idx_stash;
    }
  }
  
  // D, P
  float cdbuf[DCOEFFSNUM] = { 0 };
  // INVARIANT: ABUFNUM > HANNINGNUM + DCOEFFSNUM
  for(uint16_t i0 = (uint16_t)abuf_idx_stash + ABUFNUM - HANNINGNUM - DCOEFFSNUM, j0 = 0; j0 < DCOEFFSNUM; i0++, j0++) {
    for(uint16_t i = i0, j = 0; j < HANNINGNUM; i++, j++) {
      cdbuf[j0] += abuf_rad[i & ABUFMASK] * HANNING[j];
    }
  }
  
  float cp = cdbuf[DCOEFFSNUM - DCOEFFSDELAY - 1];
  
  float cd = 0;
  for(uint8_t i = 0; i < DCOEFFSNUM; i++) {
    cd += cdbuf[i] * DCOEFFS[i];
  }
  
  float T = PID_P * cp + PID_D * cd + PID_I * ci;
  
  Serial.print(cp, 5); Serial.print('\t'); Serial.print(cd, 5); Serial.print('\t'); Serial.print(ci, 5); Serial.print('\t'); Serial.println(T, 5);
//  delay(50);
  
  // // estimate vfly
  
  // abuf[(abuf_idx++) & ABUFMASK] = a;
  // u[0] += (P * a) >> PPOINT;
  // u[0] = constrain(u[0], UMIN, UMAX);
  // S.writeMicroseconds(1500 + (u[0] * (u[0] > 0 ? 1 : 10) + (u[0] > 0 ? DEADBAND_PLUS : -DEADBAND_MINUS))); // step over deadband
  // Serial.print(a);
  // Serial.print('\t');
  // Serial.println(u[0]);
  // delayMicroseconds(LOOP_PERIOD);
//  digitalWrite(13, HIGH);
}

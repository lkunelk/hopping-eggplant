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
#define DCOEFFSNUM 5
#define DCOEFFSDELAY 0
const float DCOEFFS[DCOEFFSNUM] = { 0.250f,-1.333f,3.000f,-4.000f,2.083f }; // { 0.167f,-1.200f,3.750f,-6.667f,7.500f,-6.000f,2.450f };

#define HANNINGNUM 9
#define HANNINGDELAY 4
const float HANNING[HANNINGNUM] = { 0.000f,0.037f,0.125f,0.213f,0.250f,0.213f,0.125f,0.037f,0.000f };

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
#define DEADBAND_MINUS 0

// controller output is flywheel acceleration  in units of m/s^2
#define PID_I 1003.1f // 0.07071068f
#define PID_P 39000.0f // 14736.6f // 14736.6f // 1.03875926f
#define PID_D 4600.0f // 20356.0f // 20356.0f // 1.43485934f

#define UMID 1500
#define UMIN 1140
#define UMAX 1860
#define BAND_MINUS 360 // DERIVED: UMID - DEADBAND_MINUS - UMIN
#define BAND_PLUS 340 // DERIVED: UMAX - DEADBAND_PLUS - UMID
#define USAFE 1900
volatile uint16_t u = 1500; // ESC input
float vfly = 146.73f; // flywheel speed in rad/s
float ci = 0.0f;
#define A0 (3.864E3f) // units of flywheel acceleration in rad/s^2
#define VMAX (1.394E3f) // units of rad/s (assume flywheel dominates)
#define VSAFE (600.0f)
#define IFLY (70.49E-6f) // units of kg-m
// timing + -> flywheel CCW accel -> arm CW torque
// arm CCW -> analog - -> angle +

uint8_t stopped_ = 0, initted_ = 0;

// TODO it's probably a lot faster to do this in units of ms directly
#define T_ADC 0.001024f
#define A0_T_ADC 506.464 // 126.616f // DERIVED: T_ADC * A0 * VFLY_UPDATE_T
#define VFLY_UPDATE_T 128
//#define LG_VFLY_UPDATE_T 5
volatile uint8_t vfly_update_ticks = 0;
ISR(ADC_vect) {
  abuf[(++abuf_idx) & ABUFMASK] = ADC + 8;
  // (((uint16_t)ADCH) << 8) | ADCL; // <- surprisingly this didn't work
  // ^ maybe those aren't volatile, or the cast made it non-volatile
  // casting as volatile uint16_t didn't work either
//  digitalWrite(13, abuf_idx & 1);
  ADCSRA |= 1 << 4;

  // update vfly estimate (for lack of a better place to do this)
  digitalWrite(13, ((vfly_update_ticks) & (VFLY_UPDATE_T - 1)) == 0);
  if(initted_ && ((++vfly_update_ticks) & (VFLY_UPDATE_T - 1)) == 0) {
    if(u > UMID) {
      vfly += ((u - (UMID + DEADBAND_PLUS)) / (float)BAND_PLUS - (vfly / VMAX)) * A0_T_ADC;
    }
    else {
      vfly -= ((UMID - DEADBAND_MINUS - u) * A0_T_ADC) / BAND_MINUS;
      vfly = fmax(0.0f, vfly);
    }
  }
}

volatile uint8_t cnt = 0;
ISR(TIMER2_COMPA_vect) {
  ADCSRA |= 1 << 6; // start ADC conversion
  // 115bpm -> 1.916Hz - 2^5 upscale -> 61.3Hz - 1024x upscale - 256x upscale -> 16MHz (yes)
}

void setup() {
  // put your setup code here, to run once:
  S.attach(2);
  Serial.begin(9600);
  pinMode(13, OUTPUT);

  // disable interrupts while setting up the conversions and spinning the wheel up
  cli();
  
  // configure ADC
  ADMUX |= (0b01 << 6) | 0b0011; // A3 input
  ADCSRA |= (0b11 << 6) | (1 << 3 | 0b110); // 64 prescaler (conversion takes 104us)

  // note: MUST USE TIMER2. Arduino millis uses timer0, servo uses timer1.
  // also can't auto-trigger: ADC doesn't accept triggers from timer2. Of course not.
  TCCR2A |= 0b11; // CTC OCR2A top
  TCCR2B |= (1 << 3) | 0b111; // 1024 prescaling
  OCR2A = 15; // subdivide further by ~16x~ 4x, get ~1ms~ 256us sampling period
  TIMSK2 |= 0b010;
  
  sei(); // begin collecting data

  pinMode(10, INPUT_PULLUP);
//  pinMode(11, OUTPUT);
//  analogWrite(11, 40);
  S.writeMicroseconds(1560); // initialize at middle velocity
//  delay(5000);
//  
//  delay(10);
}

// estimating velocity based on a model of torque (from current) and saturation velocity
// (which is non-linear with velocity possibly due to resistive torque that is proportional to velocity)
// 1520 -> 0Hz, 1607ms -> 122Hz, 1740ms -> 183Hz, saturates just above this
uint8_t debouncer[32] = { 0 };
uint8_t debouncer_idx = 0;
uint8_t ticks = 0;
void loop() {
//  return; 
//  uint16_t b = map(a_raw, 0, 1023, 1200, 1900);
//  S.writeMicroseconds(b);
//  Serial.println(b);
//  digitalWrite(13, HIGH);
//  delay(10);
//  return;
  // estimate pid
  debouncer[(++debouncer_idx) & 31] = !digitalRead(10);
  uint8_t s = 0;
  for(uint8_t i = 0; i < 32; i++) {
    s += debouncer[i];
  }
  
  if(s > 30 && !stopped_) {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      memcpy(abuf_stash, abuf, sizeof(abuf[0]) * ABUFNUM);
      abuf_idx_stash = abuf_idx;
    }
    
    // I, analog -> rad conversion
    {
      float cibuf = 0.0f;
      uint8_t i = initted_ ? (last_abuf_idx_stash + ABUFNUM - 1) : 0;
      uint8_t j = 0;
      uint8_t imax = initted_ ? abuf_idx_stash : ABUFMASK;
      for(; (i & ABUFMASK) != (imax & ABUFMASK); i++, j++) {
        uint16_t a_raw = constrain((int16_t)abuf_stash[(i + 1) & ABUFMASK] - AMIN, 0, ASTRIDE * (ANUM - 2));
        
        uint16_t aidx = a_raw >> LG_ASTRIDE;
        int8_t ares = a_raw - (aidx << LG_ASTRIDE);
        float a = ALOOKUP[aidx];
        abuf_rad[(i + 1) & ABUFMASK] = ((ares * (ALOOKUP[aidx + 1] - ALOOKUP[aidx])) / ASTRIDE) + ALOOKUP[aidx];
        cibuf += a;
      }
      if(j > 0) {
        cibuf /= j;
        ci += cibuf;
      }
      last_abuf_idx_stash = abuf_idx_stash;
    }
    
    // D, P
    float cdbuf[DCOEFFSNUM] = { 0 };
    // INVARIANT: ABUFNUM > HANNINGNUM + DCOEFFSNUM
    for(uint16_t i0 = (uint16_t)abuf_idx_stash + ABUFNUM - HANNINGNUM - DCOEFFSNUM, j0 = 0; j0 < DCOEFFSNUM; i0++, j0++) {
      for(uint16_t i = i0, j = 0; j < HANNINGNUM; i++, j++) {
        cdbuf[j0] += abuf_rad[i & ABUFMASK] * HANNING[j];
      }
    }
    
    float cp = cdbuf[DCOEFFSNUM - 1]; // DCOEFFSDELAY - 
    
    float cd = 0;
    for(uint8_t i = 0; i < DCOEFFSNUM; i++) {
      cd += cdbuf[i] * DCOEFFS[i];
    }
    cd /= T_ADC;
    
    float A = PID_P * cp + PID_D * cd + PID_I * ci;
  
    if(A > 0) {
      if(vfly > VSAFE) {
        stopped_ = 1;
        return;
      }
      u = UMID + DEADBAND_PLUS + (vfly / VMAX + A * 1.4f / A0) * BAND_PLUS;
      u = min(USAFE, u);
    }
    else {
      // braking
      // simple ratio
      float b = -fmax(A * 6.0f, -A0) / A0;
      u = UMID - DEADBAND_MINUS  - sqrt(b) * BAND_MINUS;
      u = max(UMIN, u);
    }
    S.writeMicroseconds(u);

    Serial.print(A, 4); Serial.print('\t'); Serial.print(u); Serial.print('\t'); Serial.println(abuf_rad[abuf_idx_stash & ABUFMASK], 4); /* Serial.print('\t'); Serial.print(ci, 4); Serial.print('\t');
    Serial.print(vfly, 4); Serial.print('\t'); Serial.print(u); Serial.print('\t'); Serial.println(A, 4); */
    
    initted_ = 1;
  }
  else if(initted_) {
    // reset state
    ci = vfly = u = 0;
    stopped_ = 1;
    S.writeMicroseconds(UMIN);
  }
  else {
    if((ticks++) & 31 == 0) {
      S.writeMicroseconds(1560);
    }
    delayMicroseconds(1500);
  }

//  delay(50);
  
  // // estimate vfly
  
  
  // abuf[(abuf_idx++) & ABUFMASK] = a;
  // u[0] += (P * a) >> PPOINT;
  // u[0] = constrain(u[0], UMIN, UMAX);
  // S.writeMicroseconds(UMID + (u[0] * (u[0] > 0 ? 1 : 10) + (u[0] > 0 ? DEADBAND_PLUS : -DEADBAND_MINUS))); // step over deadband
  // Serial.print(a);
  // Serial.print('\t');
  // Serial.println(u[0]);
  // delayMicroseconds(LOOP_PERIOD);
//  digitalWrite(13, HIGH);
}

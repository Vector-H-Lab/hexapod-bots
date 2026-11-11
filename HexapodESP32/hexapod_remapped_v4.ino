#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <avr/interrupt.h>
#include <SoftwareSerial.h>

#include <math.h>

// ---------------------------------------------------------------------------
// GAIT: tripod walk with straight-line feet.
//   Hip    = swings the leg forward/back.
//   Middle = lifts the leg during the swing, nothing else.
//   Lower  = leans a few degrees in or out during the stride so each foot
//            travels in a straight line, parallel to the direction of travel.
// Why the lower joints move again: the corner legs point out at 45 degrees, so
// a hip-only swing moves their feet diagonally. The two corner feet on one side
// then twist the body instead of pushing it forward, and the feet slip.
// Lifting still uses only the middle joint, so feet do not tuck under on a lift.
// ---------------------------------------------------------------------------
const float HIP_SWING_DEG  = 15.0f; // middle-leg hip swing EACH WAY at full walk stick.
                                    // Raise toward 20 for longer steps once it walks straight.
const float TURN_SWING_DEG = 20.0f; // hip swing each way at full turn stick
const float LOWER_REACH    = 1.0f;  // 1 = feet travel straight. 0 = lower joints stay fixed (old walk).
const float REACH_RATIO    = 1.14f; // A / B, measured on a standing leg:
                                    //   A = hip pivot to foot, flat along the floor
                                    //   B = lower-joint pivot to foot tip
const float TURN_TRIM      = 0.0f;  // steering correction while walking, as a fraction of full turn
                                    // stick. Drifts counter-clockwise? Try 0.05, then 0.10.
                                    // If that makes it worse, use a negative number.
const float FEMUR_LIFT_DEG = 35.0f; // femur raise at the top of each step
const float TIBIA_LIFT_DEG = 0.0f;  // keep 0. Only if toes drag, try 5-10 (+ = tuck in)
const float TURN_GAIN      = 1.0f;  // full turn stick = full TURN_SWING_DEG
const float CYCLE_SECONDS  = 4.0f;  // one full step cycle (both tripods) at normal speed
const float FAST_MULTIPLIER = 3.0f; // A button toggles normal <-> this many times faster
const int   MAX_STEP_US    = 30;    // slew limit per 40 ms tick at normal speed (scaled in fast)
const int   DEADBAND       = 10;    // stick percent ignored around center (stops drift creep)
const float US_PER_DEG     = 11.111f; // provisional: 1000 us per 90 deg

const float RAD = 0.01745329252f;
// Direction each leg points, viewed from above (x = right, y = forward), for
// legs 1..6 at -45, 0, 45, 135, 180, 225 degrees.
// Top of the user's 4/3, 5/2, 6/1 diagram is assumed to be front.
const float LEG_X[6] = { 0.70711f, 1,  0.70711f, -0.70711f, -1, -0.70711f};
const float LEG_Y[6] = {-0.70711f, 0,  0.70711f,  0.70711f,  0, -0.70711f};

inline float bounded(float x, float lo, float hi) {
  return x < lo ? lo : (x > hi ? hi : x);
}

// Fills deg[hip, femur, tibia] with offsets from the saved centers, in degrees,
// using the same sign convention the old IK used (DIRECTION is applied later).
inline void gaitOffsets(uint8_t leg, float phase, float forward, float sideways,
                        float turn, float strength, float deg[3]) {
  // Alternate tripods: legs 1/3/5 and 2/4/6.
  float p = phase + ((leg % 2) ? 0.5f : 0.0f);
  if (p >= 1) p -= 1;
  float sweep, lift;
  if (p < 0.5f) {          // swing: leg in the air, carried to the front of its stride
    float t = p*2;
    sweep = -1 + 2*(3*t*t-2*t*t*t);   // smooth start/stop
    lift  = 4*t*(1-t);                // femur up, peak mid-swing
  } else {                 // stance: foot on the ground, moving straight back
    sweep = 1-2*((p-0.5f)*2);
    lift  = 0;
  }
  // Share the hip's range between walking and turning, so combined stick
  // input never asks a hip for more than its full swing.
  float travel = sqrtf(forward*forward + sideways*sideways);
  float spin   = TURN_GAIN*turn + TURN_TRIM*travel;
  float total  = travel + fabsf(spin);
  float share  = total > 1 ? 1/total : 1;
  // Where the foot should be along the walk direction, as a fraction of the
  // leg's standing reach: ahead at the start of stance, behind at the end.
  float u = tanf(HIP_SWING_DEG*RAD) * share * sweep * strength;
  float out    = (sideways*LEG_X[leg] + forward*LEG_Y[leg]) * u;  // along the leg, + = away from body
  float across = (forward*LEG_X[leg] - sideways*LEG_Y[leg]) * u;  // across the leg, + = counter-clockwise
  float x = 1 + LOWER_REACH*out;
  float reach = sqrtf(x*x + across*across);       // reach needed / standing reach
  // Lower leg hangs about straight down when standing, so leaning it out by
  // this angle lengthens the reach by (lower leg length) * sin(angle).
  float lean = asinf(bounded((reach-1)*REACH_RATIO, -0.5f, 0.5f)) / RAD;
  // Turn is SUBTRACTED: with + the robot rotated opposite to the stick (tested).
  deg[0] = atan2f(across, x)/RAD - TURN_SWING_DEG*spin*share*sweep*strength;
  deg[1] = FEMUR_LIFT_DEG * lift * strength;
  deg[2] = TIBIA_LIFT_DEG * lift * strength - LOWER_REACH*lean;   // + = tuck in, so leaning out is negative
}


Adafruit_PWMServoDriver pwm(0x40);
// Two servos are driven straight from Uno pins by hardware timers, so their
// pulse widths are not disturbed by SoftwareSerial or any other interrupt.
//   Channel 17 = D10 (OC1B, Timer1): 16MHz/8 = 2MHz, 40000 ticks = 20 ms (50 Hz).
//   Channel 18 = D11 (OC2A, Timer2): 16MHz/256 = 16 us per tick (about 1.5 deg).
//     Timer2 wraps every 4.096 ms, too fast for a servo, so its output is
//     connected for only one period in five: one pulse every 20.48 ms.
//     The timer hardware makes each pulse. The two small interrupts below only
//     choose WHICH period carries it, so a late interrupt can delay or repeat
//     a pulse but can never change its width.
// D9 (old channel 16) is no longer driven: its output stopped working on this Uno.
// D8 cannot be used: it has no timer output, and software pulses jitter with the radio.
const int TIMER2_STEP_US = 16;
uint8_t timer2Ticks(int us) {   // Timer2 pulse is (OCR2A+1) ticks long; 496..2496 us
  return (uint8_t)constrain((us+TIMER2_STEP_US/2)/TIMER2_STEP_US-1, 30, 155);
}
volatile uint8_t timer2Period=0;          // 0..4; the pulse is sent in period 0
volatile uint8_t timer2Width=0;           // wanted pulse width (timer ticks - 1)
ISR(TIMER2_OVF_vect) {                    // start of every 4.096 ms period
  if(++timer2Period>=5) timer2Period=0;
  if(timer2Period==2) OCR2A=timer2Width;          // output is disconnected here: safe time to change width
  if(timer2Period==4) TCCR2A|=_BV(COM2A1);        // connect: the next period carries the pulse
}
ISR(TIMER2_COMPB_vect) {                  // 2.66 ms into every period, after any pulse has ended
  // Disconnect once the pulse is over: pin low and not within a few ticks of the wrap.
  if(timer2Period!=4 && !(PINB&_BV(PB3)) && TCNT2<250) TCCR2A&=~_BV(COM2A1);
}
void startServoPWM(int pulse10, int pulse11) {
  uint8_t oldSREG=SREG;
  cli();
  TCCR1A=0; TCCR1B=0; TIMSK1=0;
  digitalWrite(10,LOW); pinMode(10,OUTPUT);
  TCNT1=0;
  ICR1=39999;
  OCR1B=(uint16_t)pulse10*2;
  TCCR1A=_BV(COM1B1)|_BV(WGM11);
  TCCR1B=_BV(WGM13)|_BV(WGM12)|_BV(CS11);
  TCCR2A=0; TCCR2B=0; TIMSK2=0;
  digitalWrite(11,LOW); pinMode(11,OUTPUT);
  TCNT2=0; timer2Period=0;
  timer2Width=timer2Ticks(pulse11);
  OCR2A=timer2Width;
  OCR2B=165;                                      // 2.66 ms: after the longest pulse (2.5 ms)
  TIFR2=_BV(TOV2)|_BV(OCF2A)|_BV(OCF2B);          // clear stale interrupt flags
  TCCR2A=_BV(WGM21)|_BV(WGM20);                   // fast PWM; output connected by the interrupt
  TCCR2B=_BV(CS22)|_BV(CS21);                     // 16MHz/256
  TIMSK2=_BV(TOIE2)|_BV(OCIE2B);
  SREG=oldSREG;
}
void writeDirectServo(uint8_t channel,int pulse) {
  if(channel==18) {timer2Width=timer2Ticks(pulse);}  // picked up by the Timer2 interrupt
  else if(channel==17) {
    uint8_t oldSREG=SREG;cli();
    OCR1B=(uint16_t)pulse*2;
    SREG=oldSREG;
  }
}
void stopServoPWM() {
  uint8_t oldSREG=SREG;cli();
  TCCR1A=0;TCCR1B=0;TIMSK2=0;TCCR2A=0;TCCR2B=0;
  digitalWrite(10,LOW);digitalWrite(11,LOW);
  SREG=oldSREG;
}
SoftwareSerial radio(2,3); // ESP32 TX2 -> D2. Leave D3 unconnected.

// Physical layout, viewed from above:
//
//          FRONT
//      4           3
//      5           2
//      6           1
//
// Each row is hip, middle, lower. 17 means Uno D10; 18 means Uno D11.
// NEW port mapping (remapped wiring).
const uint8_t CHANNEL[6][3] = {
  {13, 14, 15}, // Leg 1
  { 9,  8,  7}, // Leg 2
  { 3,  2, 18}, // Leg 3 (lower on D11; was D9, whose output is dead)
  { 1,  0, 17}, // Leg 4 (lower on D10)
  { 6,  5,  4}, // Leg 5
  {10, 11, 12}  // Leg 6
};
// Saved positions in microseconds, by leg (hip, middle, lower).
// Not const: serial [ and ] nudge leg 3 lower.
int center[6][3] = {
  {1500,1425,1975}, // Leg 1
  {1500,1475,2000}, // Leg 2
  {1500,1350,1856}, // Leg 3  lower was 1850; D11 moves in 16 us steps and 1856 is the nearest
  {1575,1125,1850}, // Leg 4
  {1500,1400,2000}, // Leg 5
  {1350,1875,1800}  // Leg 6
};
// Same signs as before (femur already lifted the right way in testing).
// If one hip swings backwards relative to the others, flip only that entry.
const int8_t DIRECTION[6][3] = {
  {1,-1,-1},{1,-1,-1},{1,-1,-1},
  {1,-1,-1},{1,-1,-1},{1,-1,-1}
};
const int MAX_GAIT_OFFSET = 500;
float phase=0, strength=0;
float forward=0, sideways=0, turn=0;
int outputUS[6][3];
// ready: sticks have been seen centered since boot/reconnect, so the robot
// can't lurch off if the stick is already pushed when the link comes up.
bool ready=false, fast=false, tripped=false, seen=false;
uint8_t previousFlags=0x0F, flags=0; // 0x0F: A must be released before a press counts
unsigned long lastPacket=0, lastTick=0;
uint8_t packet[6], used=0;
bool collecting=false;

void writeJoint(uint8_t leg,uint8_t joint,int us) {
  uint8_t ch=CHANNEL[leg][joint];
  if(ch<16) pwm.writeMicroseconds(ch,us);
  else writeDirectServo(ch,us);
}
void stopOutputs() {
  for(uint8_t i=0;i<16;i++) pwm.setPWM(i,0,4096);
  stopServoPWM(); ready=false; strength=0; tripped=true;
  Serial.println(F("Signals OFF. Support robot; reset Uno to restart."));
}
// -100..100 percent -> -1..1 with a center deadband; full stick still = 1.
float stickValue(int v) {
  if(abs(v)<=DEADBAND) return 0;
  return (v-(v>0?DEADBAND:-DEADBAND))/(float)(100-DEADBAND);
}
void consumePacket() {
  uint8_t checksum=0xA5;
  for(uint8_t i=0;i<5;i++) checksum ^= packet[i];
  if(checksum!=packet[5] || packet[0]!=1) return;
  int f=(int8_t)packet[1], s=(int8_t)packet[2], t=(int8_t)packet[3];
  if(abs(f)>100 || abs(s)>100 || abs(t)>100 || (packet[4]&0xF0)) return;
  lastPacket=millis(); seen=true; flags=packet[4];
  forward=stickValue(f); sideways=stickValue(s); turn=stickValue(t);
  // Flags: A=1 (speed toggle), B=2 (stop), right bumper=4 (unused), connected=8.
  if((flags&2) && !tripped) stopOutputs();
  if(tripped) {previousFlags=flags; return;}  // after B: nothing until Uno reset
  if(!(flags&8)) {
    ready=false; forward=sideways=turn=0;
  } else {
    if(!ready && forward==0 && sideways==0 && turn==0) {
      ready=true;
      Serial.println(F("Ready: joystick drives. A = speed toggle, B = stop."));
    }
    if((flags&1) && !(previousFlags&1)) {
      fast=!fast;
      if(fast) Serial.println(F("Speed: FAST (3x)"));
      else     Serial.println(F("Speed: normal"));
    }
  }
  previousFlags=flags;
}
void readRadio() {
  while(radio.available()) {
    uint8_t b=radio.read();
    if(!collecting) {
      if(b==0xA5) {collecting=true;used=0;}
    } else {
      packet[used++]=b;
      if(used==sizeof(packet)) {collecting=false;consumePacket();}
    }
  }
}
void setup() {
  Serial.begin(9600);
  if(!pwm.begin()) {
    Serial.println(F("PCA9685 missing. Check wiring."));
    while(true) delay(100);
  }
  pwm.setPWMFreq(50);
  for(uint8_t i=0;i<16;i++) pwm.setPWM(i,0,4096);
  startServoPWM(center[3][2],center[2][2]);  // D10 = leg 4 lower, D11 = leg 3 lower
  for(uint8_t l=0;l<6;l++) for(uint8_t j=0;j<3;j++) {
    outputUS[l][j]=center[l][j]; writeJoint(l,j,outputUS[l][j]);
  }
  radio.begin(9600);
  Serial.println(F("v4: straight-line feet (lower joints lean during the stride)."));
  Serial.println(F("Saved pose, NEW port mapping. Pair Xbox, center sticks, then drive."));
  Serial.println(F("A toggles normal/3x speed. B stops all servos."));
  Serial.println(F("Serial C pauses until sticks re-center; S prints centers; X stops."));
  Serial.println(F("Leg 3 lower is on Uno pin 11 now (pin 9 unused). Serial ] and [ move it one step."));
}
void loop() {
  readRadio();
  if(Serial.available()) {
    char c=Serial.read();
    if(c=='x'||c=='X') stopOutputs();
    if(c=='c'||c=='C') ready=false;
    // Nudge leg 3 lower one step (16 us on D11). ] = higher number, [ = lower number.
    if(c==']'||c=='[') {
      center[2][2]=constrain(center[2][2]+(c==']'?TIMER2_STEP_US:-TIMER2_STEP_US),704,2304);
      Serial.print(F("Leg 3 lower = "));Serial.print(center[2][2]);Serial.println();
    }
    if(c=='s'||c=='S') for(uint8_t l=0;l<6;l++) {
      Serial.print(F("Leg "));Serial.print(l+1);Serial.print(F(": "));
      for(uint8_t j=0;j<3;j++) {Serial.print(center[l][j]);Serial.print(' ');}
      Serial.println();
    }
    static uint8_t calLeg = 0, calJoint = 0;
    if (c>='1'&&c<='6') { calLeg=c-'1'; Serial.print(F("Leg ")); Serial.println(calLeg+1); }
    if (c=='h'||c=='H')  { calJoint=0; Serial.println(F("Hip")); }
    if (c=='m'||c=='M')  { calJoint=1; Serial.println(F("Middle")); }
    if (c=='l'||c=='L')  { calJoint=2; Serial.println(F("Lower")); }
    if ((c=='+'||c=='=') && strength==0) { center[calLeg][calJoint]+=10; Serial.println(center[calLeg][calJoint]); }
    if ((c=='-'||c=='_') && strength==0) { center[calLeg][calJoint]-=10; Serial.println(center[calLeg][calJoint]); }
  }
  if(seen && millis()-lastPacket>400) {
    seen=false;ready=false;fast=false;previousFlags=0x0F;
    forward=sideways=turn=0;flags=0;
    Serial.println(F("Link lost: saved pose, normal speed. Center sticks to drive."));
  }
  if(tripped || millis()-lastTick<40) return;
  lastTick=millis();
  bool walk=ready && seen && (flags&8) &&
            (forward!=0 || sideways!=0 || turn!=0);
  strength=bounded(strength+(walk?0.20f:-0.12f),0,1);
  float speed=fast ? FAST_MULTIPLIER : 1.0f;
  // Faster cycle needs a proportionally faster slew, or the femur never gets up.
  int maxStep=(int)(MAX_STEP_US*speed);
  if(strength>0) {phase+=0.04f*speed/CYCLE_SECONDS;if(phase>=1)phase-=1;}
  else phase=0;
  for(uint8_t l=0;l<6;l++) {
    float deg[3];
    gaitOffsets(l,phase,forward,sideways,turn,strength,deg);
    for(uint8_t j=0;j<3;j++) {
      int delta=(int)lroundf(deg[j]*US_PER_DEG*DIRECTION[l][j]);
      delta=constrain(delta,-MAX_GAIT_OFFSET,MAX_GAIT_OFFSET);
      // No absolute clamp: calibrated centers include 1975 and 2000 us.
      int target=center[l][j]+delta;
      // Smooth stops and direction changes; also limits sudden packet jumps.
      int diff=constrain(target-outputUS[l][j],-maxStep,maxStep);
      if(diff) {outputUS[l][j]+=diff;writeJoint(l,j,outputUS[l][j]);}
    }
  }
}

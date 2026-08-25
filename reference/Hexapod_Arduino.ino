// Özgür Toprak Önsoy, 11.2025, Eskisehir, Turkiye
// Emre Kalem, 11.2025, Eskisehir, Turkiye

/**
*   Hexapod source code v1.0.0
*/

/**
MIT License

Copyright 2025 Emre Kalem

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the “Software”), 
to deal in the Software without restriction, including without limitation the rights to use, copy, modify, 
merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND , EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. 
IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include <Servo.h>
#include  <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"

#define _USE_MATH_DEFINES
#include <math.h>

#define RAD_TO_DEG(x) ((x) * 180.0 / M_PI)
#define DEG_TO_RAD(x) ((x) * M_PI / 180.0)
#define POW2(x)       ((x)*(x))

struct Vector
{
  float x;
  float y;
  float z;

  Vector(float _x = 0.0, float _y = 0.0, float _z = 0.0)
    : x(_x),
    y(_y),
    z(_z)
  {
  }
};

struct Leg
{
  Vector home;
  Vector origin;
  Vector target;

  int servoPins[3];
  Servo servos[3];

  Leg(int pin0, int pin1, int pin2, float _x, float _y)
    : servoPins({pin0, pin1, pin2}),
    origin(_x, _y, 0)
  {
  }

  void init()
  {
    origin.z = atan2(origin.y, origin.x); // neutral angle

    for (size_t i = 0; i < 3; ++i)
    {
      servos[i].attach(servoPins[i]);
      servos[i].write(90);
    }
  }
};

const float COXA_LENGTH  = 38.0;
const float FEMUR_LENGTH = 86.25;
const float TIBIA_LENGTH = 160.5;

const float STEP_LENGTH = 60.0;
const float STEP_HEIGHT = 100.0;
const float DEFAULT_Z = -40.0;

const float WALK_SMOOTHING_FACTOR = 0.4;
const float STOP_SMOOTHING_FACTOR = 0.4;
const float ROTATION_STEP_RAD = DEG_TO_RAD(15.0);

Leg legs[6] = 
{
  Leg(28, 29, 30, 100.94, -57.11),    // RF
  Leg(25, 26, 27, 0.0, -105.72),      // RM
  Leg(22, 23, 24, -100.94, -57.11),   // RR
  Leg(31,32,33, -100.94, 57.11),      // LR
  Leg(34,35,36, 0.0, 105.72),         // LM
  Leg(37,38,39, 100.94, 57.11)        // LF
};

const int CE = 2;
const int CS = 3;
int radioData[10] = { 500, 500, 500, 500, 500, 500, 0, 0, 0, 0 };
RF24 radio(CE, CS);
const uint64_t pipe = 0xE8E8F0F0E1LL;

float t = 0;

void calculate_fk(const Vector& angles, const Vector& pos, Vector res[4]);
void calculate_ik(const Leg& leg, Vector& res);
Vector get_foot_target(float t, size_t legIndex, float direction, float rotation);
void set_foot_positions(float direction, float rotation);
void read_radio();

void setup()
{
  Serial.begin(9600);

  radio.begin(); 
  radio.openReadingPipe(1,pipe);
  radio.startListening();

  Vector neutralPoints[4] = {};
  for (size_t i = 0; i < 6; ++i)
  {
    legs[i].init();

    const Vector neutralAngles(0, M_PI/4, -M_PI/2);
    calculate_fk(neutralAngles, legs[i].origin, neutralPoints);
    
    legs[i].home = neutralPoints[3];
    legs[i].home.z = DEFAULT_Z;

    legs[i].target = legs[i].home;
  }

  delay(1000);
}

void loop()
{
  read_radio();
  
  float direction = 0.0;
  float rotation = 0.0;

  if (radioData[0] < 200)
    direction = -1.0;
  else if (radioData[0] > 800)
    direction = 1.0;

  if (radioData[3] < 200)
    rotation = 1.0;
  else if (radioData[3] > 800)
    rotation = -1.0;

  set_foot_positions(direction, rotation);
  delay(10);
}

void calculate_fk(const Vector& angles, const Vector& pos, Vector res[4])
{
  const float alpha = angles.x + pos.z;
  const float beta = angles.y;
  const float gamma = angles.z;

  const float ca = cos(alpha);
  const float sa = sin(alpha);
  const float cb = cos(beta);
  const float sb = sin(beta);
  const float cbg = cos(beta + gamma);
  const float sbg = sin(beta + gamma);

  res[0] = Vector(pos.x, pos.y, 0);
  
  res[1].x = res[0].x + COXA_LENGTH * ca;
  res[1].y = res[0].y + COXA_LENGTH * sa;
  res[1].z = res[0].z;

  res[2].x = res[1].x + FEMUR_LENGTH * cb * ca;
  res[2].y = res[1].y + FEMUR_LENGTH * cb * sa;
  res[2].z = res[1].z + FEMUR_LENGTH * sb;

  res[3].x = res[1].x + (FEMUR_LENGTH * cb + TIBIA_LENGTH * cbg) * ca;
  res[3].y = res[1].y + (FEMUR_LENGTH * cb + TIBIA_LENGTH * cbg) * sa;
  res[3].z = res[1].z + (FEMUR_LENGTH * sb + TIBIA_LENGTH * sbg);
}

void calculate_ik(const Leg& leg, Vector& res)
{
  Vector relativeTarget(
    (leg.target.x - leg.origin.x),
    (leg.target.y - leg.origin.y),
    leg.target.z
  );

  const float c = cos(-leg.origin.z);
  const float s = sin(-leg.origin.z);

  const float tempX = relativeTarget.x;
  const float tempY = relativeTarget.y;

  relativeTarget.x = tempX * c - tempY * s;
  relativeTarget.y = tempX * s + tempY * c;

  const float l_xy = sqrt(POW2(relativeTarget.x) + POW2(relativeTarget.y));
  const float l_forward = l_xy - COXA_LENGTH;
  float D = sqrt(POW2(l_forward) + POW2(relativeTarget.z));

  if (D > (FEMUR_LENGTH + TIBIA_LENGTH)) 
  {
    D = FEMUR_LENGTH + TIBIA_LENGTH - 0.001;
  }
  if (D < fabs(FEMUR_LENGTH - TIBIA_LENGTH)) 
  {
    D = fabs(FEMUR_LENGTH - TIBIA_LENGTH) + 0.001;
  }

  res.z = RAD_TO_DEG(acos((POW2(FEMUR_LENGTH) + POW2(TIBIA_LENGTH) - POW2(D)) / (2.0 * FEMUR_LENGTH * TIBIA_LENGTH)) - M_PI);

  const float beta1 = atan2(relativeTarget.z, l_forward);
  const float beta2 = acos((POW2(FEMUR_LENGTH) + POW2(D) - POW2(TIBIA_LENGTH)) / (2.0 * FEMUR_LENGTH * D));
  res.y = RAD_TO_DEG(beta1 + beta2);

  res.x = RAD_TO_DEG(atan2(relativeTarget.y, relativeTarget.x));
}

Vector get_foot_target(float t, size_t legIndex, float direction, float rotation)
{
  float phase = fmod(t, 1.0);
  if ((legIndex % 2) != 0) // Legs 1, 3, 5
    phase = fmod(phase + 0.5, 1.0);
 

  Vector home_pos = legs[legIndex].home;
  
  if (direction == 0 && rotation == 0)
    return home_pos;
 
  float step_x_move = 0.0;
  float step_rot_rad = 0.0;
  float step_z_move = 0.0;

  if (phase < 0.5)
  {
    float phase_norm = phase * 2.0;
    step_x_move = direction * (0.5 - phase_norm) * STEP_LENGTH;
    step_rot_rad = rotation * (0.5 - phase_norm) * ROTATION_STEP_RAD;
    step_z_move = 0;
  }
  else
  {
    float phase_norm = (phase - 0.5) * 2.0; // 0 -> 1
    step_x_move = direction * (phase_norm - 0.5) * STEP_LENGTH;
    step_rot_rad = rotation * (phase_norm - 0.5) * ROTATION_STEP_RAD;
    step_z_move = sin(phase_norm * M_PI) * STEP_HEIGHT;
  }

  float c_rot = cos(step_rot_rad);
  float s_rot = sin(step_rot_rad);

  float rotated_home_x = home_pos.x * c_rot - home_pos.y * s_rot;
  float rotated_home_y = home_pos.x * s_rot + home_pos.y * c_rot;

  Vector res;
  res.x = rotated_home_x + step_x_move;
  res.y = rotated_home_y;
  res.z = home_pos.z + step_z_move;

  return res;
}

void set_foot_positions(float direction, float rotation)
{
  t += 0.015;

  Vector angles;
  for (size_t i = 0; i < 6; ++i)
  {
    const Vector goalTarget = get_foot_target(t, i, direction, rotation);
    const float smoothingFactor = (direction == 0) ? (STOP_SMOOTHING_FACTOR) : (WALK_SMOOTHING_FACTOR);

    legs[i].target.x += (goalTarget.x - legs[i].target.x) * smoothingFactor;
    legs[i].target.y += (goalTarget.y - legs[i].target.y) * smoothingFactor;
    legs[i].target.z += (goalTarget.z - legs[i].target.z) * smoothingFactor;

    calculate_ik(legs[i], angles);

    float ax = 90 + angles.x;
    float ay = 90 + angles.y;
    float az = fabs(angles.z);

    ax = constrain(ax, 45, 135);

    legs[i].servos[0].write(ax);
    legs[i].servos[1].write(ay);
    legs[i].servos[2].write(az);
  }
}

void read_radio()
{
  if (radio.available())
  {
    radio.read(radioData, sizeof(radioData));
  }
}

/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2022 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * power_distribution_quadrotor.c - Crazyflie stock power distribution code
 */

#include "power_distribution.h"

#include <string.h>
#include "log.h"
#include "param.h"
#include "num.h"
#include "autoconf.h"
#include "config.h"
#include "math.h"
#include "math3d.h"

#ifndef CONFIG_MOTORS_DEFAULT_IDLE_THRUST
#  define DEFAULT_IDLE_THRUST 0
#else
#  define DEFAULT_IDLE_THRUST CONFIG_MOTORS_DEFAULT_IDLE_THRUST
#endif

#define mass 0.55f

static uint32_t idleThrust = DEFAULT_IDLE_THRUST;
static float armLength = 0.046f; // m;
static float armlength_bigquad = 0.17f; //m
static float thrustToTorque = 0.005964552f;

// thrust = a * pwm^2 + b * pwm
static float pwmToThrustA = 0.091492681f;
static float pwmToThrustB = 0.067673604f;


// Force to pwm function coefficients
static float f1_a = 8.746454211812262e-9F;
static float f1_b = 1.280608521387491e-9F;
static float f1_c = 2.28664090792239e+8F;
static float f1_d = -5.914182129481692e+3F;

static float f2_a = 8.746454211812262e-9F;
static float f2_b = 1.280608521387491e-9F;
static float f2_c = 2.28664090792239e+8F;
static float f2_d = -5.914182129481692e+3F;

static float f3_a = 8.746454211812262e-9F;
static float f3_b = 1.280608521387491e-9F;
static float f3_c = 2.28664090792239e+8F;
static float f3_d = -5.914182129481692e+3F;

static float f4_a = 8.746454211812262e-9F;
static float f4_b = 1.280608521387491e-9F;
static float f4_c = 2.28664090792239e+8F;
static float f4_d = -5.914182129481692e+3F;

// Torque to pwm function coefficient
static float t1_a = 1.972462809575412e+6F;
static float t1_b = 1.280608521387491e-9F;
static float t1_c = 2.28664090792239e+8F;
static float t1_d = -5.914182129481692e+3F;
static float t1_e = -5.914182129481692e+3F;

static float t2_a = 8.746454211812262e-9F;
static float t2_b = 1.280608521387491e-9F;
static float t2_c = 2.28664090792239e+8F;
static float t2_d = -5.914182129481692e+3F;
static float t2_e = -5.914182129481692e+3F;

static float t3_a = 8.746454211812262e-9F;
static float t3_b = 1.280608521387491e-9F;
static float t3_c = 2.28664090792239e+8F;
static float t3_d = -5.914182129481692e+3F;
static float t3_e = -5.914182129481692e+3F;

static float t4_a = 8.746454211812262e-9F;
static float t4_b = 1.280608521387491e-9F;
static float t4_c = 2.28664090792239e+8F;
static float t4_d = -5.914182129481692e+3F;
static float t4_e = -5.914182129481692e+3F;

static float Ix = 1.0f*powf(10.0f, -1.0f);
static float Iy = 1.0f*powf(10.0f, -1.0f);
static float Iz = 1.0f*powf(10.0f, -1.0f);

struct vec4 trpy_g;                 // moter [gram]
struct vec4 Moter_g;                // moter [gram]
struct vec4 Moter_p;                // moter [gram]


static float motor1_pwm2thrust(float m1_F) {
  float m1_pwm = 0.0;
  if (m1_F >= 0) {m1_pwm = sqrtf((float)m1_F*f1_a+f1_b)*f1_c + f1_d;}
  else {m1_pwm = -(sqrtf((float)-m1_pwm*f1_a+f1_b)*f1_c + f1_d);}
  return m1_pwm;
}
static float motor2_pwm2thrust(float m2_F) {
  float m2_pwm = 0.0;
  if (m2_F >= 0) {m2_pwm = sqrtf((float)m2_F*f2_a+f2_b)*f2_c + f2_d;}
  else {m2_pwm = -(sqrtf((float)-m2_pwm*f2_a+f2_b)*f2_c + f2_d);}
  return m2_pwm;
}
static float motor3_pwm2thrust(float m3_F) {
  float m3_pwm=0.0;
  if (m3_F >= 0) {m3_pwm = sqrtf((float)m3_F*f3_a+f3_b)*f3_c + f3_d;}
  else {m3_pwm = -(sqrtf((float)-m3_pwm*f3_a+f3_b)*f3_c + f3_d);}
  return m3_pwm;
}
static float motor4_pwm2thrust(float m4_F) {
  float m4_pwm = 0.0;
  if (m4_F >= 0) {m4_pwm = sqrtf((float)m4_F*f4_a+f4_b)*f4_c + f4_d;}
  else {m4_pwm = -(sqrtf((float)-m4_pwm*f4_a+f4_b)*f4_c + f4_d);}
  return m4_pwm;
}

static float motor1_pwm2torque(float m1_T) {
  float m1_pwm = 0.0;
  if (m1_T >= 0) {m1_pwm = m1_T*t1_a-powf(m1_T, 2.0f)*t1_b+powf(m1_T, 3.0f)*t1_c-powf(m1_T, 4.0f)*t1_d+t1_e;}
  else {m1_pwm = -(-m1_T*t1_a-powf(m1_T, 2.0f)*t1_b-powf(m1_T, 3.0f)*t1_c-powf(m1_T, 4.0f)*t1_d+t1_e);}
  return m1_pwm;
}

static float motor2_pwm2torque(float m2_T) {
  float m2_pwm = 0.0;
  if (m2_T >= 0) {m2_pwm = m2_T*t2_a-powf(m2_T, 2.0f)*t2_b+powf(m2_T, 3.0f)*t2_c-powf(m2_T, 4.0f)*t2_d+t2_e;}
  else {m2_pwm = -(-m2_T*t1_a-powf(m2_T, 2.0f)*t2_b-powf(m2_T, 3.0f)*t2_c-powf(m2_T, 4.0f)*t2_d+t2_e);}
  return m2_pwm;
}

static float motor3_pwm2torque(float m3_T) {
  float m3_pwm = 0.0;
  if (m3_T >= 0) {m3_pwm = m3_T*t3_a-powf(m3_T, 2.0f)*t3_b+powf(m3_T, 3.0f)*t3_c-powf(m3_T, 4.0f)*t3_d+t3_e;}
  else {m3_pwm = -(-m3_T*t3_a-powf(m3_T, 2.0f)*t3_b-powf(m3_T, 3.0f)*t3_c-powf(m3_T, 4.0f)*t3_d+t3_e);}
  return m3_pwm;
}

static float motor4_pwm2torque(float m4_T) {
  float m4_pwm = 0.0;
  if (m4_T >= 0) {m4_pwm = m4_T*t4_a-powf(m4_T, 2.0f)*t4_b+powf(m4_T, 3.0f)*t4_c-powf(m4_T, 4.0f)*t4_d+t4_e;}
  else {m4_pwm = -(-m4_T*t4_a-powf(m4_T, 2.0f)*t4_b-powf(m4_T, 3.0f)*t4_c-powf(m4_T, 4.0f)*t4_d+t4_e);}
  return m4_pwm;
}


int powerDistributionMotorType(uint32_t id)
{
  return 1;
}

uint16_t powerDistributionStopRatio(uint32_t id)
{
  return 0;
}

void powerDistributionInit(void)
{
}

bool powerDistributionTest(void)
{
  bool pass = true;
  return pass;
}

static uint16_t capMinThrust(float thrust, uint32_t minThrust) {
  if (thrust < minThrust) {
    return minThrust;
  }

  return thrust;
}

static void powerDistributionLegacy(const control_t *control, motors_thrust_uncapped_t* motorThrustUncapped)
{
  int16_t r = control->roll / 2.0f;
  int16_t p = control->pitch / 2.0f;

  motorThrustUncapped->motors.m1 = control->thrust - r + p + control->yaw;
  motorThrustUncapped->motors.m2 = control->thrust - r - p - control->yaw;
  motorThrustUncapped->motors.m3 = control->thrust + r - p + control->yaw;
  motorThrustUncapped->motors.m4 = control->thrust + r + p - control->yaw;
}

static void powerDistributionForceTorque(const control_t *control, motors_thrust_uncapped_t* motorThrustUncapped) {
  static float motorForces[STABILIZER_NR_OF_MOTORS];

  const float arm = 0.707106781f * armLength;
  const float rollPart  = 0.25f / arm * control->torqueX;
  const float pitchPart = 0.25f / arm * control->torqueY;
  const float thrustPart = 0.25f * control->thrustSi; // N (per rotor)
  const float yawPart = 0.25f * control->torqueZ / thrustToTorque;

  motorForces[0] = thrustPart - rollPart - pitchPart - yawPart;
  motorForces[1] = thrustPart - rollPart + pitchPart + yawPart;
  motorForces[2] = thrustPart + rollPart + pitchPart - yawPart;
  motorForces[3] = thrustPart + rollPart - pitchPart + yawPart;

  for (int motorIndex = 0; motorIndex < STABILIZER_NR_OF_MOTORS; motorIndex++) {
    float motorForce = motorForces[motorIndex];
    if (motorForce < 0.0f) {
      motorForce = 0.0f;
    }

    float motor_pwm = (-pwmToThrustB + sqrtf(pwmToThrustB * pwmToThrustB + 4.0f * pwmToThrustA * motorForce)) / (2.0f * pwmToThrustA);
    motorThrustUncapped->list[motorIndex] = motor_pwm * UINT16_MAX;
  }
}

static void powerDistributionForceTorque_for_bigquad(const control_t *control, motors_thrust_uncapped_t* motorThrustUncapped) {
  struct mat44 CT_rpyt2g=Ctrl_m(mass);
  trpy_g.x = control->thrust;
  trpy_g.y = Ix * radians(control->roll) * sqrtf(2)/armlength_bigquad;
  trpy_g.z = Iy * radians(control->pitch) * sqrtf(2)/armlength_bigquad;
  trpy_g.w = 0.0;

  Moter_g = mvmul4(CT_rpyt2g, trpy_g);
  trpy_g.w = Iz * radians(control->yaw);

  
  motorThrustUncapped->list[0] = motor1_pwm2thrust(Moter_g.x) + motor1_pwm2torque(0.25f*trpy_g.w);
  motorThrustUncapped->list[1] = motor2_pwm2thrust(Moter_g.y) + motor2_pwm2torque(-0.25f*trpy_g.w);
  motorThrustUncapped->list[2] = motor3_pwm2thrust(Moter_g.z) + motor3_pwm2torque(0.25f*trpy_g.w);
  motorThrustUncapped->list[3] = motor4_pwm2thrust(Moter_g.w) + motor4_pwm2torque(-0.25f*trpy_g.w); 

  motorThrustUncapped->list[0] = control->thrust;
  motorThrustUncapped->list[1] = control->thrust;
  motorThrustUncapped->list[2] = control->thrust;
  motorThrustUncapped->list[3] = control->thrust;

}

static void powerDistributionForce(const control_t *control, motors_thrust_uncapped_t* motorThrustUncapped) {
  // Not implemented yet
}

void powerDistribution(const control_t *control, motors_thrust_uncapped_t* motorThrustUncapped)
{
  switch (control->controlMode) {
    case controlModeLegacy:
      powerDistributionLegacy(control, motorThrustUncapped);
      break;
    case controlModeForceTorque:
      powerDistributionForceTorque(control, motorThrustUncapped);
      break;
    case controlModeForce:
      powerDistributionForce(control, motorThrustUncapped);
      break;
    default:
      powerDistributionForceTorque_for_bigquad(control, motorThrustUncapped);
      break;
  }
}

void powerDistributionCap(const motors_thrust_uncapped_t* motorThrustBatCompUncapped, motors_thrust_pwm_t* motorPwm)
{
  const int32_t maxAllowedThrust = UINT16_MAX;

  // Find highest thrust
  int32_t highestThrustFound = 0;
  for (int motorIndex = 0; motorIndex < STABILIZER_NR_OF_MOTORS; motorIndex++)
  {
    const int32_t thrust = motorThrustBatCompUncapped->list[motorIndex];
    if (thrust > highestThrustFound)
    {
      highestThrustFound = thrust;
    }
  }

  int32_t reduction = 0;
  if (highestThrustFound > maxAllowedThrust)
  {
    reduction = highestThrustFound - maxAllowedThrust;
  }

  for (int motorIndex = 0; motorIndex < STABILIZER_NR_OF_MOTORS; motorIndex++)
  {
    int32_t thrustCappedUpper = motorThrustBatCompUncapped->list[motorIndex] - reduction;
    motorPwm->list[motorIndex] = capMinThrust(thrustCappedUpper, idleThrust);
  }
}

/**
 * Power distribution parameters
 */
PARAM_GROUP_START(powerDist)
/**
 * @brief Motor thrust to set at idle (default: 0)
 *
 * This is often needed for brushless motors as
 * it takes time to start up the motor. Then a
 * common value is between 3000 - 6000.
 */
PARAM_ADD_CORE(PARAM_UINT32 | PARAM_PERSISTENT, idleThrust, &idleThrust)
PARAM_GROUP_STOP(powerDist)

/**
 * System identification parameters for quad rotor
 */
PARAM_GROUP_START(quadSysId)

PARAM_ADD(PARAM_FLOAT, thrustToTorque, &thrustToTorque)
PARAM_ADD(PARAM_FLOAT, pwmToThrustA, &pwmToThrustA)
PARAM_ADD(PARAM_FLOAT, pwmToThrustB, &pwmToThrustB)

/**
 * @brief Length of arms (m)
 *
 * The distance from the center to a motor
 */
PARAM_ADD(PARAM_FLOAT, armLength, &armLength)
PARAM_GROUP_STOP(quadSysId)

#define DEBUG_MODULE "LQR"
#include "debug.h"


#include "stabilizer.h"
#include "stabilizer_types.h"

#include "attitude_controller.h"
#include "sensfusion6.h"
#include "position_controller.h"
#include "controller_lqr.h"
#include "log.h"
#include "param.h"
#include "math3d.h"
#include "matrix_ops.h"
//#include "ricatti_solver.h"

#define ATTITUDE_UPDATE_DT    (float)(1.0f/ATTITUDE_RATE)

static bool tiltCompensationEnabled = false;

static attitude_t attitudeDesired;
static attitude_t rateDesired;
static float actuatorThrust;

static float cmd_thrust;
static float cmd_roll;
static float cmd_pitch;
static float cmd_yaw;
static float r_roll;
static float r_pitch;
static float r_yaw;
static float accelz;

static struct {
  uint32_t m1;
  uint32_t m2;
  uint32_t m3;
  uint32_t m4;
} motorInputs;

// uint32_t state[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
uint32_t x_dA[] = {0, 0, 0, 0, 0, 0, 0, 0};
uint32_t x_dP[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
static const uint32_t KP[4][12] ={{-61.237,        0,  43.301, -60.713,        0,  92.665,      0,  -255.2,  50,       0,  -45.116,    23.406},
                                  {      0,  -61.237,  43.301,       0,  -60.713,  92.665,  255.2,       0, -50,  45.116,        0,   -23.406},
                                  { 61.237,        0,  43.301,  60.713,        0,  92.665,      0,   255.2,  50,       0,   45.116,    23.406},
                                  {      0,   61.237,  43.301,       0,   60.713,  92.665, -255.2,       0, -50, -45.116,        0,   -23.406}};
static const uint32_t KA[4][8] = {{35.3553,    84.0058,    0.0000,      -122.4745,    50,     0.0000,      -35.1693,    23.4056},
                                  {35.3553,    84.0058,    122.4745,    0.0000,      -50,     35.1693,     0.0000,      -23.4056},
                                  {35.3553,    84.0058,    0.00000,     122.4745,    50,      0.0000,      35.1693,     23.4056},
                                  {35.3553,    84.0058,    -122.4745,   0.0000,      -50,     -35.1693,    0.0000,      -23.4056}};
// static const uint32_t H[4][4] =  {{11363636.3636364,  401187515.044532,  -401187515.044532, -250000000},
//                                   {11363636.3636364,  -401187515.044532,  -401187515.044532, 250000000},
//                                   {11363636.3636364,  401187515.044532,  401187515.044532, 250000000},
//                                   {11363636.3636364,  -401187515.044532,  401187515.044532, -250000000}};
static const double k = 2.2e-8;
static const double b = 1e-9;
static const double l = 0.065;//0.046;
static const double m = 0.032;
static const double g = 9.81;
static const double hoverSpeed = 1888.72;//((m*g)/(4*k))^(0.5);
static const double thrustTerm = 1.5e5;
static const double torqueTerm = 5e4;//2e4;


void controllerLQRInit(void)
{
  attitudeControllerInit(ATTITUDE_UPDATE_DT);
  positionControllerInit();
  DEBUG_PRINT("Initializing LQR controller\n");
}

bool controllerLQRTest(void)
{
  bool pass = true;

  pass &= attitudeControllerTest();

  return pass;
}

static float capAngle(float angle) {
  float result = angle;

  while (result > 180.0f) {
    result -= 360.0f;
  }

  while (result < -180.0f) {
    result += 360.0f;
  }

  return result;
}

void controllerLQR(control_t *control, setpoint_t *setpoint,
                                         const sensorData_t *sensors,
                                         const state_t *state,
                                         const uint32_t tick)
{

  // //DEBUG_PRINT("calling LQR control\n");
  // if (RATE_DO_EXECUTE(ATTITUDE_RATE, tick)) {
  //   // Rate-controled YAW is moving YAW angle setpoint
  //   if (setpoint->mode.yaw == modeVelocity) {
  //      attitudeDesired.yaw += setpoint->attitudeRate.yaw * ATTITUDE_UPDATE_DT;
  //   } else {
  //     attitudeDesired.yaw = setpoint->attitude.yaw;
  //   }

  //   attitudeDesired.yaw = capAngle(attitudeDesired.yaw);
  // }

  // if (RATE_DO_EXECUTE(POSITION_RATE, tick)) {
  //   positionController(&actuatorThrust, &attitudeDesired, setpoint, state);
  // }

  // if (RATE_DO_EXECUTE(ATTITUDE_RATE, tick)) {
  //   // Switch between manual and automatic position control
  //   if (setpoint->mode.z == modeDisable) {
  //     actuatorThrust = setpoint->thrust;
  //   }

    uint32_t MS[] = {0, 0, 0, 0};
    float Thrust = 0;
    float Torque_r = 0;
    float Torque_p = 0;
    float Torque_y = 0;

    if (setpoint->mode.x == modeDisable || setpoint->mode.y == modeDisable) {
      // Attitude control
      if (RATE_DO_EXECUTE(ATTITUDE_RATE, tick)) {
          x_dA[0] = state->position.z;
          x_dA[1] = state->velocity.z;
          x_dA[2] = radians(state->attitude.roll);
          x_dA[3] = radians(state->attitude.pitch);
          x_dA[4] = radians(state->attitude.yaw);
          
          float x = state->position.x;
          float y = state->position.y;
          float roll = radians(state->attitude.roll);
          float pitch = radians(state->attitude.pitch);
          float yaw = radians(state->attitude.yaw);
          float p = (1.0f * roll) + (0.0f * pitch) + (-sinf(y) * yaw);
          float q = (0.0f * roll) + (cosf(x) * pitch) + (cosf(y) * sinf(x) * yaw);
          float r = (0.0f * roll) + (-sinf(x) * pitch) + (cosf(y) * cosf(x) * yaw);


          x_dA[5] = p;
          x_dA[6] = q;
          x_dA[7] = r;
          // x_dA[5] = radians(sensors->gyro.x); // gyro in world frame, convert to body
          // x_dA[6] = -radians(sensors->gyro.y);
          // x_dA[7] = radians(sensors->gyro.z);
      }//first 9 in world, last 3 in body
      // LQR
      double sum = 0.0;
      for(int k = 0; k < 4; k++){
        sum = 0.0;
        for(int i = 0; i < 8; i++){
          sum += x_dA[i]*KA[k][i];
        }
        MS[k] = hoverSpeed + sum;
      }
      // Convert from Motor speeds to torques
      Thrust    = k*(MS[0]^2 + MS[1]^2 + MS[2]^2 + MS[3]^2)*thrustTerm;
      Torque_r  = (k*l)*(1.414)*(-MS[0]^2 - MS[1]^2 + MS[2]^2 + MS[3]^2)*torqueTerm;
      Torque_p  = (k*l)*(1.414)*(-MS[0]^2 + MS[1]^2 + MS[2]^2 - MS[3]^2)*torqueTerm;
      // Torque_r  = (k*l)*(-MS[0]^2 + MS[2]^2)*torqueTerm;
      // Torque_p  = (k*l)*(-MS[1]^2 + MS[3]^2)*torqueTerm;
      Torque_y  = b*(-MS[0]^2 + MS[1]^2 - MS[2]^2 + MS[3]^2)*torqueTerm;
    }
    else{
      // Position control
      if (RATE_DO_EXECUTE(POSITION_RATE, tick)) {
          x_dP[0] = state->position.x - setpoint->position.x;
          x_dP[1] = state->position.y - setpoint->position.y;
          x_dP[2] = state->position.z - setpoint->position.z;
          x_dP[3] = state->velocity.x;
          x_dP[4] = state->velocity.y;
          x_dP[5] = state->velocity.z;
          x_dP[6] = radians(state->attitude.roll);
          x_dP[7] = radians(state->attitude.pitch);
          x_dP[8] = radians(state->attitude.yaw);
          x_dP[9] = radians(sensors->gyro.x);
          x_dP[10] = -radians(sensors->gyro.y);
          x_dP[11] = radians(sensors->gyro.z);
      }
      // LQR
      double sum = 0.0;
      for(int k = 0; k < 4; k++){
        sum = 0.0;
        for(int i = 0; i < 12; i++){
          sum += x_dP[i]*KP[k][i];
        }
        MS[k] = hoverSpeed + sum;
      }
      // Convert from Motor speeds to torques
      Thrust    = k*(MS[0]^2 + MS[1]^2 + MS[2]^2 + MS[3]^2)*thrustTerm;
      Torque_r  = (k*l)*(1.414)*(-MS[0]^2 - MS[1]^2 + MS[2]^2 + MS[3]^2)*torqueTerm;
      Torque_p  = (k*l)*(1.414)*(-MS[0]^2 + MS[1]^2 + MS[2]^2 - MS[3]^2)*torqueTerm;
      // Torque_r  = (k*l)*(-MS[0]^2 + MS[2]^2)*torqueTerm;
      // Torque_p  = (k*l)*(-MS[1]^2 + MS[3]^2)*torqueTerm;
      Torque_y  = b*(-MS[0]^2 + MS[1]^2 - MS[2]^2 + MS[3]^2)*torqueTerm;
    }

    if (setpoint->position.z == 0.0f){
      control->thrust = 0;
    }
    else{
      control->thrust = Thrust;
    }
    if (control->thrust > 0) {
      control->roll = clamp(Torque_r, -32000, 32000);
      control->pitch = clamp(Torque_p, -32000, 32000);
      control->yaw = clamp(Torque_y, -32000, 32000);
    } 
    else 
    {
      control->roll = 0;
      control->pitch = 0;
      control->yaw = 0;
    }
/*
	// control->thrust = H[0][0]*MS[0] + H[0][1]*MS[1] + H[0][2]*MS[2] + H[0][3]*MS[3];
	// control->roll   = H[1][0]*MS[0] + H[1][1]*MS[1] + H[1][2]*MS[2] + H[1][3]*MS[3];
	// control->pitch  = H[2][0]*MS[0] + H[2][1]*MS[1] + H[2][2]*MS[2] + H[2][3]*MS[3];
	// control->yaw    = H[3][0]*MS[0] + H[3][1]*MS[1] + H[3][2]*MS[2] + H[3][3]*MS[3];

  // control->thrust = k*(MS[0]^2 + MS[1]^2 + MS[2]^2 + MS[3]^2);
  // control->roll   = ((k*l)/1.414213)*(-MS[0]^2 - MS[1]^2 + MS[2]^2 + MS[3]^2);
  // control->pitch  = ((k*l)/1.414213)*(-MS[0]^2 + MS[1]^2 + MS[2]^2 - MS[3]^2);
  // control->yaw    = b*(MS[0]^2 + MS[1]^2 + MS[2]^2 + MS[3]^2);

  // control->thrust = k*(MS[0]^2 + MS[1]^2 + MS[2]^2 + MS[3]^2);
  // control->roll   = (k*l)*(-MS[0]^2 + MS[2]^2);
  // control->pitch  = (k*l)*(-MS[1]^2 + MS[3]^2);
  // control->yaw    = b*(-MS[0]^2 + MS[1]^2 - MS[2]^2 + MS[3]^2);
  //thrust scale
  */

//<<<<<<< HEAD
  //   // TODO: Investigate possibility to subtract gyro drift.
  //   attitudeControllerCorrectRatePID(sensors->gyro.x, -sensors->gyro.y, sensors->gyro.z,
  //                            rateDesired.roll, rateDesired.pitch, rateDesired.yaw);

  //   attitudeControllerGetActuatorOutput(&control->roll,
  //                                       &control->pitch,
  //                                       &control->yaw);

  //   control->yaw = -control->yaw;

  //   //construct the stat vector from the stat input
  //   float state[12]={state->position.x,state->position.y,state->position.z,\
  //   state->attitude.roll,state->attitude.pitch,state->attitude.yaw,\
  //   state->velocity.x,state->velocity.y,state->velocity->z,\
  //   state->}
  //   cmd_thrust = control->thrust;
  //   cmd_roll = control->roll;
  //   cmd_pitch = control->pitch;
  //   cmd_yaw = control->yaw;
  //   r_roll = radians(sensors->gyro.x);
  //   r_pitch = -radians(sensors->gyro.y);
  //   r_yaw = radians(sensors->gyro.z);
  //   accelz = sensors->acc.z;
  // }

  // if (tiltCompensationEnabled)
  // {
  //   control->thrust = actuatorThrust / sensfusion6GetInvThrustCompensationForTilt();
  // }
  // else
  // {
  //   control->thrust = actuatorThrust;
  // }

  // if (control->thrust == 0)
  // {
  //   control->thrust = 0;
  //   control->roll = 0;
  //   control->pitch = 0;
  //   control->yaw = 0;

  //   cmd_thrust = control->thrust;
  //   cmd_roll = control->roll;
  //   cmd_pitch = control->pitch;
  //   cmd_yaw = control->yaw;

  //   attitudeControllerResetAllPID();
  //   positionControllerResetAllPID();

  //   // Reset the calculated YAW angle for rate control
  //   attitudeDesired.yaw = state->attitude.yaw;
  // }

}

/**
 * Logging variables for the command and reference signals for the
 * altitude LQR controller
 */
LOG_GROUP_START(controller)
/**
 * @brief Thrust command
 */
LOG_ADD(LOG_FLOAT, cmd_thrust, &cmd_thrust)
/**
 * @brief Roll command
 */
LOG_ADD(LOG_FLOAT, cmd_roll, &cmd_roll)
/**
 * @brief Pitch command
 */
LOG_ADD(LOG_FLOAT, cmd_pitch, &cmd_pitch)
/**
 * @brief yaw command
 */
LOG_ADD(LOG_FLOAT, cmd_yaw, &cmd_yaw)
/**
 * @brief Gyro roll measurement in radians
 */
LOG_ADD(LOG_FLOAT, r_roll, &r_roll)
/**
 * @brief Gyro pitch measurement in radians
 */
LOG_ADD(LOG_FLOAT, r_pitch, &r_pitch)
/**
 * @brief Yaw  measurement in radians
 */
LOG_ADD(LOG_FLOAT, r_yaw, &r_yaw)
/**
 * @brief Acceleration in the zaxis in G-force
 */
LOG_ADD(LOG_FLOAT, accelz, &accelz)
/**
 * @brief Thrust command without (tilt)compensation
 */
LOG_ADD(LOG_FLOAT, actuatorThrust, &actuatorThrust)
/**
 * @brief Desired roll setpoint
 */
LOG_ADD(LOG_FLOAT, roll,      &attitudeDesired.roll)
/**
 * @brief Desired pitch setpoint
 */
LOG_ADD(LOG_FLOAT, pitch,     &attitudeDesired.pitch)
/**
 * @brief Desired yaw setpoint
 */
LOG_ADD(LOG_FLOAT, yaw,       &attitudeDesired.yaw)
/**
 * @brief Desired roll rate setpoint
 */
LOG_ADD(LOG_FLOAT, rollRate,  &rateDesired.roll)
/**
 * @brief Desired pitch rate setpoint
 */
LOG_ADD(LOG_FLOAT, pitchRate, &rateDesired.pitch)
/**
 * @brief Desired yaw rate setpoint
 */
LOG_ADD(LOG_FLOAT, yawRate,   &rateDesired.yaw)
LOG_GROUP_STOP(controller)


/**
 * Controller parameters
 */
PARAM_GROUP_START(controller)
/**
 * @brief Nonzero for tilt compensation enabled (default: 0)
 */
PARAM_ADD(PARAM_UINT8, tiltComp, &tiltCompensationEnabled)
PARAM_GROUP_STOP(controller)
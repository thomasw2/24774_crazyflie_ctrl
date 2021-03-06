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

#define ATTITUDE_UPDATE_DT    (float)(1.0f/ATTITUDE_RATE)

static bool tiltCompensationEnabled = false;

static attitude_t attitudeDesired;
static attitude_t rateDesired;
static float actuatorThrust;

static float r_roll;
static float r_pitch;
static float r_yaw;
static float accelz;

static float x_dP[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
//Gain Matrix with values: [1....]

//Semi-working LQR!!!
// static const float KP[4][12]={{0.0000, -0.0000, 2.0962, 0.0000, 0.0000, -0.0000, 0.0000, 0.0000, 0.3794, 0.0000, 0.0000, 0.0000},
// {-0.0000, -0.0084, -0.0000, 0.0522, -0.0000, -0.0000, -0.0000, -0.0095, -0.0000, 0.0146, 0.0000, 0.0000},
// {0.0084, 0.0000, -0.0000, -0.0000, 0.0522, -0.0000, 0.0095, 0.0000, 0.0000, 0.0000, 0.0146, -0.0000},
// {-0.0000, -0.0000, -0.0000, -0.0000, -0.0000, 0.2222, -0.0000, 0.0000, -0.0000, 0.0000, -0.0000, 0.0285}};
//better gains
// static const float KP[4][12]={{-0.0000, -0.0000, 2.0894, 0.0000, -0.0000, -0.0000, -0.0000, -0.0000, 0.4804, 0.0000, -0.0000, 0.0000},
// {0.0000, -0.0471, 0.0000, 0.0928, -0.0000, 0.0000, 0.0000, -0.0300, 0.0000, 0.0146, -0.0000, -0.0000},
// {0.0624, 0.0000, 0.0000, -0.0000, 0.1022, 0.0000, 0.0364, 0.0000, -0.0000, 0.0000, 0.0146, -0.0000},
// {-0.0000, -0.0000, -0.0000, 0.0000, -0.0000, 0.1139, -0.0000, -0.0000, -0.0000, -0.0000, -0.0000, 0.0281}};
static const float KP[4][12]={{-0.0000, 0.0000, 2.0496, -0.0000, -0.0000, 0.0000, -0.0000, 0.0000, 1.0761, 0.0000, -0.0000, -0.0000},
{-0.0000, -0.0471, -0.0000, 0.1041, -0.0000, 0.0000, -0.0000, -0.0378, -0.0000, 0.0146, -0.0000, 0.0000},
{0.0623, 0.0000, -0.0000, -0.0000, 0.1109, -0.0000, 0.0428, 0.0000, -0.0000, -0.0000, 0.0146, -0.0000},
{-0.0000, -0.0000, 0.0000, 0.0000, -0.0000, 0.1139, -0.0000, -0.0000, 0.0000, 0.0000, -0.0000, 0.0281}};


static const float m = 0.04;//0.032;            // kg
static const float g = 9.81;             // m/s2
static const float l = 0.03252;
static const float thrustTerm = 435566.0f;//132000.9*4;
static const float torqueTerm_rp = thrustTerm/l/8;
static const float torqueTerm_y = thrustTerm*0.006f*130;
static const float cr = 1.0;
static const float cp = 1.0;
static const float cy = 1.0;

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
    float MS[] = {0.0f, 0.0f, 0.0f, 0.0f};
    float Thrust = 0.0;
    float Torque_r = 0.0f;
    float Torque_p = 0.0f;
    float Torque_y = 0.0f;
    bool enable_pid = false;
    if (!RATE_DO_EXECUTE(ATTITUDE_RATE, tick)) {
      return;
    }
    else{
      if (setpoint->mode.x == modeAbs || setpoint->mode.y == modeAbs || setpoint->mode.z == modeAbs) {
        float roll = radians(capAngle(state->attitude.roll));    //--> try body frame for dr, dp, dy
        float pitch = -radians(capAngle(state->attitude.pitch));
        float yaw = radians(capAngle(state->attitude.yaw));
        float droll =  radians(sensors->gyro.x);
        float dpitch =  radians(sensors->gyro.y);
        float dyaw =  radians(sensors->gyro.z);
        float p = (1.0f*droll) + sinf(roll)*tanf(pitch)*dpitch + cosf(roll)*tanf(pitch)*dyaw;
        float q = (0.0f*droll) + cosf(roll)*dpitch             - sinf(roll)*dyaw;
        float r = (0.0f*droll) + sinf(roll)/cosf(pitch)*dpitch + cosf(roll)/cosf(pitch)*dyaw;


        x_dP[0] = state->position.x - setpoint->position.x;
        x_dP[1] = state->position.y - setpoint->position.y;
        x_dP[2] = state->position.z - setpoint->position.z;
        x_dP[3] = roll;//- setpoint->attitude.roll;
        x_dP[4] = pitch;//- setpoint->attitude.pitch;
        x_dP[5] = yaw;//- setpoint->attitude.yaw;
        x_dP[6] = state->velocity.x- setpoint->velocity.x;
        x_dP[7] = state->velocity.y- setpoint->velocity.y;
        x_dP[8] = state->velocity.z- setpoint->velocity.z;
        x_dP[9] = p;
        x_dP[10] = q;
        x_dP[11] = r;
        // LQR
        float sum = 0.0;
        for(int k = 0; k < 4; k++){
          sum = 0.0;
          for(int i = 0; i < 12; i++){
            sum += x_dP[i]*KP[k][i];
          }
          // MS[k] = (sum);//*motorConversion;
          MS[k] = -sum;//*motorConversion;
        }
        MS[0]=MS[0]+m*g;

//need to map yaw torque to motor input
        //experimental

        Thrust =clamp(MS[0]*thrustTerm/4.0f,0,65000);
        Torque_r= clamp(cr*MS[1]*torqueTerm_rp/8.0f,-32000,32000);
        Torque_p =clamp(cp*MS[2]*torqueTerm_rp/8.0f,-32000,32000);
        Torque_y = clamp(cy*MS[3]*torqueTerm_y/4.0f,-32000,32000);

        
        control->m1 = x_dP[0];//Thrust;
        control->m2 = x_dP[6];//Torque_r;
        control->m3 = setpoint->position.x;//Torque_p;
        control->m4 = setpoint->velocity.x;//Torque_y;

        // Convert Motor speeds to torques
        // Thrust    = k*(powf(MS[0], 2.0) + powf(MS[1], 2.0) + powf(MS[2], 2.0) + powf(MS[3], 2.0))*thrustTerm;                  // Thrust - Newtons (kg*m/rad2)
        // Torque_r  = ((k*l)/(1.414f))*(-powf(MS[0], 2.0) - powf(MS[1], 2.0) + powf(MS[2], 2.0) + powf(MS[3], 2.0))*torqueTerm;  // Newton * m
        // Torque_p  = ((k*l)/(1.414f))*(-powf(MS[0], 2.0) + powf(MS[1], 2.0) + powf(MS[2], 2.0) - powf(MS[3], 2.0))*torqueTerm;  // Newton * m
        // Torque_y  = b*(powf(MS[0], 2.0) - powf(MS[1], 2.0) + powf(MS[2], 2.0) - powf(MS[3], 2.0))*torqueTerm;

        // if (RATE_DO_EXECUTE(POSITION_RATE, tick)) {
        //   DEBUG_PRINT("T:%4.4f taux:%4.4f tauy:%4.4f tauz:%4.4f\n",
        //     (double)MS[0],(double)MS[1],(double)MS[2],(double)MS[3]);
        // }

        control->thrust = Thrust;//clamp(Thrust*thrustTerm/4.0f,0,65000);
        control->roll = Torque_r;//clamp(cr*Torque_r*thrustTerm*l/2.0f,-32000,32000);
        control->pitch = Torque_p;//clamp(cp*Torque_p*thrustTerm*l/2.0f,-32000,32000);
        control->yaw = Torque_y;//clamp(cy*Torque_y*torqueTerm,-32000,32000);
        control->lqr_Thrust = Thrust;
        control->lqr_Tr = Torque_r;
        control->lqr_Tp = Torque_p;
        control->lqr_Ty = Torque_y;
      }
      else{
        control->thrust = 0;
      }
    }
    if (control->thrust == 0)
    {
      control->thrust = 0;
      control->roll = 0;
      control->pitch = 0;
      control->yaw = 0;
      // control->m1 = 0;
      // control->m2 = 0;
      // control->m3 = 0;
      // control->m4 = 0;

      attitudeControllerResetAllPID();
      positionControllerResetAllPID();

      // Reset the calculated YAW angle for rate control
      attitudeDesired.yaw = state->attitude.yaw;
    }

///////////////////////////////////////           PID           ///////////////////////////////////////////////////
  if (enable_pid){
    if (RATE_DO_EXECUTE(ATTITUDE_RATE, tick)) {
      // Rate-controled YAW is moving YAW angle setpoint
      if (setpoint->mode.yaw == modeVelocity) {
        attitudeDesired.yaw += setpoint->attitudeRate.yaw * ATTITUDE_UPDATE_DT;
      } else {
        attitudeDesired.yaw = setpoint->attitude.yaw;
      }

      attitudeDesired.yaw = capAngle(attitudeDesired.yaw);
    }
    if (RATE_DO_EXECUTE(POSITION_RATE, tick)) {
      positionController(&actuatorThrust, &attitudeDesired, setpoint, state);
    }
    if (RATE_DO_EXECUTE(ATTITUDE_RATE, tick)) {
      // Switch between manual and automatic position control
      if (setpoint->mode.z == modeDisable) {
        actuatorThrust = setpoint->thrust;
      }
      if (setpoint->mode.x == modeDisable || setpoint->mode.y == modeDisable) {
        attitudeDesired.roll = setpoint->attitude.roll;
        attitudeDesired.pitch = setpoint->attitude.pitch;
      }
      attitudeControllerCorrectAttitudePID(state->attitude.roll, state->attitude.pitch, state->attitude.yaw,
                                  attitudeDesired.roll, attitudeDesired.pitch, attitudeDesired.yaw,
                                  &rateDesired.roll, &rateDesired.pitch, &rateDesired.yaw);

      // For roll and pitch, if velocity mode, overwrite rateDesired with the setpoint
      // value. Also reset the PID to avoid error buildup, which can lead to unstable
      // behavior if level mode is engaged later
      if (setpoint->mode.roll == modeVelocity) {
        rateDesired.roll = setpoint->attitudeRate.roll;
        attitudeControllerResetRollAttitudePID();
      }
      if (setpoint->mode.pitch == modeVelocity) {
        rateDesired.pitch = setpoint->attitudeRate.pitch;
        attitudeControllerResetPitchAttitudePID();
      }
      // TODO: Investigate possibility to subtract gyro drift.
      attitudeControllerCorrectRatePID(sensors->gyro.x, -sensors->gyro.y, sensors->gyro.z,
                              rateDesired.roll, rateDesired.pitch, rateDesired.yaw);

      attitudeControllerGetActuatorOutput(&control->roll,
                                          &control->pitch,
                                          &control->yaw);
      control->yaw = -control->yaw;
      r_roll = radians(sensors->gyro.x);
      r_pitch = -radians(sensors->gyro.y);
      r_yaw = radians(sensors->gyro.z);
      accelz = sensors->acc.z;
    }

    control->pid_Thrust = actuatorThrust;
    control->pid_Tr = control->roll;
    control->pid_Tp = control->pitch;
    control->pid_Ty = -control->yaw;

    if (tiltCompensationEnabled)
    {
      control->thrust = actuatorThrust / sensfusion6GetInvThrustCompensationForTilt();
    }
    else
    {
      control->thrust = actuatorThrust;
    }
    if (control->thrust == 0)
    {
      control->thrust = 0;
      control->roll = 0;
      control->pitch = 0;
      control->yaw = 0;

      attitudeControllerResetAllPID();
      positionControllerResetAllPID();

      // Reset the calculated YAW angle for rate control
      attitudeDesired.yaw = state->attitude.yaw;
    }
  }
}
/**
 * Logging variables for the command and reference signals for the
 * altitude LQR controller
 */
LOG_GROUP_START(controller)
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
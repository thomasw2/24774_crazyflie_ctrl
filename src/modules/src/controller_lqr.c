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
//#include "ricatti_solver.h"

#define ATTITUDE_UPDATE_DT    (float)(1.0f/ATTITUDE_RATE)

static bool tiltCompensationEnabled = false;

static attitude_t attitudeDesired;
static attitude_t rateDesired;
static float actuatorThrust;

static float r_roll;
static float r_pitch;
static float r_yaw;
static float accelz;

// static struct {
//   float m1;
//   float m2;
//   float m3;
//   float m4;
// } motorInputs;

// uint32_t state[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
// uint32_t x_dA[] = {0, 0, 0, 0, 0, 0, 0, 0};
static float x_dP[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
//Gain Matrix with values: [1....]
static const float KP[4][12] = {{-0.0000, -0.0000, 0.0145, -0.0000, -0.0000, -0.0145, 0.0000, -0.0000, 14.7655, 0.0000, -0.0000, 0.0000},
{0.0000, 0.0791, -0.0000, 0.0000, -0.0000, -0.0000, 0.0791, 0.0000, 0.0000, 0.0000, 0.0000, 0.0285},
{-0.0000, -0.0000, 0.0211, -0.0000, -0.0000, -0.0211, -0.0000, -0.0000, 14.7975, 0.0000, -0.0000, 0.0000},
{0.0000, 0.0146, -0.0000, 0.0000, 0.0000, -0.0000, 0.0146, 0.0000, -0.0000, 0.0000, 0.0000, 0.0285}};

// Gain Matrix with values: [0.001  0.001  1  0.001  0.001  0.001  5  5  5  0.001  0.001  0.001]
// static const float KP[4][12] = {{-0.5000, 0.5000, 0.5000, -0.5000, 0.5000, 0.5000, -0.5000, -0.5000, 15.8114, 15.8114, 15.8114, 15.8114},
//                                    {-2.1091, 2.1091, 2.1091, -2.1091, 2.1091, 2.1091, -2.1091, -2.1091, 55.1763, 55.1763, 55.1763, 55.1763},
//                                    {-41.1865, -41.1865, 41.1865, 41.1865, -41.1865, 41.1865, 41.1865, -41.1865, 35.3553, -35.3553, 35.3553, -35.3553},
//                                    {-11.1423, -11.1423, 11.1423, 11.1423, -11.1423, 11.1423, 11.1423, -11.1423, 14.5204, -14.5204, 14.5204, -14.5204}};

// Gain Matrix with values: [0.001  0.001  5  0.001  0.001  0.001  50  50  50  0.001  0.001  0.001]
// static const float KP[4][12] = {{-0.5000, 0.5000, 0.5000, -0.5000, 0.5000, 0.5000, -0.5000, -0.5000, 35.3553, 35.3553, 35.3553, 35.3553},
//                                 {-3.4948, 3.4948, 3.4948, -3.4948, 3.4948, 3.4948, -3.4948, -3.4948, 82.5059, 82.5059, 82.5059, 82.5059},
//                                 {-117.3631, -117.3631, 117.3631, 117.3631, -117.3631, 117.3631, 117.3631, -117.3631, 111.8034, -111.8034, 111.8034, -111.8034},
//                                 {-18.7966, -18.7966, 18.7966, 18.7966, -18.7966, 18.7966, 18.7966, -18.7966, 25.8108, -25.8108, 25.8108, -25.8108}};

// Gain Matrix with values: [0.001  0.001  70  0.001  0.001  0.001  50  50  50  0.001  0.001  0.001]
// static const float KP[4][12] = {{-0.5000, 0.5000, 0.5000, -0.5000, 0.5000, 0.5000, -0.5000, -0.5000, 132.2876, 132.2876, 132.2876, 132.2876},
//                                 {-3.4948, 3.4948, 3.4948, -3.4948, 3.4948, 3.4948, -3.4948, -3.4948, 159.5920, 159.5920, 159.5920, 159.5920},
//                                 {-117.3631, -117.3631, 117.3631, 117.3631, -117.3631, 117.3631, 117.3631, -117.3631, 111.8034, -111.8034, 111.8034, -111.8034},
//                                 {-18.7966, -18.7966, 18.7966, 18.7966, -18.7966, 18.7966, 18.7966, -18.7966, 25.8108, -25.8108, 25.8108, -25.8108}};

// Gain Matrix with values: [0.5  0.5  70  0.001  0.001  0.001  100  100  100  0.001  0.001  0.001]
// static const float KP[4][12] = {{-11.1803, 11.1803, 11.1803, -11.1803, 11.1803, 11.1803, -11.1803, -11.1803, 132.2876, 132.2876, 132.2876, 132.2876},
//                                 {-20.5332, 20.5332, 20.5332, -20.5332, 20.5332, 20.5332, -20.5332, -20.5332, 159.5920, 159.5920, 159.5920, 159.5920},
//                                 {-184.8578, -184.8578, 184.8578, 184.8578, -184.8578, 184.8578, 184.8578, -184.8578, 158.1139, -158.1139, 158.1139, -158.1139},
//                                 {-23.5872, -23.5872, 23.5872, 23.5872, -23.5872, 23.5872, 23.5872, -23.5872, 30.6927, -30.6927, 30.6927, -30.6927}};

// Gain Matrix with values: [0.5  0.5  70  0.001  0.001  0.001  10  10  3  0.001  0.001  0.001]
// static const float KP[4][12] = {{-11.1803, 11.1803, 11.1803, -11.1803, 11.1803, 11.1803, -11.1803, -11.1803, 132.2876, 132.2876, 132.2876, 132.2876},
//                                 {-13.4226, 13.4226, 13.4226, -13.4226, 13.4226, 13.4226, -13.4226, -13.4226, 159.5920, 159.5920, 159.5920, 159.5920},
//                                 {-78.9321, -78.9321, 78.9321, 78.9321, -78.9321, 78.9321, 78.9321, -78.9321, 27.3861, -27.3861, 27.3861, -27.3861},
//                                 {-15.4175, -15.4175, 15.4175, 15.4175, -15.4175, 15.4175, 15.4175, -15.4175, 12.7817, -12.7817, 12.7817, -12.7817}};

// K from Bayrak paper
// static const float KP[4][12] = {{-0.0157, 0.0158, 0.0223, -0.0090, 0.0091, 0.0177, -0.0259, -0.0254, -0.0220, -0.0027, -0.0025, -0.0080},
//                                 {0.0157, 0.0155, 0.0223, 0.0089, 0.0086, 0.0177, -0.0230, 0.0249, 0.0224, -0.0017, 0.0024, 0.0077},
//                                 {0.0156, -0.0155, 0.0223, 0.0088, -0.0087, 0.0177, 0.0234, 0.0242, -0.0226, 0.0018, 0.0021, -0.0076},
//                                 {-0.0156, -0.0158, 0.0223, -0.0087, -0.0090, 0.0177, 0.0254, -0.0237, 0.0222, 0.0026, -0.0020, 0.0079}};

// K from Vaibhav
// static const float KP[4][12] = {{0.000, 0.000, 0.2175, 0.000, 0.000, 0.000, 0.000, 0.000, 0.1456, 0.000, 0.000, 0.0},
//                                 {0.0024, 0.000, 0.000, 0.000, 0.0093, 0.000, 0.0026, 0.000, 0.000, 0.000, 0.0016, 0.0},
//                                 {0.000, -0.0024, 0.000, 0.0093, 0.000, 0.000, 0.000, -0.0026, 0.000, 0.0016, 0.000, 0.0},
//                                 {0.000, 0.000, 0.000, 0.000, 0.000, 0.0025, 0.000, 0.000, 0.000, 0.000, 0.000, 0.0026}};

// static const uint32_t KA[4][8] = {{35.3553,    84.0058,    0.0000,      -122.4745,    50,     0.0000,      -35.1693,    23.4056},
//                                   {35.3553,    84.0058,    122.4745,    0.0000,      -50,     35.1693,     0.0000,      -23.4056},
//                                   {35.3553,    84.0058,    0.00000,     122.4745,    50,      0.0000,      35.1693,     23.4056},
//                                   {35.3553,    84.0058,    -122.4745,   0.0000,      -50,     -35.1693,    0.0000,      -23.4056}};

//static const float k = 2.2e-8;           // kg*m/rad2
//static const float b = 1e-9;             // 
//static const float l = 0.046;            // m
static const float m = 0.032;            // kg
static const float g = 9.81;             // m/s2
//static const float hoverSpeed = 1888;//powf((m*g)/(4*k), 0.5);
//static const float motorConversion = 5.5593;
static const float thrustTerm = 132000.9;
static const float torqueTerm = 6154.0;//2.8696e6;//5e4;


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
        //float x = state->position.x;
        //float y = state->position.y;
        float roll = radians(capAngle(state->attitude.roll));    //--> try body frame for dr, dp, dy
        float pitch = -radians(capAngle(state->attitude.pitch));
        float yaw = radians(capAngle(state->attitude.yaw));
        float droll =  radians(sensors->gyro.x);
        float dpitch =  radians(sensors->gyro.y);
        float dyaw =  radians(sensors->gyro.z);
        // float p = droll;
        // float q = dpitch;
        // float r = dyaw;
        float p = (1.0f*droll) + sinf(roll)*tanf(pitch)*dpitch + cosf(roll)*tanf(pitch)*dyaw;
        float q = (0.0f*droll) + cosf(roll)*dpitch             - sinf(roll)*dyaw;
        float r = (0.0f*droll) + sinf(roll)/cosf(pitch)*dpitch + cosf(roll)/cosf(pitch)*dyaw;


        x_dP[0] = state->position.x - setpoint->position.x;
        x_dP[1] = state->position.y - setpoint->position.y;
        x_dP[2] = state->position.z - setpoint->position.z;
        x_dP[3] = roll;
        x_dP[4] = pitch;
        x_dP[5] = yaw;
        x_dP[6] = state->velocity.x;
        x_dP[7] = state->velocity.y;
        x_dP[8] = state->velocity.z;
        x_dP[9] = p;
        x_dP[10] = q;
        x_dP[11] = r;
        // x_dP[3] = state->velocity.x;
        // x_dP[4] = state->velocity.y;
        // x_dP[5] = state->velocity.z;
        // x_dP[6] = roll;
        // x_dP[7] = pitch;
        // x_dP[8] = yaw;
        // x_dP[9] = p;
        // x_dP[10] = q;
        // x_dP[11] = r;

        // control->x = roll;
        // control->y = pitch;
        // control->z = yaw;
        // control->dx = ;
        // control->dy = ;
        // control->dz = ;
        // control->p = p;
        // control->q = q;
        // control->r = r;
        // control->x = x_dP[0];
        // control->y = x_dP[1];
        // control->z = x_dP[2];
        // control->dx = x_dP[3];
        // control->dy = x_dP[4];
        // control->dz = x_dP[5];
        // control->p = x_dP[6];
        // control->q = x_dP[7];
        // control->r = x_dP[8];
        // control->dp = x_dP[9];
        // control->dq = x_dP[10];
        // control->dr = x_dP[11];

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
        control->m1 = MS[0];
        control->m2 = MS[1];
        control->m3 = MS[2];
        control->m4 = MS[3];

        // Convert Motor speeds to torques
        // Thrust    = k*(powf(MS[0], 2.0) + powf(MS[1], 2.0) + powf(MS[2], 2.0) + powf(MS[3], 2.0))*thrustTerm;                  // Thrust - Newtons (kg*m/rad2)
        // Torque_r  = ((k*l)/(1.414f))*(-powf(MS[0], 2.0) - powf(MS[1], 2.0) + powf(MS[2], 2.0) + powf(MS[3], 2.0))*torqueTerm;  // Newton * m
        // Torque_p  = ((k*l)/(1.414f))*(-powf(MS[0], 2.0) + powf(MS[1], 2.0) + powf(MS[2], 2.0) - powf(MS[3], 2.0))*torqueTerm;  // Newton * m
        // Torque_y  = b*(powf(MS[0], 2.0) - powf(MS[1], 2.0) + powf(MS[2], 2.0) - powf(MS[3], 2.0))*torqueTerm;
        Thrust = MS[0];
        Torque_r = MS[1];
        Torque_p = MS[2];
        Torque_y = MS[3];
        if (RATE_DO_EXECUTE(POSITION_RATE, tick)) {
          DEBUG_PRINT("T:%4.4f taux:%4.4f tauy:%4.4f tauz:%4.4f\n",
            (double)MS[0],(double)MS[1],(double)MS[2],(double)MS[3]);
        }
        control->thrust = Thrust*thrustTerm;
        control->roll = Torque_r*torqueTerm;
        control->pitch = Torque_p*torqueTerm;
        control->yaw = Torque_y*torqueTerm;
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



//////////////////////////////////////////////////////////////////////////////////////////
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
      // control->m1 = 0;
      // control->m2 = 0;
      // control->m3 = 0;
      // control->m4 = 0;

      attitudeControllerResetAllPID();
      positionControllerResetAllPID();

      // Reset the calculated YAW angle for rate control
      attitudeDesired.yaw = state->attitude.yaw;
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
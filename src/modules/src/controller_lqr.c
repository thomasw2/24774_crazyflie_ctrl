
#include "stabilizer.h"
#include "stabilizer_types.h"

#include "attitude_controller.h"
#include "sensfusion6.h"
#include "position_controller.h"
#include "controller_lqr.h"
#include "debug.h"
#include "log.h"
#include "param.h"
#include "math3d.h"
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
uint32_t state[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
uint32_t x_d[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
static const uint32_t K[4][12] = {{-61.237,        0,  43.301, -60.713,        0,  92.665,      0,  -255.2,  50,       0,  -45.116,    23.406},
                                  {      0,  -61.237,  43.301,       0,  -60.713,  92.665,  255.2,       0, -50,  45.116,        0,   -23.406},
                                  { 61.237,        0,  43.301,  60.713,        0,  92.665,      0,   255.2,  50,       0,   45.116,    23.406},
                                  {      0,   61.237,  43.301,       0,   60.713,  92.665, -255.2,       0, -50, -45.116,        0,   -23.406}};
static const int rowK = sizeof(K) / sizeof(K[0]);
static const int columnK = sizeof(K[0])/sizeof(K[0][0]);


void controllerLQRInit(void)
{
  attitudeControllerInit(ATTITUDE_UPDATE_DT);
  positionControllerInit();
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
    //        float stateAttitudeRateRoll = radians(sensors->gyro.x);
    //        float stateAttitudeRatePitch = -radians(sensors->gyro.y);
    //        float stateAttitudeRateYaw = radians(sensors->gyro.z);

    //DEBUG_PRINT("calling LQR control\n");
    x_d[3] = state->velocity.x;
    x_d[4] = state->velocity.y;
    x_d[5] = state->velocity.z;
    x_d[6] = state->attitude.roll;
    x_d[7] = state->attitude.pitch;
    x_d[8] = state->attitude.yaw;
    x_d[9] = radians(sensors->gyro.x);
    x_d[10] = -radians(sensors->gyro.y);
    x_d[11] = radians(sensors->gyro.z);
    // Attitude control
    if (RATE_DO_EXECUTE(ATTITUDE_RATE, tick)) {
        x_d[0] = state->position.x;
        x_d[1] = state->position.y;
        x_d[2] = state->position.z;
    }
    // Position control
    if (RATE_DO_EXECUTE(POSITION_RATE, tick)) {
        x_d[0] = state->position.x - setpoint->position.x;
        x_d[1] = state->position.y - setpoint->position.y;
        x_d[2] = state->position.z - setpoint->position.z;
    }
    double sum = 0.0;
    for(int k=0; k< 4; k++){
        sum = 0.0;
        for(int i=0; i< 12; i++){
            sum += x_d[i]*K[k][i];
        }
        if(k == 0){
            u->m1 = -sum;
        }
        else if (k == 1){
            u->m2 = -sum;
        }
        else if (k == 2){
            u->m3 = -sum;
        }
        else if (k == 3){
            u->m4 = -sum;
        }
    }
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
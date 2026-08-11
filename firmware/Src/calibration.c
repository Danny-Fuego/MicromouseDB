#include "calibration.h"
#include "turn.h"
#include "sensors.h"

extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart2;

#define TEST_ENCODER_ONLY 0
#define TEST_MOTOR_ENCODER_FOREVER 0
#define TEST_BNO055_ONLY 0
#define TEST_BNO055_WITH_MOTOR 0
#define TEST_BLUETOOTH_ONLY 0
#define TEST_ENCODER_CPR_CALIB 0
#define TEST_MOTOR_SPEED_CALIB 0
#define TEST_DEADZONE 0
#define TEST_SPEED_CONTROLLER 0
#define TEST_MOTION_PROFILE 0
#define TEST_DECEL_NATURAL 0
#define TEST_DECEL_THRESHOLD 0
#define TEST_STRAIGHT_SPEED_CONTROLLER 0
#define TEST_TURN_SIGN 0
#define TEST_TURN 0
#define TEST_TURN_PROFILE_CALIB 0
#define TEST_STRAIGHT_TURN_SEQUENCE 0
#define TEST_IR_LIVE 0
#define TEST_IR_BLUETOOTH 0
#define TEST_IR_CALIB 0
#define TEST_IR_STATUS 0
#define TEST_APPROACH_BRAKE_CALIB 0
#define TEST_RANDOM_MAZE_ROAM 0
#define TEST_LEFT_WALL_FOLLOW_CALIB     0
#define TEST_DECEL_CURVE_CALIB          1

#define DECEL_CURVE_PRECOAST_AVG_MS  100
#define DECEL_CURVE_SPEED_MATCH_TOL  60.0f
#define DECEL_CURVE_TARGET_SPEED        800.0f
#define DECEL_CURVE_ACCEL_MM_S2         800.0f
#define DECEL_CURVE_HOLD_MS             500
#define DECEL_CURVE_SAMPLE_MS           10
#define DECEL_CURVE_TIMEOUT_MS          2000
#define DECEL_CURVE_STOP_SPEED_MM_S     30.0f
#define DECEL_CURVE_STOP_SAMPLES        8
#define DECEL_CURVE_REPEATS             3

#define DECEL_CURVE_FILTER_ALPHA        0.30f
#define DECEL_CURVE_MAX_SAMPLES         220

#define WALL_FOLLOW_CALIB_DIST_MM       254.0f
#define WALL_FOLLOW_CALIB_SPEED_MM_S    400.0f
#define WALL_FOLLOW_CALIB_TIMEOUT_MS    2500
#define WALL_FOLLOW_CALIB_REPEATS       8
#define WALL_FOLLOW_CALIB_SETTLE_MS     500
#define WALL_FOLLOW_PRINT_EVERY_MS      20

#define BNO_TEST_MODE_NDOF 0
#define BNO_TEST_MODE_IMUPLUS 1

#define MM_PER_TICK 0.595f
#define CONTROL_DT_MS 10
#define CONTROL_DT_SEC 0.010f
#define TEST_DURATION_MS 3000
#define MOTION_TEST_DIST_MM      1000.0f
#define MOTION_TEST_MAX_SPEED    800.0f
#define MOTION_TEST_TIMEOUT_MS   6000
#define BRAKE_DIST_FRAC        0.6f
#define TRACK_WIDTH_MM 78.7f
#define RAD_TO_DEG     57.2958f

#define SEQ_STRAIGHT_DIST_MM    254.0f
#define SEQ_MAX_SPEED_MM_S      800.0f
#define SEQ_MOVE_TIMEOUT_MS     3000

#define APPROACH_CALIB_SPEED_MM_S   400.0f
#define APPROACH_CALIB_RUN_TIME_MS  700
#define APPROACH_CALIB_REPEATS      10
#define APPROACH_CALIB_SETTLE_MS    400

#define RANDOM_ROAM_DIST_MM       254.0f
#define RANDOM_ROAM_SPEED_MM_S    800.0f
#define RANDOM_ROAM_SETTLE_MS     200

int16_t rawL;
int16_t rawR;
int32_t cntL;
int32_t cntR;

BNO055_Status bnoStatus;
BNO055_Euler bnoEuler;
BNO055_Calib bnoCalib;
BNO055_Gyro bnoGyro;

uint8_t bnoChipID = 0;
uint8_t bnoSysStat = 0;
uint8_t bnoSysErr = 0;

float bnoHeading = 0.0f;
float bnoRoll = 0.0f;
float bnoPitch = 0.0f;
float bnoGz = 0.0f;

// IR live debug variables
volatile uint16_t dbg_irL = 0;
volatile uint16_t dbg_irF = 0;
volatile uint16_t dbg_irR = 0;
volatile float dbg_distL = 0.0f;
volatile float dbg_distF = 0.0f;
volatile float dbg_distR = 0.0f;
volatile uint8_t dbg_wallL = 0;
volatile uint8_t dbg_wallF = 0;
volatile uint8_t dbg_wallR = 0;
volatile uint8_t dbg_appL = 0;
volatile uint8_t dbg_appF = 0;
volatile uint8_t dbg_appR = 0;

#define GZ_FILTER_ALPHA    0.05f
#define GZ_DEADBAND_DPS    3.0f

static float gzFiltered = 0.0f;
static float bnoGzFiltered = 0.0f;

static void GZ_FilterReset(void)
{
    bnoGz = 0.0f;
    gzFiltered = 0.0f;
    bnoGzFiltered = 0.0f;
}

static float GZ_FilterUpdate(float rawGz)
{
    gzFiltered = gzFiltered + GZ_FILTER_ALPHA * (rawGz - gzFiltered);
    bnoGzFiltered = gzFiltered;
    return bnoGzFiltered;
}

static void BT_SendLine(const char *msg)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), 1000);
    HAL_UART_Transmit(&huart2, (uint8_t *)"\r\n", 2, 1000);
}

static void BT_Sendf(const char *fmt, ...)
{
    char buf[256];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    HAL_UART_Transmit(&huart2, (uint8_t *)buf, strlen(buf), 1000);
}

static void Wait_For_Button(void)
{
    Button_Init();
    while (!Button_WasPressed()) { HAL_Delay(5); }
    HAL_Delay(200);
}

static float AngleDiff(float now, float start)
{
    return BNO055_AngleError(now, start);
}

static void Run_TurnProfileCalibration(void)
{
    const int16_t pwmList[] = {300, 350, 400, 450, 500, 550, 600, 650, 700};
    const int pwmCount = sizeof(pwmList) / sizeof(pwmList[0]);
    float startHeading = 0.0f;
    float heading = 0.0f;
    float delta = 0.0f;
    BNO055_Gyro gyro;
    BNO055_Status st;
    uint32_t startTime;
    uint32_t now;
    uint32_t lastPrint;

    BT_SendLine("===== TURN PROFILE CALIBRATION READY =====");
    BT_SendLine("Place robot on ground. Press button.");
    Wait_For_Button();

    BT_SendLine("===== TEST 1: TURN BREAKAWAY PWM =====");
    BT_SendLine("pwm,dir,startHeading,endHeading,deltaHeading,moved");
    for (int i = 0; i < pwmCount; i++)
    {
        int16_t pwm = pwmList[i];
        BNO055_ReadHeading(&hi2c1, &startHeading);
        Motor_SetRawPWM(pwm, -pwm);
        HAL_Delay(350);
        Motor_Stop(MOTOR_BRAKE);
        HAL_Delay(250);
        BNO055_ReadHeading(&hi2c1, &heading);
        delta = AngleDiff(heading, startHeading);
        BT_Sendf("%d,RIGHT,%.2f,%.2f,%.2f,%d\r\n", pwm, startHeading, heading, delta, fabsf(delta) > 2.0f ? 1 : 0);
        HAL_Delay(800);

        BNO055_ReadHeading(&hi2c1, &startHeading);
        Motor_SetRawPWM(-pwm, pwm);
        HAL_Delay(350);
        Motor_Stop(MOTOR_BRAKE);
        HAL_Delay(250);
        BNO055_ReadHeading(&hi2c1, &heading);
        delta = AngleDiff(heading, startHeading);
        BT_Sendf("%d,LEFT,%.2f,%.2f,%.2f,%d\r\n", pwm, startHeading, heading, delta, fabsf(delta) > 2.0f ? 1 : 0);
        HAL_Delay(1000);
    }

    BT_SendLine("===== TEST 2: ANGULAR SPEED VS PWM =====");
    BT_SendLine("time_ms,pwm,dir,heading,gz");
    for (int i = 0; i < pwmCount; i++)
    {
        int16_t pwm = pwmList[i];
        startTime = HAL_GetTick();
        lastPrint = startTime;
        Motor_SetRawPWM(pwm, -pwm);
        while ((HAL_GetTick() - startTime) < 700)
        {
            now = HAL_GetTick();
            if ((now - lastPrint) >= 20)
            {
                BNO055_ReadHeading(&hi2c1, &heading);
                st = BNO055_ReadGyro(&hi2c1, &gyro);
                if (st == BNO055_OK)
                    BT_Sendf("%lu,%d,RIGHT,%.2f,%.2f\r\n", now - startTime, pwm, heading, gyro.z_dps);
                lastPrint = now;
            }
        }
        Motor_Stop(MOTOR_BRAKE);
        HAL_Delay(1000);

        startTime = HAL_GetTick();
        lastPrint = startTime;
        Motor_SetRawPWM(-pwm, pwm);
        while ((HAL_GetTick() - startTime) < 700)
        {
            now = HAL_GetTick();
            if ((now - lastPrint) >= 20)
            {
                BNO055_ReadHeading(&hi2c1, &heading);
                st = BNO055_ReadGyro(&hi2c1, &gyro);
                if (st == BNO055_OK)
                    BT_Sendf("%lu,%d,LEFT,%.2f,%.2f\r\n", now - startTime, pwm, heading, gyro.z_dps);
                lastPrint = now;
            }
        }
        Motor_Stop(MOTOR_BRAKE);
        HAL_Delay(1000);
    }

    BT_SendLine("===== TEST 3: TURN BRAKE DISTANCE =====");
    BT_SendLine("pwm,dir,gzAtBrake,headingAtBrake,finalHeading,extraDeg");
    for (int i = 0; i < pwmCount; i++)
    {
        int16_t pwm = pwmList[i];
        Motor_SetRawPWM(pwm, -pwm);
        HAL_Delay(350);
        BNO055_ReadHeading(&hi2c1, &startHeading);
        BNO055_ReadGyro(&hi2c1, &gyro);
        Motor_Stop(MOTOR_BRAKE);
        HAL_Delay(400);
        BNO055_ReadHeading(&hi2c1, &heading);
        delta = AngleDiff(heading, startHeading);
        BT_Sendf("%d,RIGHT,%.2f,%.2f,%.2f,%.2f\r\n", pwm, gyro.z_dps, startHeading, heading, delta);
        HAL_Delay(1200);

        Motor_SetRawPWM(-pwm, pwm);
        HAL_Delay(350);
        BNO055_ReadHeading(&hi2c1, &startHeading);
        BNO055_ReadGyro(&hi2c1, &gyro);
        Motor_Stop(MOTOR_BRAKE);
        HAL_Delay(400);
        BNO055_ReadHeading(&hi2c1, &heading);
        delta = AngleDiff(heading, startHeading);
        BT_Sendf("%d,LEFT,%.2f,%.2f,%.2f,%.2f\r\n", pwm, gyro.z_dps, startHeading, heading, delta);
        HAL_Delay(1200);
    }

    Motor_Stop(MOTOR_BRAKE);
    BT_SendLine("===== TURN PROFILE CALIBRATION DONE =====");
    while (1) { Motor_Stop(MOTOR_BRAKE); HAL_Delay(1000); }
}

static void Run_SpeedControllerTest(float targetL, float targetR)
{
    uint32_t startTime, lastControlTime, lastPrintTime, now, dt_ms;
    float dt_sec;
    uint32_t lastControlDtMs = 0;

    Encoder_ResetAll();
    Controller_Reset();
    BT_Sendf("\r\n===== SPEED CONTROLLER TEST L:%.1f R:%.1f =====\r\n", targetL, targetR);
    BT_SendLine("time_ms,dt_ms,targetL,targetR,measL,measR,pwmL,pwmR,cntL,cntR");
    startTime = HAL_GetTick();
    lastControlTime = startTime;
    lastPrintTime = startTime;

    while ((HAL_GetTick() - startTime) < TEST_DURATION_MS)
    {
        now = HAL_GetTick();
        dt_ms = now - lastControlTime;
        if (dt_ms >= CONTROL_DT_MS)
        {
            dt_sec = dt_ms / 1000.0f;
            Controller_UpdateSpeed(targetL, targetR, dt_sec, 0, 0.0f, 800.0f, 3000.0f);
            lastControlDtMs = dt_ms;
            lastControlTime = now;
        }
        if ((now - lastPrintTime) >= 100)
        {
            Encoder_Update();
            cntL = Encoder_GetLeftCount();
            cntR = Encoder_GetRightCount();
            BT_Sendf("%lu,%lu,%.1f,%.1f,%.2f,%.2f,%d,%d,%ld,%ld\r\n",
                     now - startTime, lastControlDtMs, targetL, targetR,
                     Controller_GetLeftMeasuredSpeed(), Controller_GetRightMeasuredSpeed(),
                     Controller_GetLeftPWM(), Controller_GetRightPWM(), cntL, cntR);
            lastPrintTime = now;
        }
    }
    Motor_Stop(MOTOR_BRAKE);
    HAL_Delay(100);
    Controller_Reset();
    HAL_Delay(1000);
}

static void Run_MotorSpeedTest(int16_t pwmL, int16_t pwmR)
{
    int32_t prevL = 0, prevR = 0, currL = 0, currR = 0, dL = 0, dR = 0;
    float speedL = 0.0f, speedR = 0.0f;
    uint32_t startTime, lastTime, now, dt_ms;
    float dt_sec;

    Encoder_ResetAll();
    Encoder_Update();
    prevL = Encoder_GetLeftCount();
    prevR = Encoder_GetRightCount();
    BT_Sendf("\r\n===== PWM TEST L:%d R:%d =====\r\n", pwmL, pwmR);
    BT_SendLine("time_ms,dt_ms,pwmL,pwmR,cntL,cntR,dL,dR,speedL_mm_s,speedR_mm_s");
    Motor_SetRawPWM(pwmL, pwmR);
    startTime = HAL_GetTick();
    lastTime = startTime;

    while ((HAL_GetTick() - startTime) < TEST_DURATION_MS)
    {
        now = HAL_GetTick();
        dt_ms = now - lastTime;
        if (dt_ms >= CONTROL_DT_MS)
        {
            dt_sec = dt_ms / 1000.0f;
            Encoder_Update();
            currL = Encoder_GetLeftCount();
            currR = Encoder_GetRightCount();
            dL = currL - prevL;
            dR = currR - prevR;
            speedL = (dL / dt_sec) * MM_PER_TICK;
            speedR = (dR / dt_sec) * MM_PER_TICK;
            BT_Sendf("%lu,%lu,%d,%d,%ld,%ld,%ld,%ld,%.2f,%.2f\r\n",
                     now - startTime, dt_ms, pwmL, pwmR, currL, currR, dL, dR, speedL, speedR);
            prevL = currL;
            prevR = currR;
            lastTime = now;
        }
    }
    Motor_Stop(MOTOR_BRAKE);
    HAL_Delay(100);
    HAL_Delay(1000);
}

static void Run_DeadzoneTest(int16_t pwm)
{
    Encoder_ResetAll();
    Encoder_Update();
    BT_Sendf("\r\n===== DEADZONE TEST PWM:%d =====\r\n", pwm);
    int32_t startL = Encoder_GetLeftCount();
    int32_t startR = Encoder_GetRightCount();
    Motor_SetRawPWM(pwm, pwm);
    HAL_Delay(2000);
    Motor_SetRawPWM(0, 0);
    Encoder_Update();
    int32_t endL = Encoder_GetLeftCount();
    int32_t endR = Encoder_GetRightCount();
    BT_Sendf("LeftTicks:%ld RightTicks:%ld\r\n", endL - startL, endR - startR);
}

static void Run_ApproachBrakeCalib(void)
{
    uint32_t startTime, lastControlTime, now, dt_ms;
    float dt_sec;

    int32_t cntL_local, cntR_local;
    float distL, distR, avgDist;

    float leftTargetSpeed = 0.0f;
    float rightTargetSpeed = 0.0f;

    float brakeStartDist;
    float brakeDist;
    float sumBrakeDist = 0.0f;
    float minBrakeDist = 9999.0f;
    float maxBrakeDist = 0.0f;

    BT_SendLine("===== APPROACH BRAKE CALIB START =====");
    BT_Sendf("speed=%.1fmm/s repeats=%d\r\n",
             APPROACH_CALIB_SPEED_MM_S,
             APPROACH_CALIB_REPEATS);
    BT_SendLine("run,brakeStartDist,finalDist,brakeDist");

    for (int run = 1; run <= APPROACH_CALIB_REPEATS; run++)
    {
        Encoder_ResetAll();
        Encoder_Update();

        Controller_Reset();
        StraightController_Reset();

        // Startup kick so the robot actually breaks static friction
        Motor_SetRawPWM(650, 650);
        HAL_Delay(120);

        Motor_Stop(MOTOR_COAST);
        HAL_Delay(20);

        Controller_Reset();
        StraightController_Reset();

        startTime = HAL_GetTick();
        lastControlTime = startTime;

        while ((HAL_GetTick() - startTime) < APPROACH_CALIB_RUN_TIME_MS)
        {
            now = HAL_GetTick();
            dt_ms = now - lastControlTime;

            if (dt_ms >= CONTROL_DT_MS)
            {
                dt_sec = dt_ms / 1000.0f;

                Encoder_Update();

                cntL_local = Encoder_GetLeftCount();
                cntR_local = Encoder_GetRightCount();

                distL = ((float)cntL_local) * MM_PER_TICK;
                distR = ((float)cntR_local) * MM_PER_TICK;

                StraightController_Update(APPROACH_CALIB_SPEED_MM_S,
                                          distL,
                                          distR,
                                          0,
                                          &leftTargetSpeed,
                                          &rightTargetSpeed,
                                          dt_sec);

                Controller_UpdateSpeed(leftTargetSpeed,
                                       rightTargetSpeed,
                                       dt_sec,
                                       0,
                                       APPROACH_CALIB_SPEED_MM_S,
                                       APPROACH_CALIB_SPEED_MM_S,
                                       1000.0f);

                lastControlTime = now;
            }
        }

        Encoder_Update();

        cntL_local = Encoder_GetLeftCount();
        cntR_local = Encoder_GetRightCount();

        distL = ((float)cntL_local) * MM_PER_TICK;
        distR = ((float)cntR_local) * MM_PER_TICK;

        brakeStartDist = 0.5f * (distL + distR);

        Motor_Stop(MOTOR_BRAKE);
        HAL_Delay(APPROACH_CALIB_SETTLE_MS);

        Encoder_Update();

        cntL_local = Encoder_GetLeftCount();
        cntR_local = Encoder_GetRightCount();

        distL = ((float)cntL_local) * MM_PER_TICK;
        distR = ((float)cntR_local) * MM_PER_TICK;

        avgDist = 0.5f * (distL + distR);
        brakeDist = avgDist - brakeStartDist;

        sumBrakeDist += brakeDist;

        if (brakeDist < minBrakeDist) minBrakeDist = brakeDist;
        if (brakeDist > maxBrakeDist) maxBrakeDist = brakeDist;

        BT_Sendf("%d,%.2f,%.2f,%.2f\r\n",
                 run,
                 brakeStartDist,
                 avgDist,
                 brakeDist);

        Motor_Stop(MOTOR_BRAKE);

        Controller_Reset();
        StraightController_Reset();

        HAL_Delay(1200);
    }

    BT_Sendf("AVG_BRAKE_DIST=%.2f MIN=%.2f MAX=%.2f\r\n",
             sumBrakeDist / APPROACH_CALIB_REPEATS,
             minBrakeDist,
             maxBrakeDist);

    BT_SendLine("===== APPROACH BRAKE CALIB DONE =====");

    while (1)
    {
        Motor_Stop(MOTOR_BRAKE);
        HAL_Delay(1000);
    }
}

static void Run_MotionProfileTest(void)
{
    uint32_t startTime, lastControlTime, lastPrintTime, now, dt_ms;
    float dt_sec;
    uint32_t lastControlDtMs = 0;

    int32_t cntL_local, cntR_local;
    float distL, distR, avgDist;
    float targetSpeed = 0.0f;
    float actualAvgSpeed = 0.0f;
    float leftTargetSpeed = 0.0f;
    float rightTargetSpeed = 0.0f;
    uint8_t isDecelerating = 0;

    Encoder_ResetAll();
    Encoder_Update();

    Controller_Reset();
    MotionProfile_Reset();
    MotionProfile_SetMove(MOTION_TEST_DIST_MM, MOTION_TEST_MAX_SPEED);
    StraightController_Reset();

    startTime = HAL_GetTick();
    lastControlTime = startTime;
    lastPrintTime = startTime;

    BT_SendLine("\r\n===== MOTION PROFILE TEST STARTED =====");
    BT_SendLine("time_ms,dt_ms,targetSpeed,leftTarget,rightTarget,actualAvgSpeed,measL,measR,pwmL,pwmR,cntL,cntR,distL,distR,avgDist,straightErr,straightCorr");

    while ((HAL_GetTick() - startTime) < MOTION_TEST_TIMEOUT_MS)
    {
        now = HAL_GetTick();
        dt_ms = now - lastControlTime;

        if (dt_ms >= CONTROL_DT_MS)
        {
            dt_sec = dt_ms / 1000.0f;

            Encoder_Update();

            cntL_local = Encoder_GetLeftCount();
            cntR_local = Encoder_GetRightCount();

            distL = ((float)cntL_local) * MM_PER_TICK;
            distR = ((float)cntR_local) * MM_PER_TICK;
            avgDist = 0.5f * (distL + distR);

            actualAvgSpeed = 0.5f * (
                fabsf(Controller_GetLeftMeasuredSpeed()) +
                fabsf(Controller_GetRightMeasuredSpeed())
            );

            targetSpeed = MotionProfile_Update(avgDist, actualAvgSpeed, dt_sec);

            if (targetSpeed <= 0.0f)
            {
                Motor_Stop(MOTOR_BRAKE);
                HAL_Delay(300);

                Encoder_Update();

                cntL_local = Encoder_GetLeftCount();
                cntR_local = Encoder_GetRightCount();

                distL = ((float)cntL_local) * MM_PER_TICK;
                distR = ((float)cntR_local) * MM_PER_TICK;
                avgDist = 0.5f * (distL + distR);

                BT_Sendf("===== FINAL: cntL=%ld cntR=%ld distL=%.1f distR=%.1f avgDist=%.1f target=%.1f error=%.1f =====\r\n",
                         cntL_local,
                         cntR_local,
                         distL,
                         distR,
                         avgDist,
                         MOTION_TEST_DIST_MM,
                         MOTION_TEST_DIST_MM - avgDist);

                BT_SendLine("===== MOTION PROFILE DONE =====");

                Controller_Reset();
                StraightController_Reset();

                while (1)
                {
                    Motor_Stop(MOTOR_BRAKE);
                    HAL_Delay(10);
                }
            }

            isDecelerating = MotionProfile_IsDecelerating();

            StraightController_Update(targetSpeed,
                                      distL,
                                      distR,
                                      isDecelerating,
                                      &leftTargetSpeed,
                                      &rightTargetSpeed,
                                      dt_sec);

            Controller_UpdateSpeed(leftTargetSpeed,
                                   rightTargetSpeed,
                                   dt_sec,
                                   isDecelerating,
                                   targetSpeed,
                                   MOTION_TEST_MAX_SPEED,
                                   MOTION_TEST_DIST_MM);

            lastControlDtMs = dt_ms;
            lastControlTime = now;
        }

        if ((now - lastPrintTime) >= 100)
        {
            Encoder_Update();

            cntL_local = Encoder_GetLeftCount();
            cntR_local = Encoder_GetRightCount();

            distL = ((float)cntL_local) * MM_PER_TICK;
            distR = ((float)cntR_local) * MM_PER_TICK;
            avgDist = 0.5f * (distL + distR);

            BT_Sendf("%lu,%lu,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%d,%d,%ld,%ld,%.2f,%.2f,%.2f,%.2f,%.2f\r\n",
                     now - startTime,
                     lastControlDtMs,
                     targetSpeed,
                     leftTargetSpeed,
                     rightTargetSpeed,
                     actualAvgSpeed,
                     Controller_GetLeftMeasuredSpeed(),
                     Controller_GetRightMeasuredSpeed(),
                     Controller_GetLeftPWM(),
                     Controller_GetRightPWM(),
                     cntL_local,
                     cntR_local,
                     distL,
                     distR,
                     avgDist,
                     StraightController_GetError(),
                     StraightController_GetCorrection());

            lastPrintTime = now;
        }
    }

    Motor_Stop(MOTOR_BRAKE);
    HAL_Delay(300);

    Encoder_Update();

    cntL_local = Encoder_GetLeftCount();
    cntR_local = Encoder_GetRightCount();

    distL = ((float)cntL_local) * MM_PER_TICK;
    distR = ((float)cntR_local) * MM_PER_TICK;
    avgDist = 0.5f * (distL + distR);

    BT_Sendf("===== TIMEOUT FINAL: cntL=%ld cntR=%ld distL=%.1f distR=%.1f avgDist=%.1f target=%.1f error=%.1f =====\r\n",
             cntL_local,
             cntR_local,
             distL,
             distR,
             avgDist,
             MOTION_TEST_DIST_MM,
             MOTION_TEST_DIST_MM - avgDist);

    BT_SendLine("===== MOTION PROFILE TIMEOUT =====");

    Controller_Reset();
    StraightController_Reset();

    while (1)
    {
        Motor_Stop(MOTOR_BRAKE);
        HAL_Delay(10);
    }
}

static void Run_StraightSpeedControllerTest(float baseSpeed)
{
    uint32_t startTime, lastControlTime, lastPrintTime, now, dt_ms;
    float dt_sec;
    uint32_t lastControlDtMs = 0;
    int32_t cntL_local, cntR_local;
    float distL, distR, measL, measR, encoderGz = 0.0f;
    float leftTargetSpeed = 0.0f, rightTargetSpeed = 0.0f;
    int32_t prevGzCntL = 0, prevGzCntR = 0;
    uint32_t prevGzTime = 0;
    float gzDtSec = 0.0f;

    Encoder_ResetAll();
    Encoder_Update();
    Controller_Reset();
    StraightController_Reset();
    GZ_FilterReset();

    BT_Sendf("\r\n===== STRAIGHT + SPEED TEST BASE:%.1f =====\r\n", baseSpeed);
    BT_SendLine("time_ms,dt_ms,baseSpeed,leftTarget,rightTarget,measL,measR,cntL,cntR,straightErr,rawGz,filteredGz,encoderGz");

    startTime = HAL_GetTick();
    lastControlTime = startTime;
    lastPrintTime = startTime;
    prevGzTime = startTime;
    prevGzCntL = Encoder_GetLeftCount();
    prevGzCntR = Encoder_GetRightCount();

    while ((HAL_GetTick() - startTime) < TEST_DURATION_MS)
    {
        now = HAL_GetTick();
        dt_ms = now - lastControlTime;
        if (dt_ms >= CONTROL_DT_MS)
        {
            dt_sec = dt_ms / 1000.0f;
            Encoder_Update();
            cntL_local = Encoder_GetLeftCount();
            cntR_local = Encoder_GetRightCount();
            distL = ((float)cntL_local) * MM_PER_TICK;
            distR = ((float)cntR_local) * MM_PER_TICK;
            bnoStatus = BNO055_ReadGyro(&hi2c1, &bnoGyro);
            if (bnoStatus == BNO055_OK) { bnoGz = bnoGyro.z_dps; GZ_FilterUpdate(bnoGz); }
            StraightController_Update(baseSpeed,
                                      distL,
                                      distR,
                                      0,
                                      &leftTargetSpeed,
                                      &rightTargetSpeed,
                                      dt_sec);
            Controller_UpdateSpeed(leftTargetSpeed, rightTargetSpeed, dt_sec, 0, baseSpeed, baseSpeed, 3000.0f);
            lastControlDtMs = dt_ms;
            lastControlTime = now;
        }
        if ((now - lastPrintTime) >= 50)
        {
            Encoder_Update();
            cntL_local = Encoder_GetLeftCount();
            cntR_local = Encoder_GetRightCount();
            measL = Controller_GetLeftMeasuredSpeed();
            measR = Controller_GetRightMeasuredSpeed();
            gzDtSec = (now - prevGzTime) / 1000.0f;
            if (gzDtSec > 0.0f)
            {
                float dDistL = ((float)(cntL_local - prevGzCntL)) * MM_PER_TICK;
                float dDistR = ((float)(cntR_local - prevGzCntR)) * MM_PER_TICK;
                encoderGz = (((dDistR - dDistL) / gzDtSec) / TRACK_WIDTH_MM) * RAD_TO_DEG;
            }
            prevGzCntL = cntL_local; prevGzCntR = cntR_local; prevGzTime = now;
            BT_Sendf("%lu,%lu,%.2f,%.2f,%.2f,%.2f,%.2f,%ld,%ld,%.2f,%.2f,%.2f,%.2f\r\n",
                     now - startTime, lastControlDtMs, baseSpeed, leftTargetSpeed, rightTargetSpeed,
                     measL, measR, cntL_local, cntR_local, StraightController_GetError(),
                     bnoGz, bnoGzFiltered, encoderGz);
            lastPrintTime = now;
        }
    }
    Motor_Stop(MOTOR_BRAKE);
    HAL_Delay(100);
    Controller_Reset();
    StraightController_Reset();
    GZ_FilterReset();
    BT_SendLine("===== STRAIGHT + SPEED TEST DONE =====");
    HAL_Delay(1000);
}

static void Run_TurnSignTest(void)
{
    float startHeading = 0.0f, endHeading = 0.0f, delta = 0.0f;
    BT_SendLine("===== TURN SIGN TEST READY =====");
    BT_SendLine("Place robot on ground. Press button.");
    Wait_For_Button();

    BNO055_ReadHeading(&hi2c1, &startHeading);
    BT_SendLine("===== TEST 1: Motor_SetRawPWM(600, -600) =====");
    BT_Sendf("startHeading: %.2f\r\n", startHeading);
    Motor_SetRawPWM(600, -600);
    HAL_Delay(1000);
    Motor_Stop(MOTOR_BRAKE);
    HAL_Delay(300);
    BNO055_ReadHeading(&hi2c1, &endHeading);
    delta = BNO055_AngleError(endHeading, startHeading);
    BT_Sendf("endHeading: %.2f  delta: %.2f\r\n", endHeading, delta);
    HAL_Delay(1500);

    BNO055_ReadHeading(&hi2c1, &startHeading);
    BT_SendLine("===== TEST 2: Motor_SetRawPWM(-600, 600) =====");
    BT_Sendf("startHeading: %.2f\r\n", startHeading);
    Motor_SetRawPWM(-600, 600);
    HAL_Delay(1000);
    Motor_Stop(MOTOR_BRAKE);
    HAL_Delay(300);
    BNO055_ReadHeading(&hi2c1, &endHeading);
    delta = BNO055_AngleError(endHeading, startHeading);
    BT_Sendf("endHeading: %.2f  delta: %.2f\r\n", endHeading, delta);

    BT_SendLine("===== TURN SIGN TEST DONE =====");
    while (1) { Motor_Stop(MOTOR_BRAKE); HAL_Delay(1000); }
}

static void Run_StraightMove(float distance_mm, float maxSpeed)
{
    uint32_t startTime, lastControlTime, now, dt_ms;
    float dt_sec;
    int32_t cntL_local, cntR_local;
    float distL, distR, avgDist;
    float targetSpeed, actualAvgSpeed;
    float leftTargetSpeed, rightTargetSpeed;

    MotionGuard_ClearAbort();
    MotionGuard_SetEnabled(1);

    Encoder_ResetAll();
    Encoder_Update();
    Controller_Reset();
    MotionProfile_Reset();
    StraightController_Reset();
    MotionProfile_SetMove(distance_mm, maxSpeed);

    startTime = HAL_GetTick();
    lastControlTime = startTime;

    BT_Sendf("===== STRAIGHT %.1fmm START =====\r\n", distance_mm);

    while ((HAL_GetTick() - startTime) < SEQ_MOVE_TIMEOUT_MS)
    {
        now = HAL_GetTick();
        dt_ms = now - lastControlTime;

        if (MotionGuard_WasAborted())
        {
            Motor_Stop(MOTOR_BRAKE);

            MotionGuard_SetEnabled(0);
            MotionGuard_ClearAbort();

            Controller_Reset();
            StraightController_Reset();

            BT_SendLine("===== STRAIGHT ABORTED: FRONT WALL =====");
            return;
        }

        if (dt_ms >= CONTROL_DT_MS)
        {
            dt_sec = dt_ms / 1000.0f;

            Encoder_Update();

            cntL_local = Encoder_GetLeftCount();
            cntR_local = Encoder_GetRightCount();

            distL = ((float)cntL_local) * MM_PER_TICK;
            distR = ((float)cntR_local) * MM_PER_TICK;
            avgDist = 0.5f * (distL + distR);

            actualAvgSpeed = 0.5f *
            (
                fabsf(Controller_GetLeftMeasuredSpeed()) +
                fabsf(Controller_GetRightMeasuredSpeed())
            );

            targetSpeed = MotionProfile_Update(avgDist,
                                               actualAvgSpeed,
                                               dt_sec);

            if (targetSpeed <= 0.0f)
            {
                Motor_Stop(MOTOR_BRAKE);
                HAL_Delay(250);

                Encoder_Update();

                cntL_local = Encoder_GetLeftCount();
                cntR_local = Encoder_GetRightCount();

                distL = ((float)cntL_local) * MM_PER_TICK;
                distR = ((float)cntR_local) * MM_PER_TICK;
                avgDist = 0.5f * (distL + distR);

                MotionGuard_SetEnabled(0);
                MotionGuard_ClearAbort();

                BT_Sendf("===== STRAIGHT DONE avgDist=%.1f L=%.1f R=%.1f =====\r\n",
                         avgDist,
                         distL,
                         distR);

                Controller_Reset();
                StraightController_Reset();
                return;
            }

            StraightController_Update(targetSpeed,
                                      distL,
                                      distR,
                                      MotionProfile_IsDecelerating(),
                                      &leftTargetSpeed,
                                      &rightTargetSpeed,
                                      dt_sec);

            Controller_UpdateSpeed(leftTargetSpeed,
                                   rightTargetSpeed,
                                   dt_sec,
                                   MotionProfile_IsDecelerating(),
                                   targetSpeed,
                                   maxSpeed,
                                   distance_mm);

            lastControlTime = now;
        }
    }

    Motor_Stop(MOTOR_BRAKE);

    MotionGuard_SetEnabled(0);
    MotionGuard_ClearAbort();

    BT_SendLine("===== STRAIGHT TIMEOUT =====");

    Controller_Reset();
    StraightController_Reset();
}

static void Run_StraightTurnSequence(void)
{
    const float angles[] = {45.0f, 90.0f, 180.0f};
    const int angleCount = sizeof(angles) / sizeof(angles[0]);

    BT_SendLine("===== STRAIGHT + TURN SEQUENCE START =====");

    for (int i = 0; i < angleCount; i++)
    {
        float a = angles[i];

        BT_Sendf("===== TEST: STRAIGHT 10in -> RIGHT %.0f -> STRAIGHT 10in =====\r\n", a);
        Run_StraightMove(SEQ_STRAIGHT_DIST_MM, SEQ_MAX_SPEED_MM_S);
        HAL_Delay(300);
        Turn_Execute(&hi2c1, a, TURN_RIGHT);
        HAL_Delay(300);
        Run_StraightMove(SEQ_STRAIGHT_DIST_MM, SEQ_MAX_SPEED_MM_S);
        HAL_Delay(800);

        BT_Sendf("===== TEST: STRAIGHT 10in -> LEFT %.0f -> STRAIGHT 10in =====\r\n", a);
        Run_StraightMove(SEQ_STRAIGHT_DIST_MM, SEQ_MAX_SPEED_MM_S);
        HAL_Delay(300);
        Turn_Execute(&hi2c1, a, TURN_LEFT);
        HAL_Delay(300);
        Run_StraightMove(SEQ_STRAIGHT_DIST_MM, SEQ_MAX_SPEED_MM_S);
        HAL_Delay(1000);
    }

    Motor_Stop(MOTOR_BRAKE);
    BT_SendLine("===== STRAIGHT + TURN SEQUENCE DONE =====");
    while (1) { Motor_Stop(MOTOR_BRAKE); HAL_Delay(1000); }
}

// ============================================================
// MAIN CALIBRATION SWITCH
// ============================================================

static void Run_DecelNatural(void)
{
    // Motion profile decels all the way to 0 — no hard brake, no decel bias.
    // Motors coast to a stop naturally. Measures where each wheel ends up.
    uint32_t startTime, lastControlTime, lastPrintTime, now, dt_ms;
    float dt_sec;
    uint32_t lastControlDtMs = 0;
    int32_t cntL_local, cntR_local;
    float distL, distR, avgDist;
    float targetSpeed = 0.0f, actualAvgSpeed = 0.0f;
    float leftTargetSpeed = 0.0f, rightTargetSpeed = 0.0f;

    Encoder_ResetAll();
    Encoder_Update();
    Controller_Reset();
    MotionProfile_Reset();
    MotionProfile_SetMove(MOTION_TEST_DIST_MM, MOTION_TEST_MAX_SPEED);
    StraightController_Reset();

    startTime = HAL_GetTick();
    lastControlTime = startTime;
    lastPrintTime = startTime;

    BT_SendLine("\r\n===== DECEL NATURAL TEST =====");
    BT_SendLine("No brake, no decel bias. Profile ramps to 0, motors coast.");
    BT_SendLine("time_ms,targetSpeed,leftTarget,rightTarget,measL,measR,pwmL,pwmR,distL,distR,avgDist");

    while ((HAL_GetTick() - startTime) < MOTION_TEST_TIMEOUT_MS)
    {
        now = HAL_GetTick();
        dt_ms = now - lastControlTime;

        if (dt_ms >= CONTROL_DT_MS)
        {
            dt_sec = dt_ms / 1000.0f;
            Encoder_Update();
            cntL_local = Encoder_GetLeftCount();
            cntR_local = Encoder_GetRightCount();
            distL = ((float)cntL_local) * MM_PER_TICK;
            distR = ((float)cntR_local) * MM_PER_TICK;
            avgDist = 0.5f * (distL + distR);
            actualAvgSpeed = 0.5f * (fabsf(Controller_GetLeftMeasuredSpeed()) +
                                     fabsf(Controller_GetRightMeasuredSpeed()));
            targetSpeed = MotionProfile_Update(avgDist, actualAvgSpeed, dt_sec);

            StraightController_Update(targetSpeed,
                                      distL,
                                      distR,
                                      0,
                                      &leftTargetSpeed,
                                      &rightTargetSpeed,
                                      dt_sec);
            // isDecelerating always 0 — bias disabled
            Controller_UpdateSpeed(leftTargetSpeed, rightTargetSpeed, dt_sec,
                                   0, targetSpeed, MOTION_TEST_MAX_SPEED, MOTION_TEST_DIST_MM);

            // Profile returned 0 — just let motors coast, no brake
            if (targetSpeed <= 0.0f)
            {
                Motor_Stop(MOTOR_COAST);
                HAL_Delay(800);
                Encoder_Update();
                cntL_local = Encoder_GetLeftCount();
                cntR_local = Encoder_GetRightCount();
                distL = ((float)cntL_local) * MM_PER_TICK;
                distR = ((float)cntR_local) * MM_PER_TICK;
                avgDist = 0.5f * (distL + distR);
                BT_Sendf("===== FINAL: distL=%.1f distR=%.1f avgDist=%.1f target=%.1f error=%.1f =====\r\n",
                         distL, distR, avgDist, MOTION_TEST_DIST_MM, MOTION_TEST_DIST_MM - avgDist);
                BT_SendLine("===== DECEL NATURAL DONE =====");
                Controller_Reset();
                StraightController_Reset();
                return;
            }

            lastControlDtMs = dt_ms;
            lastControlTime = now;
        }

        if ((now - lastPrintTime) >= 100)
        {
            Encoder_Update();
            cntL_local = Encoder_GetLeftCount();
            cntR_local = Encoder_GetRightCount();
            distL = ((float)cntL_local) * MM_PER_TICK;
            distR = ((float)cntR_local) * MM_PER_TICK;
            avgDist = 0.5f * (distL + distR);
            BT_Sendf("%lu,%lu,%.1f,%.1f,%.1f,%.1f,%.1f,%d,%d,%.1f,%.1f,%.1f\r\n",
                     now - startTime, lastControlDtMs, targetSpeed,
                     leftTargetSpeed, rightTargetSpeed,
                     Controller_GetLeftMeasuredSpeed(), Controller_GetRightMeasuredSpeed(),
                     Controller_GetLeftPWM(), Controller_GetRightPWM(),
                     distL, distR, avgDist);
            lastPrintTime = now;
        }
    }

    Motor_Stop(MOTOR_COAST);
    HAL_Delay(800);
    Encoder_Update();
    cntL_local = Encoder_GetLeftCount();
    cntR_local = Encoder_GetRightCount();
    distL = ((float)cntL_local) * MM_PER_TICK;
    distR = ((float)cntR_local) * MM_PER_TICK;
    avgDist = 0.5f * (distL + distR);
    BT_Sendf("===== TIMEOUT FINAL: distL=%.1f distR=%.1f avgDist=%.1f =====\r\n",
             distL, distR, avgDist);
    BT_SendLine("===== DECEL NATURAL TIMEOUT =====");
    Controller_Reset();
    StraightController_Reset();
}

// Brake speed threshold — hard brake when target speed drops to this value
#define DECEL_BRAKE_SPEED_MM_S  500.0f

static void Run_DecelThreshold(void)
{
    /*
       Follows the motion profile with decel bias disabled.

       Once deceleration has started, hard-brake when target speed reaches
       or passes below DECEL_BRAKE_SPEED_MM_S.

       The threshold check is performed before the zero-speed fallback so
       a profile step from above 500 mm/s directly to 0 still triggers BRAKE.
    */
    uint32_t startTime;
    uint32_t lastControlTime;
    uint32_t lastPrintTime;
    uint32_t now;
    uint32_t dt_ms;
    uint32_t lastControlDtMs = 0;

    float dt_sec;

    int32_t cntL_local;
    int32_t cntR_local;

    float distL;
    float distR;
    float avgDist;

    float targetSpeed = 0.0f;
    float previousTargetSpeed = 0.0f;
    float actualAvgSpeed = 0.0f;

    float leftTargetSpeed = 0.0f;
    float rightTargetSpeed = 0.0f;

    uint8_t decelStarted = 0;

    Encoder_ResetAll();
    Encoder_Update();

    Controller_Reset();
    MotionProfile_Reset();
    StraightController_Reset();

    MotionProfile_SetMove(MOTION_TEST_DIST_MM,
                          MOTION_TEST_MAX_SPEED);

    startTime = HAL_GetTick();
    lastControlTime = startTime;
    lastPrintTime = startTime;

    BT_SendLine("\r\n===== DECEL THRESHOLD TEST =====");
    BT_Sendf("Hard brake when target speed <= %.0f mm/s. No decel bias.\r\n",
             DECEL_BRAKE_SPEED_MM_S);

    BT_SendLine(
        "time_ms,dt_ms,targetSpeed,decelStarted,"
        "leftTarget,rightTarget,measL,measR,"
        "pwmL,pwmR,distL,distR,avgDist"
    );

    while ((HAL_GetTick() - startTime) < MOTION_TEST_TIMEOUT_MS)
    {
        now = HAL_GetTick();
        dt_ms = now - lastControlTime;

        if (dt_ms >= CONTROL_DT_MS)
        {
            dt_sec = dt_ms / 1000.0f;

            Encoder_Update();

            cntL_local = Encoder_GetLeftCount();
            cntR_local = Encoder_GetRightCount();

            distL = ((float)cntL_local) * MM_PER_TICK;
            distR = ((float)cntR_local) * MM_PER_TICK;
            avgDist = 0.5f * (distL + distR);

            actualAvgSpeed = 0.5f *
            (
                fabsf(Controller_GetLeftMeasuredSpeed()) +
                fabsf(Controller_GetRightMeasuredSpeed())
            );

            previousTargetSpeed = targetSpeed;

            targetSpeed = MotionProfile_Update(avgDist,
                                               actualAvgSpeed,
                                               dt_sec);

            /*
               Detect the beginning of deceleration only after the profile
               has reached a meaningful speed and then starts decreasing.
            */
            if (!decelStarted &&
                previousTargetSpeed > DECEL_BRAKE_SPEED_MM_S &&
                targetSpeed < previousTargetSpeed)
            {
                decelStarted = 1;
            }

            /*
               Check the threshold before checking targetSpeed <= 0.

               This ensures that if the motion profile jumps from something
               above 500 mm/s directly to 0, the robot still uses BRAKE
               instead of falling into the coast fallback.
            */
            if (decelStarted &&
                targetSpeed <= DECEL_BRAKE_SPEED_MM_S)
            {
                float brakeStartDist;
                float brakeStartLeft;
                float brakeStartRight;
                float brakeTravelLeft;
                float brakeTravelRight;
                float brakeTravelAvg;

                brakeStartLeft = distL;
                brakeStartRight = distR;
                brakeStartDist = avgDist;

                BT_Sendf(
                    "===== BRAKE TRIGGER targetSpeed=%.1f "
                    "previousTarget=%.1f avgDist=%.1f "
                    "L=%.1f R=%.1f =====\r\n",
                    targetSpeed,
                    previousTargetSpeed,
                    brakeStartDist,
                    brakeStartLeft,
                    brakeStartRight
                );

                Motor_Stop(MOTOR_BRAKE);
                HAL_Delay(400);

                Encoder_Update();

                cntL_local = Encoder_GetLeftCount();
                cntR_local = Encoder_GetRightCount();

                distL = ((float)cntL_local) * MM_PER_TICK;
                distR = ((float)cntR_local) * MM_PER_TICK;
                avgDist = 0.5f * (distL + distR);

                brakeTravelLeft = distL - brakeStartLeft;
                brakeTravelRight = distR - brakeStartRight;
                brakeTravelAvg = avgDist - brakeStartDist;

                BT_Sendf(
                    "===== FINAL (BRAKE): "
                    "distL=%.1f distR=%.1f avgDist=%.1f "
                    "target=%.1f moveError=%.1f =====\r\n",
                    distL,
                    distR,
                    avgDist,
                    MOTION_TEST_DIST_MM,
                    MOTION_TEST_DIST_MM - avgDist
                );

                BT_Sendf(
                    "===== BRAKE TRAVEL: "
                    "left=%.1f right=%.1f avg=%.1f "
                    "leftMinusRight=%.1f =====\r\n",
                    brakeTravelLeft,
                    brakeTravelRight,
                    brakeTravelAvg,
                    brakeTravelLeft - brakeTravelRight
                );

                BT_Sendf(
                    "===== FINAL WHEEL DIFFERENCE: "
                    "leftMinusRight=%.1f =====\r\n",
                    distL - distR
                );

                BT_SendLine("===== DECEL THRESHOLD DONE =====");

                Controller_Reset();
                StraightController_Reset();
                return;
            }

            /*
               Fallback only. This should not normally execute because the
               threshold condition above also accepts targetSpeed == 0.
            */
            if (targetSpeed <= 0.0f)
            {
                Motor_Stop(MOTOR_COAST);
                HAL_Delay(800);

                Encoder_Update();

                cntL_local = Encoder_GetLeftCount();
                cntR_local = Encoder_GetRightCount();

                distL = ((float)cntL_local) * MM_PER_TICK;
                distR = ((float)cntR_local) * MM_PER_TICK;
                avgDist = 0.5f * (distL + distR);

                BT_Sendf(
                    "===== FINAL (COAST FALLBACK): "
                    "distL=%.1f distR=%.1f avgDist=%.1f "
                    "target=%.1f error=%.1f =====\r\n",
                    distL,
                    distR,
                    avgDist,
                    MOTION_TEST_DIST_MM,
                    MOTION_TEST_DIST_MM - avgDist
                );

                BT_SendLine("===== DECEL THRESHOLD FALLBACK =====");

                Controller_Reset();
                StraightController_Reset();
                return;
            }

            /*
               Pass isDecelerating = 0 deliberately so this calibration uses:
               - no decel motor bias
               - no decel-specific straight-controller behavior
            */
            StraightController_Update(targetSpeed,
                                      distL,
                                      distR,
                                      0,
                                      &leftTargetSpeed,
                                      &rightTargetSpeed,
                                      dt_sec);

            Controller_UpdateSpeed(leftTargetSpeed,
                                   rightTargetSpeed,
                                   dt_sec,
                                   0,
                                   targetSpeed,
                                   MOTION_TEST_MAX_SPEED,
                                   MOTION_TEST_DIST_MM);

            lastControlDtMs = dt_ms;
            lastControlTime = now;
        }

        if ((now - lastPrintTime) >= 100)
        {
            Encoder_Update();

            cntL_local = Encoder_GetLeftCount();
            cntR_local = Encoder_GetRightCount();

            distL = ((float)cntL_local) * MM_PER_TICK;
            distR = ((float)cntR_local) * MM_PER_TICK;
            avgDist = 0.5f * (distL + distR);

            BT_Sendf(
                "%lu,%lu,%.1f,%u,"
                "%.1f,%.1f,%.1f,%.1f,"
                "%d,%d,%.1f,%.1f,%.1f\r\n",
                now - startTime,
                lastControlDtMs,
                targetSpeed,
                decelStarted,
                leftTargetSpeed,
                rightTargetSpeed,
                Controller_GetLeftMeasuredSpeed(),
                Controller_GetRightMeasuredSpeed(),
                Controller_GetLeftPWM(),
                Controller_GetRightPWM(),
                distL,
                distR,
                avgDist
            );

            lastPrintTime = now;
        }
    }

    Motor_Stop(MOTOR_BRAKE);
    HAL_Delay(400);

    Encoder_Update();

    cntL_local = Encoder_GetLeftCount();
    cntR_local = Encoder_GetRightCount();

    distL = ((float)cntL_local) * MM_PER_TICK;
    distR = ((float)cntR_local) * MM_PER_TICK;
    avgDist = 0.5f * (distL + distR);

    BT_Sendf(
        "===== TIMEOUT FINAL: "
        "distL=%.1f distR=%.1f avgDist=%.1f "
        "leftMinusRight=%.1f =====\r\n",
        distL,
        distR,
        avgDist,
        distL - distR
    );

    BT_SendLine("===== DECEL THRESHOLD TIMEOUT =====");

    Controller_Reset();
    StraightController_Reset();
}

static void Run_RandomMazeRoam(void)
{
    uint8_t wallL, wallF, wallR;
    int options[4];
    int optionCount;
    int choice;

    BT_SendLine("===== RANDOM MAZE ROAM START =====");

    srand(HAL_GetTick());

    while (1)
    {
        Run_StraightMove(RANDOM_ROAM_DIST_MM, RANDOM_ROAM_SPEED_MM_S);

        // Straight move has finished (normally or by AEB).
        // Disable MotionGuard before making any turn.
        MotionGuard_SetEnabled(0);
        MotionGuard_ClearAbort();

        Motor_Stop(MOTOR_BRAKE);
        HAL_Delay(RANDOM_ROAM_SETTLE_MS);

        wallL = IR_WallLeft_BG();
        wallF = IR_WallFront_BG();
        wallR = IR_WallRight_BG();

        optionCount = 0;

        if (!wallL) options[optionCount++] = 0;   // Left
        if (!wallF) options[optionCount++] = 1;   // Straight
        if (!wallR) options[optionCount++] = 2;   // Right

        if (optionCount == 0)
        {
            options[optionCount++] = 3;           // Turn around
        }

        choice = options[rand() % optionCount];

        BT_Sendf("walls L:%d F:%d R:%d choice:%d\r\n",
                 wallL, wallF, wallR, choice);

        switch (choice)
        {
            case 0:
                Turn_Execute(&hi2c1, 90.0f, TURN_LEFT);
                break;

            case 1:
                // Go straight next iteration.
                break;

            case 2:
                Turn_Execute(&hi2c1, 90.0f, TURN_RIGHT);
                break;

            case 3:
                Turn_Execute(&hi2c1, 180.0f, TURN_RIGHT);
                break;
        }

        HAL_Delay(RANDOM_ROAM_SETTLE_MS);
    }
}

static void Run_LeftWallFollowCalib(void)
{
    BT_SendLine("===== LEFT WALL FOLLOW MULTI-RUN CALIB START =====");
    BT_SendLine("run,time_ms,dt_ms,targetSpeed,isDecel,leftTarget,rightTarget,measL,measR,distL_mm,distR_mm,avgDist_mm,leftRaw,leftDist_in,wallL,straightErr,totalCorr");

    for (int run = 1; run <= WALL_FOLLOW_CALIB_REPEATS; run++)
    {
        uint32_t startTime, lastControlTime, lastPrintTime, now, dt_ms;
        float dt_sec;

        int32_t cntL_local, cntR_local;
        float distL_mm = 0.0f;
        float distR_mm = 0.0f;
        float avgDist_mm = 0.0f;

        float targetSpeed = 0.0f;
        float actualAvgSpeed = 0.0f;
        float leftTargetSpeed = 0.0f;
        float rightTargetSpeed = 0.0f;
        uint8_t isDecelerating = 0;

        float startLeftWall_in = 999.0f;
        float endLeftWall_in   = 999.0f;
        float minLeftWall_in   = 999.0f;
        float maxLeftWall_in   = 0.0f;
        float sumLeftWall_in   = 0.0f;
        uint32_t sampleCount   = 0;

        uint8_t startWallSeen = 0;
        uint8_t endWallSeen   = 0;

        Motor_Stop(MOTOR_BRAKE);
        HAL_Delay(WALL_FOLLOW_CALIB_SETTLE_MS);

        Encoder_ResetAll();
        Encoder_Update();

        Controller_Reset();
        MotionProfile_Reset();
        StraightController_Reset();

        MotionProfile_SetMove(WALL_FOLLOW_CALIB_DIST_MM,
                              WALL_FOLLOW_CALIB_SPEED_MM_S);

        HAL_Delay(100);

        startWallSeen = IR_WallLeft_BG();
        startLeftWall_in = IR_GetDistanceL_BG();

        startTime = HAL_GetTick();
        lastControlTime = startTime;
        lastPrintTime = startTime;

        BT_Sendf("===== RUN %d START startSeen=%u startLeft=%.2fin =====\r\n",
                 run,
                 startWallSeen,
                 startLeftWall_in);

        while ((HAL_GetTick() - startTime) < WALL_FOLLOW_CALIB_TIMEOUT_MS)
        {
            now = HAL_GetTick();
            dt_ms = now - lastControlTime;

            if (dt_ms >= CONTROL_DT_MS)
            {
                dt_sec = dt_ms / 1000.0f;

                Encoder_Update();

                cntL_local = Encoder_GetLeftCount();
                cntR_local = Encoder_GetRightCount();

                distL_mm = ((float)cntL_local) * MM_PER_TICK;
                distR_mm = ((float)cntR_local) * MM_PER_TICK;
                avgDist_mm = 0.5f * (distL_mm + distR_mm);

                actualAvgSpeed = 0.5f *
                (
                    fabsf(Controller_GetLeftMeasuredSpeed()) +
                    fabsf(Controller_GetRightMeasuredSpeed())
                );

                targetSpeed = MotionProfile_Update(avgDist_mm,
                                                   actualAvgSpeed,
                                                   dt_sec);

                isDecelerating = MotionProfile_IsDecelerating();

                if (IR_WallLeft_BG())
                {
                    float d = IR_GetDistanceL_BG();

                    sumLeftWall_in += d;
                    sampleCount++;

                    if (d < minLeftWall_in) minLeftWall_in = d;
                    if (d > maxLeftWall_in) maxLeftWall_in = d;
                }

                if (targetSpeed <= 0.0f)
                {
                    Motor_Stop(MOTOR_BRAKE);
                    HAL_Delay(250);

                    Encoder_Update();

                    cntL_local = Encoder_GetLeftCount();
                    cntR_local = Encoder_GetRightCount();

                    distL_mm = ((float)cntL_local) * MM_PER_TICK;
                    distR_mm = ((float)cntR_local) * MM_PER_TICK;
                    avgDist_mm = 0.5f * (distL_mm + distR_mm);

                    break;
                }

                StraightController_Update(targetSpeed,
                                          distL_mm,
                                          distR_mm,
                                          isDecelerating,
                                          &leftTargetSpeed,
                                          &rightTargetSpeed,
                                          dt_sec);

                Controller_UpdateSpeed(leftTargetSpeed,
                                       rightTargetSpeed,
                                       dt_sec,
                                       isDecelerating,
                                       targetSpeed,
                                       WALL_FOLLOW_CALIB_SPEED_MM_S,
                                       WALL_FOLLOW_CALIB_DIST_MM);

                lastControlTime = now;
            }

            if ((now - lastPrintTime) >= WALL_FOLLOW_PRINT_EVERY_MS)
            {
                Encoder_Update();

                cntL_local = Encoder_GetLeftCount();
                cntR_local = Encoder_GetRightCount();

                distL_mm = ((float)cntL_local) * MM_PER_TICK;
                distR_mm = ((float)cntR_local) * MM_PER_TICK;
                avgDist_mm = 0.5f * (distL_mm + distR_mm);

                BT_Sendf("%d,%lu,%lu,%.2f,%u,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%u,%.2f,%u,%.2f,%.2f\r\n",
                         run,
                         now - startTime,
                         dt_ms,
                         targetSpeed,
                         isDecelerating,
                         leftTargetSpeed,
                         rightTargetSpeed,
                         Controller_GetLeftMeasuredSpeed(),
                         Controller_GetRightMeasuredSpeed(),
                         distL_mm,
                         distR_mm,
                         avgDist_mm,
                         IR_GetL_BG(),
                         IR_GetDistanceL_BG(),
                         IR_WallLeft_BG(),
                         StraightController_GetError(),
                         StraightController_GetCorrection());

                lastPrintTime = now;
            }
        }

        endWallSeen = IR_WallLeft_BG();
        endLeftWall_in = IR_GetDistanceL_BG();

        BT_Sendf("===== RUN %d FINAL avgDist=%.2f target=%.2f moveErr=%.2f Lmm=%.2f Rmm=%.2f =====\r\n",
                 run,
                 avgDist_mm,
                 WALL_FOLLOW_CALIB_DIST_MM,
                 WALL_FOLLOW_CALIB_DIST_MM - avgDist_mm,
                 distL_mm,
                 distR_mm);

        BT_Sendf("===== RUN %d WALL startSeen=%u endSeen=%u start=%.2fin end=%.2fin drift=%.2fin =====\r\n",
                 run,
                 startWallSeen,
                 endWallSeen,
                 startLeftWall_in,
                 endLeftWall_in,
                 endLeftWall_in - startLeftWall_in);

        if (sampleCount > 0)
        {
            BT_Sendf("===== RUN %d WALL avg=%.2fin min=%.2fin max=%.2fin samples=%lu =====\r\n",
                     run,
                     sumLeftWall_in / sampleCount,
                     minLeftWall_in,
                     maxLeftWall_in,
                     sampleCount);
        }
        else
        {
            BT_Sendf("===== RUN %d WALL no valid wall samples =====\r\n", run);
        }

        Controller_Reset();
        StraightController_Reset();

        HAL_Delay(WALL_FOLLOW_CALIB_SETTLE_MS);
    }

    Motor_Stop(MOTOR_BRAKE);
    BT_SendLine("===== LEFT WALL FOLLOW MULTI-RUN CALIB DONE =====");

    while (1)
    {
        Motor_Stop(MOTOR_BRAKE);
        HAL_Delay(1000);
    }
}

typedef struct
{
    uint32_t time_ms;
    uint32_t dt_ms;

    int32_t countL;
    int32_t countR;

    int32_t deltaL;
    int32_t deltaR;

    float rawSpeedL;
    float rawSpeedR;

    float filteredSpeedL;
    float filteredSpeedR;

    float decelL;
    float decelR;

    float coastDistL;
    float coastDistR;
} DecelCurveSample;

static DecelCurveSample decelCurveSamples[DECEL_CURVE_MAX_SAMPLES];

static void Run_DecelCurveCalib(void)
{
    BT_SendLine("===== NATURAL DECELERATION CURVE CALIBRATION =====");
    BT_SendLine("Ramps to target speed, holds, then coasts.");
    BT_SendLine("Reposition robot and press button before each run.");

    for (int run = 1; run <= DECEL_CURVE_REPEATS; run++)
    {
        uint32_t rampStartTime;
        uint32_t holdStartTime;
        uint32_t averageStartTime;
        uint32_t coastStartTime;

        uint32_t lastControlTime;
        uint32_t lastSampleTime;

        uint32_t now;
        uint32_t dt_ms;
        uint32_t sampleCount = 0;
        uint32_t stoppedSampleCount = 0;

        float dt_sec;
        float targetSpeed = 0.0f;

        float measuredL = 0.0f;
        float measuredR = 0.0f;

        float speedSumL = 0.0f;
        float speedSumR = 0.0f;
        uint32_t speedSamples = 0;

        int16_t coastTriggerPwmL = 0;
        int16_t coastTriggerPwmR = 0;

        int32_t coastStartCountL;
        int32_t coastStartCountR;

        int32_t previousCountL;
        int32_t previousCountR;

        float filteredSpeedL = 0.0f;
        float filteredSpeedR = 0.0f;

        float previousFilteredSpeedL = 0.0f;
        float previousFilteredSpeedR = 0.0f;

        uint8_t targetReached = 0;
        uint8_t firstCoastSample = 1;

        BT_Sendf("\r\n===== RUN %d READY =====\r\n", run);
        BT_SendLine("Place robot on clear ground and press button.");

        Wait_For_Button();

        Motor_Stop(MOTOR_BRAKE);
        HAL_Delay(300);

        Encoder_ResetAll();
        Encoder_Update();

        Controller_Reset();
        StraightController_Reset();

        /*
         * Phase 1:
         * Ramp both wheel speed targets smoothly to the test speed.
         */
        rampStartTime = HAL_GetTick();
        lastControlTime = rampStartTime;

        BT_Sendf("===== RUN %d RAMP TO %.1f mm/s =====\r\n",
                 run,
                 DECEL_CURVE_TARGET_SPEED);

        while (!targetReached)
        {
            now = HAL_GetTick();
            dt_ms = now - lastControlTime;

            if (dt_ms >= CONTROL_DT_MS)
            {
                dt_sec = dt_ms / 1000.0f;

                targetSpeed += DECEL_CURVE_ACCEL_MM_S2 * dt_sec;

                if (targetSpeed >= DECEL_CURVE_TARGET_SPEED)
                {
                    targetSpeed = DECEL_CURVE_TARGET_SPEED;
                    targetReached = 1;
                }

                Controller_UpdateSpeed(targetSpeed,
                                       targetSpeed,
                                       dt_sec,
                                       0,
                                       targetSpeed,
                                       DECEL_CURVE_TARGET_SPEED,
                                       5000.0f);

                lastControlTime = now;
            }

            if ((HAL_GetTick() - rampStartTime) > 3000U)
            {
                BT_SendLine("===== RAMP TIMEOUT =====");

                Motor_Stop(MOTOR_BRAKE);
                Controller_Reset();
                StraightController_Reset();
                return;
            }
        }

        /*
         * Phase 2:
         * Hold both wheels at the test speed.
         */
        holdStartTime = HAL_GetTick();
        lastControlTime = holdStartTime;

        BT_Sendf("===== RUN %d HOLD %.1f mm/s FOR %lu ms =====\r\n",
                 run,
                 DECEL_CURVE_TARGET_SPEED,
                 (uint32_t)DECEL_CURVE_HOLD_MS);

        while ((HAL_GetTick() - holdStartTime) < DECEL_CURVE_HOLD_MS)
        {
            now = HAL_GetTick();
            dt_ms = now - lastControlTime;

            if (dt_ms >= CONTROL_DT_MS)
            {
                dt_sec = dt_ms / 1000.0f;

                Controller_UpdateSpeed(DECEL_CURVE_TARGET_SPEED,
                                       DECEL_CURVE_TARGET_SPEED,
                                       dt_sec,
                                       0,
                                       DECEL_CURVE_TARGET_SPEED,
                                       DECEL_CURVE_TARGET_SPEED,
                                       5000.0f);

                lastControlTime = now;
            }
        }

        /*
         * Phase 3:
         * Average the measured powered wheel speeds during the final
         * pre-coast window.
         */
        averageStartTime = HAL_GetTick();
        lastControlTime = averageStartTime;

        while ((HAL_GetTick() - averageStartTime) <
               DECEL_CURVE_PRECOAST_AVG_MS)
        {
            now = HAL_GetTick();
            dt_ms = now - lastControlTime;

            if (dt_ms >= CONTROL_DT_MS)
            {
                dt_sec = dt_ms / 1000.0f;

                Controller_UpdateSpeed(DECEL_CURVE_TARGET_SPEED,
                                       DECEL_CURVE_TARGET_SPEED,
                                       dt_sec,
                                       0,
                                       DECEL_CURVE_TARGET_SPEED,
                                       DECEL_CURVE_TARGET_SPEED,
                                       5000.0f);

                speedSumL +=
                    fabsf(Controller_GetLeftMeasuredSpeed());

                speedSumR +=
                    fabsf(Controller_GetRightMeasuredSpeed());

                speedSamples++;

                lastControlTime = now;
            }
        }

        if (speedSamples > 0U)
        {
            measuredL = speedSumL / (float)speedSamples;
            measuredR = speedSumR / (float)speedSamples;
        }

        coastTriggerPwmL = Controller_GetLeftPWM();
        coastTriggerPwmR = Controller_GetRightPWM();

        /*
         * All Bluetooth output must happen before capturing the coast
         * encoder baseline. BT_Sendf() blocks while the robot is moving.
         */
        BT_Sendf("===== RUN %d PRECOAST AVG "
                 "speedL=%.2f speedR=%.2f difference=%.2f "
                 "samples=%lu =====\r\n",
                 run,
                 measuredL,
                 measuredR,
                 measuredL - measuredR,
                 speedSamples);

        if (fabsf(measuredL - measuredR) >
            DECEL_CURVE_SPEED_MATCH_TOL)
        {
            BT_Sendf("===== WARNING: PRECOAST SPEED DIFFERENCE "
                     "EXCEEDS %.1f mm/s =====\r\n",
                     DECEL_CURVE_SPEED_MATCH_TOL);
        }

        BT_Sendf("===== RUN %d COAST TRIGGER "
                 "avgSpeedL=%.2f avgSpeedR=%.2f "
                 "pwmL=%d pwmR=%d =====\r\n",
                 run,
                 measuredL,
                 measuredR,
                 coastTriggerPwmL,
                 coastTriggerPwmR);

        /*
         * Critical ordering:
         *
         * 1. Synchronize encoder counts.
         * 2. Capture the exact coast baseline.
         * 3. Immediately remove motor drive.
         *
         * Nothing blocking is allowed between these operations.
         */
        Encoder_Update();

        coastStartCountL = Encoder_GetLeftCount();
        coastStartCountR = Encoder_GetRightCount();

        previousCountL = coastStartCountL;
        previousCountR = coastStartCountR;

        Motor_Stop(MOTOR_COAST);

        coastStartTime = HAL_GetTick();
        lastSampleTime = coastStartTime;

        sampleCount = 0;
        stoppedSampleCount = 0;
        firstCoastSample = 1;

        /*
         * Phase 4:
         * Measure the natural unpowered coast.
         *
         * No speed controller, straight controller, wall following,
         * motion profile, or deceleration bias is called here.
         */
        while (((HAL_GetTick() - coastStartTime) <
                DECEL_CURVE_TIMEOUT_MS) &&
               (sampleCount < DECEL_CURVE_MAX_SAMPLES))
        {
            int32_t currentCountL;
            int32_t currentCountR;

            int32_t deltaL;
            int32_t deltaR;

            float rawSpeedL;
            float rawSpeedR;

            float decelL;
            float decelR;

            float coastDistL;
            float coastDistR;

            now = HAL_GetTick();
            dt_ms = now - lastSampleTime;

            if (dt_ms < DECEL_CURVE_SAMPLE_MS)
            {
                continue;
            }

            dt_sec = dt_ms / 1000.0f;

            Encoder_Update();

            currentCountL = Encoder_GetLeftCount();
            currentCountR = Encoder_GetRightCount();

            deltaL = currentCountL - previousCountL;
            deltaR = currentCountR - previousCountR;

            /*
             * Ignore reverse jitter after the wheel has stopped.
             * Do not turn a negative tick into positive speed.
             */
            if (deltaL < 0)
            {
                deltaL = 0;
            }

            if (deltaR < 0)
            {
                deltaR = 0;
            }

            rawSpeedL =
                ((float)deltaL * MM_PER_TICK) / dt_sec;

            rawSpeedR =
                ((float)deltaR * MM_PER_TICK) / dt_sec;

            /*
             * Use the first genuine coast sample as the filter's initial
             * state. There is no valid deceleration value for this row
             * because no earlier coast sample exists.
             */
            if (firstCoastSample)
            {
                filteredSpeedL = rawSpeedL;
                filteredSpeedR = rawSpeedR;

                previousFilteredSpeedL = filteredSpeedL;
                previousFilteredSpeedR = filteredSpeedR;

                decelL = 0.0f;
                decelR = 0.0f;

                firstCoastSample = 0;
            }
            else
            {
                filteredSpeedL +=
                    DECEL_CURVE_FILTER_ALPHA *
                    (rawSpeedL - filteredSpeedL);

                filteredSpeedR +=
                    DECEL_CURVE_FILTER_ALPHA *
                    (rawSpeedR - filteredSpeedR);

                decelL =
                    (filteredSpeedL -
                     previousFilteredSpeedL) / dt_sec;

                decelR =
                    (filteredSpeedR -
                     previousFilteredSpeedR) / dt_sec;

                previousFilteredSpeedL = filteredSpeedL;
                previousFilteredSpeedR = filteredSpeedR;
            }

            coastDistL =
                ((float)(currentCountL - coastStartCountL)) *
                MM_PER_TICK;

            coastDistR =
                ((float)(currentCountR - coastStartCountR)) *
                MM_PER_TICK;

            decelCurveSamples[sampleCount].time_ms =
                now - coastStartTime;

            decelCurveSamples[sampleCount].dt_ms =
                dt_ms;

            decelCurveSamples[sampleCount].countL =
                currentCountL;

            decelCurveSamples[sampleCount].countR =
                currentCountR;

            decelCurveSamples[sampleCount].deltaL =
                deltaL;

            decelCurveSamples[sampleCount].deltaR =
                deltaR;

            decelCurveSamples[sampleCount].rawSpeedL =
                rawSpeedL;

            decelCurveSamples[sampleCount].rawSpeedR =
                rawSpeedR;

            decelCurveSamples[sampleCount].filteredSpeedL =
                filteredSpeedL;

            decelCurveSamples[sampleCount].filteredSpeedR =
                filteredSpeedR;

            decelCurveSamples[sampleCount].decelL =
                decelL;

            decelCurveSamples[sampleCount].decelR =
                decelR;

            decelCurveSamples[sampleCount].coastDistL =
                coastDistL;

            decelCurveSamples[sampleCount].coastDistR =
                coastDistR;

            sampleCount++;

            previousCountL = currentCountL;
            previousCountR = currentCountR;

            lastSampleTime = now;

            if ((filteredSpeedL <=
                 DECEL_CURVE_STOP_SPEED_MM_S) &&
                (filteredSpeedR <=
                 DECEL_CURVE_STOP_SPEED_MM_S))
            {
                stoppedSampleCount++;

                if (stoppedSampleCount >=
                    DECEL_CURVE_STOP_SAMPLES)
                {
                    break;
                }
            }
            else
            {
                stoppedSampleCount = 0;
            }
        }

        Motor_Stop(MOTOR_BRAKE);
        HAL_Delay(200);

        /*
         * Print after sampling so blocking UART transmission cannot alter
         * the coast sampling interval.
         */
        BT_Sendf("\r\n===== RUN %d DECEL DATA samples=%lu =====\r\n",
                 run,
                 sampleCount);

        BT_SendLine(
            "run,time_ms,dt_ms,"
            "countL,countR,deltaL,deltaR,"
            "rawSpeedL,rawSpeedR,"
            "filteredSpeedL,filteredSpeedR,"
            "decelL,decelR,"
            "coastDistL,coastDistR"
        );

        for (uint32_t i = 0; i < sampleCount; i++)
        {
            BT_Sendf(
                "%d,%lu,%lu,"
                "%ld,%ld,%ld,%ld,"
                "%.2f,%.2f,"
                "%.2f,%.2f,"
                "%.2f,%.2f,"
                "%.2f,%.2f\r\n",

                run,

                decelCurveSamples[i].time_ms,
                decelCurveSamples[i].dt_ms,

                decelCurveSamples[i].countL,
                decelCurveSamples[i].countR,

                decelCurveSamples[i].deltaL,
                decelCurveSamples[i].deltaR,

                decelCurveSamples[i].rawSpeedL,
                decelCurveSamples[i].rawSpeedR,

                decelCurveSamples[i].filteredSpeedL,
                decelCurveSamples[i].filteredSpeedR,

                decelCurveSamples[i].decelL,
                decelCurveSamples[i].decelR,

                decelCurveSamples[i].coastDistL,
                decelCurveSamples[i].coastDistR
            );
        }

        if (sampleCount > 0U)
        {
            DecelCurveSample *finalSample =
                &decelCurveSamples[sampleCount - 1U];

            BT_Sendf(
                "===== RUN %d SUMMARY "
                "precoastAvgL=%.2f precoastAvgR=%.2f "
                "precoastDifference=%.2f "
                "coastTime=%lu "
                "coastDistL=%.2f coastDistR=%.2f "
                "leftMinusRight=%.2f =====\r\n",

                run,

                measuredL,
                measuredR,
                measuredL - measuredR,

                finalSample->time_ms,

                finalSample->coastDistL,
                finalSample->coastDistR,

                finalSample->coastDistL -
                finalSample->coastDistR
            );
        }

        Controller_Reset();
        StraightController_Reset();

        BT_Sendf("===== RUN %d COMPLETE =====\r\n", run);
    }

    Motor_Stop(MOTOR_BRAKE);

    BT_SendLine(
        "===== NATURAL DECELERATION CURVE CALIBRATION DONE ====="
    );

    while (1)
    {
        Motor_Stop(MOTOR_BRAKE);
        HAL_Delay(1000);
    }
}

void Calibration_Run(void)
{

#if TEST_BLUETOOTH_ONLY
    HAL_Delay(2000);
    while (1) { BT_SendLine("HELLO"); HAL_Delay(2000); }
#endif

#if TEST_ENCODER_ONLY
    Encoder_Init(); Encoder_ResetAll(); HAL_Delay(500); Wait_For_Button(); Encoder_ResetAll();
    while (1) { Encoder_Update(); rawL = Encoder_GetLeftRaw(); rawR = Encoder_GetRightRaw(); cntL = Encoder_GetLeftCount(); cntR = Encoder_GetRightCount(); HAL_Delay(10); }
#endif

#if TEST_MOTOR_ENCODER_FOREVER
    Encoder_Init(); Encoder_ResetAll(); Motor_Init(); Motor_Enable(); HAL_Delay(500); Wait_For_Button(); Encoder_ResetAll(); Motor_SetRawPWM(700, 700);
    while (1) { Encoder_Update(); cntL = Encoder_GetLeftCount(); cntR = Encoder_GetRightCount(); HAL_Delay(10); }
#endif

#if TEST_BNO055_ONLY
    Wait_For_Button(); BNO055_Reset();
    bnoStatus = BNO055_ReadChipID(&hi2c1, &bnoChipID);
    if ((bnoStatus != BNO055_OK) || (bnoChipID != BNO055_CHIP_ID_VALUE)) { while (1) { BT_SendLine("BNO NOT DETECTED"); HAL_Delay(1000); } }
    bnoStatus = BNO055_Init(&hi2c1);
    if (bnoStatus != BNO055_OK) { while (1) { BT_SendLine("BNO INIT FAILED"); HAL_Delay(1000); } }
#if BNO_TEST_MODE_IMUPLUS
    bnoStatus = BNO055_SetMode(&hi2c1, BNO055_MODE_IMUPLUS);
#else
    bnoStatus = BNO055_SetMode(&hi2c1, BNO055_MODE_NDOF);
#endif
    if (bnoStatus != BNO055_OK) { while (1) { BT_SendLine("BNO MODE SET FAILED"); HAL_Delay(1000); } }
    HAL_Delay(100); BT_SendLine("BNO ready. Press button."); Wait_For_Button(); BT_SendLine("===== BNO055 ONLY TEST STARTED =====");
    while (1)
    {
        bnoStatus = BNO055_ReadEuler(&hi2c1, &bnoEuler); bnoStatus = BNO055_ReadGyro(&hi2c1, &bnoGyro);
        if (bnoStatus == BNO055_OK) { bnoHeading = bnoEuler.heading_deg; bnoRoll = bnoEuler.roll_deg; bnoPitch = bnoEuler.pitch_deg; bnoGz = bnoGyro.z_dps; }
        BNO055_ReadCalibration(&hi2c1, &bnoCalib); BNO055_ReadStatus(&hi2c1, &bnoSysStat, &bnoSysErr);
        BT_Sendf("HEAD: %.2f | ROLL: %.2f | PITCH: %.2f\r\n", bnoHeading, bnoRoll, bnoPitch);
        BT_Sendf("GZ: %.2f deg/s\r\n", bnoGz);
        BT_Sendf("CAL SYS:%d GYR:%d ACC:%d MAG:%d | STAT:%d ERR:%d\r\n", bnoCalib.sys, bnoCalib.gyro, bnoCalib.accel, bnoCalib.mag, bnoSysStat, bnoSysErr);
        HAL_Delay(200);
    }
#endif

#if TEST_BNO055_WITH_MOTOR
    Encoder_Init(); Encoder_ResetAll(); Motor_Init(); Motor_Enable(); BNO055_Reset();
    bnoStatus = BNO055_ReadChipID(&hi2c1, &bnoChipID);
    if ((bnoStatus != BNO055_OK) || (bnoChipID != BNO055_CHIP_ID_VALUE)) { while (1) { BT_SendLine("BNO NOT DETECTED"); HAL_Delay(1000); } }
    bnoStatus = BNO055_Init(&hi2c1);
    if (bnoStatus != BNO055_OK) { while (1) { BT_SendLine("BNO INIT FAILED"); HAL_Delay(1000); } }
#if BNO_TEST_MODE_IMUPLUS
    bnoStatus = BNO055_SetMode(&hi2c1, BNO055_MODE_IMUPLUS);
#else
    bnoStatus = BNO055_SetMode(&hi2c1, BNO055_MODE_NDOF);
#endif
    if (bnoStatus != BNO055_OK) { while (1) { BT_SendLine("BNO MODE SET FAILED"); HAL_Delay(1000); } }
    HAL_Delay(500); Wait_For_Button(); Encoder_ResetAll(); Motor_SetRawPWM(700, 700); BT_SendLine("===== BNO055 + MOTOR + ENCODER TEST =====");
    while (1)
    {
        Encoder_Update(); cntL = Encoder_GetLeftCount(); cntR = Encoder_GetRightCount();
        bnoStatus = BNO055_ReadEuler(&hi2c1, &bnoEuler); bnoStatus = BNO055_ReadGyro(&hi2c1, &bnoGyro);
        if (bnoStatus == BNO055_OK) { bnoHeading = bnoEuler.heading_deg; bnoGz = bnoGyro.z_dps; }
        BT_Sendf("HEAD: %.2f  GZ: %.2f  L: %ld  R: %ld\r\n", bnoHeading, bnoGz, cntL, cntR);
        HAL_Delay(200);
    }
#endif

#if TEST_ENCODER_CPR_CALIB
    Encoder_Init(); Encoder_ResetAll(); Wait_For_Button(); Encoder_ResetAll();
    BT_SendLine("===== ENCODER CPR CALIBRATION =====");
    while (1) { Encoder_Update(); cntL = Encoder_GetLeftCount(); cntR = Encoder_GetRightCount(); BT_Sendf("LEFT: %ld  RIGHT: %ld\r\n", cntL, cntR); HAL_Delay(100); }
#endif

#if TEST_MOTOR_SPEED_CALIB
    Wait_For_Button(); Encoder_Init(); Motor_Init(); Motor_Enable(); HAL_Delay(500);
    BT_SendLine("===== MOTOR SPEED CALIBRATION READY ====="); Wait_For_Button(); BT_SendLine("===== MOTOR SPEED CALIBRATION STARTED =====");
    for (int16_t p = 400; p <= 1000; p += 50) Run_MotorSpeedTest(p, 0);
    for (int16_t p = 400; p <= 1000; p += 50) Run_MotorSpeedTest(0, p);
    Motor_SetRawPWM(0, 0); BT_SendLine("===== MOTOR SPEED CALIBRATION DONE =====");
    while (1) { HAL_Delay(1000); }
#endif

#if TEST_DEADZONE
    Encoder_Init(); Encoder_ResetAll(); Motor_Init(); Motor_Enable();
    BT_SendLine("===== DEADZONE TEST ====="); Wait_For_Button(); Run_DeadzoneTest(500); HAL_Delay(3000);
    BT_SendLine("===== DEADZONE TEST DONE ====="); while (1);
#endif

#if TEST_SPEED_CONTROLLER
    Wait_For_Button(); Encoder_Init(); Motor_Init(); Motor_Enable(); Controller_Init();
    BT_SendLine("===== SPEED CONTROLLER READY ====="); Wait_For_Button(); BT_SendLine("===== SPEED CONTROLLER STARTED =====");
    Run_SpeedControllerTest(800.0f, 800.0f);
    Motor_Stop(MOTOR_COAST); BT_SendLine("===== SPEED CONTROLLER DONE =====");
    while (1) { HAL_Delay(1000); }
#endif

#if TEST_MOTION_PROFILE
    Wait_For_Button(); Encoder_Init(); Encoder_ResetAll(); Motor_Init(); Motor_Enable(); Controller_Init();
    BT_SendLine("===== MOTION PROFILE READY ====="); BT_SendLine("Place robot safely on ground."); BT_SendLine("Connect Bluetooth, then press button."); Wait_For_Button();
    Run_MotionProfileTest();
    while (1) { HAL_Delay(1000); }
#endif

#if TEST_DECEL_NATURAL
    Wait_For_Button(); Encoder_Init(); Encoder_ResetAll(); Motor_Init(); Motor_Enable(); Controller_Init();
    BT_SendLine("===== DECEL NATURAL READY =====");
    BT_SendLine("Place robot on ground. Press button.");
    Wait_For_Button();
    Run_DecelNatural();
    HAL_Delay(1500);
    while (1) { Motor_Stop(MOTOR_COAST); HAL_Delay(1000); }
#endif

#if TEST_DECEL_THRESHOLD
    Wait_For_Button(); Encoder_Init(); Encoder_ResetAll(); Motor_Init(); Motor_Enable(); Controller_Init();
    BT_SendLine("===== DECEL THRESHOLD READY =====");
    BT_SendLine("Place robot on ground. Press button.");
    Wait_For_Button();
    Run_DecelThreshold();
    HAL_Delay(1500);
    while (1) { Motor_Stop(MOTOR_BRAKE); HAL_Delay(1000); }
#endif

#if TEST_STRAIGHT_SPEED_CONTROLLER
    Wait_For_Button(); Encoder_Init(); Encoder_ResetAll(); Motor_Init(); Motor_Enable(); Controller_Init(); BNO055_Reset();
    bnoStatus = BNO055_ReadChipID(&hi2c1, &bnoChipID);
    if ((bnoStatus != BNO055_OK) || (bnoChipID != BNO055_CHIP_ID_VALUE)) { while (1) { BT_SendLine("BNO NOT DETECTED"); HAL_Delay(1000); } }
    bnoStatus = BNO055_Init(&hi2c1);
    if (bnoStatus != BNO055_OK) { while (1) { BT_SendLine("BNO INIT FAILED"); HAL_Delay(1000); } }
    bnoStatus = BNO055_SetMode(&hi2c1, BNO055_MODE_IMUPLUS);
    if (bnoStatus != BNO055_OK) { while (1) { BT_SendLine("BNO MODE SET FAILED"); HAL_Delay(1000); } }
    HAL_Delay(100); BT_SendLine("===== STRAIGHT + SPEED CONTROLLER READY ====="); Wait_For_Button(); BT_SendLine("===== STRAIGHT + SPEED CONTROLLER STARTED =====");
    Run_StraightSpeedControllerTest(1000.0f);
    Motor_Stop(MOTOR_COAST); BT_SendLine("===== STRAIGHT + SPEED CONTROLLER DONE =====");
    while (1) { HAL_Delay(1000); }
#endif

#if TEST_TURN
    Motor_Init(); Motor_Enable(); Encoder_Init(); Encoder_ResetAll(); BNO055_Reset();
    bnoStatus = BNO055_ReadChipID(&hi2c1, &bnoChipID);
    if ((bnoStatus != BNO055_OK) || (bnoChipID != BNO055_CHIP_ID_VALUE)) { while (1) { BT_SendLine("BNO NOT DETECTED"); HAL_Delay(1000); } }
    bnoStatus = BNO055_Init(&hi2c1);
    if (bnoStatus != BNO055_OK) { while (1) { BT_SendLine("BNO INIT FAILED"); HAL_Delay(1000); } }
    bnoStatus = BNO055_SetMode(&hi2c1, BNO055_MODE_IMUPLUS);
    if (bnoStatus != BNO055_OK) { while (1) { BT_SendLine("BNO MODE SET FAILED"); HAL_Delay(1000); } }
    HAL_Delay(500);
    BT_SendLine("===== TURN TEST READY =====");
    BT_SendLine("Place robot on ground. Press button to turn RIGHT 90 degrees.");
    Wait_For_Button();
    BT_SendLine("===== TURNING RIGHT 90 =====");
    TurnResult result = Turn_Execute(&hi2c1, 180.0f, TURN_LEFT);
    if (result == TURN_OK) BT_SendLine("===== TURN OK =====");
    else if (result == TURN_TIMEOUT) BT_SendLine("===== TURN TIMEOUT =====");
    else BT_SendLine("===== TURN IMU ERROR =====");
    Motor_Stop(MOTOR_BRAKE);
    while (1) { HAL_Delay(1000); }
#endif

#if TEST_TURN_SIGN
    Motor_Init(); Motor_Enable(); Encoder_Init(); Encoder_ResetAll(); BNO055_Reset();
    bnoStatus = BNO055_ReadChipID(&hi2c1, &bnoChipID);
    if ((bnoStatus != BNO055_OK) || (bnoChipID != BNO055_CHIP_ID_VALUE)) { while (1) { BT_SendLine("BNO NOT DETECTED"); HAL_Delay(1000); } }
    bnoStatus = BNO055_Init(&hi2c1);
    if (bnoStatus != BNO055_OK) { while (1) { BT_SendLine("BNO MODE SET FAILED"); HAL_Delay(1000); } }
    bnoStatus = BNO055_SetMode(&hi2c1, BNO055_MODE_IMUPLUS);
    if (bnoStatus != BNO055_OK) { while (1) { BT_SendLine("BNO MODE SET FAILED"); HAL_Delay(1000); } }
    HAL_Delay(500);
    Run_TurnSignTest();
#endif

#if TEST_TURN_PROFILE_CALIB
    Motor_Init(); Motor_Enable(); Encoder_Init(); Encoder_ResetAll(); BNO055_Reset();
    bnoStatus = BNO055_ReadChipID(&hi2c1, &bnoChipID);
    if ((bnoStatus != BNO055_OK) || (bnoChipID != BNO055_CHIP_ID_VALUE)) { while (1) { BT_SendLine("BNO NOT DETECTED"); HAL_Delay(1000); } }
    bnoStatus = BNO055_Init(&hi2c1);
    if (bnoStatus != BNO055_OK) { while (1) { BT_SendLine("BNO INIT FAILED"); HAL_Delay(1000); } }
    bnoStatus = BNO055_SetMode(&hi2c1, BNO055_MODE_IMUPLUS);
    if (bnoStatus != BNO055_OK) { while (1) { BT_SendLine("BNO MODE SET FAILED"); HAL_Delay(1000); } }
    HAL_Delay(500);
    Run_TurnProfileCalibration();
#endif

#if TEST_STRAIGHT_TURN_SEQUENCE
    Motor_Init(); Motor_Enable(); Encoder_Init(); Encoder_ResetAll(); Controller_Init(); BNO055_Reset();
    bnoStatus = BNO055_ReadChipID(&hi2c1, &bnoChipID);
    if ((bnoStatus != BNO055_OK) || (bnoChipID != BNO055_CHIP_ID_VALUE)) { while (1) { BT_SendLine("BNO NOT DETECTED"); HAL_Delay(1000); } }
    bnoStatus = BNO055_Init(&hi2c1);
    if (bnoStatus != BNO055_OK) { while (1) { BT_SendLine("BNO INIT FAILED"); HAL_Delay(1000); } }
    bnoStatus = BNO055_SetMode(&hi2c1, BNO055_MODE_IMUPLUS);
    if (bnoStatus != BNO055_OK) { while (1) { BT_SendLine("BNO MODE SET FAILED"); HAL_Delay(1000); } }
    HAL_Delay(500);
    BT_SendLine("===== STRAIGHT + TURN SEQUENCE READY =====");
    BT_SendLine("Place robot on ground. Press button.");
    Wait_For_Button();
    Run_StraightTurnSequence();
#endif

#if TEST_IR_LIVE
    // ── Live Expressions / debugger watch ────────────────────────────────────
    // Watch: dbg_irL, dbg_irF, dbg_irR, dbg_distL, dbg_distF, dbg_distR,
    //        dbg_wallL, dbg_wallF, dbg_wallR, dbg_appL, dbg_appF, dbg_appR
    IR_Init();
    IR_BackgroundReset();
    Wait_For_Button();
    while (1)
    {
        dbg_irL   = IR_GetL_BG();
        dbg_irF   = IR_GetF_BG();
        dbg_irR   = IR_GetR_BG();
        dbg_distL = IR_GetDistanceL_BG();
        dbg_distF = IR_GetDistanceF_BG();
        dbg_distR = IR_GetDistanceR_BG();
        dbg_wallL = IR_WallLeft_BG();
        dbg_wallF = IR_WallFront_BG();
        dbg_wallR = IR_WallRight_BG();
        dbg_appL  = IR_WallApproachingLeft_BG();
        dbg_appF  = IR_WallApproachingFront_BG();
        dbg_appR  = IR_WallApproachingRight_BG();
        HAL_Delay(20);
    }
#endif

#if TEST_IR_BLUETOOTH
    // ── Bluetooth IR sensor test ──────────────────────────────────────────────
    // Prints raw ADC, distance, and wall flags for all 3 sensors every 100ms.
    // Format: rawL,rawF,rawR,distL,distF,distR,wallL,wallF,wallR,appL,appF,appR
    IR_Init();
    IR_BackgroundReset();
    HAL_Delay(500);
    BT_SendLine("===== IR BLUETOOTH TEST =====");
    BT_SendLine("Press button to start.");
    Wait_For_Button();
    BT_SendLine("rawL,rawF,rawR,distL,distF,distR,wallL,wallF,wallR,appL,appF,appR");
    uint32_t lastPrint = HAL_GetTick();
    while (1)
    {
        uint32_t now = HAL_GetTick();
        if ((now - lastPrint) >= 100)
        {
            BT_Sendf("%u,%u,%u,%.2f,%.2f,%.2f,%u,%u,%u,%u,%u,%u\r\n",
                     IR_GetL_BG(),
                     IR_GetF_BG(),
                     IR_GetR_BG(),
                     IR_GetDistanceL_BG(),
                     IR_GetDistanceF_BG(),
                     IR_GetDistanceR_BG(),
                     IR_WallLeft_BG(),
                     IR_WallFront_BG(),
                     IR_WallRight_BG(),
                     IR_WallApproachingLeft_BG(),
                     IR_WallApproachingFront_BG(),
                     IR_WallApproachingRight_BG());
            lastPrint = now;
        }
    }
#endif

#if TEST_IR_STATUS
    // ── IR Status Monitor ─────────────────────────────────────────────────────
    // Prints human-readable messages only when sensor state changes.
    // Shows distance and wall/approach flags per sensor.
    {
        IR_Init();
        IR_BackgroundReset();
        HAL_Delay(500);

        BT_SendLine("===== IR STATUS MONITOR =====");
        BT_SendLine("Watching for changes...");

        float prevDistL = 999.0f, prevDistF = 999.0f, prevDistR = 999.0f;
        uint8_t prevWallL = 0, prevWallF = 0, prevWallR = 0;
        uint8_t prevAppL  = 0, prevAppF  = 0, prevAppR  = 0;
        uint8_t firstPrint = 1;

        while (1)
        {
            HAL_Delay(50);

            float dL = IR_GetDistanceL_BG();
            float dF = IR_GetDistanceF_BG();
            float dR = IR_GetDistanceR_BG();
            uint8_t wL = IR_WallLeft_BG();
            uint8_t wF = IR_WallFront_BG();
            uint8_t wR = IR_WallRight_BG();
            uint8_t aL = IR_WallApproachingLeft_BG();
            uint8_t aF = IR_WallApproachingFront_BG();
            uint8_t aR = IR_WallApproachingRight_BG();

            uint8_t changed = firstPrint ||
                (wL != prevWallL) || (wF != prevWallF) || (wR != prevWallR) ||
                (aL != prevAppL)  || (aF != prevAppF)  || (aR != prevAppR)  ||
                (dL < 999.0f && fabsf(dL - prevDistL) > 0.2f) ||
                (dF < 999.0f && fabsf(dF - prevDistF) > 0.2f) ||
                (dR < 999.0f && fabsf(dR - prevDistR) > 0.2f);

            if (!changed) continue;

            firstPrint = 0;
            prevDistL = dL; prevDistF = dF; prevDistR = dR;
            prevWallL = wL; prevWallF = wF; prevWallR = wR;
            prevAppL  = aL; prevAppF  = aF; prevAppR  = aR;

            uint8_t anything = wL || wF || wR;

            if (!anything)
            {
                BT_SendLine("[ ALL CLEAR ]");
                continue;
            }

            BT_SendLine("------------------------------");
            if (wL)
            {
                if (aL)
                    BT_Sendf("  LEFT:  APPROACHING  %.1fin\r\n", dL);
                else
                    BT_Sendf("  LEFT:  wall detected  %.1fin\r\n", dL);
            }
            if (wF)
            {
                if (aF)
                    BT_Sendf("  FRONT: APPROACHING  %.1fin\r\n", dF);
                else
                    BT_Sendf("  FRONT: wall detected  %.1fin\r\n", dF);
            }
            if (wR)
            {
                if (aR)
                    BT_Sendf("  RIGHT: APPROACHING  %.1fin\r\n", dR);
                else
                    BT_Sendf("  RIGHT: wall detected  %.1fin\r\n", dR);
            }
            BT_SendLine("------------------------------");
        }
    }
#endif

#if TEST_IR_CALIB
        const float distances[] = {1.0f, 2.0f, 3.0f, 4.0f, 5.0f,
                                   6.0f, 7.0f, 8.0f, 9.0f, 10.0f};
        const int numDist    = 10;
        const int numSamples = 20;

        IR_Init();
        IR_BackgroundReset();
        HAL_Delay(500);

        BT_SendLine("===== IR CALIBRATION =====");

        // ── LEFT SENSOR ───────────────────────────────────────────────────────
        BT_SendLine("\r\n--- LEFT SENSOR ---");
        BT_SendLine("Point LEFT side at wall. Press button at each distance.");
        BT_SendLine("dist_in,avgRaw,minRaw,maxRaw");
        for (int i = 0; i < numDist; i++)
        {
            BT_Sendf("Place LEFT sensor %.0fin from wall. Press button.\r\n", distances[i]);
            Wait_For_Button();
            uint32_t sum = 0;
            uint16_t mn = 65535, mx = 0, r;
            for (int s = 0; s < numSamples; s++)
            {
                HAL_Delay(50);
                r = IR_GetL_BG();
                sum += r;
                if (r < mn) mn = r;
                if (r > mx) mx = r;
            }
            BT_Sendf("%.0f,%u,%u,%u\r\n", distances[i], (uint16_t)(sum/numSamples), mn, mx);
        }
        BT_SendLine("--- LEFT SENSOR DONE ---");

        // ── FRONT SENSOR ──────────────────────────────────────────────────────
        BT_SendLine("\r\n--- FRONT SENSOR ---");
        BT_SendLine("Point FRONT at wall. Press button at each distance.");
        BT_SendLine("dist_in,avgRaw,minRaw,maxRaw");
        for (int i = 0; i < numDist; i++)
        {
            BT_Sendf("Place FRONT sensor %.0fin from wall. Press button.\r\n", distances[i]);
            Wait_For_Button();
            uint32_t sum = 0;
            uint16_t mn = 65535, mx = 0, r;
            for (int s = 0; s < numSamples; s++)
            {
                HAL_Delay(50);
                r = IR_GetF_BG();
                sum += r;
                if (r < mn) mn = r;
                if (r > mx) mx = r;
            }
            BT_Sendf("%.0f,%u,%u,%u\r\n", distances[i], (uint16_t)(sum/numSamples), mn, mx);
        }
        BT_SendLine("--- FRONT SENSOR DONE ---");

        // ── RIGHT SENSOR ──────────────────────────────────────────────────────
        BT_SendLine("\r\n--- RIGHT SENSOR ---");
        BT_SendLine("Point RIGHT side at wall. Press button at each distance.");
        BT_SendLine("dist_in,avgRaw,minRaw,maxRaw");
        for (int i = 0; i < numDist; i++)
        {
            BT_Sendf("Place RIGHT sensor %.0fin from wall. Press button.\r\n", distances[i]);
            Wait_For_Button();
            uint32_t sum = 0;
            uint16_t mn = 65535, mx = 0, r;
            for (int s = 0; s < numSamples; s++)
            {
                HAL_Delay(50);
                r = IR_GetR_BG();
                sum += r;
                if (r < mn) mn = r;
                if (r > mx) mx = r;
            }
            BT_Sendf("%.0f,%u,%u,%u\r\n", distances[i], (uint16_t)(sum/numSamples), mn, mx);
        }
        BT_SendLine("--- RIGHT SENSOR DONE ---");

        BT_SendLine("\r\n===== IR CALIBRATION DONE =====");
        BT_SendLine("Use avgRaw values to rebuild lookup table in sensors.c");
        while (1) { HAL_Delay(1000); }
    }
#endif

#if TEST_APPROACH_BRAKE_CALIB
    Wait_For_Button();

    Encoder_Init();
    Encoder_ResetAll();

    Motor_Init();
    Motor_Enable();

    Controller_Init();

    BT_SendLine("===== APPROACH BRAKE CALIB READY =====");
    BT_SendLine("Place robot on ground. Press button.");

    Wait_For_Button();

    Run_ApproachBrakeCalib();
#endif

#if TEST_RANDOM_MAZE_ROAM
    Motor_Init();
    Motor_Enable();
    Encoder_Init();
    Encoder_ResetAll();
    Controller_Init();
    IR_Init();
    IR_BackgroundReset();
    BNO055_Reset();

    bnoStatus = BNO055_ReadChipID(&hi2c1, &bnoChipID);
    if ((bnoStatus != BNO055_OK) || (bnoChipID != BNO055_CHIP_ID_VALUE))
    {
        while (1) { BT_SendLine("BNO NOT DETECTED"); HAL_Delay(1000); }
    }

    bnoStatus = BNO055_Init(&hi2c1);
    if (bnoStatus != BNO055_OK)
    {
        while (1) { BT_SendLine("BNO INIT FAILED"); HAL_Delay(1000); }
    }

    bnoStatus = BNO055_SetMode(&hi2c1, BNO055_MODE_IMUPLUS);
    if (bnoStatus != BNO055_OK)
    {
        while (1) { BT_SendLine("BNO MODE SET FAILED"); HAL_Delay(1000); }
    }

    HAL_Delay(500);

    BT_SendLine("===== RANDOM MAZE ROAM READY =====");
    BT_SendLine("Place robot in maze. Press button.");
    Wait_For_Button();

    Run_RandomMazeRoam();
#endif

#if TEST_LEFT_WALL_FOLLOW_CALIB
    Motor_Init();
    Motor_Enable();
    Encoder_Init();
    Encoder_ResetAll();
    Controller_Init();
    IR_Init();
    IR_BackgroundReset();

    HAL_Delay(500);

    BT_SendLine("===== LEFT WALL FOLLOW CALIB READY =====");
    BT_SendLine("Place robot beside LEFT wall. Press button.");
    Wait_For_Button();

    Run_LeftWallFollowCalib();
#endif

#if TEST_DECEL_CURVE_CALIB
    Encoder_Init();
    Encoder_ResetAll();

    Motor_Init();
    Motor_Enable();

    Controller_Init();

    BT_SendLine("===== DECEL CURVE CALIB READY =====");
    BT_SendLine("The robot will ramp to 800 mm/s, hold, then coast.");
    BT_SendLine("Use a long clear test area.");

    Run_DecelCurveCalib();
#endif

}

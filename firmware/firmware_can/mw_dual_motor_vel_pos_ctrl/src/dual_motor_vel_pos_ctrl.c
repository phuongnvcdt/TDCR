// BSD 3-Clause License
// Copyright (c) 2015, Texas Instruments Incorporated
// Copyright (c) 2019, Max Planck Gesellschaft, New York University
// Copyright (c) 2024, Extended for velocity/position control
// All rights reserved.
// (See LICENSE file)

/**
 * \file dual_motor_vel_pos_ctrl.c
 * \brief Dual-motor velocity / position controller over CAN.
 *
 * Based on mw_dual_motor_torque_ctrl (udriver_firmware).
 * Adds per-motor control mode switching:
 *
 *   CTRL_MODE_TORQUE   (0)  IqRef direct from CAN mailbox 1  [original]
 *   CTRL_MODE_VELOCITY (1)  Velocity PID  → IqRef
 *   CTRL_MODE_POSITION (2)  Position PID outer → velocity setpoint → vel PID → IqRef
 *
 * CAN protocol additions
 * ----------------------
 *  Mailbox 2 IN  (arb-ID 0x06): velocity or position reference
 *      MDL = motor 1 ref  (_iq, krpm or mrev depending on mode)
 *      MDH = motor 2 ref
 *
 *  New command IDs (sent to mailbox 0):
 *      40 / 41  — set control mode motor 1 / motor 2
 *      50-55    — tune velocity PID gains  (Kp/Ki/Kd per motor)
 *      60-65    — tune position PID gains  (Kp/Ki/Kd per motor)
 *
 *  All other command IDs unchanged from original firmware.
 *
 * Velocity PID
 * ------------
 *  Runs at the same decimation rate as the original speed loop
 *  (numCtrlTicksPerSpeedTick).  Feedback = SpinTAC velocity (krpm).
 *  Output = IqRef_A, clamped to ±VEL_PID_IQREF_MAX_A.
 *
 * Position PID
 * ------------
 *  Outer loop: position error (mrev) → velocity setpoint (krpm).
 *  Inner loop: velocity PID (same as velocity mode).
 *  Outer loop runs at POS_LOOP_DECIMATION × speed-tick period.
 *
 * \author Phuong — continuum robotics project
 */

// ============================================================
// Includes
// ============================================================
#include <amd_motorware_ext/button.h>
#include <amd_motorware_ext/utils.h>
#include <math.h>

#include "canapi.h"          // extended version in this project
#include "ctrl_pid.h"        // our lightweight PID
#include "main_2mtr.h"
#include "main_helper.h"
#include "spintac.h"
#include "virtualspring.h"

#ifdef FLASH
#pragma CODE_SECTION(motor1_ISR, "ramfuncs");
#pragma CODE_SECTION(motor2_ISR, "ramfuncs");
#endif


// ============================================================
// Tuning constants  — adjust before first run
// ============================================================

//! Sample period fed to velocity PID (seconds, IQ24).
//! Must match numCtrlTicksPerSpeedTick * ISR period.
//! For default MotorWare: ISR=10kHz, speedTick decimation=10 → 1 ms
#define VEL_PID_DT_S        _IQ(0.001)

//! Max IqRef the velocity PID is allowed to command (Ampere, IQ24).
#define VEL_PID_IQREF_MAX_A _IQ(10.0)

//! Default velocity PID gains (can be overridden via CAN cmd 50-55).
#define VEL_KP_DEFAULT      _IQ(0.5)
#define VEL_KI_DEFAULT      _IQ(0.05)
#define VEL_KD_DEFAULT      _IQ(0.0)

//! Max velocity the position PID outer loop may command (krpm, IQ24).
#define POS_VEL_CMD_MAX_KRPM _IQ(1.0)

//! Position PID runs every POS_LOOP_DECIMATION velocity-tick periods.
#define POS_LOOP_DECIMATION  5

//! Default position PID gains.
#define POS_KP_DEFAULT      _IQ(2.0)
#define POS_KI_DEFAULT      _IQ(0.0)
#define POS_KD_DEFAULT      _IQ(0.1)

//! Sample period for position PID (IQ24).
#define POS_PID_DT_S        _IQ(0.001 * POS_LOOP_DECIMATION)


// ============================================================
// Global objects (HAL, FOC, SpinTAC — identical to original)
// ============================================================
#pragma DATA_SECTION(ECanaRegs,   "ECanaRegsFile");
volatile struct ECAN_REGS   ECanaRegs;
#pragma DATA_SECTION(ECanaMboxes, "ECanaMboxesFile");
volatile struct ECAN_MBOXES ECanaMboxes;

int32_t gFoobar = 0;

HAL_Handle     halHandle;
HAL_Obj        hal;
HAL_Handle_mtr halHandleMtr[2];
HAL_Obj_mtr    halMtr[2];

CLARKE_Handle  clarkeHandle_I[2];
CLARKE_Obj     clarke_I[2];
PARK_Handle    parkHandle[2];
PARK_Obj       park[2];
CLARKE_Handle  clarkeHandle_V[2];
CLARKE_Obj     clarke_V[2];
IPARK_Handle   iparkHandle[2];
IPARK_Obj      ipark[2];

EST_Handle     estHandle[2];

PID_Handle     pidHandle[2][3];
PID_Obj        pid[2][3];

SVGEN_Handle   svgenHandle[2];
SVGEN_Obj      svgen[2];
ENC_Handle     encHandle[2];
ENC_Obj        enc[2];
SLIP_Handle    slipHandle[2];
SLIP_Obj       slip[2];
ANGLE_COMP_Handle angleCompHandle[2];
ANGLE_COMP_Obj    angleComp[2];
FILTER_FO_Handle  filterHandle[2][6];
FILTER_FO_Obj     filter[2][6];
ST_Handle      stHandle[2];
ST_Obj         st_obj[2];
VIRTUALSPRING_Handle springHandle[2];
VIRTUALSPRING_Obj    spring[2];


// ============================================================
// Counters
// ============================================================
uint16_t stCntSpeed[2]    = {0, 0};
uint16_t stCntPosConv[2]  = {0, 0};
uint32_t gOffsetCalcCount[2] = {0, 0};
uint32_t gAlignCount[2]   = {0, 0};

//! Decimation counter for position outer loop
uint16_t gPosCntDecim[2]  = {0, 0};


// ============================================================
// Control mode
// ============================================================

//! Per-motor control mode.
volatile CtrlMode_e gCtrlMode[2] = {CTRL_MODE_TORQUE, CTRL_MODE_TORQUE};

//! Velocity PID objects (one per motor).
CTRL_PID_Obj gVelPid[2];

//! Position PID objects (one per motor).
CTRL_PID_Obj gPosPid[2];

//! Velocity setpoint commanded by position outer loop (krpm, IQ24).
volatile _iq gPosToVelCmd[2] = {_IQ(0.0), _IQ(0.0)};

//! Current velocity / position reference received from CAN mailbox 2.
volatile _iq gVelPosRef[2] = {_IQ(0.0), _IQ(0.0)};


// ============================================================
// Enable flags (identical to original)
// ============================================================
bool gFlag_enableVirtualSpring[2]  = {false, false};
bool gFlag_enableCan               = true;
bool gFlag_resetZeroPositionOffset = false;
bool gFlag_enablePosRolloverError  = false;


// ============================================================
// Data variables
// ============================================================
HAL_PwmData_t gPwmData[2] = {{_IQ(0.0), _IQ(0.0), _IQ(0.0)},
                              {_IQ(0.0), _IQ(0.0), _IQ(0.0)}};
HAL_AdcData_t gAdcData[2];
MATH_vec3 gOffsets_I_pu[2] = {{_IQ(0.0), _IQ(0.0), _IQ(0.0)},
                               {_IQ(0.0), _IQ(0.0), _IQ(0.0)}};
MATH_vec3 gOffsets_V_pu[2] = {{_IQ(0.0), _IQ(0.0), _IQ(0.0)},
                               {_IQ(0.0), _IQ(0.0), _IQ(0.0)}};
MATH_vec2 gIdq_ref_pu[2]   = {{_IQ(0.0), _IQ(0.0)}, {_IQ(0.0), _IQ(0.0)}};
MATH_vec2 gVdq_out_pu[2]   = {{_IQ(0.0), _IQ(0.0)}, {_IQ(0.0), _IQ(0.0)}};
MATH_vec2 gIdq_pu[2]       = {{_IQ(0.0), _IQ(0.0)}, {_IQ(0.0), _IQ(0.0)}};

_iq gTorque_Ls_Id_Iq_pu_to_Nm_sf[2];
_iq gTorque_Flux_Iq_pu_to_Nm_sf[2];
_iq gSpeed_pu_to_krpm_sf[2];
_iq gCurrent_A_to_pu_sf[2];
_iq gZeroPositionOffset[2] = {0, 0};


// ============================================================
// SpinTAC decimation
// ============================================================
const uint16_t gNumIsrTicksPerPosConvTick[2] = {ISR_TICKS_PER_POSCONV_TICK,
                                                ISR_TICKS_PER_POSCONV_TICK_2};
USER_Params gUserParams[2];
volatile MOTOR_Vars_t gMotorVars[2] = {MOTOR_Vars_INIT_Mtr1,
                                       MOTOR_Vars_INIT_Mtr2};

#ifdef FLASH
extern uint16_t *RamfuncsLoadStart, *RamfuncsLoadEnd, *RamfuncsRunStart;
#endif

#ifdef DRV8305_SPI
DRV_SPI_8305_Vars_t gDrvSpi8305Vars[2];
#endif
#ifdef DRV8301_SPI
DRV_SPI_8301_Vars_t gDrvSpi8301Vars[2];
#endif

uint32_t gTimer0_stamp                   = 0;
uint32_t gStatusLedBlinkLastToggleTime   = 0;
uint32_t gCanLastStatusMsgTime           = 0;
uint32_t gCanLastReceivedIqRef_stamp     = 0;
uint32_t gCanReceiveIqRefTimeout         = 0;
uint32_t gEnabledCanMessages             = 0;
bool     gCanAbortingMessages            = false;

Error_t  gErrors;
QepIndexWatchdog_t gQepIndexWatchdog[2] = {
    {.isInitialized = false, .indexError_counts = 0},
    {.isInitialized = false, .indexError_counts = 0}};


// ============================================================
// Helpers
// ============================================================
inline void setCanMboxStatus(const uint32_t mbox, const uint32_t status)
{
    if (status) gEnabledCanMessages |= mbox;
    else        gEnabledCanMessages &= ~mbox;
}

/**
 * \brief Initialise both PIDs for one motor with default gains.
 */
static void initCtrlPids(uint_least8_t mtrNum)
{
    CTRL_PID_init(&gVelPid[mtrNum],
                  VEL_KP_DEFAULT, VEL_KI_DEFAULT, VEL_KD_DEFAULT,
                  -VEL_PID_IQREF_MAX_A, VEL_PID_IQREF_MAX_A,
                  VEL_PID_DT_S);

    CTRL_PID_init(&gPosPid[mtrNum],
                  POS_KP_DEFAULT, POS_KI_DEFAULT, POS_KD_DEFAULT,
                  -POS_VEL_CMD_MAX_KRPM, POS_VEL_CMD_MAX_KRPM,
                  POS_PID_DT_S);
}

/**
 * \brief Reset PID state and velocity setpoints when switching modes.
 */
static void onCtrlModeChange(uint_least8_t mtrNum)
{
    CTRL_PID_reset(&gVelPid[mtrNum]);
    CTRL_PID_reset(&gPosPid[mtrNum]);
    gPosToVelCmd[mtrNum]         = _IQ(0.0);
    gMotorVars[mtrNum].IqRef_A   = _IQ(0.0);
}


// ============================================================
// main()
// ============================================================
void main(void)
{
#ifdef FLASH
    memCopy((uint16_t *)&RamfuncsLoadStart, (uint16_t *)&RamfuncsLoadEnd,
            (uint16_t *)&RamfuncsRunStart);
#endif

    gErrors.all = 0;

    halHandle = HAL_init(&hal, sizeof(hal));
    USER_setParamsMtr1(&gUserParams[HAL_MTR1]);
    USER_setParamsMtr2(&gUserParams[HAL_MTR2]);
    HAL_setParams(halHandle, &gUserParams[HAL_MTR1]);

    // QEP qualification — fast encoder support
    GPIO_setQualificationPeriod(hal.gpioHandle, GPIO_Number_16, 11);
    GPIO_setQualificationPeriod(hal.gpioHandle, GPIO_Number_50, 11);
    GPIO_setQualificationPeriod(hal.gpioHandle, GPIO_Number_56, 11);

    // Timer 0
    uint32_t timerPeriod_cnts =
        ((uint32_t)gUserParams[0].systemFreq_MHz * 1e6l) / TIMER0_FREQ_Hz - 1;
    overwriteSetupTimer0(halHandle, timerPeriod_cnts);

    estHandle[HAL_MTR1] = EST_init((void *)USER_EST_HANDLE_ADDRESS,   0x200);
    estHandle[HAL_MTR2] = EST_init((void *)USER_EST_HANDLE_ADDRESS_1, 0x200);

    {
        uint_least8_t mtrNum;
        for (mtrNum = HAL_MTR1; mtrNum <= HAL_MTR2; mtrNum++) {

            halHandleMtr[mtrNum] =
                HAL_init_mtr(&halMtr[mtrNum], sizeof(halMtr[mtrNum]),
                             (HAL_MtrSelect_e)mtrNum);
            HAL_setParamsMtr(halHandleMtr[mtrNum], halHandle,
                             &gUserParams[mtrNum]);

            {
                CTRL_Handle ctrlHandle =
                    CTRL_init((void *)USER_CTRL_HANDLE_ADDRESS, 0x200);
                CTRL_Obj *obj = (CTRL_Obj *)ctrlHandle;
                obj->estHandle = estHandle[mtrNum];
                CTRL_setParams(ctrlHandle, &gUserParams[mtrNum]);
                CTRL_setUserMotorParams(ctrlHandle);
                CTRL_setupEstIdleState(ctrlHandle);
            }

            angleCompHandle[mtrNum] =
                ANGLE_COMP_init(&angleComp[mtrNum], sizeof(angleComp[mtrNum]));
            ANGLE_COMP_setParams(angleCompHandle[mtrNum],
                                 gUserParams[mtrNum].iqFullScaleFreq_Hz,
                                 gUserParams[mtrNum].pwmPeriod_usec,
                                 gUserParams[mtrNum].numPwmTicksPerIsrTick);

            clarkeHandle_I[mtrNum] =
                CLARKE_init(&clarke_I[mtrNum], sizeof(clarke_I[mtrNum]));
            clarkeHandle_V[mtrNum] =
                CLARKE_init(&clarke_V[mtrNum], sizeof(clarke_V[mtrNum]));
            parkHandle[mtrNum] = PARK_init(&park[mtrNum], sizeof(park[mtrNum]));

            gTorque_Ls_Id_Iq_pu_to_Nm_sf[mtrNum] =
                USER_computeTorque_Ls_Id_Iq_pu_to_Nm_sf(&gUserParams[mtrNum]);
            gTorque_Flux_Iq_pu_to_Nm_sf[mtrNum] =
                USER_computeTorque_Flux_Iq_pu_to_Nm_sf(&gUserParams[mtrNum]);
            gSpeed_pu_to_krpm_sf[mtrNum] =
                _IQ((gUserParams[mtrNum].iqFullScaleFreq_Hz * 60.0) /
                    ((float_t)gUserParams[mtrNum].motor_numPolePairs * 1000.0));
            gCurrent_A_to_pu_sf[mtrNum] =
                _IQ(1.0 / gUserParams[mtrNum].iqFullScaleCurrent_A);

            EST_setFlag_enableRsRecalc(estHandle[mtrNum], false);
            setupClarke_I(clarkeHandle_I[mtrNum],
                          gUserParams[mtrNum].numCurrentSensors);
            setupClarke_V(clarkeHandle_V[mtrNum],
                          gUserParams[mtrNum].numVoltageSensors);

            // MotorWare current PIDs (unchanged)
            pidHandle[mtrNum][1] =
                PID_init(&pid[mtrNum][1], sizeof(pid[mtrNum][1]));
            pidHandle[mtrNum][2] =
                PID_init(&pid[mtrNum][2], sizeof(pid[mtrNum][2]));
            pidSetup(pidHandle[mtrNum], gUserParams[mtrNum]);

            iparkHandle[mtrNum] =
                IPARK_init(&ipark[mtrNum], sizeof(ipark[mtrNum]));
            svgenHandle[mtrNum] =
                SVGEN_init(&svgen[mtrNum], sizeof(svgen[mtrNum]));

            {
                uint16_t cnt = 0;
                _iq b0 = _IQ(gUserParams[mtrNum].offsetPole_rps /
                             (float_t)gUserParams[mtrNum].ctrlFreq_Hz);
                _iq a1 = (b0 - _IQ(1.0));
                _iq b1 = _IQ(0.0);
                for (cnt = 0; cnt < 6; cnt++) {
                    filterHandle[mtrNum][cnt] = FILTER_FO_init(
                        &filter[mtrNum][cnt], sizeof(filter[mtrNum][0]));
                    FILTER_FO_setDenCoeffs(filterHandle[mtrNum][cnt], a1);
                    FILTER_FO_setNumCoeffs(filterHandle[mtrNum][cnt], b0, b1);
                    FILTER_FO_setInitialConditions(filterHandle[mtrNum][cnt],
                                                   _IQ(0.0), _IQ(0.0));
                }
                gMotorVars[mtrNum].Flag_enableOffsetcalc = false;
            }

            encHandle[mtrNum]  = ENC_init(&enc[mtrNum], sizeof(enc[mtrNum]));
            slipHandle[mtrNum] = SLIP_init(&slip[mtrNum], sizeof(slip[mtrNum]));
            SLIP_setup(slipHandle[mtrNum],
                       _IQ(gUserParams[mtrNum].ctrlPeriod_sec));

            HAL_setupFaults(halHandleMtr[mtrNum]);
            stHandle[mtrNum] = ST_init(&st_obj[mtrNum], sizeof(st_obj[mtrNum]));

            springHandle[mtrNum] =
                VIRTUALSPRING_init(&spring[mtrNum], sizeof(spring[mtrNum]));
            VIRTUALSPRING_setup(
                springHandle[mtrNum], 10, _IQ(2.0),
                STPOSCONV_getMRevMaximum_mrev(st_obj[mtrNum].posConvHandle));

            // Initialise our velocity / position PIDs
            initCtrlPids(mtrNum);
        }
    }

    ENC_setup(encHandle[HAL_MTR1], 1, USER_MOTOR_NUM_POLE_PAIRS,
              USER_MOTOR_ENCODER_LINES, 0, USER_IQ_FULL_SCALE_FREQ_Hz,
              USER_ISR_FREQ_Hz, 8000.0);
    ENC_setup(encHandle[HAL_MTR2], 1, USER_MOTOR_NUM_POLE_PAIRS_2,
              USER_MOTOR_ENCODER_LINES_2, 0, USER_IQ_FULL_SCALE_FREQ_Hz_2,
              USER_ISR_FREQ_Hz_2, 8000.0);

    setupQepIndexInterrupt(halHandle, halHandleMtr,
                           &qep1IndexISR, &qep2IndexISR);

    ST_setupPosConv_mtr1(stHandle[HAL_MTR1]);
    ST_setupPosConv_mtr2(stHandle[HAL_MTR2]);

    // Current / voltage feedback offsets (same as original)
    gOffsets_I_pu[HAL_MTR1].value[0] = _IQ(I_A_offset);
    gOffsets_I_pu[HAL_MTR1].value[1] = _IQ(I_B_offset);
    gOffsets_I_pu[HAL_MTR1].value[2] = _IQ(I_C_offset);
    gOffsets_V_pu[HAL_MTR1].value[0] = _IQ(V_A_offset);
    gOffsets_V_pu[HAL_MTR1].value[1] = _IQ(V_B_offset);
    gOffsets_V_pu[HAL_MTR1].value[2] = _IQ(V_C_offset);
    gOffsets_I_pu[HAL_MTR2].value[0] = _IQ(I_A_offset_2);
    gOffsets_I_pu[HAL_MTR2].value[1] = _IQ(I_B_offset_2);
    gOffsets_I_pu[HAL_MTR2].value[2] = _IQ(I_C_offset_2);
    gOffsets_V_pu[HAL_MTR2].value[0] = _IQ(V_A_offset_2);
    gOffsets_V_pu[HAL_MTR2].value[1] = _IQ(V_B_offset_2);
    gOffsets_V_pu[HAL_MTR2].value[2] = _IQ(V_C_offset_2);

    HAL_initIntVectorTable(halHandle);
    HAL_enableAdcInts(halHandle);
    HAL_enableGlobalInts(halHandle);
    HAL_enableDebugInt(halHandle);
    HAL_enableTimer0Int(halHandle);
    PIE_registerTimer0IntHandler(hal.pieHandle, &timer0_ISR);

    HAL_disablePwm(halHandleMtr[HAL_MTR1]);
    HAL_disablePwm(halHandleMtr[HAL_MTR2]);
    HAL_overwriteSetupGpio(halHandle);

    // CAN setup — registers our extended mailbox 2 as well
    setupCan(halHandle, &can1_ISR);

    gMotorVars[HAL_MTR1].Flag_enableSys = true;

#ifdef DRV8305_SPI
    HAL_enableDrv(halHandleMtr[HAL_MTR1]);
    HAL_enableDrv(halHandleMtr[HAL_MTR2]);
    HAL_setupDrvSpi(halHandleMtr[HAL_MTR1], &gDrvSpi8305Vars[HAL_MTR1]);
    HAL_setupDrvSpi(halHandleMtr[HAL_MTR2], &gDrvSpi8305Vars[HAL_MTR2]);
#endif
#ifdef DRV8301_SPI
    HAL_enableDrv(halHandleMtr[HAL_MTR1]);
    HAL_enableDrv(halHandleMtr[HAL_MTR2]);
    HAL_setupDrvSpi(halHandleMtr[HAL_MTR1], &gDrvSpi8301Vars[HAL_MTR1]);
    HAL_setupDrvSpi(halHandleMtr[HAL_MTR2], &gDrvSpi8301Vars[HAL_MTR2]);
#endif

    // ------------------------------------------------------------------
    // Background loop
    // ------------------------------------------------------------------
    for (;;) {
        while (!(gMotorVars[HAL_MTR1].Flag_enableSys)) {
            LED_run(halHandle);
            maybeSendCanStatusMsg();
        }

        while (gMotorVars[HAL_MTR1].Flag_enableSys) {
            uint_least8_t mtrNum = HAL_MTR1;

            checkErrors();
            LED_run(halHandle);

            if (gErrors.all) {
                gMotorVars[HAL_MTR1].Flag_enableSys = false;
                break;
            }

            maybeSendCanStatusMsg();

            if (BUTTON_isPressed(hal.gpioHandle)) {
                gMotorVars[HAL_MTR1].Flag_enableSys = false;
            }

            for (mtrNum = HAL_MTR1; mtrNum <= HAL_MTR2; mtrNum++) {

                if (gMotorVars[mtrNum].Flag_Run_Identify) {
                    bool vspringChanged;

                    EST_updateState(estHandle[mtrNum], 0);

#ifdef FAST_ROM_V1p6
                    softwareUpdate1p6(estHandle[mtrNum], &gUserParams[mtrNum]);
#endif

                    vspringChanged = VIRTUALSPRING_setEnabled(
                        springHandle[mtrNum],
                        gFlag_enableVirtualSpring[mtrNum]);
                    if (vspringChanged) {
                        if (VIRTUALSPRING_isEnabled(springHandle[mtrNum])) {
                            VIRTUALSPRING_scheduleResetOffset(
                                springHandle[mtrNum]);
                        } else {
                            gMotorVars[mtrNum].IqRef_A = 0;
                        }
                    }

                    HAL_enablePwm(halHandleMtr[mtrNum]);

                } else {
                    EST_setIdle(estHandle[mtrNum]);
                    HAL_disablePwm(halHandleMtr[mtrNum]);

                    // Reset all integrators on motor disable
                    PID_setUi(pidHandle[mtrNum][0], _IQ(0.0));
                    PID_setUi(pidHandle[mtrNum][1], _IQ(0.0));
                    PID_setUi(pidHandle[mtrNum][2], _IQ(0.0));
                    CTRL_PID_reset(&gVelPid[mtrNum]);
                    CTRL_PID_reset(&gPosPid[mtrNum]);

                    gIdq_ref_pu[mtrNum].value[0] = _IQ(0.0);
                    gIdq_ref_pu[mtrNum].value[1] = _IQ(0.0);
                }

                updateGlobalVariables(mtrNum);
                EST_setFlag_enableForceAngle(estHandle[mtrNum],
                    gMotorVars[mtrNum].Flag_enableForceAngle);

#ifdef DRV8305_SPI
                HAL_writeDrvData(halHandleMtr[mtrNum], &gDrvSpi8305Vars[mtrNum]);
                HAL_readDrvData(halHandleMtr[mtrNum], &gDrvSpi8305Vars[mtrNum]);
#endif
#ifdef DRV8301_SPI
                HAL_writeDrvData(halHandleMtr[mtrNum], &gDrvSpi8301Vars[mtrNum]);
                HAL_readDrvData(halHandleMtr[mtrNum], &gDrvSpi8301Vars[mtrNum]);
#endif
            }
        }
    }
}


// ============================================================
// Helper functions called from ISRs
// ============================================================

static inline void sendMotorDataViaCan(const uint_least8_t mtrNum, ST_Handle st)
{
    ST_Obj *st_obj_ptr = (ST_Obj *)st;
    _iq current_iq = _IQmpy(gIdq_pu[mtrNum].value[1],
                            _IQ(gUserParams[mtrNum].iqFullScaleCurrent_A));
    _iq position = STPOSCONV_getPosition_mrev(st_obj_ptr->posConvHandle);
    _iq speed = _IQmpy(STPOSCONV_getVelocityFiltered(st_obj_ptr->posConvHandle),
                       gSpeed_pu_to_krpm_sf[mtrNum]);
    if (mtrNum == HAL_MTR1) {
        CAN_setDataMotor1(current_iq, position, speed);
    } else {
        CAN_setDataMotor2(current_iq, position, speed);
    }
}

bool checkEncoderError(const QepIndexWatchdog_t qiwd)
{
    return abs(qiwd.indexError_counts) > QEP_MAX_INDEX_ERROR;
}

static void checkQepIndexError(HAL_Handle_mtr *halHandleMtrArg,
                               QepIndexWatchdog_t *qiwd)
{
    (void)halHandleMtrArg;
    gErrors.bit.qep_error = (checkEncoderError(qiwd[0]) ||
                             checkEncoderError(qiwd[1]));
}

static void checkPosRolloverError(ST_Handle *st, Error_t *errors)
{
    errors->bit.pos_rollover =
        ((STPOSCONV_getPositionRollOver(
              ((ST_Obj *)st[HAL_MTR1])->posConvHandle) != 0) ||
         (STPOSCONV_getPositionRollOver(
              ((ST_Obj *)st[HAL_MTR2])->posConvHandle) != 0));
}


// ============================================================
// Motor ISR — called at PWM / ADC frequency (~10 kHz per motor)
// This is the innermost real-time loop.
// ============================================================

/**
 * \brief Common ISR body for one motor.
 *
 * Identical to original up to the point where IqRef is set.
 * At that point the control mode is checked and the appropriate
 * PID loop runs instead of the raw CAN IqRef.
 */
void generic_motor_ISR(const HAL_MtrSelect_e mtrNum)
{
    static _iq angle_pu[2] = {_IQ(0.0), _IQ(0.0)};
    _iq speed_pu = _IQ(0.0);

    HAL_readAdcDataWithOffsets(halHandle, halHandleMtr[mtrNum], &gAdcData[mtrNum]);

    // ---- Offset calibration (unchanged) ----
    if (gMotorVars[mtrNum].Flag_enableOffsetcalc == true) {
        runOffsetsCalculation(mtrNum);
    } else {
        // Apply offsets
        gAdcData[mtrNum].I.value[0] -= gOffsets_I_pu[mtrNum].value[0];
        gAdcData[mtrNum].I.value[1] -= gOffsets_I_pu[mtrNum].value[1];
        gAdcData[mtrNum].I.value[2] -= gOffsets_I_pu[mtrNum].value[2];
        gAdcData[mtrNum].V.value[0] -= gOffsets_V_pu[mtrNum].value[0];
        gAdcData[mtrNum].V.value[1] -= gOffsets_V_pu[mtrNum].value[1];
        gAdcData[mtrNum].V.value[2] -= gOffsets_V_pu[mtrNum].value[2];
    }

    // ---- Clarke / Park (unchanged) ----
    MATH_vec3 Iabc_pu, Vabc_pu;
    Iabc_pu.value[0] = gAdcData[mtrNum].I.value[0];
    Iabc_pu.value[1] = gAdcData[mtrNum].I.value[1];
    Iabc_pu.value[2] = gAdcData[mtrNum].I.value[2];
    CLARKE_run(clarkeHandle_I[mtrNum], &Iabc_pu, &(gIdq_pu[mtrNum]));

    Vabc_pu.value[0] = gAdcData[mtrNum].V.value[0];
    Vabc_pu.value[1] = gAdcData[mtrNum].V.value[1];
    Vabc_pu.value[2] = gAdcData[mtrNum].V.value[2];
    MATH_vec2 Vab_pu;
    CLARKE_run(clarkeHandle_V[mtrNum], &Vabc_pu, &Vab_pu);

    // ---- Estimator (unchanged) ----
    EST_run(estHandle[mtrNum], &(gIdq_pu[mtrNum]), &Vab_pu,
            gAdcData[mtrNum].dcBus, angle_pu[mtrNum]);

    // ---- Park ----
    MATH_vec2 Iab_pu;
    Iab_pu.value[0] = gIdq_pu[mtrNum].value[0];
    Iab_pu.value[1] = gIdq_pu[mtrNum].value[1];
    MATH_vec2 phasor;
    phasor.value[0] = _IQcosPU(angle_pu[mtrNum]);
    phasor.value[1] = _IQsinPU(angle_pu[mtrNum]);
    PARK_setPhasor(parkHandle[mtrNum], &phasor);
    PARK_run(parkHandle[mtrNum], &Iab_pu, &gIdq_pu[mtrNum]);

    // ---- Encoder / SpinTAC position-converter ----
    ENC_calcElecAngle(encHandle[mtrNum],
                      HAL_getQepPosnCounts(halHandleMtr[mtrNum]));

    if (++stCntPosConv[mtrNum] >= gNumIsrTicksPerPosConvTick[mtrNum]) {
        stCntPosConv[mtrNum] = 0;
        ST_runPosConv(stHandle[mtrNum], encHandle[mtrNum], slipHandle[mtrNum],
                      &gIdq_pu[mtrNum], gUserParams[mtrNum].motor_type);
    }

    // ---- Upper-level control (speed-tick decimation) ----
    if (gMotorVars[mtrNum].Flag_Run_Identify) {
        _iq refValue, fbackValue, outMax_pu, oneOverDcBus;

        if (gMotorVars[mtrNum].Flag_enableAlignment == false) {

            if (++stCntSpeed[mtrNum] >=
                gUserParams[mtrNum].numCtrlTicksPerSpeedTick) {

                stCntSpeed[mtrNum] = 0;

                // ================================================
                //  CONTROL MODE DISPATCH
                // ================================================
                if (VIRTUALSPRING_isEnabled(springHandle[mtrNum])) {
                    // Virtual spring overrides all modes
                    VIRTUALSPRING_run(springHandle[mtrNum],
                                      STPOSCONV_getPosition_mrev(
                                          st_obj[mtrNum].posConvHandle));
                    gMotorVars[mtrNum].IqRef_A =
                        VIRTUALSPRING_getIqRef_A(springHandle[mtrNum]);

                } else {

                    switch (gCtrlMode[mtrNum]) {

                    // ---- TORQUE mode: original behaviour ----
                    case CTRL_MODE_TORQUE:
                        if (gFlag_enableCan) {
                            gMotorVars[mtrNum].IqRef_A =
                                CAN_getIqRef(mtrNum);
                        }
                        // else: set by GUI/debugger
                        break;

                    // ---- VELOCITY mode ----
                    case CTRL_MODE_VELOCITY:
                    {
                        // Feedback: velocity in krpm
                        _iq vel_krpm = _IQmpy(
                            STPOSCONV_getVelocity(st_obj[mtrNum].posConvHandle),
                            gSpeed_pu_to_krpm_sf[mtrNum]);

                        gMotorVars[mtrNum].IqRef_A =
                            CTRL_PID_run(&gVelPid[mtrNum],
                                         gVelPosRef[mtrNum],
                                         vel_krpm);
                        break;
                    }

                    // ---- POSITION mode ----
                    case CTRL_MODE_POSITION:
                    {
                        // Outer loop runs at reduced rate
                        if (++gPosCntDecim[mtrNum] >= POS_LOOP_DECIMATION) {
                            gPosCntDecim[mtrNum] = 0;

                            _iq pos_mrev = STPOSCONV_getPosition_mrev(
                                st_obj[mtrNum].posConvHandle);

                            // Position PID → velocity setpoint
                            gPosToVelCmd[mtrNum] = CTRL_PID_run(
                                &gPosPid[mtrNum],
                                gVelPosRef[mtrNum],  // position reference
                                pos_mrev);
                        }

                        // Inner velocity loop runs every speed tick
                        _iq vel_krpm = _IQmpy(
                            STPOSCONV_getVelocity(st_obj[mtrNum].posConvHandle),
                            gSpeed_pu_to_krpm_sf[mtrNum]);

                        gMotorVars[mtrNum].IqRef_A =
                            CTRL_PID_run(&gVelPid[mtrNum],
                                         gPosToVelCmd[mtrNum],
                                         vel_krpm);
                        break;
                    }

                    default:
                        gMotorVars[mtrNum].IqRef_A = _IQ(0.0);
                        break;
                    }
                }
                // ================================================

                gIdq_ref_pu[mtrNum].value[0] = _IQmpy(
                    gMotorVars[mtrNum].IdRef_A, gCurrent_A_to_pu_sf[mtrNum]);
                gIdq_ref_pu[mtrNum].value[1] = _IQmpy(
                    gMotorVars[mtrNum].IqRef_A, gCurrent_A_to_pu_sf[mtrNum]);
            }

            // ---- Electrical angle ----
            if (gUserParams[mtrNum].motor_type == MOTOR_Type_Induction) {
                SLIP_setElectricalAngle(slipHandle[mtrNum],
                                        ENC_getElecAngle(encHandle[mtrNum]));
                SLIP_run(slipHandle[mtrNum]);
                angle_pu[mtrNum] = SLIP_getMagneticAngle(slipHandle[mtrNum]);
            } else {
                angle_pu[mtrNum] = ENC_getElecAngle(encHandle[mtrNum]);
            }
            speed_pu = STPOSCONV_getVelocity(st_obj[mtrNum].posConvHandle);

        } else {
            // Alignment
            angle_pu[mtrNum] = _IQ(0.0);
            speed_pu = _IQ(0.0);
            gIdq_ref_pu[mtrNum].value[0] = _IQmpy(
                _IQ(gUserParams[mtrNum].maxCurrent_resEst),
                gCurrent_A_to_pu_sf[mtrNum]);
            gIdq_ref_pu[mtrNum].value[1] = _IQ(0.0);

            if (gUserParams[mtrNum].motor_type == MOTOR_Type_Pm) {
                ENC_setZeroOffset(encHandle[mtrNum],
                    (uint32_t)(HAL_getQepPosnMaximum(halHandleMtr[mtrNum]) -
                               HAL_getQepPosnCounts(halHandleMtr[mtrNum])));
            }
            if (gAlignCount[mtrNum]++ >=
                gUserParams[mtrNum].ctrlWaitTime[CTRL_State_OffLine]) {
                gMotorVars[mtrNum].Flag_enableAlignment = false;
                gAlignCount[mtrNum] = 0;
                gIdq_ref_pu[mtrNum].value[0] = _IQ(0.0);
                CTRL_PID_reset(&gVelPid[mtrNum]);
                CTRL_PID_reset(&gPosPid[mtrNum]);
            }
        }

        // ---- Angle compensation (same as torque ctrl) ----
        ANGLE_COMP_run(angleCompHandle[mtrNum], speed_pu, angle_pu[mtrNum]);
        angle_pu[mtrNum] = ANGLE_COMP_getAngleComp_pu(angleCompHandle[mtrNum]);

        // ---- Current PIDs + inverse Park + SVGEN ----
        phasor.value[0] = _IQcosPU(angle_pu[mtrNum]);
        phasor.value[1] = _IQsinPU(angle_pu[mtrNum]);

        refValue   = gIdq_ref_pu[mtrNum].value[0];
        fbackValue = gIdq_pu[mtrNum].value[0];
        outMax_pu  = _IQ(0.5);
        PID_setMinMax(pidHandle[mtrNum][1], -outMax_pu, outMax_pu);
        PID_run(pidHandle[mtrNum][1], refValue, fbackValue,
                &(gVdq_out_pu[mtrNum].value[0]));

        refValue   = gIdq_ref_pu[mtrNum].value[1];
        fbackValue = gIdq_pu[mtrNum].value[1];
        outMax_pu = _IQsqrt(
            _IQ(gUserParams[mtrNum].maxVsMag_pu * gUserParams[mtrNum].maxVsMag_pu) -
            _IQmpy(gVdq_out_pu[mtrNum].value[0], gVdq_out_pu[mtrNum].value[0]));
        PID_setMinMax(pidHandle[mtrNum][2], -outMax_pu, outMax_pu);
        PID_run(pidHandle[mtrNum][2], refValue, fbackValue,
                &(gVdq_out_pu[mtrNum].value[1]));

        IPARK_setPhasor(iparkHandle[mtrNum], &phasor);
        IPARK_run(iparkHandle[mtrNum], &gVdq_out_pu[mtrNum], &Vab_pu);

        oneOverDcBus = _IQdiv(_IQ(1.0), gAdcData[mtrNum].dcBus);
        Vab_pu.value[0] = _IQmpy(Vab_pu.value[0], oneOverDcBus);
        Vab_pu.value[1] = _IQmpy(Vab_pu.value[1], oneOverDcBus);

        SVGEN_run(svgenHandle[mtrNum], &Vab_pu, &(gPwmData[mtrNum].Tabc));
    } else {
        HAL_disablePwm(halHandleMtr[mtrNum]);
        gPwmData[mtrNum].Tabc.value[0] = _IQ(0.0);
        gPwmData[mtrNum].Tabc.value[1] = _IQ(0.0);
        gPwmData[mtrNum].Tabc.value[2] = _IQ(0.0);
    }

    // Write PWM
    HAL_writePwmData(halHandleMtr[mtrNum], &gPwmData[mtrNum]);

    // Send CAN data
    sendMotorDataViaCan(mtrNum, stHandle[mtrNum]);
}


// ---- Per-motor ISR entry points (needed by HAL vector table) ----
interrupt void motor1_ISR(void)
{
    HAL_acqAdcInt(halHandle, ADC_IntNumber_1);
    generic_motor_ISR(HAL_MTR1);
}
interrupt void motor2_ISR(void)
{
    HAL_acqAdcInt(halHandle, ADC_IntNumber_2);
    generic_motor_ISR(HAL_MTR2);
}


// ============================================================
// CAN ISR — runs when a CAN message is received
// ============================================================
interrupt void can1_ISR(void)
{
    // ---- Update velocity/position reference from mailbox 2 ----
    if (CAN_checkAndClearRMP(CAN_MBOX_IN_VelPosRef)) {
        gVelPosRef[HAL_MTR1] = CAN_getVelPosRef(HAL_MTR1);
        gVelPosRef[HAL_MTR2] = CAN_getVelPosRef(HAL_MTR2);
    }

    // ---- Update CAN RX timeout watchdog (torque mode only) ----
    if (CAN_checkAndClearRMP(CAN_MBOX_IN_IqRef)) {
        gCanLastReceivedIqRef_stamp = gTimer0_stamp;
    }

    // ---- Command mailbox ----
    if (CAN_checkAndClearRMP(CAN_MBOX_IN_COMMANDS)) {
        CAN_Command_t cmd = CAN_getCommand();

        switch (cmd.id) {

        // ---- Original commands (unchanged) ----
        case CAN_CMD_ENABLE_SYS:
            gMotorVars[HAL_MTR1].Flag_enableSys = cmd.value;
            break;
        case CAN_CMD_ENABLE_MTR1:
            gMotorVars[HAL_MTR1].Flag_Run_Identify = cmd.value;
            break;
        case CAN_CMD_ENABLE_MTR2:
            gMotorVars[HAL_MTR2].Flag_Run_Identify = cmd.value;
            break;
        case CAN_CMD_ENABLE_VSPRING1:
            spring[HAL_MTR1].enabled = cmd.value;
            break;
        case CAN_CMD_ENABLE_VSPRING2:
            spring[HAL_MTR2].enabled = cmd.value;
            break;
        case CAN_CMD_SEND_CURRENT:
            setCanMboxStatus(CAN_MBOX_OUT_Iq, cmd.value);
            break;
        case CAN_CMD_SEND_POSITION:
            setCanMboxStatus(CAN_MBOX_OUT_ENC_POS, cmd.value);
            break;
        case CAN_CMD_SEND_VELOCITY:
            setCanMboxStatus(CAN_MBOX_OUT_SPEED, cmd.value);
            break;
        case CAN_CMD_SEND_ADC6:
            setCanMboxStatus(CAN_MBOX_OUT_ADC6, cmd.value);
            break;
        case CAN_CMD_SEND_ENC_INDEX:
            setCanMboxStatus(CAN_MBOX_OUT_ENC_INDEX, cmd.value);
            break;
        case CAN_CMD_SEND_ALL:
            gEnabledCanMessages = cmd.value
                ? (CAN_MBOX_OUT_Iq | CAN_MBOX_OUT_ENC_POS |
                   CAN_MBOX_OUT_SPEED | CAN_MBOX_OUT_ADC6 |
                   CAN_MBOX_OUT_ENC_INDEX)
                : 0;
            break;
        case CAN_CMD_SET_CAN_RECV_TIMEOUT:
            gCanReceiveIqRefTimeout = cmd.value;
            break;
        case CAN_CMD_ENABLE_POS_ROLLOVER_ERROR:
            gFlag_enablePosRolloverError = cmd.value;
            break;

        // ---- NEW: control mode switching ----
        case CAN_CMD_SET_CTRL_MODE_MTR1:
            if (cmd.value <= CTRL_MODE_POSITION) {
                gCtrlMode[HAL_MTR1] = (CtrlMode_e)cmd.value;
                onCtrlModeChange(HAL_MTR1);
            }
            break;
        case CAN_CMD_SET_CTRL_MODE_MTR2:
            if (cmd.value <= CTRL_MODE_POSITION) {
                gCtrlMode[HAL_MTR2] = (CtrlMode_e)cmd.value;
                onCtrlModeChange(HAL_MTR2);
            }
            break;

        // ---- NEW: velocity PID gains ----
        case CAN_CMD_SET_VEL_KP_MTR1:
            gVelPid[HAL_MTR1].Kp = (_iq)cmd.value;
            CTRL_PID_reset(&gVelPid[HAL_MTR1]);
            break;
        case CAN_CMD_SET_VEL_KI_MTR1:
            gVelPid[HAL_MTR1].Ki = _IQmpy((_iq)cmd.value, VEL_PID_DT_S);
            CTRL_PID_reset(&gVelPid[HAL_MTR1]);
            break;
        case CAN_CMD_SET_VEL_KD_MTR1:
            gVelPid[HAL_MTR1].Kd = _IQdiv((_iq)cmd.value, VEL_PID_DT_S);
            CTRL_PID_reset(&gVelPid[HAL_MTR1]);
            break;
        case CAN_CMD_SET_VEL_KP_MTR2:
            gVelPid[HAL_MTR2].Kp = (_iq)cmd.value;
            CTRL_PID_reset(&gVelPid[HAL_MTR2]);
            break;
        case CAN_CMD_SET_VEL_KI_MTR2:
            gVelPid[HAL_MTR2].Ki = _IQmpy((_iq)cmd.value, VEL_PID_DT_S);
            CTRL_PID_reset(&gVelPid[HAL_MTR2]);
            break;
        case CAN_CMD_SET_VEL_KD_MTR2:
            gVelPid[HAL_MTR2].Kd = _IQdiv((_iq)cmd.value, VEL_PID_DT_S);
            CTRL_PID_reset(&gVelPid[HAL_MTR2]);
            break;

        // ---- NEW: position PID gains ----
        case CAN_CMD_SET_POS_KP_MTR1:
            gPosPid[HAL_MTR1].Kp = (_iq)cmd.value;
            CTRL_PID_reset(&gPosPid[HAL_MTR1]);
            break;
        case CAN_CMD_SET_POS_KI_MTR1:
            gPosPid[HAL_MTR1].Ki = _IQmpy((_iq)cmd.value, POS_PID_DT_S);
            CTRL_PID_reset(&gPosPid[HAL_MTR1]);
            break;
        case CAN_CMD_SET_POS_KD_MTR1:
            gPosPid[HAL_MTR1].Kd = _IQdiv((_iq)cmd.value, POS_PID_DT_S);
            CTRL_PID_reset(&gPosPid[HAL_MTR1]);
            break;
        case CAN_CMD_SET_POS_KP_MTR2:
            gPosPid[HAL_MTR2].Kp = (_iq)cmd.value;
            CTRL_PID_reset(&gPosPid[HAL_MTR2]);
            break;
        case CAN_CMD_SET_POS_KI_MTR2:
            gPosPid[HAL_MTR2].Ki = _IQmpy((_iq)cmd.value, POS_PID_DT_S);
            CTRL_PID_reset(&gPosPid[HAL_MTR2]);
            break;
        case CAN_CMD_SET_POS_KD_MTR2:
            gPosPid[HAL_MTR2].Kd = _IQdiv((_iq)cmd.value, POS_PID_DT_S);
            CTRL_PID_reset(&gPosPid[HAL_MTR2]);
            break;

        default:
            break;
        }
    }

    // CAN timeout check (same as original — only applies in TORQUE mode)
    bool canTimeout =
        (gFlag_enableCan
         && gCanReceiveIqRefTimeout != 0
         && ((gMotorVars[HAL_MTR1].Flag_Run_Identify &&
              gMotorVars[HAL_MTR1].IqRef_A != 0) ||
             (gMotorVars[HAL_MTR2].Flag_Run_Identify &&
              gMotorVars[HAL_MTR2].IqRef_A != 0))
         && (gCanLastReceivedIqRef_stamp <
             gTimer0_stamp - gCanReceiveIqRefTimeout));

    if (canTimeout) gErrors.bit.can_recv_timeout = 1;

    // QEP index error check
    checkQepIndexError(halHandleMtr, gQepIndexWatchdog);

    // Position rollover error
    if (gFlag_enablePosRolloverError) {
        checkPosRolloverError(stHandle, &gErrors);
    }

    // PIE acknowledge
    HAL_Obj *obj = (HAL_Obj *)halHandle;
    PIE_clearInt(obj->pieHandle, PIE_GroupNumber_9);
}


// ============================================================
// Timer 0 ISR (heartbeat, unchanged)
// ============================================================
interrupt void timer0_ISR(void)
{
    ++gTimer0_stamp;

    uint32_t mbox_mask = gEnabledCanMessages;
    if (CAN_checkTransmissionPending(mbox_mask)) {
        if (!gCanAbortingMessages) {
            gCanAbortingMessages = true;
            CAN_abort(mbox_mask);
        }
    } else {
        gCanAbortingMessages = false;
    }

    CAN_setAdcIn6Values(HAL_readAdcResult(halHandle, POTI_RESULT1),
                        HAL_readAdcResult(halHandle, POTI_RESULT2));
    CAN_send(mbox_mask);

    HAL_Obj *obj = (HAL_Obj *)halHandle;
    PIE_clearInt(obj->pieHandle, PIE_GroupNumber_1);
}


// ============================================================
// Offset calibration
// ============================================================
void runOffsetsCalculation(HAL_MtrSelect_e mtrNum)
{
    uint16_t cnt;

    HAL_enablePwm(halHandleMtr[mtrNum]);

    for (cnt = 0; cnt < 3; cnt++) {
        gPwmData[mtrNum].Tabc.value[cnt] = _IQ(0.0);
        gOffsets_I_pu[mtrNum].value[cnt] = _IQ(0.0);
        gOffsets_V_pu[mtrNum].value[cnt] = _IQ(0.0);
        FILTER_FO_run(filterHandle[mtrNum][cnt],     gAdcData[mtrNum].I.value[cnt]);
        FILTER_FO_run(filterHandle[mtrNum][cnt + 3], gAdcData[mtrNum].V.value[cnt]);
    }

    if (gOffsetCalcCount[mtrNum]++ >=
        gUserParams[mtrNum].ctrlWaitTime[CTRL_State_OffLine]) {
        gMotorVars[mtrNum].Flag_enableOffsetcalc = false;
        gOffsetCalcCount[mtrNum] = 0;
        for (cnt = 0; cnt < 3; cnt++) {
            gOffsets_I_pu[mtrNum].value[cnt] =
                FILTER_FO_get_y1(filterHandle[mtrNum][cnt]);
            gOffsets_V_pu[mtrNum].value[cnt] =
                FILTER_FO_get_y1(filterHandle[mtrNum][cnt + 3]);
            FILTER_FO_setInitialConditions(filterHandle[mtrNum][cnt],
                                           _IQ(0.0), _IQ(0.0));
            FILTER_FO_setInitialConditions(filterHandle[mtrNum][cnt + 3],
                                           _IQ(0.0), _IQ(0.0));
        }
    }
}


// ============================================================
// Global variable update
// ============================================================
void updateGlobalVariables(const uint_least8_t mtrNum)
{
    gMotorVars[mtrNum].Speed_krpm =
        _IQmpy(STPOSCONV_getVelocityFiltered(st_obj[mtrNum].posConvHandle),
               gSpeed_pu_to_krpm_sf[mtrNum]);

    gMotorVars[mtrNum].VdcBus_kV =
        _IQmpy(gAdcData[mtrNum].dcBus,
               _IQ(gUserParams[mtrNum].iqFullScaleVoltage_V / 1000.0));

    gMotorVars[mtrNum].Vd = gVdq_out_pu[mtrNum].value[0];
    gMotorVars[mtrNum].Vq = gVdq_out_pu[mtrNum].value[1];

    gMotorVars[mtrNum].Vs =
        _IQsqrt(_IQmpy(gMotorVars[mtrNum].Vd, gMotorVars[mtrNum].Vd) +
                _IQmpy(gMotorVars[mtrNum].Vq, gMotorVars[mtrNum].Vq));

    gMotorVars[mtrNum].Id_A =
        _IQmpy(gIdq_pu[mtrNum].value[0],
               _IQ(gUserParams[mtrNum].iqFullScaleCurrent_A));
    gMotorVars[mtrNum].Iq_A =
        _IQmpy(gIdq_pu[mtrNum].value[1],
               _IQ(gUserParams[mtrNum].iqFullScaleCurrent_A));

    gMotorVars[mtrNum].Is_A =
        _IQsqrt(_IQmpy(gMotorVars[mtrNum].Id_A, gMotorVars[mtrNum].Id_A) +
                _IQmpy(gMotorVars[mtrNum].Iq_A, gMotorVars[mtrNum].Iq_A));

    gMotorVars[mtrNum].Torque_Nm = UTILS_computeTorque_Nm(
        estHandle[mtrNum], gIdq_pu[mtrNum],
        gTorque_Flux_Iq_pu_to_Nm_sf[mtrNum],
        gTorque_Ls_Id_Iq_pu_to_Nm_sf[mtrNum]);

    gMotorVars[mtrNum].SpinTAC.PosConvErrorID =
        STPOSCONV_getErrorID(st_obj[mtrNum].posConvHandle);
}


// ============================================================
// Error checking
// ============================================================
void checkErrors(void)
{
    gErrors.bit.qep_error =
        (checkEncoderError(gQepIndexWatchdog[0]) ||
         checkEncoderError(gQepIndexWatchdog[1]));

    gErrors.bit.posconv_error =
        ((STPOSCONV_getErrorID(st_obj[HAL_MTR1].posConvHandle) != 0) ||
         (STPOSCONV_getErrorID(st_obj[HAL_MTR2].posConvHandle) != 0));

    gErrors.bit.pos_rollover =
        gFlag_enablePosRolloverError &&
        ((STPOSCONV_getPositionRollOver(
              st_obj[HAL_MTR1].posConvHandle) != 0) ||
         (STPOSCONV_getPositionRollOver(
              st_obj[HAL_MTR2].posConvHandle) != 0));
}


// ============================================================
// CAN status message
// ============================================================
inline void setCanStatusMsg(void)
{
    CAN_StatusMsg_t status;
    status.all = 0;
    status.bit.system_enabled = gMotorVars[HAL_MTR1].Flag_enableSys;
    status.bit.motor1_enabled = gMotorVars[HAL_MTR1].Flag_Run_Identify;
    status.bit.motor1_ready   = !gMotorVars[HAL_MTR1].Flag_enableAlignment;
    status.bit.motor2_enabled = gMotorVars[HAL_MTR2].Flag_Run_Identify;
    status.bit.motor2_ready   = !gMotorVars[HAL_MTR2].Flag_enableAlignment;
    if (gErrors.bit.qep_error) {
        status.bit.error_code = CAN_ERROR_ENCODER;
    } else if (gErrors.bit.can_error) {
        status.bit.error_code = CAN_ERROR_OTHER;
    } else if (gErrors.bit.can_recv_timeout) {
        status.bit.error_code = CAN_ERROR_CAN_RECV_TIMEOUT;
    } else if (gErrors.bit.posconv_error) {
        status.bit.error_code = CAN_ERROR_POSCONV;
    } else if (gErrors.bit.pos_rollover) {
        status.bit.error_code = CAN_ERROR_POS_ROLLOVER;
    } else {
        status.bit.error_code = CAN_ERROR_NO_ERROR;
    }
    CAN_setStatusMsg(status);
}

void maybeSendCanStatusMsg(void)
{
    if (gCanLastStatusMsgTime <
        (gTimer0_stamp - TIMER0_FREQ_Hz / CAN_STATUSMSG_TRANS_FREQ_Hz)) {
        setCanStatusMsg();
        CAN_send(CAN_MBOX_OUT_STATUSMSG);
        gCanLastStatusMsgTime = gTimer0_stamp;
    }
}


// ============================================================
// QEP index ISRs
// ============================================================
interrupt void qep1IndexISR() { genericQepIndexISR(HAL_MTR1); }
interrupt void qep2IndexISR() { genericQepIndexISR(HAL_MTR2); }

inline void genericQepIndexISR(const HAL_MtrSelect_e mtrNum)
{
    HAL_Obj *obj = (HAL_Obj *)halHandle;
    HAL_Obj_mtr *halMtrObj = (HAL_Obj_mtr *)halHandleMtr[mtrNum];

    uint32_t index_posn = QEP_read_posn_index_latch(halMtrObj->qepHandle);

    if (gEnabledCanMessages & CAN_MBOX_OUT_ENC_INDEX) {
        _iq index_pos_mrev =
            STPOSCONV_getPosition_mrev(st_obj[mtrNum].posConvHandle);
        CAN_setEncoderIndex(mtrNum, index_pos_mrev);
        CAN_send(CAN_MBOX_OUT_ENC_INDEX);
    }

    if (gQepIndexWatchdog[mtrNum].isInitialized) {
        gQepIndexWatchdog[mtrNum].indexError_counts =
            index_posn - gQepIndexWatchdog[mtrNum].indexPosition_counts;
    } else {
        gQepIndexWatchdog[mtrNum].isInitialized = true;
        gQepIndexWatchdog[mtrNum].indexPosition_counts = index_posn;
    }

    QEP_clear_all_interrupt_flags(halMtrObj->qepHandle);
    PIE_clearInt(obj->pieHandle, PIE_GroupNumber_5);
}


// ============================================================
// LED status
// ============================================================
void LED_run(HAL_Handle halHandle)
{
    if (gMotorVars[0].Flag_enableSys) {
        if (gMotorVars[0].Flag_Run_Identify ||
            gMotorVars[1].Flag_Run_Identify) {
            uint32_t blink_duration = TIMER0_FREQ_Hz / LED_BLINK_FREQ_Hz;
            if ((gMotorVars[0].Flag_Run_Identify &&
                 gMotorVars[0].Flag_enableAlignment) ||
                (gMotorVars[1].Flag_Run_Identify &&
                 gMotorVars[1].Flag_enableAlignment)) {
                blink_duration /= 4;
            }
            if (gStatusLedBlinkLastToggleTime <
                (gTimer0_stamp - blink_duration)) {
                HAL_toggleLed(halHandle, LED_ONBOARD_BLUE);
                HAL_toggleLed(halHandle, LED_EXTERN_GREEN);
                gStatusLedBlinkLastToggleTime = gTimer0_stamp;
            }
        } else {
            HAL_turnLedOn(halHandle, LED_ONBOARD_BLUE);
            HAL_turnLedOn(halHandle, LED_EXTERN_GREEN);
        }
    } else {
        HAL_turnLedOff(halHandle, LED_ONBOARD_BLUE);
        HAL_turnLedOff(halHandle, LED_EXTERN_GREEN);
    }

    if (gCanAbortingMessages) {
        HAL_turnLedOn(halHandle, LED_EXTERN_YELLOW);
    } else {
        HAL_turnLedOff(halHandle, LED_EXTERN_YELLOW);
    }

    if (gErrors.all) {
        HAL_turnLedOn(halHandle, LED_ONBOARD_RED);
        HAL_turnLedOn(halHandle, LED_EXTERN_RED);
    } else {
        HAL_turnLedOff(halHandle, LED_ONBOARD_RED);
        HAL_turnLedOff(halHandle, LED_EXTERN_RED);
    }
}

//@} //defgroup
// end of file

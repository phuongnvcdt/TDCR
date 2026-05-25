/**
 * \file ctrl_pid.h
 * \brief Simple discrete PID controller using TI IQ-math.
 *
 * Implements a standard position-form PID with anti-windup (output clamping).
 * Uses _iq fixed-point arithmetic so it compiles cleanly with TI MotorWare.
 *
 * Usage
 * -----
 *   CTRL_PID_Obj pid;
 *   CTRL_PID_init(&pid, Kp, Ki, Kd, outMin, outMax, dt_s);
 *   _iq output = CTRL_PID_run(&pid, setpoint, feedback);
 *   CTRL_PID_reset(&pid);        // reset integrator / derivative state
 */

#ifndef SRC_CTRL_PID_H_
#define SRC_CTRL_PID_H_

#include "sw/modules/iqmath/src/32b/IQmathLib.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif


// ============================================================
// Object
// ============================================================

typedef struct _CTRL_PID_Obj_
{
    // Gains (IQ24)
    _iq Kp;
    _iq Ki;          //!< Ki already multiplied by dt: Ki_raw * dt
    _iq Kd;          //!< Kd already divided  by dt: Kd_raw / dt

    // Output limits
    _iq outMin;
    _iq outMax;

    // State
    _iq integrator;  //!< Accumulated integral term
    _iq prevError;   //!< Previous error for derivative

    bool  firstRun;  //!< Suppress derivative spike on first call
} CTRL_PID_Obj;

typedef CTRL_PID_Obj *CTRL_PID_Handle;


// ============================================================
// Functions
// ============================================================

/**
 * \brief Initialise a PID object.
 *
 * \param obj      Pointer to the PID object.
 * \param Kp       Proportional gain  (IQ24)
 * \param Ki_raw   Integral gain      (IQ24) — will be multiplied by dt internally
 * \param Kd_raw   Derivative gain    (IQ24) — will be divided   by dt internally
 * \param outMin   Lower output limit (IQ24)
 * \param outMax   Upper output limit (IQ24)
 * \param dt_iq    Sample period in seconds (IQ24)
 */
static inline void CTRL_PID_init(CTRL_PID_Obj *obj,
                                 _iq Kp, _iq Ki_raw, _iq Kd_raw,
                                 _iq outMin, _iq outMax,
                                 _iq dt_iq)
{
    obj->Kp       = Kp;
    obj->Ki       = _IQmpy(Ki_raw, dt_iq);   // pre-multiply
    obj->Kd       = (dt_iq != 0) ? _IQdiv(Kd_raw, dt_iq) : _IQ(0.0);
    obj->outMin   = outMin;
    obj->outMax   = outMax;
    obj->integrator = _IQ(0.0);
    obj->prevError  = _IQ(0.0);
    obj->firstRun   = true;
}

/**
 * \brief Reset integrator and derivative state (e.g. after mode switch).
 */
static inline void CTRL_PID_reset(CTRL_PID_Obj *obj)
{
    obj->integrator = _IQ(0.0);
    obj->prevError  = _IQ(0.0);
    obj->firstRun   = true;
}

/**
 * \brief Set output clamp limits at runtime (e.g. to reduce max current).
 */
static inline void CTRL_PID_setLimits(CTRL_PID_Obj *obj, _iq outMin, _iq outMax)
{
    obj->outMin = outMin;
    obj->outMax = outMax;
}

/**
 * \brief Update gains at runtime (over CAN).
 *
 * \param dt_iq  Must be the same dt used during init so Ki/Kd scaling is correct.
 */
static inline void CTRL_PID_setGains(CTRL_PID_Obj *obj,
                                     _iq Kp, _iq Ki_raw, _iq Kd_raw,
                                     _iq dt_iq)
{
    obj->Kp = Kp;
    obj->Ki = _IQmpy(Ki_raw, dt_iq);
    obj->Kd = (dt_iq != 0) ? _IQdiv(Kd_raw, dt_iq) : _IQ(0.0);
    CTRL_PID_reset(obj);
}

/**
 * \brief Run one PID iteration.
 *
 * \param obj       PID object.
 * \param setpoint  Desired value (IQ24).
 * \param feedback  Measured value (IQ24).
 * \returns         Control output, clamped to [outMin, outMax] (IQ24).
 */
static inline _iq CTRL_PID_run(CTRL_PID_Obj *obj, _iq setpoint, _iq feedback)
{
    _iq error = setpoint - feedback;

    // --- Proportional ---
    _iq Pterm = _IQmpy(obj->Kp, error);

    // --- Integral with clamping anti-windup ---
    _iq Iterm = obj->integrator + _IQmpy(obj->Ki, error);
    // Clamp integrator alone so it does not wind up past output limits
    if      (Iterm > obj->outMax) Iterm = obj->outMax;
    else if (Iterm < obj->outMin) Iterm = obj->outMin;
    obj->integrator = Iterm;

    // --- Derivative (on error, suppress first-run spike) ---
    _iq Dterm;
    if (obj->firstRun) {
        Dterm = _IQ(0.0);
        obj->firstRun = false;
    } else {
        Dterm = _IQmpy(obj->Kd, error - obj->prevError);
    }
    obj->prevError = error;

    // --- Sum and clamp output ---
    _iq output = Pterm + Iterm + Dterm;
    if      (output > obj->outMax) output = obj->outMax;
    else if (output < obj->outMin) output = obj->outMin;

    return output;
}


#ifdef __cplusplus
}
#endif

#endif /* SRC_CTRL_PID_H_ */

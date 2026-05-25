// BSD 3-Clause License
// Copyright (c) 2019, Max Planck Gesellschaft, New York University
// Copyright (c) 2024, Extended for velocity/position control
// All rights reserved.
// (License text unchanged — see original udriver_firmware)

/**
 * \brief CAN API — extended for velocity and position control modes.
 *
 * Added vs. original:
 *   - CAN_MBOX_IN_VelPosRef  (mailbox 2): receive velocity or position
 *     reference for both motors.
 *   - CAN_CMD_SET_CTRL_MODE_MTR1/2 (cmd 40/41): switch control mode.
 *   - CAN_CMD_SET_VEL_PID_MTRx / CAN_CMD_SET_POS_PID_MTRx: tune PID gains
 *     over CAN without reflashing.
 *
 * Control modes
 * -------------
 *   CTRL_MODE_TORQUE   (0) — original behaviour, IqRef comes from CAN mailbox 1
 *   CTRL_MODE_VELOCITY (1) — velocity PID, SpeedRef from mailbox 2
 *   CTRL_MODE_POSITION (2) — position PID outer + velocity PID inner,
 *                            PosRef from mailbox 2
 *
 * Mailbox 2 layout (IN_VelPosRef):
 *   MDL = reference for motor 1  (_iq, krpm or mrev)
 *   MDH = reference for motor 2  (_iq, krpm or mrev)
 *
 * \author Extended by Phuong / continuum robotics project
 */

#ifndef SRC_CANAPI_H_
#define SRC_CANAPI_H_

#include "sw/drivers/can/src/32b/f28x/f2806x/can.h"
#include "hal_2mtr.h"

#ifdef __cplusplus
extern "C" {
#endif


// ============================================================
// Mailbox bitmasks
// ============================================================
#define CAN_MBOX_OUT_STATUSMSG  (uint32_t)(1 << 15)
#define CAN_MBOX_OUT_Iq         (uint32_t)(1 << 14)
#define CAN_MBOX_OUT_ENC_POS    (uint32_t)(1 << 13)
#define CAN_MBOX_OUT_SPEED      (uint32_t)(1 << 12)
#define CAN_MBOX_OUT_ADC6       (uint32_t)(1 << 11)
#define CAN_MBOX_OUT_ENC_INDEX  (uint32_t)(1 << 10)
#define CAN_MBOX_IN_COMMANDS    (uint32_t)(1 << 0)
#define CAN_MBOX_IN_IqRef       (uint32_t)(1 << 1)
//! NEW: velocity / position reference input (mailbox 2)
#define CAN_MBOX_IN_VelPosRef   (uint32_t)(1 << 2)

#define CAN_MBOX_ALL  (CAN_MBOX_OUT_STATUSMSG  \
    | CAN_MBOX_OUT_Iq       \
    | CAN_MBOX_OUT_ENC_POS  \
    | CAN_MBOX_OUT_SPEED    \
    | CAN_MBOX_OUT_ADC6     \
    | CAN_MBOX_OUT_ENC_INDEX\
    | CAN_MBOX_IN_COMMANDS  \
    | CAN_MBOX_IN_IqRef     \
    | CAN_MBOX_IN_VelPosRef)


// ============================================================
// Arbitration IDs
// ============================================================
#define CAN_ID_COMMANDS     0x00
#define CAN_ID_IqRef        0x05
#define CAN_ID_VelPosRef    0x06   // NEW
#define CAN_ID_STATUSMSG    0x10
#define CAN_ID_Iq           0x20
#define CAN_ID_POS          0x30
#define CAN_ID_SPEED        0x40
#define CAN_ID_ADC6         0x50
#define CAN_ID_ENC_INDEX    0x60


// ============================================================
// Command IDs (sent in MDH of COMMANDS mailbox)
// ============================================================
// --- original ---
#define CAN_CMD_ENABLE_SYS              1
#define CAN_CMD_ENABLE_MTR1             2
#define CAN_CMD_ENABLE_MTR2             3
#define CAN_CMD_ENABLE_VSPRING1         4
#define CAN_CMD_ENABLE_VSPRING2         5
#define CAN_CMD_SEND_CURRENT            12
#define CAN_CMD_SEND_POSITION           13
#define CAN_CMD_SEND_VELOCITY           14
#define CAN_CMD_SEND_ADC6               15
#define CAN_CMD_SEND_ENC_INDEX          16
#define CAN_CMD_SEND_ALL                20
#define CAN_CMD_SET_CAN_RECV_TIMEOUT    30
#define CAN_CMD_ENABLE_POS_ROLLOVER_ERROR 31

// --- NEW: control mode ---
//! Set control mode for motor 1.  value = CTRL_MODE_TORQUE/VELOCITY/POSITION
#define CAN_CMD_SET_CTRL_MODE_MTR1      40
//! Set control mode for motor 2.  value = CTRL_MODE_TORQUE/VELOCITY/POSITION
#define CAN_CMD_SET_CTRL_MODE_MTR2      41

// --- NEW: PID gain tuning (value encoded as _IQ fixed-point) ---
//! Set velocity PID Kp for motor 1
#define CAN_CMD_SET_VEL_KP_MTR1         50
//! Set velocity PID Ki for motor 1
#define CAN_CMD_SET_VEL_KI_MTR1         51
//! Set velocity PID Kd for motor 1
#define CAN_CMD_SET_VEL_KD_MTR1         52
//! Set velocity PID Kp for motor 2
#define CAN_CMD_SET_VEL_KP_MTR2         53
//! Set velocity PID Ki for motor 2
#define CAN_CMD_SET_VEL_KI_MTR2         54
//! Set velocity PID Kd for motor 2
#define CAN_CMD_SET_VEL_KD_MTR2         55
//! Set position PID Kp for motor 1
#define CAN_CMD_SET_POS_KP_MTR1         60
//! Set position PID Ki for motor 1
#define CAN_CMD_SET_POS_KI_MTR1         61
//! Set position PID Kd for motor 1
#define CAN_CMD_SET_POS_KD_MTR1         62
//! Set position PID Kp for motor 2
#define CAN_CMD_SET_POS_KP_MTR2         63
//! Set position PID Ki for motor 2
#define CAN_CMD_SET_POS_KI_MTR2         64
//! Set position PID Kd for motor 2
#define CAN_CMD_SET_POS_KD_MTR2         65


// ============================================================
// Control mode enum
// ============================================================
typedef enum {
    CTRL_MODE_TORQUE   = 0,  //!< Direct IqRef from CAN mailbox 1 (original)
    CTRL_MODE_VELOCITY = 1,  //!< Velocity PID — SpeedRef from mailbox 2
    CTRL_MODE_POSITION = 2   //!< Position PID outer + velocity inner
} CtrlMode_e;


// ============================================================
// Error codes (status message bits 5-7)
// ============================================================
#define CAN_ERROR_NO_ERROR          0
#define CAN_ERROR_ENCODER           1
#define CAN_ERROR_CAN_RECV_TIMEOUT  2
#define CAN_ERROR_CRIT_TEMP         3
#define CAN_ERROR_POSCONV           4
#define CAN_ERROR_POS_ROLLOVER      5
#define CAN_ERROR_OTHER             7


// ============================================================
// Types
// ============================================================
struct CAN_STATUSMSG_BITS {
    uint16_t system_enabled:1;   // 0
    uint16_t motor1_enabled:1;   // 1
    uint16_t motor1_ready:1;     // 2
    uint16_t motor2_enabled:1;   // 3
    uint16_t motor2_ready:1;     // 4
    uint16_t error_code:3;       // 5-7
    uint16_t rsvd:8;             // 8-15
};

typedef union _CAN_StatusMsg_t_ {
    uint16_t              all;
    struct CAN_STATUSMSG_BITS  bit;
} CAN_StatusMsg_t;

typedef struct _CAN_Command_t_ {
    uint32_t id;
    uint32_t value;
} CAN_Command_t;


// ============================================================
// Function prototypes
// ============================================================
extern void CAN_initECanaGpio(HAL_Handle halHandle);
extern void CAN_initECana(void);
extern void CAN_setupMboxes(void);


// ============================================================
// Inline accessors
// ============================================================

inline void CAN_setStatusMsg(CAN_StatusMsg_t statusmsg)
{
    ECanaMboxes.MBOX15.MDL.byte.BYTE0 = statusmsg.all;
}

inline void CAN_setDataMotor1(_iq current_iq, _iq position, _iq velocity)
{
    ECanaMboxes.MBOX14.MDL.all = current_iq;
    ECanaMboxes.MBOX13.MDL.all = position;
    ECanaMboxes.MBOX12.MDL.all = velocity;
}

inline void CAN_setDataMotor2(_iq current_iq, _iq encoder_position, _iq velocity)
{
    ECanaMboxes.MBOX14.MDH.all = current_iq;
    ECanaMboxes.MBOX13.MDH.all = encoder_position;
    ECanaMboxes.MBOX12.MDH.all = velocity;
}

inline void CAN_setAdcIn6Values(_iq adcin_a6, _iq adcin_b6)
{
    ECanaMboxes.MBOX11.MDL.all = adcin_a6;
    ECanaMboxes.MBOX11.MDH.all = adcin_b6;
}

inline void CAN_setEncoderIndex(uint16_t mtrNum, _iq index_position)
{
    ECanaMboxes.MBOX10.MDH.byte.BYTE4 = mtrNum & 0xFF;
    ECanaMboxes.MBOX10.MDL.all = index_position;
}

inline CAN_Command_t CAN_getCommand(void)
{
    CAN_Command_t cmd;
    cmd.id    = ECanaMboxes.MBOX0.MDH.all;
    cmd.value = ECanaMboxes.MBOX0.MDL.all;
    return cmd;
}

inline _iq CAN_getIqRef(uint16_t mtrNum)
{
    return (mtrNum == HAL_MTR1) ? ECanaMboxes.MBOX1.MDL.all
                                : ECanaMboxes.MBOX1.MDH.all;
}

//! \brief Get velocity or position reference for a motor from mailbox 2.
//! Unit: krpm (velocity mode) or mrev (position mode) — caller interprets.
inline _iq CAN_getVelPosRef(uint16_t mtrNum)
{
    return (mtrNum == HAL_MTR1) ? ECanaMboxes.MBOX2.MDL.all
                                : ECanaMboxes.MBOX2.MDH.all;
}

inline void CAN_send(uint32_t mailboxes)
{
    ECanaRegs.CANTRS.all |= mailboxes;
}

inline void CAN_abort(uint32_t mailboxes)
{
    ECanaRegs.CANTRR.all |= mailboxes;
}

inline bool CAN_checkReceivedMessagePending(uint32_t mailbox_mask)
{
    return ECanaRegs.CANRMP.all & mailbox_mask;
}

inline void CAN_clearReceivedMessagePending(uint32_t mailbox_mask)
{
    ECanaRegs.CANRMP.all = mailbox_mask;
}

inline bool CAN_checkAndClearRMP(uint32_t mailbox_mask)
{
    if (CAN_checkReceivedMessagePending(mailbox_mask)) {
        CAN_clearReceivedMessagePending(mailbox_mask);
        return true;
    }
    return false;
}

inline bool CAN_checkTransmissionPending(uint32_t mailbox_mask)
{
    return ECanaRegs.CANTRS.all & mailbox_mask;
}


#ifdef __cplusplus
}
#endif

#endif /* SRC_CANAPI_H_ */

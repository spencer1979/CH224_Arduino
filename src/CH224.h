/**
 * @file    CH224.h
 * @brief   CH224 test program with dynamic I2C pin configuration.
 * @author  Spencer Chen
 * @version V1.0.0
 * @date    2025/01/02
 *
 * @details
 * This file defines the I2C communication protocol, register definitions, PD protocol data structures (PDO, APDO, etc.), and related function prototypes for the CH224 series (such as CH224A/CH224Q) USB PD Sink chips.
 * It supports dynamic I2C pin configuration, PD Source Capability parsing, and advanced PD features detection and request (PPS/AVS/EPR).
 *
 * Main contents include:
 * - CH224 register definitions and status bit descriptions
 * - USB PD protocol structures (Message Header, PDO, APDO, etc.)
 * - CH224 data structure and global variable declarations
 * - I2C access function prototypes
 * - CH224 operation function prototypes (initialization, capability parsing, voltage/current request, etc.)
 *
 * @note
 * This file can be used with Arduino Wire.h/I2C driver and is suitable for embedded platform development and testing of CH224 PD Sink.
 *
 * @copyright
 * Copyright (c) 2025 Spencer Chen. All rights reserved.
 */

#ifndef CH224_H_
#define CH224_H_

#include <Wire.h>
#include <stdint.h> 

#define CH224_DEBUG 0 ///< Set to 1 for debug messages, 0 for no debug messages

#define CH224_I2C_ADDRESS      0x23

#define CH224_STATUS_BC_ACT      (1 << 0)
#define CH224_STATUS_QC2_ACT     (1 << 1)
#define CH224_STATUS_QC3_ACT     (1 << 2)
#define CH224_STATUS_PD_ACT      (1 << 3)
#define CH224_STATUS_EPR_ACT     (1 << 4)
#define CH224_STATUS_EPR_EXIST   (1 << 5)
#define CH224_STATUS_AVS_EXIST   (1 << 6)
#define CH224_STATUS_RESERVED    (1 << 7)

#define CH224_REG_STATUS      0x09 ///< Protocol status register

#define CH224_REG_VOL_CFG 0x0A ///< Voltage control register

#define CH224_VOL_5V 0
#define CH224_VOL_9V 1
#define CH224_VOL_12V 2
#define CH224_VOL_15V 3
#define CH224_VOL_20V 4
#define CH224_VOL_28V 5
#define CH224_MODE_PPS 6 ///< PPS mode (only for CH224Q)
#define CH224_MODE_AVS 7 ///< AVS mode (only for CH224Q)

#define CH224_REG_CURRENT_STATUS  0x0B ///< Current status register (read-only)
#define CH224_READ_CUR_STEP 50  ///< Current step unit: 50mA

#define CH224_REG_AVS_CFG_H    0x51  ///< AVS voltage config high byte
#define CH224_REG_AVS_CFG_L    0x52  ///< AVS voltage config low byte
#define CH224_READ_AVS_CFG_STEP 25  ///< AVS config step: 25mV

#define CH224_REG_PPS_CFG      0x53  ///< PPS voltage config register (write-only, unit: 100mV)
#define CH224_PPS_CFG_MAX    20.0  ///< PPS voltage config max value (20V)
#define CH224_PPS_CFG_MIN    3.3   ///< PPS voltage config min value (3.3V)
#define CH224_PPS_CFG_STEP   0.1   ///< PPS voltage config step (100mV)

#define PD_FIXED_SUPPLY 0x01 ///< Fixed Supply PDO
#define PD_PPS_SUPPLY 0x02   ///< Programmable Power Supply PDO
#define PD_AVS_SUPPLY 0x03   ///< Adjustable Voltage Supply PDO

#define CH224_REG_PD_SRCCAP_START 0x60  ///< PD SRCCAP data register start address
#define CH224_REG_PD_SRCCAP_END   0x8F  ///< PD SRCCAP data register end address
#define CH224_REG_PD_SRCCAP_LEN   (CH224_REG_PD_SRCCAP_END - CH224_REG_PD_SRCCAP_START + 1) ///< Data length

#define MAX_PDO_COUNT 7 ///< Maximum number of PDOs supported

/**
 * @brief I2C status register (0x09) bit definition.
 */
typedef union {
    struct {
        uint8_t BC_ACT : 1;      ///< Bit 0: BC activation
        uint8_t QC2_ACT : 1;     ///< Bit 1: QC2 activation
        uint8_t QC3_ACT : 1;     ///< Bit 2: QC3 activation
        uint8_t PD_ACT : 1;      ///< Bit 3: PD activation
        uint8_t EPR_ACT : 1;     ///< Bit 4: EPR activation
        uint8_t EPR_EXIST : 1;   ///< Bit 5: EPR exist
        uint8_t AVR_EXIST : 1;   ///< Bit 6: AVS exist
        uint8_t REV : 1;         ///< Bit 7: Reserved
    };
    uint8_t Data;
} I2C_status_t;

/**
 * @brief CH224 main data structure.
 */
typedef struct {
    I2C_status_t I2C_status; ///< I2C status
    uint8_t vol_status;      ///< Voltage status
    uint8_t current_status;  ///< Current status
    uint8_t AVS_H;           ///< AVS high byte
    uint8_t AVS_L;           ///< AVS low byte
    uint8_t PPS_ctrl;        ///< PPS control
    uint8_t PD_data[48];     ///< PD data buffer
} CH224_t;

// Message Header
/**
 * @brief USB Power Delivery (PD) Message Header definition.
 *
 * bit15    bit14..bit12   bit11..bit9   bit8     bit7..bit6   bit5     bit4..bit0
 *   ↓           ↓             ↓          ↓          ↓          ↓           ↓
 *  Ext     NumDO(3)      MsgID(3)      PRRole     SpecRev(2)  PDRole   MsgType(5)
 *
 * - Ext (bit 15): Extended Message Indicator (1 bit)
 *      - 0: Not an extended message
 *      - 1: Extended message
 * - NumDO (bits 12-14): Number of Data Objects (3 bits)
 *      - Indicates the number of 32-bit Data Objects following the header (0-7).
 * - MsgID (bits 9-11: Message ID (3 bits)
 *      - Used for message identification and sequencing.
 * - PRRole (bit 8): Port Power Role (1 bit)
 *      - 0: Sink
 *      - 1: Source
 * - SpecRev (bits 6-7): Specification Revision (2 bits)
 *      - 00: USB PD Revision 1.0
 *      - 01: USB PD Revision 2.0
 *      - 10: USB PD Revision 3.0
 *      - 11: Reserved
 * - PDRole (bit 5): Power/Data Role (1 bit)
 *      - 0: UFP (Upstream Facing Port)
 *      - 1: DFP (Downstream Facing Port)
 * - MsgType (bits 0-4): Message Type (5 bits)
 *      - Specifies the type of message (e.g., Control, Data, Extended).
 *
 * The union allows access to the header as a whole (Data) or by individual fields (bits).
 */
typedef union {
    struct {
        uint16_t MsgType : 5;   // bits 0~4
        uint16_t PDRole : 1;    // bit 5
        uint16_t SpecRev : 2;   // bits 6~7
        uint16_t PRRole : 1;    // bit 8
        uint16_t MsgID : 3;     // bits 9~11
        uint16_t NumDO : 3;     // bits 12~14
        uint16_t Ext : 1;       // bit 15
    };
    uint16_t Data;
} PD_Message_Header_t;

// Extended Message Header
// USB PD Extended Message Header definition (see USB PD Spec 3.0 Table 6-29)
/**
 * @brief USB PD Extended Message Header
 *
 * 位元分佈（MSB→LSB）:
 *  15   14..11   10    9    8........0
 * +---+-------+----+----+-----------+
 * |Ch |Chunk# |Req |Rev | DataSize  |
 * +---+-------+----+----+-----------+
 *  |      |     |    |        |
 *  |      |     |    |        +-- [8:0]   資料長度 (9 bits)
 *  |      |     |    +----------- [9]     保留 (1 bit)
 *  |      |     +---------------- [10]    請求分塊 (1 bit)
 *  |      +---------------------- [14:11] 分塊編號 (4 bits)
 *  +----------------------------- [15]    分塊訊息 (1 bit)
 */
typedef union {
    struct {
        uint16_t DataSize : 9;
        uint16_t Rev : 1;
        uint16_t RequestChunk : 1;
        uint16_t ChunkNumber : 4;
        uint16_t Chunked : 1;
    } ;
    uint16_t Data;
} PD_Message_ExtHeader_t;

// Fixed Supply PDO - Source
/**
 * @brief USB Power Delivery (PD) Source Power Data Object (PDO) - Fixed Supply
 *
 * 位元分佈（MSB→LSB）:
 *  31  30 29 28 27 26 25 24 23 22 21 20 19........10 9.........0
 * +---+--+--+--+--+--+--+--+--+--+--+--+------------+----------+
 * |FS |DR|SU|UC|UC|DC|UC|EM| R|PC|      Voltage     |MaxCurrent|
 * +---+--+--+--+--+--+--+--+--+--+--+--+------------+----------+
 *  |   |  |  |  |  |  |  |  |  |  |  |         |           |
 *  |   |  |  |  |  |  |  |  |  |  |  |         |           +-- [9:0]    最大輸出電流 (10 bits, 單位 10mA)
 *  |   |  |  |  |  |  |  |  |  |  |  |         +-------------- [19:10]  輸出電壓 (10 bits, 單位 50mV)
 *  |   |  |  |  |  |  |  |  |  |  +------------------------ [21:20]  峰值電流能力 (2 bits)
 *  |   |  |  |  |  |  |  |  |  +--------------------------- [22]     保留 (1 bit)
 *  |   |  |  |  |  |  +--------------------------------- [23]     是否支援 EPR 模式 (1 bit)
 *  |   |  |  |  |  +------------------------------------ [24]     是否支援 Unchunked Extended Message (1 bit)
 *  |   |  |  |  +--------------------------------------- [25]     是否支援雙角色資料 (1 bit)
 *  |   |  |  +------------------------------------------ [26]     是否支援 USB 通訊 (1 bit)
 *  |   |  +--------------------------------------------- [27]     是否為不受限電源 (1 bit)
 *  |   +------------------------------------------------ [28]     是否支援 USB Suspend (1 bit)
 *  +---------------------------------------------------- [31:30]  固定電源類型 (2 bits)
 */
typedef union {
    struct {
        uint32_t MaxCurrent : 10;
        uint32_t Voltage : 10;
        uint32_t PeakCurrent : 2;
        uint32_t Reserved : 1;
        uint32_t EPRModeCap : 1;
        uint32_t Unchunked : 1;
        uint32_t DualRoleData : 1;
        uint32_t USBComCap : 1;
        uint32_t UnconstrainedPWR : 1;
        uint32_t USBSuspendSupported : 1;
        uint32_t DualRolePWR : 1;
        uint32_t FixedSupply : 2;
    };
    uint32_t Data;
} FixedSupply;

//SPR Programmable Power Supply APDO - Source
/**
 * @brief Represents a USB Power Delivery (USB PD) Programmable Power Supply (PPS) mode data structure.
 *
 * 位元分佈（MSB→LSB）:
 *  31 30 29 28 27 26 25 24 23........17 16........9 8.......1 0
 * +--+--+--+--+--+--+--+--+-------------+----------+--------+-+
 * |AP|AP|PP|PP|PL|R3|R3|MV|   MaxVolt   |MinVolt   |R1|MaxCur|
 * +--+--+--+--+--+--+--+--+-------------+----------+--------+-+
 *  |  |  |  |  |  |  |  |        |          |     |      |
 *  |  |  |  |  |  |  |  |        |          |     +------ [6:0]   最大電流 (MaxCurrent, 7 bits, 50mA units)
 *  |  |  |  |  |  |  |  |        |          +------------ [8:7]   保留 (Reserved1, 1 bit)
 *  |  |  |  |  |  |  |  |        |          |     |      |
 *  |  |  |  |  |  |  |  |        |          |     +------ [6:0]   最大電流 (MaxCurrent, 7 bits, 50mA units)
 *  |  |  |  |  |  |  |  |        |          +------------ [8:7]   保留 (Reserved1, 1 bit)
 *  |  |  |  |  |  |  |  |        |          +------------------------ [16:9] 最小電壓 (MinVoltage, 8 bits, 20mV units)
 *  |  |  |  |  |  |  |  |        |          |     |      |
 *  |  |  |  |  |  |  |  |        |          |     +------ [6:0]   最大電流 (MaxCurrent, 7 bits, 50mA units)
 *  |  |  |  |  |  |  |  |        |          +------------ [8:7]   保留 (Reserved1, 1 bit)
 *  |  |  |  |  |  |  |  |        |          +------------------------ [16:9] 最小電壓 (MinVoltage, 8 bits, 20mV units)
 *  |  |  |  |  |  |  |  |        |          +------------------------ [24:17] 最大電壓 (MaxVoltage, 8 bits, 100mV units)
 *  |  |  |  |  |  |  |  |        |          +--------------------------- [25]    PPS 電源受限 (PPSPWRLimited, 1 bit)
 *  |  |  |  |  |  |  |  |        |          +--------------------------- [26]    是否支援雙角色資料 (1 bit)
 *  |  |  |  |  |  |  |  |        |          +--------------------------- [27]    是否支援 USB 通訊 (1 bit)
 *  |  |  |  |  |  |  |  |        |          +--------------------------- [28]    是否支援 USB Suspend (1 bit)
 *  +---------------------------------------------------- [32:31] APDO 類型 (APDO, 2 bits)
 *
 * Fields:
 * - MaxCurrent (7 bits): Maximum current in 50mA units that the PPS source can supply.
 * - Reserved1 (1 bit): Reserved, must be set to zero.
 * - MinVoltage (8 bits): Minimum voltage in 20mV units that the PPS source can supply.
 * - Reserved2 (1 bit): Reserved, must be set to zero.
 * - MaxVoltage (8 bits): Maximum voltage in 100mV units that the PPS source can supply.
 * - Reserved3 (2 bits): Reserved, must be set to zero.
 * - PPSPWRLimited (1 bit): Indicates if the PPS power is limited (1 = limited, 0 = not limited).
 * - PPS (2 bits): PPS mode indicator.
 * - APDO (2 bits): Augmented Power Data Object (APDO) type.
 *
 * @note This structure is specific to USB PD PPS mode and is used for negotiating programmable power supply parameters.
 */
typedef union {
    struct {
        uint32_t MaxCurrent : 7;
        uint32_t Reserved1 : 1;
        uint32_t MinVoltage : 8;
        uint32_t Reserved2 : 1;
        uint32_t MaxVoltage : 8;
        uint32_t Reserved3 : 2;
        uint32_t PPSPWRLimited : 1;
        uint32_t PPS : 2;
        uint32_t APDO : 2;
    } ;
    uint32_t Data;
} PPSupply;

// EPR Adjustable Voltage Supply APDO - Source
/**
 * @brief USB Power Delivery (PD) Source Power Data Object (PDO) - EPR Adjustable Voltage Supply (APDO)
 *
 * 位元分佈（MSB→LSB）:
 *  31..30   29..28   27..26   25..17   16   15..8   7..0
 * +------+--------+--------+---------+----+-------+------+
 * |APDO  |PeakCur | MaxVolt|Reserved |MinV| PDP   |      |
 * +------+--------+--------+---------+----+-------+------+
 *  |         |        |        |      |      |      |
 *  |         |        |        |      |      +------ [7:0]   PDP in 1W increments (8 bits)
 *  |         |        |        |      +------------- [15:8]  Minimum Voltage in 100mV increments (8 bits)
 *  |         |        |        +-------------------- [16]    Reserved, shall be set to zero (1 bit)
 *  |         |        +---------------------------- [25:17]  Maximum Voltage in 100mV increments (9 bits)
 *  |         +------------------------------------- [27:26]  Peak Current (2 bits, see Table 6-15)
 *  +----------------------------------------------- [31:30]  APDO type (2 bits, 11b for EPR AVS)
 *
 * Fields:
 * - PDP (8 bits): Power Data in 1W increments.
 * - MinVoltage (8 bits): Minimum voltage in 100mV increments.
 * - Reserved2 (1 bit): Reserved, shall be set to zero.
 * - MaxVoltage (9 bits): Maximum voltage in 100mV increments.
 * - PeakCurrent (2 bits): Peak current (see USB PD Spec Table 6-15).
 * - EPRAVS (2 bits): 01b for EPR Adjustable Voltage Supply.
 * - APDO (2 bits): Augmented Power Data Object type (11b for AVS).
 *
 * Reference: USB Power Delivery Specification Revision 3.1, Table 6-14.
 */
typedef union {
    struct {
        uint32_t PDP         : 8;  // Power Data Profile
        uint32_t MinVoltage  : 8;  // Minimum Voltage
        uint32_t Reserved2   : 1;  // Reserved
        uint32_t MaxVoltage  : 9;  // Maximum Voltage
        uint32_t PeakCurrent : 2;  // Peak Current
        uint32_t EPRAVS      : 2;  // EPR AVS
        uint32_t APDO        : 2;  // APDO
    };
    uint32_t Data;
} AVSupply;



extern CH224_t CH224;

extern FixedSupply Rx_FixedSupply;
extern PPSupply Rx_PPSupply;
extern AVSupply Rx_AVSupply;

extern PD_Message_Header_t Rx_Header;
extern PD_Message_ExtHeader_t Rx_Ext_Header;

//PD_Msg[10][4] represents up to 10 PDOs, each PDO takes up 4 uint16_t
//PD_Msg[0][0] represents PDO type, 1 represents fixed voltage, 2 represents PPS, 3 represents AVS, 4 represents invalid PDO
//PD_Msg[0][1] represents PDO voltage (fixd mode), minimum voltage (PPS, AVS mode)
//PD_Msg[0][2] represents the maximum voltage of PDO (PPS, AVS mode) and the maximum current (fixd mode)
//PD_Msg[0][3] represents PDO maximum power (AVS mode), maximum current (PPS mode), 0 (fixd mode)
extern uint16_t PD_Msg[10][4];

//----------------------------------------------------------------------------------------------//
//----------------------------------------------------------------------------------------------//
// I2C Read write Function prototypes
static void I2C_WriteByte(uint8_t addr, uint8_t reg, uint8_t data);
static uint8_t I2C_ReadByte(uint8_t addr, uint8_t reg);
static void I2C_SequentialRead(uint8_t addr, uint8_t reg, uint8_t len, uint8_t* buffer);
//CH224 Function prototypes
/**
 * @brief Initialize CH224 with specified I2C pins.
 * @param sda I2C SDA pin
 * @param scl I2C SCL pin
 */
void CH224_Init(uint8_t sda, uint8_t scl);

/**
 * @brief Reset CH224 data structure.
 */
void CH224_DataInit(void);

/**
 * @brief Read PD source capabilities and parse PDOs.
 * @return true if successful, false otherwise.
 */
bool CH224_readSourceCap(void);

/**
 * @brief Check if PD is supported.
 * @return true if supported, false otherwise.
 */
bool CH224_isPDSupported(void);

/**
 * @brief Check if PPS is supported.
 * @return true if supported, false otherwise.
 */
bool CH224_HasPPS(void);

/**
 * @brief Check if AVS is supported.
 * @return true if supported, false otherwise.
 */
bool CH224_HasAVS(void);

/**
 * @brief Check if EPR is supported.
 * @return true if supported, false otherwise.
 */
bool CH224_HasEPR(void);

/**
 * @brief Request AVS voltage.
 * @param vol Voltage in volts
 * @return true if successful, false otherwise.
 */
bool CH224_AVS_Request(float vol);

/**
 * @brief Request PPS voltage.
 * @param vol Voltage in volts
 * @return true if successful, false otherwise.
 */
bool CH224_PPS_Request(float vol);

/**
 * @brief Request fixed voltage.
 * @param vol Voltage in volts (e.g., 5 for 5V)
 * @return true if successful, false otherwise.
 */
bool CH224_Fixed_Request(uint8_t vol);

/**
 * @brief Check if requested PD voltage is valid.
 * @param type PDO type (1=Fixed, 2=PPS, 3=AVS)
 * @param req_mv Requested voltage in mV
 * @return true if valid, false otherwise.
 */
bool CH224_HasValidPdVoltage(uint8_t type, uint16_t req_mv);

/**
 * @brief Get PPS/AVS min or max voltage.
 * @param type PDO type (2=PPS, 3=AVS)
 * @param get_max true for max, false for min
 * @return Voltage in mV
 */
uint16_t CH224_GetPPSAVSLimitVoltage(uint8_t type, bool get_max);

/**
 * @brief Get current fixed voltage.
 * @return Voltage in mV
 */
uint16_t CH224_GetFixedVoltage(void);

/**
 * @brief Get current PPS voltage.
 * @return Voltage in mV
 */
uint16_t CH224_GetPPSVoltage(void);

/**
 * @brief Get current AVS voltage.
 * @return Voltage in mV
 */
uint16_t CH224_GetAVSVoltage(void);

/**
 * @brief Get the number of fixed PDOs.
 * @return Number of fixed PDOs
 */
uint8_t CH224_GetFixedPDO_Count(void);

#endif /* CH224_H_ */
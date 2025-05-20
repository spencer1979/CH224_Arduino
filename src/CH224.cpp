/**
 * @file    CH224.cpp
 * @brief   Implementation of CH224 test program with dynamic I2C pin configuration.
 * @author  WCH-ZJH
 * @modifier Spencer Chen
 * @version V1.0.0
 * @date    2025/01/02
 */

#include "CH224.h"
#include <Arduino.h>

// Debug macro definitions
#if defined(CH224_DEBUG) && (CH224_DEBUG == 1)
#define DEBUG_PRINTLN(x) Serial.println(x)
#define DEBUG_PRINTF(...) Serial.printf(__VA_ARGS__)
#else
#define DEBUG_PRINTLN(x)
#define DEBUG_PRINTF(...)
#endif

CH224_t CH224;

FixedSupply Rx_FixedSupply;
PPSupply Rx_PPSupply;
AVSupply Rx_AVSupply;

PD_Message_Header_t Rx_Header;
PD_Message_ExtHeader_t Rx_Ext_Header;
uint16_t PD_Msg[10][4];

/**
 * @brief Initializes the CH224 device with specified I2C pins.
 * 
 * @param sda The SDA pin for I2C communication.
 * @param scl The SCL pin for I2C communication.
 */
void CH224_Init(uint8_t sda, uint8_t scl) {
    Wire.begin(sda, scl); // ESP32/ESP8266 supports specifying SDA/SCL pins
    CH224_DataInit(); // Initialize CH224 data structure
    DEBUG_PRINTF("I2C Initialized with SDA: %d, SCL: %d\n", sda, scl);
    memset(PD_Msg, 0, sizeof(PD_Msg)); // Initialize PD_Msg array to zero
}

/**
 * @brief Resets the CH224 data structure to its default state.
 */
void CH224_DataInit() {
    CH224.AVS_H = 0;
    CH224.AVS_L = 0;
    for (uint8_t i = 0; i < 48; i++) {
        CH224.PD_data[i] = 0;
    }
    CH224.PPS_ctrl = 0;
    CH224.current_status = 0;
    CH224.I2C_status.Data = 0;
    CH224.vol_status = 0;
}

/**
 * @brief Writes a single byte to a specific register of an I2C device.
 * 
 * @param addr The 7-bit I2C address of the target device.
 * @param reg The register address to write to.
 * @param data The data byte to write.
 */
void I2C_WriteByte(uint8_t addr, uint8_t reg, uint8_t data) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    Wire.write(data);
    Wire.endTransmission();
}

/**
 * @brief Reads a single byte from a specific register of an I2C device.
 * 
 * @param addr The 7-bit I2C address of the target device.
 * @param reg The register address to read from.
 * @return uint8_t The byte read from the register, or 0xFF if an error occurs.
 */
uint8_t I2C_ReadByte(uint8_t addr, uint8_t reg) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom(addr, (uint8_t)1);
    if (Wire.available()) {
        return Wire.read();
    }
    return 0xFF; // Return error value if read fails
}

/**
 * @brief Reads multiple bytes from consecutive registers of an I2C device.
 * 
 * @param addr The 7-bit I2C address of the target device.
 * @param reg The starting register address to read from.
 * @param len The number of bytes to read.
 * @param buffer Pointer to the buffer where the read bytes will be stored.
 */
void I2C_SequentialRead(uint8_t addr, uint8_t reg, uint8_t len, uint8_t* buffer) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom(addr, len);
    for (uint8_t i = 0; i < len; i++) {
        if (Wire.available()) {
            buffer[i] = Wire.read();
        }
    }
}

/**
 * @brief Reads the status registers and PD source capabilities from the CH224 device.
 *        Parses the PDOs and fills the PD_Msg array with available power profiles.
 *        Prints debug information if enabled.
 * @return true if PD source capabilities are successfully read and parsed, false otherwise.
 */
bool CH224_readSourceCap() {
    CH224_DataInit(); // Reset data structure

    // Read I2C status register
    CH224.I2C_status.Data = I2C_ReadByte(CH224_I2C_ADDRESS, CH224_REG_STATUS);
    delay(2);
    if (CH224.I2C_status.PD_ACT != 1)
        return false;

    // Force output to 5V for initialization
    I2C_WriteByte(CH224_I2C_ADDRESS, CH224_REG_VOL_CFG, CH224_VOL_5V);
    delay(2);
    if (I2C_ReadByte(CH224_I2C_ADDRESS, CH224_REG_VOL_CFG) & CH224_VOL_5V) {
        DEBUG_PRINTLN("Configured to 5V");
    } else {
        DEBUG_PRINTLN("Configured to 5V failed");
        // Try to set 5V again
        I2C_WriteByte(CH224_I2C_ADDRESS, CH224_REG_VOL_CFG, CH224_VOL_5V);
        DEBUG_PRINTLN("Configured to 5V again");
        delay(2);
    }
    delay(2);

    DEBUG_PRINTLN("\n/-----------------------------------/");
    DEBUG_PRINTLN("            CH224 Status           ");
    DEBUG_PRINTLN("/-----------------------------------/");

    DEBUG_PRINTF("\nPWR_status(0:disable 1:enable):\n[ BC -> %d | QC2 -> %d | QC3 -> %d | PD -> %d | EPR -> %d ]\n\n",
                 CH224.I2C_status.BC_ACT, CH224.I2C_status.QC2_ACT, CH224.I2C_status.QC3_ACT, CH224.I2C_status.PD_ACT, CH224.I2C_status.EPR_EXIST);
    DEBUG_PRINTF("CH224.I2C_status:      0x09->%02x\n", CH224.I2C_status.Data);

    // Read voltage status register
    CH224.vol_status = I2C_ReadByte(CH224_I2C_ADDRESS, CH224_REG_VOL_CFG);
    delay(2);
    DEBUG_PRINTF("CH224.vol_status:      0x0A->%02x\n", CH224.vol_status);

    // Read current status register
    CH224.current_status = I2C_ReadByte(CH224_I2C_ADDRESS, CH224_REG_CURRENT_STATUS);
    delay(2);
    DEBUG_PRINTF("CH224.current_status:  0x50->%02x\n", CH224.current_status);

    // Read AVS high and low registers
    CH224.AVS_H = I2C_ReadByte(CH224_I2C_ADDRESS, CH224_REG_AVS_CFG_H);
    delay(2);
    DEBUG_PRINTF("CH224.AVS_H:           0x51->%02x\n", CH224.AVS_H);

    CH224.AVS_L = I2C_ReadByte(CH224_I2C_ADDRESS, CH224_REG_AVS_CFG_L);
    delay(2);
    DEBUG_PRINTF("CH224.AVS_L:           0x52->%02x\n", CH224.AVS_L);

    // Read PPS control register
    CH224.PPS_ctrl = I2C_ReadByte(CH224_I2C_ADDRESS, CH224_REG_PPS_CFG);
    delay(2);
    DEBUG_PRINTF("CH224.PPS_ctrl:        0x53->%02x\n", CH224.PPS_ctrl);

    // Read PD Source Capabilities data into CH224.PD_data buffer
    I2C_SequentialRead(CH224_I2C_ADDRESS, CH224_REG_PD_SRCCAP_START, CH224_REG_PD_SRCCAP_LEN, CH224.PD_data);
    delay(2);

    // Print the PD data buffer
    DEBUG_PRINTLN("\n/-----------------------------------/");
    DEBUG_PRINTLN("              Power Data            ");
    DEBUG_PRINTLN("/-----------------------------------/");
    for (uint8_t i = 0; i < 48; i++) {
        DEBUG_PRINTF("0x%02x->%02x\n", 0x60 + i, CH224.PD_data[i]);
    }

    // Parse each 32-bit PDO in PD_data[], assign to PD_Msg according to type
    memcpy(&Rx_Header.Data, &CH224.PD_data[0], 2);
    uint8_t offset = 2;
    if (Rx_Header.Ext == 1) {
        memcpy(&Rx_Ext_Header.Data, &CH224.PD_data[2], 2);
        offset += 2;
    }

    if (Rx_Header.NumDO > 10 || Rx_Header.NumDO == 0) {
        DEBUG_PRINTLN("Data error: NumDO out of range or zero");
        return false;
    }

    for (uint8_t i = 0; i < Rx_Header.NumDO && i < 10; i++) {
        uint32_t pdo = 0;
        memcpy(&pdo, &CH224.PD_data[offset + 4 * i], 4);
        uint8_t type = (pdo >> 30) & 0x03;
        if (type == 0b00) { // Fixed Supply PDO
            Rx_FixedSupply.Data = pdo;
            PD_Msg[i][0] = 1;
            PD_Msg[i][1] = Rx_FixedSupply.Voltage * 50;      // Voltage in mV
            PD_Msg[i][2] = Rx_FixedSupply.MaxCurrent * 10;   // Current in mA
            PD_Msg[i][3] = 0;
        }
        else if (type == 0x03) { // APDO
            // Determine APDO subtype (bits 29-28)
            uint8_t apdo_type = (pdo >> 28) & 0x03;
            if (apdo_type == 0x00) {
                // SPR Programmable Power Supply (PPS)
                PD_Msg[i][0] = 2;
                Rx_PPSupply.Data = pdo;
                PD_Msg[i][1] = Rx_PPSupply.MinVoltage * 100; // Min voltage in mV
                PD_Msg[i][2] = Rx_PPSupply.MaxVoltage * 100; // Max voltage in mV
                PD_Msg[i][3] = Rx_PPSupply.MaxCurrent * 50;  // Max current in mA
            }
            else if (apdo_type == 0x01) {
                // EPR Adjustable Voltage Supply (AVS)
                PD_Msg[i][0] = 3;
                Rx_AVSupply.Data = pdo;
                PD_Msg[i][1] = Rx_AVSupply.MinVoltage * 100; // Min voltage in mV
                PD_Msg[i][2] = Rx_AVSupply.MaxVoltage * 100; // Max voltage in mV
                PD_Msg[i][3] = Rx_AVSupply.PDP;              // Power in W
            }
        }
        else {
            PD_Msg[i][0] = 4; // Reserved/invalid PDO
        }
    }

    // Print parsed power supply profiles
    DEBUG_PRINTLN("\n/-----------------------------------/");
    DEBUG_PRINTLN("             Power Supply            ");
    DEBUG_PRINTLN("/-----------------------------------/");
    for (uint8_t i = 0; i < 10; i++) {
        switch (PD_Msg[i][0]) {
        case 0:
            break;
        case 1:
            DEBUG_PRINTF("[%02d] Fixed    %.2fV   %.2fA\n", i + 1, (float)PD_Msg[i][1] / 1000, (float)PD_Msg[i][2] / 1000);
            break;
        case 2:
            DEBUG_PRINTF("[%02d]  PPS     %.2fV - %.2fV   %.2fA\n", i + 1, (float)PD_Msg[i][1] / 1000, (float)PD_Msg[i][2] / 1000, (float)PD_Msg[i][3] / 1000);
            break;
        case 3:
            DEBUG_PRINTF("[%02d]  AVS     %.2fV - %.2fV   %-3dW\n", i + 1, (float)PD_Msg[i][1] / 1000, (float)PD_Msg[i][2] / 1000, PD_Msg[i][3]);
            break;
        case 4:
            DEBUG_PRINTF("[%02d]  ----------Reserved----------\n", i + 1);
            break;
        }
    }

    // Print all PD_Msg[10][4] raw values
    DEBUG_PRINTLN("\n/-----------------------------------/");
    DEBUG_PRINTLN("           PD_Msg Raw Data           ");
    DEBUG_PRINTLN("/-----------------------------------/");
    for (uint8_t i = 0; i < 10; i++) {
        DEBUG_PRINTF("PD_Msg[%d]: ", i);
        for (uint8_t j = 0; j < 4; j++) {
            DEBUG_PRINTF("%5d ", PD_Msg[i][j]);
        }
        DEBUG_PRINTLN("");
    }
    return true;
}

/**
 * @brief Checks if the CH224 device supports PD protocol.
 * @return true if PD is supported, false otherwise.
 */
bool CH224_isPDSupported(void) {
    uint8_t read_val = I2C_ReadByte(CH224_I2C_ADDRESS, CH224_REG_STATUS);
    if (read_val & CH224_STATUS_PD_ACT) {
        DEBUG_PRINTLN("CH224 supported !");
        return true;
    }
    DEBUG_PRINTLN("CH224 not supported !");
    return false;
}

/**
 * @brief Checks if the requested voltage is supported by the PD_Msg array.
 * 
 * @param type The type of power supply (1=Fixed, 2=PPS, 3=AVS).
 * @param req_mv The requested voltage in millivolts (e.g., 5000 for 5V).
 * @return true if the requested voltage is supported, false otherwise.
 */
bool CH224_HasValidPdVoltage(uint8_t type, uint16_t req_mv) {
    for (uint8_t i = 0; i < 10; i++) {
        // type: 1=Fixed, 2=PPS, 3=AVS
        if (PD_Msg[i][0] == PD_FIXED_SUPPLY) {
            if (PD_Msg[i][1] == req_mv) {
                DEBUG_PRINTF("Found Fixed voltage: %dmV at index %d\n", req_mv, i);
                return true;
            }
        } 
        else if (PD_Msg[i][0] == PD_AVS_SUPPLY || PD_Msg[i][0] == PD_PPS_SUPPLY) {
            if (PD_Msg[i][1] <= req_mv && req_mv <= PD_Msg[i][2]) {
                DEBUG_PRINTF("Found %s voltage: %dmV at index %d\n", (type == 2) ? "PPS" : "AVS", req_mv, i);
                return true;
            }
        }
    }
    DEBUG_PRINTF("PD type %d voltage: %dmV not found\n", type, req_mv);
    return false;
}

/**
 * @brief Sends a request to set the fixed voltage mode with the specified voltage.
 *
 * @param vol The desired voltage in volts (e.g., 5 for 5V).
 * @return true if the request was successful, false otherwise.
 */
bool CH224_Fixed_Request(uint8_t vol) {
    if (!CH224_HasValidPdVoltage(PD_FIXED_SUPPLY, vol * 1000)) {
        DEBUG_PRINTF("Requested voltage %dmV not supported.\r\n", vol * 1000);
        return false;
    }

    uint8_t read_val = I2C_ReadByte(CH224_I2C_ADDRESS, CH224_REG_VOL_CFG);
    uint8_t set_val = CH224_VOL_5V;
    bool valid = true;

    switch (vol) {
        case 5:  set_val = CH224_VOL_5V;  break;
        case 9:  set_val = CH224_VOL_9V;  break;
        case 12: set_val = CH224_VOL_12V; break;
        case 15: set_val = CH224_VOL_15V; break;
        case 20: set_val = CH224_VOL_20V; break;
        case 28: set_val = CH224_VOL_28V; break;
        default: valid = false;           break;
    }

    if (!valid) {
        DEBUG_PRINTF("Request error: invalid voltage\r\n");
        return false;
    }

    DEBUG_PRINTF("Read before set (%dV): 0x%02x\n", vol, read_val);
    if (read_val != set_val) {
        delay(2);
        I2C_WriteByte(CH224_I2C_ADDRESS, CH224_REG_VOL_CFG, set_val);
        DEBUG_PRINTF("Request %dV...\r\n", vol);
    } else {
        DEBUG_PRINTF("Already set to %dV...\r\n", vol);
    }
    // Confirm by reading back the voltage register
    uint8_t confirm_val = I2C_ReadByte(CH224_I2C_ADDRESS, CH224_REG_VOL_CFG);
    bool success = (confirm_val == set_val);
    DEBUG_PRINTF("Confirm set voltage: 0x%02x, %s\n", confirm_val, success ? "OK" : "FAIL");
    return success;
}

/**
 * @brief Sends a request to set the AVS (Adjustable Voltage Supply) mode with the specified voltage.
 *
 * @param vol The desired voltage in volts (e.g., 5.0 for 5V).
 * @return true if the request was successful, false otherwise.
 */
bool CH224_AVS_Request(float vol) {
    // Check if the requested voltage is within the valid range for AVS
    if (!CH224_HasAVS()) {
        DEBUG_PRINTLN("AVS not supported !");
        return false;
    }

    uint16_t req_mv = (uint16_t)(vol * 1000);
    if (!CH224_HasValidPdVoltage(PD_AVS_SUPPLY, req_mv)) {
        DEBUG_PRINTF("AVS voltage %.2fV (=%dmV) not supported by PD_Msg[]\n", vol, req_mv);
        return false;
    }

    // Calculate AVS register value (AVS_H: high byte, AVS_L: low byte, value in 25mV steps)
    uint16_t avs_val = req_mv / 25;
    uint8_t avs_h = (avs_val >> 8) & 0xFF;
    uint8_t avs_l = avs_val & 0xFF;

    // Write AVS voltage registers
    I2C_WriteByte(CH224_I2C_ADDRESS, CH224_REG_AVS_CFG_H, avs_h);
    delay(2);
    I2C_WriteByte(CH224_I2C_ADDRESS, CH224_REG_AVS_CFG_L, avs_l);
    delay(2);

    // Switch to AVS mode
    I2C_WriteByte(CH224_I2C_ADDRESS, CH224_REG_VOL_CFG, CH224_MODE_AVS);
    delay(2);

    // Read back AVS_H, AVS_L, and VOL_CFG for confirmation
    uint8_t read_avs_h = I2C_ReadByte(CH224_I2C_ADDRESS, CH224_REG_AVS_CFG_H);
    delay(2);
    uint8_t read_avs_l = I2C_ReadByte(CH224_I2C_ADDRESS, CH224_REG_AVS_CFG_L);
    delay(2);
    uint8_t read_mode = I2C_ReadByte(CH224_I2C_ADDRESS, CH224_REG_VOL_CFG);

    bool success = (read_avs_h == avs_h) && (read_avs_l == avs_l) && (read_mode == CH224_MODE_AVS);

    DEBUG_PRINTF("AVS request: %.2fV (AVS_H=0x%02x, AVS_L=0x%02x), readback H=0x%02x, L=0x%02x, MODE=0x%02x, %s\n",
        vol, avs_h, avs_l, read_avs_h, read_avs_l, read_mode, success ? "OK" : "FAIL");

    return success;
}

/**
 * @brief Sends a request to set the PPS (Programmable Power Supply) mode with the specified voltage.
 *
 * @param vol The desired voltage in volts (e.g., 5.0 for 5V).
 * @return true if the request was successful, false otherwise.
 */
bool CH224_PPS_Request(float vol) {
    // Check if the requested voltage is within the valid range for PPS
    if (!CH224_HasPPS()) {
        DEBUG_PRINTLN("PPS not supported !");
        return false;
    }

    uint16_t req_mv = (uint16_t)(vol * 1000);
    if (!CH224_HasValidPdVoltage(PD_PPS_SUPPLY, req_mv)) {
        DEBUG_PRINTF("PPS voltage %.2fV (=%dmV) not supported by PD_Msg[]\n", vol, req_mv);
        return false;
    }

    // Read current VOL_CFG register to determine if already in PPS mode
    uint8_t vol_cfg_val = I2C_ReadByte(CH224_I2C_ADDRESS, CH224_REG_VOL_CFG);
    uint8_t set_val = (uint8_t)(vol * 10.0f);

    if (vol_cfg_val != CH224_MODE_PPS) {
        // First time applying for PPS: set PPS voltage first, then switch to PPS mode
        I2C_WriteByte(CH224_I2C_ADDRESS, CH224_REG_PPS_CFG, set_val);
        delay(2);
        I2C_WriteByte(CH224_I2C_ADDRESS, CH224_REG_VOL_CFG, CH224_MODE_PPS);
        delay(2);
        DEBUG_PRINTF("First PPS request: Set PPS_CFG=0x%02x, then VOL_CFG=PPS mode\n", set_val);
    } else {
        // Already in PPS mode: just update PPS voltage
        I2C_WriteByte(CH224_I2C_ADDRESS, CH224_REG_PPS_CFG, set_val);
        delay(2);
        DEBUG_PRINTF("PPS mode already set: Update PPS_CFG=0x%02x\n", set_val);
    }

    // Read back and compare for confirmation
    uint8_t pps_cfg_val = I2C_ReadByte(CH224_I2C_ADDRESS, CH224_REG_PPS_CFG);
    DEBUG_PRINTF("PPS_CFG reg readback: 0x%02x (%d)\n", pps_cfg_val, pps_cfg_val);

    bool success = (pps_cfg_val == set_val);
    DEBUG_PRINTF("PPS set %s\n", success ? "OK" : "FAIL");
    return success;
}

/**
 * @brief Gets the minimum or maximum voltage limit for PPS or AVS mode.
 *
 * @param type The type of power supply (2=PPS, 3=AVS).
 * @param get_max If true, get the maximum voltage; otherwise, get the minimum voltage.
 * @return uint16_t The voltage limit in millivolts.
 */
uint16_t CH224_GetPPSAVSLimitVoltage(uint8_t type, bool get_max) {
    if (type != PD_PPS_SUPPLY && type != PD_AVS_SUPPLY) {
        DEBUG_PRINTLN("Invalid type for limit voltage");
        return 0;
    }

    // Check if PD_Msg[i][0] contains valid data (0 means invalid/unused)
    for (uint8_t i = 0; i < 10; i++) {
        if (PD_Msg[i][0] == 0)
            continue; // Skip invalid entries

        if (PD_Msg[i][0] == type) {
            if (get_max) {
                DEBUG_PRINTF("Found %s max voltage: %dmV at index %d\n", (type == PD_PPS_SUPPLY) ? "PPS" : "AVS", PD_Msg[i][2], i);
                return PD_Msg[i][2];
            } else {
                DEBUG_PRINTF("Found %s min voltage: %dmV at index %d\n", (type == PD_PPS_SUPPLY) ? "PPS" : "AVS", PD_Msg[i][1], i);
                return PD_Msg[i][1];
            }
        }
    }
    return 0;
}

/**
 * @brief Gets the fixed voltage setting from the CH224 device.
 *
 * @return uint16_t The fixed voltage in millivolts.
 */
uint16_t CH224_GetFixedVoltage() {   
    uint8_t fixed_vol = I2C_ReadByte(CH224_I2C_ADDRESS, CH224_REG_VOL_CFG);
    uint16_t vol = 0;
    switch (fixed_vol) {
        case CH224_VOL_5V:  vol = 5000;  break;
        case CH224_VOL_9V:  vol = 9000;  break;
        case CH224_VOL_12V: vol = 12000; break;
        case CH224_VOL_15V: vol = 15000; break;
        case CH224_VOL_20V: vol = 20000; break;
        case CH224_VOL_28V: vol = 28000; break;
        default:            vol = 0;     break; // Invalid voltage
    }
    DEBUG_PRINTF("Fixed voltage: %d mV\n", vol);
    return vol;
}

/**
 * @brief Gets the PPS voltage setting from the CH224 device.
 *
 * @return uint16_t The PPS voltage in millivolts.
 */
uint16_t CH224_GetPPSVoltage() {
    uint8_t pps_vol = I2C_ReadByte(CH224_I2C_ADDRESS, CH224_REG_PPS_CFG);
    uint16_t vol = (uint16_t)(pps_vol * 100);
    DEBUG_PRINTF("PPS voltage: %d mV\n", vol);
    return vol;
}

/**
 * @brief Gets the AVS (Adaptive Voltage Scaling) voltage from the CH224 device.
 *
 * @return uint16_t The AVS voltage in millivolts.
 */
uint16_t CH224_GetAVSVoltage() {
    // AVS_H and AVS_L are combined to form the full voltage value (unit: 25mV)
    uint8_t avs_h = I2C_ReadByte(CH224_I2C_ADDRESS, CH224_REG_AVS_CFG_H);
    uint8_t avs_l = I2C_ReadByte(CH224_I2C_ADDRESS, CH224_REG_AVS_CFG_L);
    uint16_t vol = ((uint16_t)avs_h << 8 | avs_l) * 25;
    DEBUG_PRINTF("AVS voltage: %d mV\n", vol);
    return vol;
}

/**
 * @brief Checks if PPS (Programmable Power Supply) is supported by the current PD_Msg array.
 * @return true if PPS is supported, false otherwise.
 */
bool CH224_HasPPS(void) {
    for (uint8_t i = 0; i < 10; i++) {
        if (PD_Msg[i][0] == PD_PPS_SUPPLY) {
            DEBUG_PRINTLN("PPS supported");
            return true;
        }
    }
    DEBUG_PRINTLN("PPS not supported");
    return false;
}

/**
 * @brief Checks if AVS (Adjustable Voltage Supply) is supported by the current PD_Msg array.
 * @return true if AVS is supported, false otherwise.
 */
bool CH224_HasAVS(void) {
    for (uint8_t i = 0; i < 10; i++) {
        if (PD_Msg[i][0] == PD_AVS_SUPPLY) {
            DEBUG_PRINTLN("AVS supported");
            return true;
        }
    }
    DEBUG_PRINTLN("AVS not supported");
    return false;
}

/**
 * @brief Checks if EPR (Extended Power Range) is supported by the current I2C status.
 * @return true if EPR is supported, false otherwise.
 */
bool CH224_HasEPR(void) {
    if (CH224.I2C_status.EPR_EXIST == 1) {
        DEBUG_PRINTLN("EPR supported");
        return true;
    }
    DEBUG_PRINTLN("EPR not supported");
    return false;
}

/**
 * @brief Returns the number of fixed voltage PDOs available in the PD_Msg array.
 * @return Number of fixed PDOs.
 */
uint8_t CH224_GetFixedPDO_Count() {
    uint8_t count = 0;
    for (uint8_t i = 0; i < 10; i++) {
        if (PD_Msg[i][0] == PD_FIXED_SUPPLY) {
            count++;
        }
    }
    DEBUG_PRINTF("Fixed PDO count: %d\n", count);
    return count;
}

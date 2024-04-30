/*
* Copyright Novelda AS 2024.
*/
#pragma once

#define SET_PROGRAMMING_MODE    0x01
#define SET_READBACK_MODE       0x02
#define SET_NORMAL_MODE         0x00
#define FIFO_FROM_MEM_DATA_VALID_FLAG     0x04
#define FIFO_FROM_CPU_DATA_VALID 0x02
#define FIFO_TO_MEM_FIFO_EMPTY  0x02
#define FIFO_TO_MEM_FIFO_FULL   0x01

#define TO_CPU_EMPTY_BIT 2
#define FROM_CPU_VALID_BIT 1

#define SPI_ADDRESS_WRITE 0x80
#define PIF_ADDRESS_WRITE 0x80

#define BIT_SET 1
#define BIT_NOT_SET 0

#define PIF_COMMAND_MAX_RETRIES 200

#define BOOT_FROM_SRAM 0x00
#define START_OF_SRAM_LSB       0x00
#define START_OF_SRAM_MSB       0x00

#define OTP_SAMPLE_ID_LSB       0xFB
#define OTP_SAMPLE_ID_MSB       0xFF

/**
 * In order to solve the ODS problem when doing unity builds, the X4 addresses
 * are protected by a guard to allow multiple definitions. We assume these are
 * identical. Specifically X4Sensor and X4Source both define these.
 * 
 */
#ifndef X4_ADDRESSES_13f01334_f233_49a4_917b_d178058f545e
#define X4_ADDRESSES_13f01334_f233_49a4_917b_d178058f545e
/**
 * Addresses of all the SPI register on X4.
 */
typedef enum {
    ADDR_SPI_FORCE_ZERO_R                      = 0,
    ADDR_SPI_FORCE_ONE_R                       = 1,
    ADDR_SPI_CHIP_ID_DIG_R                     = 2,
    ADDR_SPI_CHIP_ID_SYS_R                     = 3,
    ADDR_SPI_DEBUG_RW                          = 4,
    ADDR_SPI_RADAR_DATA_SPI_RE                 = 5,
    ADDR_SPI_RADAR_DATA_SPI_STATUS_R           = 6,
    ADDR_SPI_SPI_RADAR_DATA_CLEAR_STATUS_WE    = 7,
    ADDR_SPI_SPI_RADAR_DATA0_FIFO_STATUS_R     = 8,
    ADDR_SPI_SPI_RADAR_DATA0_CLEAR_STATUS_WE   = 9,
    ADDR_SPI_SPI_RADAR_DATA1_FIFO_STATUS_R     = 10,
    ADDR_SPI_SPI_RADAR_DATA1_CLEAR_STATUS_WE   = 11,
    ADDR_SPI_RADAR_BIST_CTRL_RW                = 12,
    ADDR_SPI_RADAR_BIST_STATUS_R               = 13,
    ADDR_SPI_FIRMWARE_VERSION_SPI_R            = 14,
    ADDR_SPI_TO_CPU_WRITE_DATA_WE              = 15,
    ADDR_SPI_SPI_MB_FIFO_STATUS_R              = 16,
    ADDR_SPI_FROM_CPU_READ_DATA_RE             = 17,
    ADDR_SPI_SPI_MB_CLEAR_STATUS_WE            = 18,
    ADDR_SPI_TO_MEM_WRITE_DATA_WE              = 19,
    ADDR_SPI_SPI_MEM_FIFO_STATUS_R             = 20,
    ADDR_SPI_FROM_MEM_READ_DATA_RE             = 21,
    ADDR_SPI_SPI_MEM_CLEAR_STATUS_WE           = 22,
    ADDR_SPI_MEM_MODE_RW                       = 23,
    ADDR_SPI_MEM_FIRST_ADDR_MSB_RW             = 24,
    ADDR_SPI_MEM_FIRST_ADDR_LSB_RW             = 25,
    ADDR_SPI_BOOT_FROM_OTP_SPI_RWE             = 26,
    ADDR_SPI_MCU_BIST_CTRL_RW                  = 27,
    ADDR_SPI_MCU_BIST_STATUS_R                 = 28,
    ADDR_SPI_SPI_CONFIG_WE                     = 29,
    ADDR_SPI_CPU_RESET_RW                      = 127,
} xtx4_spi_register_address_t;


/**
 * Addresses of all the PIF register on X4.
 */
typedef enum {
    ADDR_PIF_SP_RW                              = 1,
    ADDR_PIF_DPL_RW                             = 2,
    ADDR_PIF_DPH_RW                             = 3,
    ADDR_PIF_DPL1_RW                            = 4,
    ADDR_PIF_DPH1_RW                            = 5,
    ADDR_PIF_PCON_RW                            = 7,
    ADDR_PIF_TCON_RW                            = 8,
    ADDR_PIF_TMOD_RW                            = 9,
    ADDR_PIF_TL0_RW                             = 10,
    ADDR_PIF_TH0_RW                             = 12,
    ADDR_PIF_DPS_RW                             = 18,
    ADDR_PIF_DPC_RW                             = 19,
    ADDR_PIF_IEN2_RW                            = 26,
    ADDR_PIF_P2_RW                              = 32,
    ADDR_PIF_IEN0_RW                            = 40,
    ADDR_PIF_IP0_RW                             = 41,
    ADDR_PIF_IEN1_RW                            = 56,
    ADDR_PIF_IP1_RW                             = 57,
    ADDR_PIF_IRCON2_RW                          = 63,
    ADDR_PIF_IRCON_RW                           = 64,
    ADDR_PIF_T2CON_RW                           = 72,
    ADDR_PIF_PSW_RW                             = 80,
    ADDR_PIF_ACC_RW                             = 96,
    ADDR_PIF_B_RW                               = 112,
    ADDR_PIF_SRST_RW                            = 119,
    ADDR_PIF_FIRMWARE_VERSION_RW                = 0,
    ADDR_PIF_GPIO_OUT_RW                        = 6,
    ADDR_PIF_GPIO_IN_R                          = 11,
    ADDR_PIF_GPIO_OE_RW                         = 13,
    ADDR_PIF_RX_MFRAMES_RW                      = 14,
    ADDR_PIF_SMPL_MODE_RW                       = 15,
    ADDR_PIF_RX_DOWNCONVERSION_COEFF_I1_WE      = 16,
    ADDR_PIF_RX_DOWNCONVERSION_COEFF_I2_WE      = 17,
    ADDR_PIF_RX_DOWNCONVERSION_COEFF_Q1_WE      = 20,
    ADDR_PIF_RX_DOWNCONVERSION_COEFF_Q2_WE      = 21,
    ADDR_PIF_RX_RAM_WRITE_OFFSET_MSB_RW         = 22,
    ADDR_PIF_RX_RAM_LINE_FIRST_MSB_RW           = 23,
    ADDR_PIF_RX_RAM_LINE_LAST_MSB_RW            = 24,
    ADDR_PIF_RX_RAM_LSBS_RW                     = 25,
    ADDR_PIF_RX_COUNTER_NUM_BYTES_RW            = 27,
    ADDR_PIF_RX_COUNTER_LSB_RW                  = 28,
    ADDR_PIF_FETCH_RADAR_DATA_SPI_W             = 29,
    ADDR_PIF_FETCH_RADAR_DATA_PIF_W             = 30,
    ADDR_PIF_RADAR_DATA_PIF_RE                  = 31,
    ADDR_PIF_RADAR_DATA_PIF_STATUS_R            = 33,
    ADDR_PIF_PIF_RADAR_DATA_CLEAR_STATUS_WE     = 34,
    ADDR_PIF_PIF_RADAR_DATA0_FIFO_STATUS_R      = 35,
    ADDR_PIF_PIF_RADAR_DATA0_CLEAR_STATUS_WE    = 36,
    ADDR_PIF_PIF_RADAR_DATA1_FIFO_STATUS_R      = 37,
    ADDR_PIF_PIF_RADAR_DATA1_CLEAR_STATUS_WE    = 38,
    ADDR_PIF_RAM_SELECT_RW                      = 39,
    ADDR_PIF_RADAR_READOUT_IDLE_R               = 42,
    ADDR_PIF_RX_RESET_COUNTERS_W                = 43,
    ADDR_PIF_TRX_CLOCKS_PER_PULSE_RW            = 44,
    ADDR_PIF_RX_MFRAMES_COARSE_RW               = 45,
    ADDR_PIF_TRX_PULSES_PER_STEP_MSB_RW         = 46,
    ADDR_PIF_TRX_PULSES_PER_STEP_LSB_RW         = 47,
    ADDR_PIF_TRX_DAC_MAX_H_RW                   = 48,
    ADDR_PIF_TRX_DAC_MAX_L_RW                   = 49,
    ADDR_PIF_TRX_DAC_MIN_H_RW                   = 50,
    ADDR_PIF_TRX_DAC_MIN_L_RW                   = 51,
    ADDR_PIF_TRX_DAC_STEP_RW                    = 52,
    ADDR_PIF_TRX_ITERATIONS_RW                  = 53,
    ADDR_PIF_TRX_START_W                        = 54,
    ADDR_PIF_TRX_CTRL_DONE_R                    = 55,
    ADDR_PIF_TRX_BACKEND_DONE_R                 = 58,
    ADDR_PIF_TRX_CTRL_MODE_RW                   = 59,
    ADDR_PIF_TRX_LFSR_TAPS_0_RW                 = 60,
    ADDR_PIF_TRX_LFSR_TAPS_1_RW                 = 61,
    ADDR_PIF_TRX_LFSR_TAPS_2_RW                 = 62,
    ADDR_PIF_TRX_LFSR_RESET_WS                  = 65,
    ADDR_PIF_RX_WAIT_RW                         = 66,
    ADDR_PIF_TX_WAIT_RW                         = 67,
    ADDR_PIF_TRX_DAC_OVERRIDE_H_RW              = 68,
    ADDR_PIF_TRX_DAC_OVERRIDE_L_RW              = 69,
    ADDR_PIF_TRX_DAC_OVERRIDE_LOAD_WS           = 70,
    ADDR_PIF_CPU_SPI_MASTER_CLK_CTRL_RW         = 71,
    ADDR_PIF_MCLK_TRX_BACKEND_CLK_CTRL_RW       = 73,
    ADDR_PIF_OSC_CTRL_RW                        = 74,
    ADDR_PIF_IO_CTRL_1_RW                       = 75,
    ADDR_PIF_IO_CTRL_2_RW                       = 76,
    ADDR_PIF_IO_CTRL_3_RW                       = 77,
    ADDR_PIF_IO_CTRL_4_RW                       = 78,
    ADDR_PIF_IO_CTRL_5_RW                       = 79,
    ADDR_PIF_IO_CTRL_6_RW                       = 81,
    ADDR_PIF_PIF_MB_FIFO_STATUS_R               = 82,
    ADDR_PIF_TO_CPU_READ_DATA_RE                = 83,
    ADDR_PIF_FROM_CPU_WRITE_DATA_WE             = 84,
    ADDR_PIF_PIF_MB_CLEAR_STATUS_WE             = 85,
    ADDR_PIF_PIF_MEM_FIFO_STATUS_R              = 86,
    ADDR_PIF_PIF_MEM_CLEAR_STATUS_WE            = 87,
    ADDR_PIF_SPI_MASTER_SEND_RW                 = 88,
    ADDR_PIF_SPI_MASTER_RADAR_BURST_KICK_W      = 89,
    ADDR_PIF_SPI_MASTER_IDLE_R                  = 90,
    ADDR_PIF_SPI_MASTER_MODE_RW                 = 91,
    ADDR_PIF_SPI_MASTER_RADAR_BURST_SIZE_LSB_RW = 92,
    ADDR_PIF_OTP_CTRL_RW                        = 93,
    ADDR_PIF_BOOT_FROM_OTP_PIF_RWE              = 94,
    ADDR_PIF_RX_PLL_CTRL_1_RW                   = 95,
    ADDR_PIF_RX_PLL_CTRL_2_RW                   = 97,
    ADDR_PIF_RX_PLL_SKEW_CTRL_RW                = 98,
    ADDR_PIF_RX_PLL_SKEWCALIN_RW                = 99,
    ADDR_PIF_RX_PLL_STATUS_R                    = 100,
    ADDR_PIF_TX_PLL_CTRL_1_RW                   = 101,
    ADDR_PIF_TX_PLL_CTRL_2_RW                   = 102,
    ADDR_PIF_TX_PLL_SKEW_CTRL_RW                = 103,
    ADDR_PIF_TX_PLL_SKEWCALIN_RW                = 104,
    ADDR_PIF_TX_PLL_STATUS_R                    = 105,
    ADDR_PIF_COMMON_PLL_CTRL_1_RW               = 106,
    ADDR_PIF_COMMON_PLL_CTRL_2_RW               = 107,
    ADDR_PIF_COMMON_PLL_CTRL_3_RW               = 108,
    ADDR_PIF_COMMON_PLL_CTRL_4_RW               = 109,
    ADDR_PIF_COMMON_PLL_FRAC_2_RW               = 110,
    ADDR_PIF_COMMON_PLL_FRAC_1_RW               = 111,
    ADDR_PIF_COMMON_PLL_FRAC_0_RW               = 113,
    ADDR_PIF_COMMON_PLL_STATUS_R                = 114,
    ADDR_PIF_CLKOUT_SEL_RW                      = 115,
    ADDR_PIF_APC_DVDD_TESTMODE_RW               = 116,
    ADDR_PIF_MISC_CTRL                          = 117,
    ADDR_PIF_DVDD_RX_CTRL_RW                    = 118,
    ADDR_PIF_DVDD_TX_CTRL_RW                    = 120,
    ADDR_PIF_DVDD_TESTMODE_RW                   = 121,
    ADDR_PIF_AVDD_RX_CTRL_RW                    = 122,
    ADDR_PIF_AVDD_TX_CTRL_RW                    = 123,
    ADDR_PIF_AVDD_TESTMODE_RW                   = 124,
    ADDR_PIF_LDO_STATUS_1_R                     = 125,
    ADDR_PIF_LDO_STATUS_2_R                     = 126,
    ADDR_PIF_SPI_CONFIG_PIF_RWE                 = 127,
} xtx4_pif_register_address_t;


/**
 * Addresses of all the XIF register on X4.
 */
typedef enum {
    ADDR_XIF_DEBUG_RW                           = 0,
    ADDR_XIF_SAMPLER_PRESET_MSB_RW              = 1,
    ADDR_XIF_SAMPLER_PRESET_LSB_RW              = 2,
    ADDR_XIF_DAC_TRIM_RW                        = 3,
    ADDR_XIF_PREAMP_TRIM_RW                     = 4,
    ADDR_XIF_RX_FE_ANATESTREQ_RW                = 5,
    ADDR_XIF_LNA_ANATESTREQ_RW                  = 6,
    ADDR_XIF_DAC_ANATESTREQ_RW                  = 7,
    ADDR_XIF_VREF_TRIM_RW                       = 8,
    ADDR_XIF_IREF_TRIM_RW                       = 9,
    ADDR_XIF__APC_TEMP_TRIM_RW                  = 10,
} xtx4_xif_register_address_t;
#endif // X4_ADDRESSES_13f01334_f233_49a4_917b_d178058f545e

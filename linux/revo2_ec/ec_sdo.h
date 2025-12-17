/**
 * @file ec_sdo.h
 * @brief EtherCAT SDO operations
 *
 * This header provides shared SDO read/write functions used across multiple
 * EtherCAT example programs.
 */

#ifndef EC_SDO_H
#define EC_SDO_H

#include "ec_constants.h"
#include "ec_types.h"
#include <ecrt.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/****************************************************************************/
// SDO initialization
/****************************************************************************/

/**
 * @brief Set the EtherCAT master for SDO operations
 * @param master EtherCAT master pointer
 */
void ec_sdo_set_master(ec_master_t *master);

/****************************************************************************/
// SDO read/write functions
/****************************************************************************/

/**
 * @brief Read SDO uint8 value
 * @param slave_config EtherCAT slave configuration
 * @param slave_pos Slave position
 * @param index SDO object index
 * @param subindex SDO subindex
 * @param value Pointer to store the read value
 * @return 0 on success, -1 on failure
 */
int read_sdo_uint8(ec_slave_config_t *slave_config, uint16_t slave_pos,
                   uint16_t index, uint8_t subindex, uint8_t *value);

/**
 * @brief Read SDO uint16 value
 * @param slave_config EtherCAT slave configuration
 * @param slave_pos Slave position
 * @param index SDO object index
 * @param subindex SDO subindex
 * @param value Pointer to store the read value
 * @return 0 on success, -1 on failure
 */
int read_sdo_uint16(ec_slave_config_t *slave_config, uint16_t slave_pos,
                    uint16_t index, uint8_t subindex, uint16_t *value);

/**
 * @brief Read SDO uint32 value
 * @param slave_config EtherCAT slave configuration
 * @param slave_pos Slave position
 * @param index SDO object index
 * @param subindex SDO subindex
 * @param value Pointer to store the read value
 * @return 0 on success, -1 on failure
 */
int read_sdo_uint32(ec_slave_config_t *slave_config, uint16_t slave_pos,
                    uint16_t index, uint8_t subindex, uint32_t *value);

/**
 * @brief Read SDO string value
 * @param slave_config EtherCAT slave configuration
 * @param slave_pos Slave position
 * @param index SDO object index
 * @param subindex SDO subindex
 * @param buffer Buffer to store the string
 * @param buffer_size Size of the buffer
 * @return 0 on success, -1 on failure
 */
int read_sdo_string(ec_slave_config_t *slave_config, uint16_t slave_pos,
                    uint16_t index, uint8_t subindex, char *buffer, size_t buffer_size);

/**
 * @brief Write SDO uint8 value
 * @param slave_config EtherCAT slave configuration
 * @param slave_pos Slave position
 * @param index SDO object index
 * @param subindex SDO subindex
 * @param value Value to write
 * @return 0 on success, -1 on failure
 */
int write_sdo_uint8(ec_slave_config_t *slave_config, uint16_t slave_pos,
                    uint16_t index, uint8_t subindex, uint8_t value);

/**
 * @brief Write SDO uint16 value
 * @param slave_config EtherCAT slave configuration
 * @param slave_pos Slave position
 * @param index SDO object index
 * @param subindex SDO subindex
 * @param value Value to write
 * @return 0 on success, -1 on failure
 */
int write_sdo_uint16(ec_slave_config_t *slave_config, uint16_t slave_pos,
                     uint16_t index, uint8_t subindex, uint16_t value);

/**
 * @brief Write SDO uint32 value
 * @param slave_config EtherCAT slave configuration
 * @param slave_pos Slave position
 * @param index SDO object index
 * @param subindex SDO subindex
 * @param value Value to write
 * @return 0 on success, -1 on failure
 */
int write_sdo_uint32(ec_slave_config_t *slave_config, uint16_t slave_pos,
                     uint16_t index, uint8_t subindex, uint32_t value);

/****************************************************************************/
// Control functions
/****************************************************************************/

/**
 * @brief Set LED control
 * @param slave_config Slave configuration
 * @param slave_pos Slave position
 * @param state Switch state (SWITCH_ON/SWITCH_OFF)
 * @return 0 on success, -1 on failure
 */
int set_led_control(ec_slave_config_t *slave_config, uint16_t slave_pos, uint8_t state);

/**
 * @brief Set buzzer control
 * @param slave_config Slave configuration
 * @param slave_pos Slave position
 * @param state Switch state (SWITCH_ON/SWITCH_OFF)
 * @return 0 on success, -1 on failure
 */
int set_buzzer_control(ec_slave_config_t *slave_config, uint16_t slave_pos, uint8_t state);

/**
 * @brief Set vibrator control
 * @param slave_config Slave configuration
 * @param slave_pos Slave position
 * @param state Switch state (SWITCH_ON/SWITCH_OFF)
 * @return 0 on success, -1 on failure
 */
int set_vibrator_control(ec_slave_config_t *slave_config, uint16_t slave_pos, uint8_t state);

/**
 * @brief Set unit mode
 * @param slave_config Slave configuration
 * @param slave_pos Slave position
 * @param mode Unit mode (NORMAL/PHYSICAL)
 * @return 0 on success, -1 on failure
 */
int set_unit_mode(ec_slave_config_t *slave_config, uint16_t slave_pos, uint8_t mode);

/**
 * @brief Set turbo mode
 * @param slave_config Slave configuration
 * @param slave_pos Slave position
 * @param state Switch state (SWITCH_ON/SWITCH_OFF)
 * @param interval Turbo interval (when state is ON)
 * @param duration Turbo duration (when state is ON)
 * @return 0 on success, -1 on failure
 */
int set_turbo_mode(ec_slave_config_t *slave_config, uint16_t slave_pos, switch_state_t state,
                   uint16_t interval, uint16_t duration);

/**
 * @brief Set pressure touch data type
 * @param slave_config Slave configuration
 * @param slave_pos Slave position
 * @param data_type Data type (RAW/CALIBRATED/FORCE)
 * @return 0 on success, -1 on failure
 */
int set_pressure_touch_data_type(ec_slave_config_t *slave_config, uint16_t slave_pos,
                                 pressure_data_type_t data_type);

/****************************************************************************/
// High-level configuration functions
/****************************************************************************/

/**
 * @brief Read general configuration from device
 * @param slave_config EtherCAT slave configuration
 * @param slave_pos Slave position
 * @param config Pointer to store the configuration
 * @return 0 on success, -1 on failure
 */
int read_general_config(ec_slave_config_t *slave_config, uint16_t slave_pos,
                        general_config_t *config);

/**
 * @brief Write general configuration to device
 * @param slave_config EtherCAT slave configuration
 * @param slave_pos Slave position
 * @param config Configuration to write
 * @return 0 on success, -1 on failure
 */
int write_general_config(ec_slave_config_t *slave_config, uint16_t slave_pos,
                         const general_config_t *config);

/**
 * @brief Read protection current configuration from device
 * @param slave_config EtherCAT slave configuration
 * @param slave_pos Slave position
 * @param config Pointer to store the configuration
 * @return 0 on success, -1 on failure
 */
int read_protection_current_config(ec_slave_config_t *slave_config, uint16_t slave_pos,
                                   protection_current_config_t *config);

/**
 * @brief Write protection current configuration to device
 * @param slave_config EtherCAT slave configuration
 * @param slave_pos Slave position
 * @param config Configuration to write
 * @return 0 on success, -1 on failure
 */
int write_protection_current_config(ec_slave_config_t *slave_config, uint16_t slave_pos,
                                    const protection_current_config_t *config);

/**
 * @brief Read firmware information from device
 * @param slave_config EtherCAT slave configuration
 * @param slave_pos Slave position
 * @param info Pointer to store the firmware information
 * @return 0 on success, -1 on failure
 */
int read_firmware_info(ec_slave_config_t *slave_config, uint16_t slave_pos, firmware_info_t *info);

/**
 * @brief Read touch configuration from device
 * @param slave_config EtherCAT slave configuration
 * @param slave_pos Slave position
 * @param config Pointer to store the touch configuration
 * @return 0 on success, -1 on failure
 */
int read_touch_config(ec_slave_config_t *slave_config, uint16_t slave_pos, touch_config_t *config);

/**
 * @brief Read pressure touch configuration from device
 * @param slave_config EtherCAT slave configuration
 * @param slave_pos Slave position
 * @param config Pointer to store the pressure touch configuration
 * @return 0 on success, -1 on failure
 */
int read_pressure_touch_config(ec_slave_config_t *slave_config, uint16_t slave_pos,
                               pressure_touch_config_t *config);

/**
 * @brief Set auto calibration
 * @param slave_config EtherCAT slave configuration
 * @param slave_pos Slave position
 * @param state Switch state (ON/OFF)
 * @return 0 on success, -1 on failure
 */
int set_auto_calibration(ec_slave_config_t *slave_config, uint16_t slave_pos, switch_state_t state);

/****************************************************************************/
// Demo helper functions
/****************************************************************************/

/**
 * @brief Print firmware information
 * @param slave_config EtherCAT slave configuration
 * @param slave_pos Slave position
 */
void print_firmware_info(ec_slave_config_t *slave_config, uint16_t slave_pos);

/**
 * @brief Print general configuration
 * @param slave_config EtherCAT slave configuration
 * @param slave_pos Slave position
 */
void print_general_config(ec_slave_config_t *slave_config, uint16_t slave_pos);

/**
 * @brief Print protection current configuration
 * @param slave_config EtherCAT slave configuration
 * @param slave_pos Slave position
 */
void print_protection_current_config(ec_slave_config_t *slave_config, uint16_t slave_pos);

/**
 * @brief Print touch configuration
 * @param slave_config EtherCAT slave configuration
 * @param slave_pos Slave position
 */
void print_touch_config(ec_slave_config_t *slave_config, uint16_t slave_pos);

/**
 * @brief Print pressure touch configuration
 * @param slave_config EtherCAT slave configuration
 * @param slave_pos Slave position
 */
void print_pressure_touch_config(ec_slave_config_t *slave_config, uint16_t slave_pos);

/**
 * @brief Demonstrate basic control operations (LED, buzzer, unit mode, turbo)
 * @param slave_config EtherCAT slave configuration
 * @param slave_pos Slave position
 */
void demo_basic_controls(ec_slave_config_t *slave_config, uint16_t slave_pos);

#ifdef __cplusplus
}
#endif

#endif // EC_SDO_H

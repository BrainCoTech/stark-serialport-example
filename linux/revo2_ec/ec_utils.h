/**
 * @file ec_utils.h
 * @brief Common utility functions for EtherCAT applications
 *
 * This header provides utility functions for time management and string
 * conversion.
 */

#ifndef EC_UTILS_H
#define EC_UTILS_H

#include "ec_types.h"
#include <arpa/inet.h> // For ntohs, ntohl
#include <ecrt.h>
#include <stdint.h>
#include <time.h>

#ifdef __cplusplus
extern "C" {
#endif

/****************************************************************************/
// Byte Order Conversion Functions
/****************************************************************************/

/**
 * @brief Convert uint16 value from network byte order (big-endian) to host byte
 * order
 * @param value Pointer to uint16 value
 */
static inline void convert_uint16_from_be(uint16_t *value) {
  *value = ntohs(*value);
}

/**
 * @brief Convert uint32 value from network byte order (big-endian) to host byte
 * order
 * @param value Pointer to uint32 value
 */
static inline void convert_uint32_from_be(uint32_t *value) {
  *value = ntohl(*value);
}

/**
 * @brief Convert uint16 value from host byte order to network byte order
 * (big-endian)
 * @param value Pointer to uint16 value
 */
static inline void convert_uint16_to_be(uint16_t *value) {
  *value = htons(*value);
}

/**
 * @brief Convert uint32 value from host byte order to network byte order
 * (big-endian)
 * @param value Pointer to uint32 value
 */
static inline void convert_uint32_to_be(uint32_t *value) {
  *value = htonl(*value);
}

/****************************************************************************/
// Time Utility Functions
/****************************************************************************/

/**
 * @brief Get current time in milliseconds
 * @return Current time in milliseconds
 */
int get_mills(void);

/**
 * @brief Add two timespec structures
 * @param time1 First timespec
 * @param time2 Second timespec
 * @return Sum of two timespecs
 */
struct timespec timespec_add(struct timespec time1, struct timespec time2);

/****************************************************************************/
// String Utility Functions
/****************************************************************************/

/**
 * @brief Get joint name string
 * @param joint_id Joint index
 * @return Joint name string
 */
const char *get_joint_name(finger_index_t joint_id);

/**
 * @brief Get control mode name string
 * @param mode Control mode value
 * @return Control mode name string
 */
const char *get_control_mode_name(uint16_t mode);

/****************************************************************************/
// EtherCAT State Monitoring
/****************************************************************************/

/**
 * @brief Check and print EtherCAT domain state changes
 * @param domain EtherCAT domain pointer
 * @param domain_state Pointer to domain state structure
 */
void check_domain_state(ec_domain_t *domain, ec_domain_state_t *domain_state);

/**
 * @brief Check and print EtherCAT master state changes
 * @param master EtherCAT master pointer
 * @param master_state Pointer to master state structure
 */
void check_master_state(ec_master_t *master, ec_master_state_t *master_state);

#ifdef __cplusplus
}
#endif

#endif // EC_UTILS_H

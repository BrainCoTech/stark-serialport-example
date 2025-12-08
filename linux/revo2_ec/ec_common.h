#ifndef EC_COMMON_H
#define EC_COMMON_H

/****************************************************************************/
// System headers
/****************************************************************************/
#include <ecrt.h>
#include <stdbool.h>
#include <stdint.h>
#include <time.h>

/****************************************************************************/
// Core EtherCAT library modules
/****************************************************************************/
#include "ec_types.h"
#include "ec_constants.h"
#include "ec_macros.h"

/****************************************************************************/
// Functional modules
/****************************************************************************/
#include "ec_utils.h"
#include "ec_pdo.h"
#include "ec_control.h"

/****************************************************************************/
// High-level modules
/****************************************************************************/
#include "ec_feedback.h"
#include "ec_sdo.h"
#include "ec_app.h"
#include "ec_demo_helpers.h"  // Includes gesture and trajectory functions

#ifdef __cplusplus
extern "C" {
#endif

#ifdef __cplusplus
}
#endif

#endif // EC_COMMON_H

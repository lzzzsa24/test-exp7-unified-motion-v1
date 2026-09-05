#ifndef LINE_SEARCH_MODEL_H
#define LINE_SEARCH_MODEL_H

#include <stdint.h>
#include "vehicle_geometry.h"

/* Extended differential model for nominal search speed, NOT pivot control.
   N and chi reproduce encoder_turn.c's existing conversion. Chi is a prior
   terrain-dependent estimate, not freshly identified from chassis motion. */
#define LINE_SEARCH_COUNTS_PER_REV       1040L
#define LINE_SEARCH_SKID_PERMILLE        2619L
#define LINE_SEARCH_EFFECTIVE_TRACK_MM \
  ((VEHICLE_TRACK_WIDTH_MM * LINE_SEARCH_SKID_PERMILLE + 500L) / 1000L)

/* Engineering speed choice: reduce the previous nominal 173 deg/s to 120.
   90 deg/s is a lower-speed comparison. Neither value is a turn-angle limit. */
#ifndef LINE_SEARCH_NOMINAL_YAW_MDEG_S
#define LINE_SEARCH_NOMINAL_YAW_MDEG_S 120000L
#endif

#define LINE_SEARCH_CPS_DENOMINATOR (360000LL * VEHICLE_WHEEL_DIAMETER_MM)
#define LINE_SEARCH_TARGET_CPS \
  ((int32_t)(((int64_t)LINE_SEARCH_NOMINAL_YAW_MDEG_S * \
      LINE_SEARCH_EFFECTIVE_TRACK_MM * LINE_SEARCH_COUNTS_PER_REV + \
      LINE_SEARCH_CPS_DENOMINATOR / 2LL) / LINE_SEARCH_CPS_DENOMINATOR))

/* Existing continuous-speed operating band: do not silently clamp a nominal
   yaw request and thereby hide that the requested conversion is infeasible. */
_Static_assert(LINE_SEARCH_TARGET_CPS >= 1412L &&
               LINE_SEARCH_TARGET_CPS <= 3600L,
               "Search speed outside the baseline continuous CPS band");

#endif

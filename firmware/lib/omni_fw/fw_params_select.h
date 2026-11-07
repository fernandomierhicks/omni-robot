// Which robot's compiled-in constants the core builds against. The Teensy build picks with -DFW_PARAMS_HEADER
// (platformio.ini: env arm -> "fw_params_arm.h", env catch -> fw_params.h); the SIL build stages the variant's header
// as fw_params.h (sil/build_sil.py), so it leaves FW_PARAMS_HEADER unset.
#pragma once

#ifdef FW_PARAMS_HEADER
#include FW_PARAMS_HEADER
#else
#include "fw_params.h"
#endif

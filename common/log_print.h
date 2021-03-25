/***************************************************************************************************
** Copyright (C), 2019-2020, OPLUS Mobile Comm Corp., Ltd.
**
** File: data_convert.cpp
** Description: Include different log print header files according to different platforms
** Version: 1.0
** Date : 2020/03/06
** Author: baijin@oplus.com
**
** -------------------------------------- Revision History : ---------------------------------------
**  <author>	      <date> 	 <version >	 <desc>
**  baijin@oplus.com   2020/05/21 1.0         build this module
** -------------------------------------------- Notes : --------------------------------------------
***************************************************************************************************/

#ifndef LOG_PRINT_H_
#define LOG_PRINT_H_

# include <stdio.h>
#include "config.h"

//#ifdef __cplusplus
//extern "C" {
//#endif

#define RELEASE_LOG 1
//#define TIME_COST_LOG 1
//#define DEBUG_LOG 1

#ifdef QUALCOMM_DSP_AVAILABLE
#include "HAP_farf.h"
#define FARF_HIGH  1
#define FARF_ERROR 1
#define FARF_FATAL 1
#define oplusAudioLogPrint(a, b, c, d, e, f, g, h, i, j, k) FARF(HIGH, a, b, c, d, e, f, g, h, i, j, k)
// usage
// oplusAudioLogPrint("ktv2.0: malloc in file %s : %d", __FILE__, __LINE__, 0, 0, 0, 0, 0, 0, 0, 0);
#endif

#ifdef PC_AVAILABLE
#define oplusAudioLogPrint(a, b, c, d, e, f, g, h, i, j, k) printf(a, b, c, d, e, f, g, h, i, j, k)
// usage
// oplusAudioLogPrint("ktv2.0: malloc in file %s : %d", __FILE__, __LINE__, 0, 0, 0, 0, 0, 0, 0, 0);
#endif

//#ifdef __cplusplus
//} // extern "C"
//#endif

#endif // LOG_PRINT_H_

/***************************************************************************************************
** Copyright (C), 2019-2020, OPLUS Mobile Comm Corp., Ltd.
**
** File: config.h
** Description: control the effectiveness of some specific codes and control log print
** Version: 1.0
** Date : 2020/03/06
** Author: baijin@oplus.com
**
** -------------------------------------- Revision History : ---------------------------------------
**  <author>	      <date> 	 <version >	 <desc>
**  baijin@oplus.com   2019/12/20 1.0         build this module
**  baijin@oplus.com   2020/03/06 1.1         add comment
** -------------------------------------------- Notes : --------------------------------------------
** This document does not contain any algorithm code, only contains the macro definition which is 
** used to control the effectiveness of some specific codes and control log print, and define global
** variables                                              
***************************************************************************************************/

#ifndef CONFIG_H_
#define CONFIG_H_

//#include "log_print.h"
//#ifdef __cplusplus
//extern "C" {
//#endif

/***************************************************************************************************
**                                     Common macro definition
***************************************************************************************************/
//#define ANDROID_NAME_SPACE 1

/***************************************************************************************************
**             Set the static switch of different code blocks on different platforms
***************************************************************************************************/
//#define MTK_AP_AVAILABLE
//#define MTK_DSP_AVAILABLE
#define QUALCOMM_DSP_AVAILABLE
//#define PC_AVAILABLE

//--------------------------------------------------------------------------------------------------
#define HIGH_PASS_FILTER_OPEN
#define NS_OPEN
#define VOLUME_PROCESS_OPEN
#define PARAMETRIC_EQ_PROC_OPEN
#define PITCH_CHANGE_PROC_OPEN
#define LOW_PASS_FILTER_FOR_PITCHCHANGE_OPEN
#define REVERB_PROC_OPEN
#define COMPRESSOR_PROC_OPEN

/***************************************************************************************************
**                                         Return value definition
***************************************************************************************************/
#define SUCCESS          (0)
#define FAIL             (-1)
#define MEM_LEAK         (-2) // memory leak
#define MEM_REQ_FAIL     (-3) // memory requst fail
#define MEM_FREE_FAIL    (-4) // memory free fail


/***************************************************************************************************
**                                         Global variables
***************************************************************************************************/
/* Record the number of malloc and release */
extern long long gMemReqNums;
extern long long gMemFreeNums;

//#ifdef __cplusplus
//} // extern "C"
//#endif

#endif // CONFIG_H_


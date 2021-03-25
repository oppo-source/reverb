/***************************************************************************************************
** Copyright (C), 2019-2020, OPLUS Mobile Comm Corp., Ltd.
**
** File: config.cpp
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
#include "config.h"
//#ifdef __cplusplus
//extern "C" {
//#endif

/***************************************************************************************************
**                                         Global variables
***************************************************************************************************/
/* Record the number of malloc and release */
long long gMemReqNums = 0;
long long gMemFreeNums = 0;

//#ifdef __cplusplus
//} // extern "C"
//#endif

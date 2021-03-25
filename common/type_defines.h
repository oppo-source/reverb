/******************************************************************************
  Copyright (C)2018-2020.xxx Technology(Shenzhen) Co.Ltd
                        All rights reserved
  Developed by the Audio Dept.
File        : basic_defines.h
Description : basic type defines for all modules.
Author      : Nic
History     : 
   Author        Date      Version      Description of Changes
  ---------- ----------- ----------- ------------------------------------------
   Nic       2020.03.25   V1.1.1.10    new. basic type defines for Audio.
*******************************************************************************/
#ifndef BASIC_DEFINES_H_
#define BASIC_DEFINES_H_

#if defined(_WIN32)                 //platform:Visual Studio
typedef signed char        int8_t;
typedef short              int16_t;
typedef int                int32_t;
typedef long long          int64_t;
typedef unsigned char      uint8_t;
typedef unsigned short     uint16_t;
typedef unsigned int       uint32_t;
typedef unsigned long long uint64_t;
#else                               //platform:default

#endif
 

#endif      //BASIC_DEFINES_H_//




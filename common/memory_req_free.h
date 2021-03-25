/***************************************************************************************************
** Copyright (C), 2019-2020, OPLUS Mobile Comm Corp., Ltd.
**
** File: memory_req_free.h
** Description: Memory request and free
** Version: 1.0
** Date : 2020/03/06
** Author: baijin@oplus.com
**
** -------------------------------------- Revision History : ---------------------------------------
**  <author>	      <date> 	 <version >	 <desc>
**  baijin@oplus.com   2020/05/21 1.0         build this module
** -------------------------------------------- Notes : --------------------------------------------
***************************************************************************************************/

#ifndef MEMORY_REQ_FREE_H_
#define MEMORY_REQ_FREE_H_

/* common include */
#include "config.h"
#include "log_print.h"

//#ifdef __cplusplus
//extern "C" {
//#endif

/* Call the function before memory request to determine if there is memory leak */
#define memLeakConfirm(ptr) \
if (ptr != NULL) {\
    oplusAudioLogPrint("ktv2.0: warming !!! memory leak in file %s : %d\n", __FILE__, __LINE__, 0, 0, \
0, 0, 0, 0, 0, 0);\
}

/* Call the function after  memory request to determine if memory request success */
#define memReqConfirm(ptr) \
if (ptr == NULL) { \
    oplusAudioLogPrint("ktv2.0: error !!! memory request fail in file %s : %d\n", __FILE__, __LINE__, \
0, 0, 0, 0, 0, 0, 0, 0);\
} else {\
    gMemReqNums = gMemReqNums + 1;\
    oplusAudioLogPrint("ktv2.0: memory request success in file %s : %d, gMemReqNums = %d\n", __FILE__, \
__LINE__, gMemReqNums, 0, 0, 0, 0, 0, 0, 0);\
}

/* Encapsulation of free function*/
#define memFree(ptr) \
if (ptr != NULL) {\
    free(ptr);\
    ptr = NULL;\
    gMemFreeNums = gMemFreeNums + 1;\
    oplusAudioLogPrint("ktv2.0: memory request success in file %s : %d, gMemFreeNums = %d\n", __FILE__, \
__LINE__, gMemFreeNums, 0, 0, 0, 0, 0, 0, 0);\
} else { \
}

//#ifdef __cplusplus
//} // extern "C"
//#endif

#endif // MEMORY_REQ_FREE_H_

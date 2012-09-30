/*
 * Copyright (c) 2010 Yamaha Corporation
 * 
 * This software is provided 'as-is', without any express or implied
 * warranty.  In no event will the authors be held liable for any damages
 * arising from the use of this software.
 * 
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 * 
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software
 *    in a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 */

#include "yas.h"

#if YAS_ACC_DRIVER == YAS_ACC_DRIVER_ADXL345
#include "yas_acc_driver-adxl345.c"
#elif YAS_ACC_DRIVER == YAS_ACC_DRIVER_ADXL346
#include "yas_acc_driver-adxl345.c"
#elif YAS_ACC_DRIVER == YAS_ACC_DRIVER_BMA150
#include "yas_acc_driver-bma150.c"
#elif YAS_ACC_DRIVER == YAS_ACC_DRIVER_BMA222
#if defined (CONFIG_MACH_ARIESVE)
#include "yas_acc_driver-bma222.c"       // Galaxy S+  (REV 0.0 bma023 REV 0.1 bma222)
#elif defined (CONFIG_MACH_ANCORA)
#include "yas_acc_driver-bma222.c"            // Ancora
#elif defined (CONFIG_MACH_ANCORA_TMO)
#include "yas_acc_driver-bma222.c"            // Ancora
#elif defined (CONFIG_MACH_GODART)
#include "yas_acc_driver-bma222.c"            // Godart
#elif defined (CONFIG_MACH_APACHE)
#include "yas_acc_driver-bma250.c"            // Apache
#else
#include "yas_acc_driver-bma222.c"
#endif
#elif YAS_ACC_DRIVER == YAS_ACC_DRIVER_BMA250
#include "yas_acc_driver-bma250.c"
#elif YAS_ACC_DRIVER == YAS_ACC_DRIVER_KXSD9
#include "yas_acc_driver-kxsd9.c"
#elif YAS_ACC_DRIVER == YAS_ACC_DRIVER_KXTE9
#include "yas_acc_driver-kxte9.c"
#elif YAS_ACC_DRIVER == YAS_ACC_DRIVER_KXTF9
#include "yas_acc_driver-kxtf9.c"
#elif YAS_ACC_DRIVER == YAS_ACC_DRIVER_LIS331DL
#include "yas_acc_driver-lis331dl.c"
#elif YAS_ACC_DRIVER == YAS_ACC_DRIVER_LIS331DLH
#include "yas_acc_driver-lis331dlh.c"
#elif YAS_ACC_DRIVER == YAS_ACC_DRIVER_LIS331DLM
#include "yas_acc_driver-lis331dlm.c"
#elif YAS_ACC_DRIVER == YAS_ACC_DRIVER_LIS3DH
#include "yas_acc_driver-lis3dh.c"
#elif YAS_ACC_DRIVER == YAS_ACC_DRIVER_MMA8453Q
#include "yas_acc_driver-mma8453q.c"
#elif YAS_ACC_DRIVER == YAS_ACC_DRIVER_MMA8452Q
#include "yas_acc_driver-mma8452q.c"
#endif

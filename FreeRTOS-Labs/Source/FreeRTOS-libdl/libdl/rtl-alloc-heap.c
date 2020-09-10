/*
 *  COPYRIGHT (c) 2012 Chris Johns <chrisj@rtems.org>
 *
 *  The license and distribution terms for this file may be
 *  found in the file LICENSE in this distribution or at
 *  http://www.rtems.org/license/LICENSE.
 */
/**
 * @file
 *
 * @ingroup rtems_rtl
 *
 * @brief RTEMS Run-Time Linker Allocator for the standard heap.
 */

#include <stdlib.h>

#include "rtl-alloc-heap.h"
#include "rtl-alloc-lock.h"

#include <FreeRTOS.h>
#include "semphr.h"

void
rtems_rtl_alloc_heap (rtems_rtl_alloc_cmd cmd,
                      rtems_rtl_alloc_tag tag,
                      void**              address,
                      size_t              size)
{
  switch (cmd)
  {
    case RTEMS_RTL_ALLOC_NEW:
      *address = pvPortMalloc (size);
      break;
    case RTEMS_RTL_ALLOC_DEL:
      vPortFree (*address);
      *address = NULL;
      break;
    case RTEMS_RTL_ALLOC_LOCK:
      _RTEMS_Lock_allocator();
      break;
    case RTEMS_RTL_ALLOC_UNLOCK:
      _RTEMS_Unlock_allocator();
      break;
    case RTEMS_RTL_ALLOC_WR_DISABLE:
#ifdef __CHERI_PURE_CAPABILITY__
      address = __builtin_cheri_perms_and(address,
                   ~(__CHERI_CAP_PERMISSION_PERMIT_LOAD__ | \
                   __CHERI_CAP_PERMISSION_PERMIT_STORE__));
    case RTEMS_RTL_ALLOC_SET_PERMS: {
      switch (tag)
      {
        case RTEMS_RTL_ALLOC_READ:
          address = __builtin_cheri_perms_and(address,
                       ~(__CHERI_CAP_PERMISSION_PERMIT_EXECUTE__ | \
                       __CHERI_CAP_PERMISSION_PERMIT_STORE__));
        case RTEMS_RTL_ALLOC_READ_WRITE:
          address = __builtin_cheri_perms_and(address,
                       ~(__CHERI_CAP_PERMISSION_PERMIT_EXECUTE__));
        case RTEMS_RTL_ALLOC_READ_EXEC:
          address = __builtin_cheri_perms_and(address,
                       ~(__CHERI_CAP_PERMISSION_PERMIT_STORE__));
        default:
          break;
      }
    } break;
#endif
    default:
      break;
  }
}

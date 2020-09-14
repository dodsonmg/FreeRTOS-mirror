#if !defined (_FREERTOS_RTL_COMPARTMENTS_H_)
#define _FREERTOS_RTL_COMPARTMENTS_H_

#include <FreeRTOS.h>
#include <stdbool.h>
#include <rtl/rtl-obj.h>

typedef struct compartment {
  void*       cap;
  void**      cap_list;
  char*       name;
  size_t      size;
  // TCB?
  // Symtab?
} compartment_t;

extern compartment_t comp_list[configCOMPARTMENTS_NUM];
extern char comp_strtab[configCOMPARTMENTS_NUM][configMAXLEN_COMPNAME];

int rtl_freertos_compartment_open(const char *name);
ssize_t rtl_freertos_compartment_read(int fd, void *buffer, UBaseType_t offset, size_t count);
size_t rtl_freertos_compartment_getsize(int fd);
size_t rtl_freertos_global_symbols_add(rtems_rtl_obj* obj);

/**
 * Allocate a new array-based captable for an object.
 *
 * @param obj The object compartment to allocate a captable for.
 * @param caps_count The number of capabilities to allocate a table for.
 * @retval true If allocated successfully
 * @retval false The table could not be created. The RTL error has the error.
 */
bool
rtl_cherifreertos_captable_alloc(rtems_rtl_obj* obj, size_t caps_count);

/**
 * Install a new capability in the first free slot in the captable of an object.
 * If the table is full, it will try to re-alloc a new captable to increase the
 * size and fit in the new cap.
 *
 * @param obj The object compartment to allocate a captable for.
 * @param new_cap The content of the capability to install in the table.
 * @retval A pointer to the slot in the captable the new_cap got installed into
 * returns NULL if failed and The RTL error has the error.
 */
void**
rtl_cherifreertos_captable_install_new_cap(rtems_rtl_obj* obj, void* new_cap);
#endif

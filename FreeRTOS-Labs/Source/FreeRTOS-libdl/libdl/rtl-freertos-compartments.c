#ifdef ipconfigUSE_FAT_LIBDL
#include "ff_headers.h"
#include "ff_stdio.h"
#endif

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <rtl/rtl.h>
#include <rtl/rtl-freertos-compartments.h>
#include <rtl/rtl-allocator.h>
#include <rtl/rtl-obj.h>
#include <rtl/rtl-trace.h>
#include <errno.h>
#include "rtl-error.h"

#include <FreeRTOS.h>

#if __riscv_xlen == 32
#define ELFSIZE 32
#elif __riscv_xlen == 64
#define ELFSIZE 64
#endif

#include <sys/exec_elf.h>

#ifdef __CHERI_PURE_CAPABILITY__
#include <cheric.h>
#include <cheri/cheri-utility.h>
extern void *pvAlmightyDataCap;
extern void *pvAlmightyCodeCap;
#endif

extern Elf_Sym*  __symtab_start;
extern Elf_Sym*  __symtab_end;
extern char*  __strtab_start;
extern char*  __strtab_end;

compartment_t comp_list[configCOMPARTMENTS_NUM];
static uint32_t comp_id_free = 0;

void* rtl_freertos_compartment_open(const char *name)
{
#ifdef ipconfigUSE_FAT_LIBDL
  FF_FILE *file = ff_fopen( name, "r" );

  if (file == NULL) {
    rtems_rtl_set_error (EBADF, "Failed to open the file");
    return -1;
  }

  return file;

#else
  const char* comp_name = NULL;
  char name_buff[configMAXLEN_COMPNAME];

  char *name_dot = strrchr(name, '.');
  if (name_dot == NULL) {
    return -1;
  }

  comp_name = rtems_rtl_alloc_new (RTEMS_RTL_ALLOC_OBJECT, name_dot - name + 1, true);
  if (!comp_name) {
    rtems_rtl_set_error (ENOMEM, "no memory for comp file name");
    return -1;
  }

  memcpy(comp_name, name, name_dot - name);

  for(int i = 0; i < configCOMPARTMENTS_NUM; i++) {
    if (strncmp(comp_list[i].name, comp_name, configMAXLEN_COMPNAME) == 0) {
      return i;
    }
  }

  return -1;
#endif
}

size_t rtl_freertos_compartment_read(void* fd, void *buffer, UBaseType_t offset, size_t count)
{
#ifdef ipconfigUSE_FAT_LIBDL
  return ff_fread( buffer, 1, count, (FF_FILE *) fd );
#else
  if (fd < 0 || fd >= configCOMPARTMENTS_NUM) {
    rtems_rtl_set_error (EBADF, "Invalid compartment/fd number");
    return -1;
  }

  // If trying to read past the file size, trim down the count to read only to
  // the EoF.
  if (offset + count > comp_list[fd].size) {
    count -= (offset + count) - comp_list[fd].size;
  }

  if (memcpy(buffer, comp_list[fd].cap + offset, count)) {
    return count;
  }

  return -1;
#endif
}

size_t rtl_freertos_compartment_getsize(void *fd) {
#ifdef ipconfigUSE_FAT_LIBDL
  return ((FF_FILE *) fd)->ulFileSize;
#else
  return comp_list[fd].size;
#endif
}

void
rtems_rtl_symbol_global_insert (rtems_rtl_symbols* symbols,
                                rtems_rtl_obj_sym* symbol);

size_t
rtl_freertos_global_symbols_add(rtems_rtl_obj* obj) {
  Elf_Sym*  symtab_start = &__symtab_start;
  Elf_Sym*  symtab_end = &__symtab_end;
  char*  strtab_start = &__strtab_start;
  char*  strtab_end = &__strtab_end;
  uint32_t syms_count =  ((size_t) &__symtab_end - (size_t) &__symtab_start) / sizeof(Elf_Sym);

#ifdef __CHERI_PURE_CAPABILITY__
  size_t strtab_size = ((size_t) &__strtab_end - (size_t) &__strtab_start);
  symtab_start = cheri_build_data_cap((ptraddr_t) symtab_start, syms_count * sizeof(Elf_Sym), 0xff);
  strtab_start = cheri_build_data_cap((ptraddr_t) strtab_start, strtab_size, 0xff);
#endif

  rtems_rtl_symbols* symbols;
  rtems_rtl_obj_sym* sym;
  uint32_t globals_count = 0;

  for(int i = 0; i < syms_count; i++) {
    if (ELF_ST_BIND(symtab_start[i].st_info) == STB_GLOBAL) {
      globals_count++;
    }
  }

  if (rtems_rtl_trace (RTEMS_RTL_TRACE_GLOBAL_SYM))
    printf ("rtl: global symbol add: %zi\n", globals_count);

  obj->global_size = globals_count * sizeof (rtems_rtl_obj_sym);
  obj->global_table = rtems_rtl_alloc_new (RTEMS_RTL_ALLOC_SYMBOL,
                                           obj->global_size, true);
  if (!obj->global_table)
  {
    obj->global_size = 0;
    rtems_rtl_set_error (ENOMEM, "no memory for global symbols");
    return false;
  }

#ifdef __CHERI_PURE_CAPABILITY__
  obj->captable = NULL;
  if (!rtl_cherifreertos_captable_alloc(obj, globals_count))
  {
    if (rtems_rtl_trace (RTEMS_RTL_TRACE_CHERI))
      printf("rtl:cheri: Failed to alloc a global cap table for %s\n", obj->oname);

    return 0;
  }
#endif

  symbols = rtems_rtl_global_symbols ();

  sym = obj->global_table;

  for(int i = 0; i < syms_count; i++) {
    if (ELF_ST_BIND(symtab_start[i].st_info) == STB_GLOBAL) {
      sym->name =  &strtab_start[symtab_start[i].st_name];
      uint32_t str_idx = symtab_start[i].st_name;
      char *cap_str = strtab_start + str_idx;
#ifdef __CHERI_PURE_CAPABILITY__
      void *cap = NULL;
      if (ELF_ST_TYPE(symtab_start[i].st_info) == STT_OBJECT) {
        cap = cheri_build_data_cap((ptraddr_t) symtab_start[i].st_value,
        symtab_start[i].st_size,
        __CHERI_CAP_PERMISSION_GLOBAL__ | \
        __CHERI_CAP_PERMISSION_PERMIT_LOAD__ | \
        __CHERI_CAP_PERMISSION_PERMIT_STORE__);
      } else if (ELF_ST_TYPE(symtab_start[i].st_info) == STT_FUNC) {
        cap = cheri_build_code_cap_unbounded((ptraddr_t) symtab_start[i].st_value,
        __CHERI_CAP_PERMISSION_GLOBAL__ | \
        __CHERI_CAP_PERMISSION_PERMIT_EXECUTE__ | \
        __CHERI_CAP_PERMISSION_PERMIT_LOAD__ | \
        __CHERI_CAP_PERMISSION_PERMIT_LOAD_CAPABILITY__ | \
        __CHERI_CAP_PERMISSION_PERMIT_STORE__ | \
        __CHERI_CAP_PERMISSION_PERMIT_STORE_CAPABILITY__);
      }

      sym->capability = rtl_cherifreertos_captable_install_new_cap(obj, cap);
      if (!sym->capability) {
        if (rtems_rtl_trace (RTEMS_RTL_TRACE_CHERI))
          printf("rtl:cheri: Failed to install a new cap in %s captable\n", obj->oname);
        return 0;
      }
#endif
      sym->value = symtab_start[i].st_value;

      if (rtems_rtl_symbol_global_find (sym->name) == NULL) {
        rtems_rtl_symbol_global_insert (symbols, sym);
        ++sym;
      }
    }
  }

  obj->global_syms = globals_count;

  return globals_count;
}

uint32_t rtl_cherifreertos_compartment_get_free_compid(void) {

  if (comp_id_free >= configCOMPARTMENTS_NUM) {
    printf("Too many compartments, only %d are supported\n", configCOMPARTMENTS_NUM);
  }

  return comp_id_free++;
}

#ifdef __CHERI_PURE_CAPABILITY__
bool
rtl_cherifreertos_compartment_set_captable(rtems_rtl_obj* obj) {

  if (!obj->captable) {
    rtems_rtl_set_error (EINVAL, "Cap table hasn't been set yet");
    return false;
  }

  if (obj->comp_id >= configCOMPARTMENTS_NUM)
    return false;

  comp_list[obj->comp_id].captable = obj->captable;

  return true;
}

bool
rtl_cherifreertos_compartment_set_obj(rtems_rtl_obj* obj) {

  if (!obj) {
    rtems_rtl_set_error (EINVAL, "Invalid object to set for a compartment");
    return false;
  }

  if (obj->comp_id >= configCOMPARTMENTS_NUM)
    return false;

  comp_list[obj->comp_id].obj = obj;

  return true;
}

rtems_rtl_obj *
rtl_cherifreertos_compartment_get_obj(size_t comp_id) {

  if (comp_id >= configCOMPARTMENTS_NUM)
    return NULL;

  return (rtems_rtl_obj *) comp_list[comp_id].obj;
}

void **
rtl_cherifreertos_compartment_get_captable(size_t comp_id) {

  if (comp_id >= configCOMPARTMENTS_NUM)
    return NULL;

  return comp_list[comp_id].captable;
}

static bool
rtl_cherifreertos_captable_copy(void **dest_captable, void **src_captable, size_t caps_count) {
  void** cap_table = NULL;

  if (!dest_captable || !src_captable) {
    rtems_rtl_set_error (EINVAL, "Invalid captables to copy");
    return false;
  }

  memcpy(dest_captable, src_captable, caps_count * sizeof(void *));

  return true;
}

static bool
rtl_cherifreertos_captable_realloc(rtems_rtl_obj* obj, size_t new_caps_count) {
  void** cap_table = NULL;

  if (!obj->captable) {
    rtems_rtl_set_error (ENOTEMPTY, "There is no cap table for this object");
    return false;
  }

  cap_table = (void **) rtems_rtl_alloc_new (RTEMS_RTL_ALLOC_CAPTAB,
                                   new_caps_count * sizeof(void *), true);
  if (!cap_table) {
    rtems_rtl_set_error (ENOMEM, "no memory to re-create a new captable");
    return false;
  }

  if (!rtl_cherifreertos_captable_copy(cap_table, obj->captable, obj->caps_count)) {
    rtems_rtl_set_error (ENOMEM, "Failed to copy cap tables");
  }

  memset(obj->captable, 0, obj->caps_count * sizeof(void *));
  rtems_rtl_alloc_del(RTEMS_RTL_ALLOC_CAPTAB, obj->captable);

  obj->captable = cap_table;
  obj->caps_count = new_caps_count;

  if (!rtl_cherifreertos_compartment_set_captable(obj))
    return false;

  return true;
}

static uint32_t
rtl_cherifreertos_captable_get_free_slot(rtems_rtl_obj* obj) {

  if (obj->captable_free_slot >= obj->caps_count) {
    return 0;
  }

  return obj->captable_free_slot++;
}

uint32_t
rtl_cherifreertos_captable_install_new_cap(rtems_rtl_obj* obj, void* new_cap) {
  uint32_t free_slot;

  if (!obj->captable) {
    rtems_rtl_set_error (EINVAL, "There is no cap table for this object");
    return NULL;
  }

  free_slot = rtl_cherifreertos_captable_get_free_slot(obj);
  if (!free_slot) {
    // Try to realloc a new captable to install a new slot
    if (rtems_rtl_trace (RTEMS_RTL_TRACE_CHERI)) {
      printf("rtl:captable: no empty slot for a new cap, trying to realloc\n");
    }

    // Re-alloc a new captable with an extra slot for a new cap. We may want to
    // increase the number of slots to add when re-allocating if it's expected
    // to on-demand allocate many new caps for (i.e., if the object does many
    // externals accesses.
    if (!rtl_cherifreertos_captable_realloc(obj, obj->caps_count + 1)) {
      rtems_rtl_set_error (ENOMEM, "Couldn't realloc a new captable to install a new cap in");
      return NULL;
    }

    // Try again after increasing the cap table size
    free_slot = rtl_cherifreertos_captable_get_free_slot(obj);
    if (!free_slot) {
      rtems_rtl_set_error (ENOMEM, "Still can not find a free slot after realloc");
      return NULL;
    }
  }


  if (rtems_rtl_trace (RTEMS_RTL_TRACE_CHERI)) {
    printf("rtl:captable: Installing a new cap: "); cheri_print_cap(new_cap);
  }

  *(obj->captable + free_slot) = new_cap;

  return free_slot;
}

bool
rtl_cherifreertos_captable_alloc(rtems_rtl_obj* obj, size_t caps_count) {
  void** cap_table = NULL;

  if (obj->captable) {
    rtems_rtl_set_error (ENOTEMPTY, "There is already a cap table for this object");
    return false;
  }

  cap_table = (void **) rtems_rtl_alloc_new (RTEMS_RTL_ALLOC_CAPTAB,
                                   caps_count * sizeof(void *), true);
  if (!cap_table) {
    rtems_rtl_set_error (ENOMEM, "no memory to create a new captable");
    return false;
  }

  obj->captable = cap_table;
  obj->caps_count = caps_count;

  if (!rtl_cherifreertos_compartment_set_captable(obj))
    return false;

  return true;
}

bool
rtl_cherifreertos_capstack_alloc(rtems_rtl_obj* obj, size_t stack_depth) {
  void* stack = NULL;
  size_t stacks_bytes = stack_depth * sizeof(void *);

  if (!obj->captable) {
    rtems_rtl_set_error (ENOTEMPTY, "There is no captable for this object");
    return false;
  }

  if (*obj->captable) {
    rtems_rtl_set_error (ENOTEMPTY, "There is already an installed stack for this object");
    return false;
  }

  stack = (void *) rtems_rtl_alloc_new (RTEMS_RTL_ALLOC_OBJECT,
                                   stacks_bytes, true);

  stack = __builtin_cheri_offset_set(stack, stacks_bytes - sizeof(void *));

  if (!stack) {
    rtems_rtl_set_error (ENOMEM, "Failed to allocate a stack for this object");
    return false;
  }

  *obj->captable = stack;

  return true;
}
#endif

/*
 *  COPYRIGHT (c) 2012-2014, 2018 Chris Johns <chrisj@rtems.org>
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
 * @brief RTEMS Run-Time Linker Object File Symbol Table.
 */

#if HAVE_CONFIG_H
#include "config.h"
#endif

#include <errno.h>
#include <inttypes.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include <rtl/rtl.h>
#include "rtl-error.h"
#include <rtl/rtl-sym.h>
#include <rtl/rtl-obj.h>
#include <rtl/rtl-trace.h>
#include <rtl/rtl-freertos-compartments.h>

#ifdef __CHERI_PURE_CAPABILITY__
#include <cheri/cheri-utility.h>
#endif

/**
 * The single symbol forced into the global symbol table that is used to load a
 * symbol table from an object file.
 */
static rtems_rtl_obj_sym global_sym_add =
{
  .name  = "rtems_rtl_base_sym_global_add",
  .value = (void*) rtems_rtl_base_sym_global_add
};

static uint_fast32_t
rtems_rtl_symbol_hash (const char *s)
{
  uint_fast32_t h = 5381;
  unsigned char c;
  for (c = *s; c != '\0'; c = *++s)
    h = h * 33 + c;
  return h & 0xffffffff;
}

void
rtems_rtl_symbol_global_insert (rtems_rtl_symbols* symbols,
                                rtems_rtl_obj_sym* symbol)
{
  uint_fast32_t hash = rtems_rtl_symbol_hash (symbol->name);
  vListInsertEnd (&symbols->buckets[hash % symbols->nbuckets],
                      &symbol->node);
}

bool
rtems_rtl_symbol_table_open (rtems_rtl_symbols* symbols,
                             size_t             buckets)
{
  symbols->buckets = rtems_rtl_alloc_new (RTEMS_RTL_ALLOC_SYMBOL,
                                          buckets * sizeof (List_t),
                                          true);
  if (!symbols->buckets)
  {
    rtems_rtl_set_error (ENOMEM, "no memory for global symbol table");
    return false;
  }
  symbols->nbuckets = buckets;
  for (buckets = 0; buckets < symbols->nbuckets; ++buckets)
    vListInitialise (&symbols->buckets[buckets]);
  rtems_rtl_symbol_global_insert (symbols, &global_sym_add);
  return true;
}

void
rtems_rtl_symbol_table_close (rtems_rtl_symbols* symbols)
{
  rtems_rtl_alloc_del (RTEMS_RTL_ALLOC_SYMBOL, symbols->buckets);
}

bool
rtems_rtl_symbol_global_add (rtems_rtl_obj*       obj,
                             const unsigned char* esyms,
                             unsigned int         size)
{
  rtems_rtl_symbols* symbols;
  rtems_rtl_obj_sym* sym;
  size_t             count;
  size_t             s;
  uint32_t           marker;

  count = 0;
  s = 0;
  while ((s < size) && (esyms[s] != 0))
  {
    int l = strlen ((char*) &esyms[s]);
    if ((esyms[s + l] != '\0') || ((s + l) > size))
    {
      rtems_rtl_set_error (EINVAL, "invalid exported symbol table");
      return false;
    }
    ++count;
    s += l + sizeof (unsigned long) + 1;
  }

  /*
   * Check this is the correct end of the table.
   */
  marker = esyms[s + 1];
  marker <<= 8;
  marker |= esyms[s + 2];
  marker <<= 8;
  marker |= esyms[s + 3];
  marker <<= 8;
  marker |= esyms[s + 4];

  if (marker != 0xdeadbeefUL)
  {
    rtems_rtl_set_error (ENOMEM, "invalid export symbol table");
    return false;
  }

  if (rtems_rtl_trace (RTEMS_RTL_TRACE_GLOBAL_SYM))
    printf ("rtl: global symbol add: %zi\n", count);

  obj->global_size = count * sizeof (rtems_rtl_obj_sym);
  obj->global_table = rtems_rtl_alloc_new (RTEMS_RTL_ALLOC_SYMBOL,
                                           obj->global_size, true);
  if (!obj->global_table)
  {
    obj->global_size = 0;
    rtems_rtl_set_error (ENOMEM, "no memory for global symbols");
    return false;
  }

  symbols = rtems_rtl_global_symbols ();

  s = 0;
  sym = obj->global_table;

  while ((s < size) && (esyms[s] != 0))
  {
    /*
     * Copy the void* using a union and memcpy to avoid any strict aliasing or
     * alignment issues. The variable length of the label and the packed nature
     * of the table means casting is not suitable.
     */
    union {
      uint8_t data[sizeof (void*)];
      void*   value;
    } copy_voidp;
    int b;

    sym->name = (const char*) &esyms[s];
    s += strlen (sym->name) + 1;
    for (b = 0; b < sizeof (void*); ++b, ++s)
      copy_voidp.data[b] = esyms[s];
    sym->value = copy_voidp.value;
    if (rtems_rtl_trace (RTEMS_RTL_TRACE_GLOBAL_SYM))
      printf ("rtl: esyms: %s -> %8p\n", sym->name, sym->value);
    if (rtems_rtl_symbol_global_find (sym->name) == NULL)
      rtems_rtl_symbol_global_insert (symbols, sym);
    ++sym;
  }

  obj->global_syms = count;

  return true;
}

rtems_rtl_obj_sym*
rtems_rtl_symbol_global_find (const char* name)
{
  rtems_rtl_symbols*   symbols;
  uint_fast32_t        hash;
  List_t* bucket;
  ListItem_t*    node;

  symbols = rtems_rtl_global_symbols ();

  hash = rtems_rtl_symbol_hash (name);
  bucket = &symbols->buckets[hash % symbols->nbuckets];
  node = listGET_HEAD_ENTRY (bucket);

  while (listGET_END_MARKER (bucket) != node)
  {
    rtems_rtl_obj_sym* sym = (rtems_rtl_obj_sym*) node;
    /*
     * Use the hash. I could add this to the symbol but it uses more memory.
     */
    if (strcmp (name, sym->name) == 0)
      return sym;
    node = listGET_NEXT (node);
  }

  return NULL;
}

static int
rtems_rtl_symbol_obj_compare (const void* a, const void* b)
{
  const rtems_rtl_obj_sym* sa;
  const rtems_rtl_obj_sym* sb;
  sa = (const rtems_rtl_obj_sym*) a;
  sb = (const rtems_rtl_obj_sym*) b;
  return strcmp (sa->name, sb->name);
}

void
rtems_rtl_symbol_obj_sort (rtems_rtl_obj* obj)
{
/*
  qsort (obj->local_table,
         obj->local_syms,
         sizeof (rtems_rtl_obj_sym),
         rtems_rtl_symbol_obj_compare);
  qsort (obj->global_table,
         obj->global_syms,
         sizeof (rtems_rtl_obj_sym),
         rtems_rtl_symbol_obj_compare);
*/
}

static rtems_rtl_obj_sym*
rtems_rtl_symbol_list_find (List_t* list, const char* name)
{
  ListItem_t *node = listGET_HEAD_ENTRY (list);

  while (listGET_END_MARKER (list) != node)
  {
    rtems_rtl_obj_sym* sym = (rtems_rtl_obj_sym*) node;

    if (strcmp(sym->name, name) == 0) {
      return sym;
    }

    node = listGET_NEXT (node);
  }

  return NULL;
}

rtems_rtl_obj_sym*
rtems_rtl_lsymbol_obj_find (rtems_rtl_obj* obj, const char* name)
{
  return rtems_rtl_symbol_list_find(&obj->locals_list, name);
}

rtems_rtl_obj_sym*
rtems_rtl_gsymbol_obj_find (rtems_rtl_obj* obj, const char* name)
{
  return rtems_rtl_symbol_list_find(&obj->globals_list, name);
}

rtems_rtl_obj_sym*
rtems_rtl_isymbol_obj_find (rtems_rtl_obj* obj, const char* name)
{
  return rtems_rtl_symbol_list_find(&obj->interface_list, name);
}

rtems_rtl_obj_sym*
rtems_rtl_esymbol_obj_find (rtems_rtl_obj* obj, const char* name)
{
  return rtems_rtl_symbol_list_find(&obj->externals_list, name);
}

rtems_rtl_obj_sym*
rtems_rtl_symbol_obj_find (rtems_rtl_obj* obj, const char* name)
{
  rtems_rtl_obj_sym* match = NULL;
  /*
   * Check the object file's symbols first. If not found search the
   * global symbol table.
   */
  if (obj->local_syms)
  {
    match = rtems_rtl_lsymbol_obj_find (obj, name);
    if (match != NULL)
      return match;
  }

  if (obj->global_syms)
  {
    match = rtems_rtl_gsymbol_obj_find (obj, name);
    if (match != NULL)
      return match;
  }

  if (obj->interface_syms)
  {
    match = rtems_rtl_isymbol_obj_find (obj, name);
    if (match != NULL)
      return match;
  }

  if (obj->externals_syms)
  {
    match = rtems_rtl_esymbol_obj_find (obj, name);
    if (match != NULL)
      return match;
  }

  /*
   * If the symbol is found in the public global list (FreeRTOS/libc) mint it to
   * the obj cap table.
   */
  match = rtems_rtl_symbol_global_find (name);
  if (match) {
    return rtems_rtl_isymbol_obj_mint(NULL, obj, match->name);
  }

#if 0
    rtems_rtl_obj_sym* match;
    rtems_rtl_obj_sym  key = { 0 };
    key.name = name;
    match = bsearch (&key, obj->local_table,
                     obj->local_syms,
                     sizeof (rtems_rtl_obj_sym),
                     rtems_rtl_symbol_obj_compare);
    if (match != NULL)
      return match;
    rtems_rtl_obj_sym* match;
    rtems_rtl_obj_sym  key = { 0 };
    key.name = name;
    match = bsearch (&key, obj->global_table,
                     obj->global_syms,
                     sizeof (rtems_rtl_obj_sym),
                     rtems_rtl_symbol_obj_compare);
    if (match != NULL)
      return match;
#endif


  return NULL;
}

rtems_rtl_obj_sym*
rtems_rtl_symbol_obj_find_namevalue (rtems_rtl_obj* obj, const char* name, UBaseType_t value)
{
  ListItem_t *node = listGET_HEAD_ENTRY (&obj->locals_list);

  while (listGET_END_MARKER (&obj->locals_list) != node)
  {
    rtems_rtl_obj_sym* sym = (rtems_rtl_obj_sym*) node;

    if (strcmp(sym->name, name) == 0 && (UBaseType_t) sym->value == value) {
      return sym;
    }

    node = listGET_NEXT (node);
  }

  return NULL;
}

rtems_rtl_obj_sym*
rtems_rtl_symbol_obj_extract (rtems_rtl_obj* obj, const char* name)
{
  ListItem_t *node = listGET_HEAD_ENTRY (&obj->locals_list);

  while (listGET_END_MARKER (&obj->locals_list) != node)
  {
    rtems_rtl_obj_sym* sym = (rtems_rtl_obj_sym*) node;

    if (strcmp(sym->name, name) == 0) {
      uxListRemove(node);
      return sym;
    }

    node = listGET_NEXT (node);
  }

  return NULL;
}

bool
rtems_rtl_isymbol_create (rtems_rtl_obj* obj, isymbol_table_mode_t mode)
{
  char *istring = NULL;
  char *name = NULL;
  size_t slen = 0;

  // Copy all the globals to the interface list. Ang global symbol in this
  // compartment can then be accessed from other compartments. That is the
  // simplest and most straightforward policy, following standard C static
  // linking behavior.
  if (mode == RTL_INTERFACE_SYMBOL_ALL_GLOBALS) {
    obj->interface_table = rtems_rtl_alloc_new (RTEMS_RTL_ALLOC_SYMBOL,
                                            obj->global_size, true);
    if (!obj->interface_table) {
      obj->interface_syms = 0;
      rtems_rtl_set_error (ENOMEM, "no memory for obj interface syms");
      return false;
    }

    memcpy(obj->interface_table, obj->global_table, obj->global_size);

    istring = (char*) obj->interface_table + (obj->global_syms * sizeof(rtems_rtl_obj_sym));

    for (int i = 0; i < obj->global_syms; i++) {

       vListInitialiseItem(&obj->interface_table[i].node);
       vListInsertEnd(&obj->interface_list, &obj->interface_table[i].node);

       name = obj->global_table[i].name;
       slen = strlen (name) + 1;
       memcpy(istring, name, slen);

       obj->interface_table[i].name = istring;
       istring += slen;
    }

    obj->interface_syms = obj->global_syms;

    return true;
  } else {
    rtems_rtl_set_error (EINVAL, "Invalid mode for creating a new interface list");
    return false;
  }

  return false;
}

rtems_rtl_obj_sym*
rtems_rtl_isymbol_obj_mint (rtems_rtl_obj* src_obj, rtems_rtl_obj* dest_obj, const char* name)
{
  char *estring = NULL;
  size_t slen = 0;
  rtems_rtl_obj_sym *esym = NULL;
  rtems_rtl_obj_sym *sym = NULL;

  // If src_obj is NULL, search the global symbol table (FreeRTOS/libc) as they
  // do not have an allocated object.
  if (src_obj == NULL) {
    sym = rtems_rtl_symbol_global_find (name);
    if (!sym) {
      rtems_rtl_set_error (ENOMEM, "Could not find %s in the global symbol table", name);
      return NULL;
    }
  } else {
    // Seach the interface list of the src_obj to check if it does own that symbol
    // TODO: check of dest_obj is allowed to call src_obj:name
    sym = rtems_rtl_isymbol_obj_find(src_obj, name);
    if (!sym) {
      return NULL;
    }
  }

  slen = strlen(name) + 1;

  // Allocate a new buffer for the symbol and its name
  esym = rtems_rtl_alloc_new (RTEMS_RTL_ALLOC_SYMBOL, sizeof(rtems_rtl_obj_sym) + slen, true);
  if (!esym) {
    rtems_rtl_set_error (ENOMEM, "no memory for an external symbol");
    return NULL;
  }

  estring = (char*) esym + (sizeof(rtems_rtl_obj_sym));

  // Copy the symbol from interface table to externals table
  memcpy(esym, sym, sizeof(rtems_rtl_obj_sym));
  memcpy(estring, name, slen);

  // Allocate a new cap slot in the interface captable and install it
  // For now, just copy the same copy, but we may want to version/ID them for
  // each different object compartment.
  esym->capability = rtl_cherifreertos_captable_install_new_cap(dest_obj, *sym->capability);
  if (!esym->capability) {
    rtems_rtl_set_error (ENOMEM, "Could not mint a new cap to the dest obj");
    return NULL;
  }

  // TODO: We may need to bookkeep the owner of that object (src_obj) in the
  // symbol struct (e.g., capability).
  // Add the symbol to the dest_obj extenals list
  vListInitialiseItem(&esym->node);
  vListInsertEnd(&dest_obj->externals_list, &esym->node);

  return esym;
}

void
rtems_rtl_symbol_obj_add (rtems_rtl_obj* obj)
{
  rtems_rtl_symbols* symbols;
  rtems_rtl_obj_sym* sym;
  size_t             s;

  symbols = rtems_rtl_global_symbols ();

  for (s = 0, sym = obj->global_table; s < obj->global_syms; ++s, ++sym)
    rtems_rtl_symbol_global_insert (symbols, sym);
}

void
rtems_rtl_symbol_obj_erase_local (rtems_rtl_obj* obj)
{
  if (obj->local_table)
  {
    rtems_rtl_alloc_del (RTEMS_RTL_ALLOC_SYMBOL, obj->local_table);
    obj->local_table = NULL;
    obj->local_size = 0;
    obj->local_syms = 0;
  }
}

void
rtems_rtl_symbol_obj_erase (rtems_rtl_obj* obj)
{
  rtems_rtl_symbol_obj_erase_local (obj);
  if (obj->global_table)
  {
    rtems_rtl_obj_sym* sym;
    size_t             s;
    for (s = 0, sym = obj->global_table; s < obj->global_syms; ++s, ++sym)
        if (listLIST_ITEM_CONTAINER (&sym->node))
          uxListRemove (&sym->node);
    rtems_rtl_alloc_del (RTEMS_RTL_ALLOC_SYMBOL, obj->global_table);
    obj->global_table = NULL;
    obj->global_size = 0;
    obj->global_syms = 0;
  }
}

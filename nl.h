/** \file nl.h include file for libnl.
 *
 * The library under this name was created in November 2018
 * based on nortlib2.
 *
 * The MSG_ codes are documented on the \ref MSG_CODES page.
 */
#ifndef _NL_H_INCLUDED
#define _NL_H_INCLUDED

#include <stdio.h>
#include <stdarg.h>
#include <sys/types.h>

#ifdef __cplusplus
extern "C" {
#endif

int Skel_open(const char *name);
int Skel_copy(FILE *ofp, const char *label, int copyout);

extern int (*msg)(int level, const char *s, ...);
int nl_err(int level, const char *s, ...);
extern int (*msgv)(int level, const char *fmt, va_list args);
int nl_verr(int level, const char *fmt, va_list args);

/**
 * \defgroup MSG_CODES msg() severity level codes
 * @{
 * The MSG_ codes are msg()-compatible severity level codes.
 */
#define MSG_DEBUG -2
#define MSG_EXIT -1
#define MSG_EXIT_NORM MSG_EXIT
#define MSG 0
#define MSG_PASS MSG
#define MSG_WARN 1
#define MSG_ERROR 2
#define MSG_FAIL MSG_ERROR
#define MSG_FATAL 3
#define MSG_EXIT_ABNORM 4
#define MSG_DBG(X) (MSG_DEBUG-(X))
/** @} */

/**
 * nl_debug_level controls which debug messages are output in
 * msg() and msgv() functions. It is initialized to -1.
 * If the level code passed to msg() or msgv() is less than
 * or equal to MSG_DEBUG (-2) but also less than nl_debug_level,
 * the message is not displayed. Hence by lowering the
 * value of nl_debug_level, more messages are enabled.
 *
 * nl_debug_level can be set directly by program code,
 * but the most common method to adjust it is via the
 * -v option defined in msg.oui. Each occurance of -v
 * decrements nl_debug_level by one, enabling another
 * level of messages.
 *
 * Messages of level MSG_DEBUG (which is the same as
 * MSG_DBG(0)) will not be displayed by default, but
 * will be with the -v option. Messages of level
 * MSG_DBG(1) will display only if the -v option
 * appears at least twice on the command line
 * (e.g. -vv).
 */
extern int nl_debug_level;
extern int nl_response;
int set_response(int newval);
#define NLRSP_DIE 3
#define NLRSP_WARN 1
#define NLRSP_QUIET 0

/* guaranteed memory allocator, memlib.h subset.
 * Include memlib.h to obtain the full definition
 */
#ifndef MEMLIB_H_INCLUDED
  #define new_memory(x) nl_new_memory(x)
  #define free_memory(x) nl_free_memory(x)
  #define nl_strdup(x) nrtl_strdup(x)
#endif
void *nl_new_memory(size_t size);
void nl_free_memory(void *p);
char *nrtl_strdup(const char *s);

#if defined(__QNXNTO__)
  #define OPTIND_RESET 1
#else
  #define OPTIND_RESET 0
#endif

#ifdef __cplusplus
};
#else
/** ascii_escape.c
 * Only expose definition to C programs to avoid conflict with C++ version.
 */
const char *ascii_escape(const char *ibuf);
#endif

#endif

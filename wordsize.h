 /* Determine the wordsize from the preprocessor defines.  */
#ifndef __wordSize_H
#define __wordSize_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#if defined __x86_64__
# define __WORDSIZE 64u
# define __WORDSIZE_COMPAT32 1u
#else
# define __WORDSIZE 32u
#endif

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif
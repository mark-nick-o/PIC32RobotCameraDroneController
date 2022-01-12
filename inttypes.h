#ifndef INTTYPES_H
#define INTTYPES_H
//
//    Copyright (C) 2015 Martine Lenders <mlenders@inf.fu-berlin.de>
//
//    This file is subject to the terms and conditions of the GNU Lesser
//    General Public License v2.1. See the file LICENSE in the top level
//    directory for more details.
//    Ported  Copyright (C) 2020 A C P Avaiation Walkerburn Scotland
//#include_next <inttypes.h>

#ifdef __cplusplus
 extern "C" {
#endif

#define PRIo64  "llo"
#define PRIx64  "llx"
#define PRIu64  "llu"
#define PRId64  "lld"

#ifdef __cplusplus
}
#endif

#endif /* INTTYPES_H */
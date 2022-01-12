//#pragma once
#ifndef HAVE_MAVLINK_SHA256                                                     // Allow implementation to provide their own sha256 with the same API

//#ifdef MAVLINK_USE_CXX_NAMESPACE
//namespace mavlink {
//#endif

#ifndef MAVLINK_SHA256
#define MAVLINK_SHA256
//#endif

/*
  sha-256 implementation for MAVLink based on Heimdal sources, with
  modifications to suit mavlink headers
 */
/*
 * Copyright (c) 1995 - 2001 Kungliga Tekniska Högskolan
 * (Royal Institute of Technology, Stockholm, Sweden).
 * All rights reserved.
 *
 * Ported by (C) 2020 A C P Avaiation Walkerburn Scotland
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

typedef union {
  unsigned char save_bytes[64u];
  uint32_t save_u32[16u];
} mavlink_sha_u;

typedef struct {
  uint32_t sz[2u];
  uint32_t counter[8u];
  mavlink_sha_u u;
} mavlink_sha256_ctx;
  

#define Ch(x,y,z) (((x) & (y)) ^ ((~(x)) & (z)))
#define Maj(x,y,z) (((x) & (y)) ^ ((x) & (z)) ^ ((y) & (z)))

#define ROTR(x,n)   (((x)>>(n)) | ((x) << (32u - (n))))

#define Sigma0(x) (ROTR(x,2u)  ^ ROTR(x,13u) ^ ROTR(x,22u))
#define Sigma1(x) (ROTR(x,6u)  ^ ROTR(x,11u) ^ ROTR(x,25u))
#define sigma0(x) (ROTR(x,7u)  ^ ROTR(x,18u) ^ ((x)>>3u))
#define sigma1(x) (ROTR(x,17u) ^ ROTR(x,19u) ^ ((x)>>10u))

#define mCount1 m->counter[0u]
#define mCount2 m->counter[1u]
#define mCount3 m->counter[2u]
#define mCount4 m->counter[3u]
#define mCount5 m->counter[4u]
#define mCount6 m->counter[5u]
#define mCount7 m->counter[6u]
#define mCount8 m->counter[7u]

static const uint32_t mavlink_sha256_constant_256[64u] = {
    0x428a2f98L, 0x71374491L, 0xb5c0fbcfL, 0xe9b5dba5L,
    0x3956c25bL, 0x59f111f1L, 0x923f82a4L, 0xab1c5ed5L,
    0xd807aa98L, 0x12835b01L, 0x243185beL, 0x550c7dc3L,
    0x72be5d74L, 0x80deb1feL, 0x9bdc06a7L, 0xc19bf174L,
    0xe49b69c1L, 0xefbe4786L, 0x0fc19dc6L, 0x240ca1ccL,
    0x2de92c6fL, 0x4a7484aaL, 0x5cb0a9dcL, 0x76f988daL,
    0x983e5152L, 0xa831c66dL, 0xb00327c8L, 0xbf597fc7L,
    0xc6e00bf3L, 0xd5a79147L, 0x06ca6351L, 0x14292967L,
    0x27b70a85L, 0x2e1b2138L, 0x4d2c6dfcL, 0x53380d13L,
    0x650a7354L, 0x766a0abbL, 0x81c2c92eL, 0x92722c85L,
    0xa2bfe8a1L, 0xa81a664bL, 0xc24b8b70L, 0xc76c51a3L,
    0xd192e819L, 0xd6990624L, 0xf40e3585L, 0x106aa070L,
    0x19a4c116L, 0x1e376c08L, 0x2748774cL, 0x34b0bcb5L,
    0x391c0cb3L, 0x4ed8aa4aL, 0x5b9cca4fL, 0x682e6ff3L,
    0x748f82eeL, 0x78a5636fL, 0x84c87814L, 0x8cc70208L,
    0x90befffaL, 0xa4506cebL, 0xbef9a3f7L, 0xc67178f2L
};

MAVLINK_HELPER void mavlink_sha256_init(mavlink_sha256_ctx *m)
{
    m->sz[0u] = 0u;
    m->sz[1u] = 0u;
    mCount1 = 0x6a09e667L;
    mCount2 = 0xbb67ae85L;
    mCount3 = 0x3c6ef372L;
    mCount4 = 0xa54ff53aL;
    mCount5 = 0x510e527fL;
    mCount6 = 0x9b05688cL;
    mCount7 = 0x1f83d9abL;
    mCount8 = 0x5be0cd19L;
}

static inline void mavlink_sha256_calc(mavlink_sha256_ctx *m, uint32_t *in)
{
    uint32_t AA, BB, CC, DD, EE, FF, GG, HH;
    uint32_t dataV[64u];
    int16_t i;
    uint32_t T1, T2;

    AA = mCount1;
    BB = mCount2;
    CC = mCount3;
    DD = mCount4;
    EE = mCount5;
    FF = mCount6;
    GG = mCount7;
    HH = mCount8;

    for (i = 0; i < 16; ++i)
        dataV[i] = in[i];
    for (i = 16; i < 64; ++i)
        dataV[i] = sigma1(dataV[i-2u]) + dataV[i-7u] + sigma0(dataV[i-15u]) + dataV[i-16u];

    for (i = 0; i < 64; i++)
    {
        T1 = HH + Sigma1(EE) + Ch(EE, FF, GG) + mavlink_sha256_constant_256[i] + dataV[i];
        T2 = Sigma0(AA) + Maj(AA,BB,CC);

        HH = GG;
        GG = FF;
        FF = EE;
        EE = DD + T1;
        DD = CC;
        CC = BB;
        BB = AA;
        AA = T1 + T2;
    }

    mCount1 += AA;
    mCount2 += BB;
    mCount3 += CC;
    mCount4 += DD;
    mCount5 += EE;
    mCount6 += FF;
    mCount7 += GG;
    mCount8 += HH;
}

MAVLINK_HELPER void mavlink_sha256_update(mavlink_sha256_ctx *m, const void *v, uint32_t len)
{
    const unsigned char *p = (const unsigned char *)v;
    uint32_t old_sz = m->sz[0u];
    uint32_t offset;
    int16_t i;
    uint32_t current[16u];
    const uint32_t *u;
    const uint8_t *p1;
    uint8_t *p2;
    uint32_t l;
            
    m->sz[0u] += len * 8u;
    if (m->sz[0u] < old_sz)
        ++m->sz[1u];
    offset = (old_sz / 8u) % 64u;
    while(len > 0u)
    {
        l = 64u - offset;
        if (len < l) 
        {
            l = len;
        }
        memcpy((void*)m->u.save_bytes + offset,(void*) p,(int16_t) l);
        offset += l;
        p += l;
        len -= l;
        if(offset == 64u)
        {
            u = m->u.save_u32;
            for (i = 0; i < 16; i++)
            {
                p1 = (const uint8_t *)&u[i];
                p2 = (uint8_t *)&current[i];
                p2[0u] = p1[3u];
                p2[1u] = p1[2u];
                p2[2u] = p1[1u];
                p2[3u] = p1[0u];
            }
            mavlink_sha256_calc(m, current);
            offset = 0u;
        }
    }
}

/*
  get first 48 bits of final sha256 hash
 */
MAVLINK_HELPER void mavlink_sha256_final_48(mavlink_sha256_ctx *m, uint8_t result[6u])
{
    unsigned char zeros[72u];
    unsigned offset = (m->sz[0u] / 8u) % 64u;
    unsigned int dstart = (120u - offset - 1u) % 64u + 1u;
    uint8_t *p = (uint8_t *)&m->counter[0u];

    *zeros = 0x80u;
    memset (zeros + 1u, 0u, sizeof(zeros) - 1u);
    zeros[dstart+7u] = (m->sz[0u] >> 0u) & 0xffu;
    zeros[dstart+6u] = (m->sz[0u] >> 8u) & 0xffu;
    zeros[dstart+5u] = (m->sz[0u] >> 16u) & 0xffu;
    zeros[dstart+4u] = (m->sz[0u] >> 24u) & 0xffu;
    zeros[dstart+3u] = (m->sz[1u] >> 0u) & 0xffu;
    zeros[dstart+2u] = (m->sz[1u] >> 8u) & 0xffu;
    zeros[dstart+1u] = (m->sz[1u] >> 16u) & 0xffu;
    zeros[dstart+0u] = (m->sz[1u] >> 24u) & 0xffu;

    mavlink_sha256_update(m, zeros, dstart + 8u);

    // this ordering makes the result consistent with taking the first
    // 6 bytes of more conventional sha256 functions. It assumes
    // little-endian ordering of m->counter
    result[0u] = p[3u];
    result[1u] = p[2u];
    result[2u] = p[1u];
    result[3u] = p[0u];
    result[4u] = p[7u];
    result[5u] = p[6u];
}

// prevent conflicts with users of the header
#undef A
#undef B
#undef C
#undef D
#undef E
#undef F
#undef G
#undef H
#undef Ch
#undef ROTR
#undef Sigma0
#undef Sigma1
#undef sigma0
#undef sigma1

//#ifdef MAVLINK_USE_CXX_NAMESPACE
//} // namespace mavlink
//#endif

//#endif // HAVE_MAVLINK_SHA256
#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif  // end library

#endif // end exclusion
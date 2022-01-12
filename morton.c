#include <stdint.h>
#include "definitions.h"

#if defined(USE_MORTON_3D)
#ifdef __cplusplus
 extern "C" {
#endif

#include "Struts.h"
#include "morton_3D_LUTs.h"
/* ===========================================================================================
  https://github.com/Kugelstadt/CompactNSearch/blob/master/extern/libmorton/libmorton/include
  Tassilo Kugelstadt Aachen University 
  Ported by ACP Aviation

  Libmorton - Methods to encode/decode 64-bit/32-bit morton codes from/to (x,y,z) coordinates
  ============================================================================================ */
uint32_t m3D_e_sLUTU32vec(const dVectr vec);
uint32_t m3D_e_LUTU32vec(const dVectr vec);
uint64_t m3D_e_sLUTU64vec(const dVectr vec);
uint64_t m3D_e_LUTU64vec(const dVectr vec);
uint64_t m3D_e_LUTU64(const uint64_t x, const uint64_t y, const uint64_t z );
uint64_t m3D_e_sLUTU64(const uint64_t x, const uint64_t y, const uint64_t z);
uint32_t m3D_e_sLUTU32(const uint32_t x, const uint32_t y, const uint32_t z);
uint32_t m3D_e_LUTU32(const uint32_t x, const uint32_t y, const uint32_t z);
uint64_t splitNumAtBitAndInvert( uint64_t word, uint8_t bitNo, mortonBitDirection_e direction );
int8_t BitScanReverseU32( const uint32_t word, mortonBitDirection_e direction );
int8_t BitScanReverseU64( const uint64_t word, mortonBitDirection_e direction );
uint32_t compute3D_ET_LUT_encodeU32( const uint32_t c, const uint32_t *LUT);
uint64_t compute3D_ET_LUT_encodeU64( const uint32_t c, const uint32_t *LUT);
uint32_t m3D_e_sLUT_ETU32(const uint32_t x, const uint32_t y, const uint32_t z);
uint64_t m3D_e_sLUT_ETU64(const uint64_t x, const uint64_t y, const uint64_t z);
float64_t m3D_e_sLUT_ET_dVectr(const dVectr vec);
uint32_t m3D_e_LUT_ETU32(const uint32_t x, const uint32_t y, const uint32_t z);
uint64_t m3D_e_LUT_ETU64(const uint64_t x, const uint64_t y, const uint64_t z);
float64_t m3D_e_LUT_ET_dVectr(const dVectr vec);
uint32_t morton3D_SplitBy3bitsU32(const uint32_t a);
uint64_t morton3D_SplitBy3bitsU64(const uint64_t a);
uint32_t m3D_e_magicbitsU32(const uint32_t x, const uint32_t y, const uint32_t z);
uint64_t m3D_e_magicbitsU64(const uint64_t x, const uint64_t y, const uint64_t z);
uint64_t m3D_e_magicbitsMortonVec(const MORTON_3DVectr_t mVec, MORTON3DVectr_Type_e Typ);
uint32_t m3D_e_forU32(const uint32_t x, const uint32_t y, const uint32_t z);
uint64_t m3D_e_forU64(const uint64_t x, const uint64_t y, const uint64_t z);
uint64_t m3D_e_fordVectr(const dVectr vec);
uint64_t m3D_e_forMortonVec(const MORTON_3DVectr_t mVec, MORTON3DVectr_Type_e Typ);
uint64_t m3D_e_for_ET_U64(const uint64_t x, const uint64_t y, const uint64_t z);
uint32_t m3D_e_for_ET_U32(const uint32_t x, const uint32_t y, const uint32_t z);
uint64_t m3D_e_for_ET_dVectr(const dVectr vec);
uint64_t m3D_e_for_ET_MortonVec(const MORTON_3DVectr_t mVec, MORTON3DVectr_Type_e Typ);
uint64_t morton3D_DecodeCoord_LUT256_U64(const uint64_t m, const uint8_t *LUT, const uint16_t startshift);
uint32_t morton3D_DecodeCoord_LUT256_U32(const uint32_t m, const uint8_t *LUT, const uint16_t startshift);
void m3D_d_sLUT_U64(const uint64_t m, uint64_t *x, uint64_t *y, uint64_t *z);
void m3D_d_sLUT_U32(const uint32_t m, uint32_t *x, uint32_t *y, uint32_t *z);
void m3D_d_sLUT_dVectr(const uint64_t m, dVectr *vec);
void m3D_d_sLUT_MortonVec(const morton_type_u m, MORTON_3DVectr_t *mVec, MORTON3DVectr_Type_e Typ);
void m3D_d_LUT_U64(const uint64_t m, uint64_t *x, uint64_t *y, uint64_t *z);
void m3D_d_LUT_U32(const uint32_t m, uint32_t *x, uint32_t *y, uint32_t *z);
void m3D_d_LUT_dVectr(const uint64_t m, dVectr *vec);
void m3D_d_LUT_MortonVec(const morton_type_u m, MORTON_3DVectr_t *mVec, MORTON3DVectr_Type_e Typ);
int8_t m3D_d_sLUT_ET_U64(const uint64_t m, uint64_t *x, uint64_t *y, uint64_t *z);
int8_t m3D_d_sLUT_ET_U32(const uint32_t m, uint32_t *x, uint32_t *y, uint32_t *z);
int8_t m3D_d_sLUT_ET_dVectr(const uint64_t m, dVectr *vec);
int8_t m3D_d_LUT_ET_U64(const uint64_t m, uint64_t *x, uint64_t *y, uint64_t *z);
int8_t m3D_d_LUT_ET_U32(const uint32_t m, uint32_t *x, uint32_t *y, uint32_t *z);
int8_t m3D_d_LUT_ET_dVectr(const uint64_t m, dVectr *vec);
uint32_t morton3D_GetThirdBitsU32(const uint32_t m);
uint64_t morton3D_GetThirdBitsU64(const uint64_t m);
void m3D_d_magicbitsU64(const uint64_t m, uint64_t *x, uint64_t *y, uint64_t *z);
void m3D_d_magicbitsU32(const uint32_t m, uint32_t *x, uint32_t *y, uint32_t *z);
void m3D_d_magicbitsdVectr(const uint64_t m, dVectr *vec);
void m3D_d_forU64(const uint64_t m, uint64_t *x, uint64_t *y, uint64_t *z);
void m3D_d_forU32(const uint32_t m, uint32_t *x, uint32_t *y, uint32_t *z);
int8_t m3D_d_for_ETU32(const uint32_t m, uint32_t *x, uint32_t *y, uint32_t *z);
int8_t m3D_d_for_ETU64(const uint64_t m, uint64_t *x, uint64_t *y, uint64_t *z);
int8_t m3D_d_for_ETUdVectr(const uint64_t m, dVectr *rec);

/*-----------------------------------------------------------------------------
 *  m3D_e_sLUTU32vec:  ENCODE 3D Morton code : Pre-Shifted LookUpTable (sLUT)
 *
 *  Parameters: const dVectr vec  
 *   
 *
 *  Return: uint32_t
 *----------------------------------------------------------------------------*/
uint32_t m3D_e_sLUTU32vec(const dVectr vec) 
{
    uint32_t answer = 0.0f;
    const uint32_t EIGHTBITMASK = 0x000000FFul;
    uint16_t i,shift;
        
    for (i = sizeof(dVectr); i > 0; --i) 
    {
        shift = (i - 1) * 8;
        answer = ((uint32_t) (answer << 24u | (Morton3D_encode_z_256[(((uint32_t)vec.z) >> shift) & EIGHTBITMASK] | Morton3D_encode_y_256[(((uint32_t)vec.y) >> shift) & EIGHTBITMASK] | Morton3D_encode_x_256[(((uint32_t)vec.x) >> shift) & EIGHTBITMASK])));
    }
   return answer;
}

/*-----------------------------------------------------------------------------
 *  m3D_e_LUTU32vec:  ENCODE 3D Morton code : LookUpTable (LUT)
 *
 *  Parameters: const dVectr vec  
 *   
 *
 *  Return: uint32_t
 *----------------------------------------------------------------------------*/
uint32_t m3D_e_LUTU32vec(const dVectr vec) 
{
    uint32_t answer = 0.0f;
    const static uint32_t EIGHTBITMASK = 0x000000FFul;
    uint16_t i,shift;
        
    for (i = sizeof(dVectr); i > 0; --i) 
    {
                shift = (i - 1) * 8u;
                answer = (answer << 24u | (Morton3D_encode_x_256[ (((uint32_t) vec.z) >> shift) & EIGHTBITMASK] << 2u) | (Morton3D_encode_x_256[ (((uint32_t) vec.y) >> shift) & EIGHTBITMASK] << 1u) |        Morton3D_encode_x_256[ (((uint32_t) vec.x) >> shift) & EIGHTBITMASK]);
     }
     return answer;
}

/*-----------------------------------------------------------------------------
 *  m3D_e_sLUTU64vec:  ENCODE 3D Morton code : Pre-Shifted LookUpTable (sLUT)
 *
 *  Parameters: const dVectr vec  
 *   
 *
 *  Return: uint64_t
 *----------------------------------------------------------------------------*/
uint64_t m3D_e_sLUTU64vec(const dVectr vec) 
{
    uint64_t answer = 0ull;
    const uint64_t EIGHTBITMASK = 0x000000FFull;
    uint16_t i,shift;
        
    for (i = sizeof(dVectr); i > 0; --i) 
    {
                shift = (i - 1) * 8;
                answer = ((uint64_t) (answer << 24u | (Morton3D_encode_z_256[(((uint64_t)vec.z) >> shift) & EIGHTBITMASK] | Morton3D_encode_y_256[(((uint64_t)vec.y) >> shift) & EIGHTBITMASK] | Morton3D_encode_x_256[(((uint64_t)vec.x) >> shift) & EIGHTBITMASK])));
    }
    return answer;
}

/*-----------------------------------------------------------------------------
 *  m3D_e_LUTU64vec:  ENCODE 3D Morton code : LookUpTable (LUT)
 *
 *  Parameters: const dVectr vec  
 *   
 *
 *  Return: uint64_t
 *----------------------------------------------------------------------------*/
uint64_t m3D_e_LUTU64vec(const dVectr vec) 
{
    uint64_t answer = 0ull;
    const uint64_t EIGHTBITMASK = 0x000000FFull;
    uint16_t i,shift;
        
    for (i = sizeof(dVectr); i > 0; --i) 
    {
         shift = (i - 1) * 8u;
         answer = (answer << 24u | (Morton3D_encode_x_256[ (((uint64_t) vec.z) >> shift) & EIGHTBITMASK] << 2u) | (Morton3D_encode_x_256[ (((uint64_t) vec.y) >> shift) & EIGHTBITMASK] << 1u) |        Morton3D_encode_x_256[ (((uint64_t) vec.x) >> shift) & EIGHTBITMASK]);
    }
    return answer;
}

/*-----------------------------------------------------------------------------
 *  m3D_e_LUTU64:  ENCODE 3D Morton code : LookUpTable (LUT)
 *
 *  Parameters: const uint64_t x, const uint64_t y, const uint64_t z  
 *   
 *
 *  Return: uint64_t
 *----------------------------------------------------------------------------*/
uint64_t m3D_e_LUTU64(const uint64_t x, const uint64_t y, const uint64_t z ) 
{
    uint64_t answer = 0ull;
    const uint64_t EIGHTBITMASK = 0x000000FFull;
    uint16_t i,shift;
        
    for (i = sizeof(dVectr); i > 0; --i) 
    {
        shift = (i - 1) * 8u;
        answer = (answer << 24u | (Morton3D_encode_x_256[ (((uint64_t) z) >> shift) & EIGHTBITMASK] << 2u) | (Morton3D_encode_x_256[ (((uint64_t) y) >> shift) & EIGHTBITMASK] << 1u) |        Morton3D_encode_x_256[ (((uint64_t) x) >> shift) & EIGHTBITMASK]);
    }
    return answer;
}

/*-----------------------------------------------------------------------------
 *  m3D_e_sLUTU64vec:  ENCODE 3D Morton code : Pre-Shifted LookUpTable (sLUT)
 *
 *  Parameters: const uint64_t x, const uint64_t y, const uint64_t z  
 *   
 *
 *  Return: uint64_t
 *----------------------------------------------------------------------------*/
uint64_t m3D_e_sLUTU64(const uint64_t x, const uint64_t y, const uint64_t z) 
{
    uint64_t answer = 0ull;
    const uint64_t EIGHTBITMASK = 0x000000FFull;
    uint16_t i,shift;
        
    for (i = sizeof(dVectr); i > 0; --i) 
    {
                shift = (i - 1) * 8;
                answer = ((uint64_t) (answer << 24u | (Morton3D_encode_z_256[(((uint64_t)z) >> shift) & EIGHTBITMASK] | Morton3D_encode_y_256[(((uint64_t)y) >> shift) & EIGHTBITMASK] | Morton3D_encode_x_256[(((uint64_t)x) >> shift) & EIGHTBITMASK])));
     }
     return answer;
}

/*-----------------------------------------------------------------------------
 *  m3D_e_sLUTU32:  ENCODE 3D Morton code : Pre-Shifted LookUpTable (sLUT)
 *
 *  Parameters: const uint32_t x, const uint32_t y, const uint32_t z  
 *   
 *
 *  Return: uint32_t
 *----------------------------------------------------------------------------*/
uint32_t m3D_e_sLUTU32(const uint32_t x, const uint32_t y, const uint32_t z) 
{
    uint32_t answer = 0ul;
    const uint32_t EIGHTBITMASK = 0x000000FFul;
    uint16_t i,shift;
        
    for (i = sizeof(dVectr); i > 0; --i) 
    {
                shift = (i - 1) * 8;
                answer = ((uint32_t) (answer << 24u | (Morton3D_encode_z_256[(((uint32_t)z) >> shift) & EIGHTBITMASK] | Morton3D_encode_y_256[(((uint32_t)y) >> shift) & EIGHTBITMASK] | Morton3D_encode_x_256[(((uint32_t)x) >> shift) & EIGHTBITMASK])));
     }
     return answer;
}

/*-----------------------------------------------------------------------------
 *  m3D_e_LUTU32:  ENCODE 3D Morton code : LookUpTable (LUT)
 *
 *  Parameters: const uint32_t x, const uint32_t y, const uint32_t z  
 *   
 *
 *  Return: uint32_t
 *----------------------------------------------------------------------------*/
uint32_t m3D_e_LUTU32(const uint32_t x, const uint32_t y, const uint32_t z) 
{
    uint32_t answer = 0ul;
    const static uint32_t EIGHTBITMASK = 0x000000FFul;
    uint16_t i,shift;
        
    for (i = sizeof(dVectr); i > 0; --i) 
    {
                shift = (i - 1) * 8u;
                answer = (answer << 24u | (Morton3D_encode_x_256[ (((uint32_t) z) >> shift) & EIGHTBITMASK] << 2u) | (Morton3D_encode_x_256[ (((uint32_t) y) >> shift) & EIGHTBITMASK] << 1u) |        Morton3D_encode_x_256[ (((uint32_t) x) >> shift) & EIGHTBITMASK]);
    }
    return answer;
}

/*-----------------------------------------------------------------------------
 *  splitNumAtBitAndInvert:  Splits number at bit ID and invert one half of it 
 *
 *  Parameters: uint64_t word, uint8_t bitNo, mortonBitDirection_e direction 
 *   
 *
 *  Return: uint64_t
 *----------------------------------------------------------------------------*/
uint64_t splitNumAtBitAndInvert( uint64_t word, uint8_t bitNo, mortonBitDirection_e direction )
{
    uint64_t left_hnd;
    uint64_t right_hnd;
        
    if (direction == 0u)
    {
            left_hnd = ~(word >> bitNo);
            right_hnd = word % (bitNo+1ull);
    }
    else
    {
            left_hnd = (word >> bitNo);
            right_hnd = ~(word % (bitNo+1ull));                
    }
    return (left_hnd | right_hnd);
}

/*-----------------------------------------------------------------------------
 *  BitScanReverseU32:  return the first bit set position 
 *
 *  Parameters: const uint32_t word, mortonBitDirection_e direction 
 *   
 *
 *  Return: int8_t
 *----------------------------------------------------------------------------*/
int8_t BitScanReverseU32( const uint32_t word, mortonBitDirection_e direction )
{
    int8_t iter = -1;
    if (direction == 0u)
    {
        for (iter = 0u; iter < 32u; iter++)
        {
            if ((word & ((uint32_t)pow(2.0f,((float64_t)iter)))) != 0ull)
            {
                return iter;
            }
        }
    }
    else
    {
        for (iter = 31u; iter >= 0u; iter--)
        {
            if ((word & ((uint32_t)pow(2.0f,((float64_t)iter)))) != 0ull)
            {
                return iter;
            }
        }                
    }
    return iter;
}

/*-----------------------------------------------------------------------------
 *  BitScanReverseU64:  return the first bit set position  
 *
 *  Parameters: const uint64_t word, mortonBitDirection_e direction 
 *   
 *
 *  Return: int8_t
 *----------------------------------------------------------------------------*/
int8_t BitScanReverseU64( const uint64_t word, mortonBitDirection_e direction )
{
    int8_t iter = -1;
    if (direction == 0u)
    {
        for (iter = 0u; iter < 64u; iter++)
        {
            if ((word & ((uint64_t)pow(2.0f,((float64_t)iter)))) != 0ull)
            {
                return iter;
            }
        }
    }
    else
    {
        for (iter = 63u; iter >= 0u; iter--)
        {
            if ((word & ((uint64_t)pow(2.0f,((float64_t)iter)))) != 0ull)
            {
                return iter;
            }
        }                
    }
    return iter;
}

/*-----------------------------------------------------------------------------
 *  compute3D_ET_LUT_encodeU32:  HELPER METHOD for ET LUT encode
 *
 *  Parameters: const uint64_t word, mortonBitDirection_e direction 
 *   
 *
 *  Return: int8_t
 *----------------------------------------------------------------------------*/
uint32_t compute3D_ET_LUT_encodeU32( const uint32_t c, const uint32_t *LUT) 
{
    const uint32_t EIGHTBITMASK = 0x000000FFul;
    uint64_t maxbit = 0ull;
    uint32_t answer = 0ul;
    uint32_t i,shift;
        
    maxbit = (BitScanReverseU32(c,MORTON_FROM_LHS));
    if (maxbit != -1) /* a bit has been set in the word */
    {
        for (i = ((uint32_t)ceil((((float64_t)maxbit) + 1.0f) / 8.0f)) ; i >= 0ul; --i)
        {
                    shift = i* 8ul;
                    answer = answer << 24ul | (LUT[(c >> shift) & EIGHTBITMASK]);
        }
    }
    return answer;
}

/*-----------------------------------------------------------------------------
 *  compute3D_ET_LUT_encodeU64:  HELPER METHOD for ET LUT encode
 *
 *  Parameters: const uint64_t word, mortonBitDirection_e direction 
 *   
 *
 *  Return: int8_t
 *----------------------------------------------------------------------------*/
uint64_t compute3D_ET_LUT_encodeU64( const uint32_t c, const uint32_t *LUT) 
{
    const uint64_t EIGHTBITMASK = 0x000000FFull;
    uint64_t maxbit = 0ull;
    uint64_t answer = 0ul;
    uint64_t i,shift;
        
    maxbit = (BitScanReverseU32(c,MORTON_FROM_LHS));
    if (maxbit != -1) /* a bit has been set in the word */
    {
        for (i = ((uint32_t)ceil((((float64_t)maxbit) + 1.0f) / 8.0f)) ; i >= 0ul; --i)
        {
                    shift = i* 8ul;
                    answer = answer << 24ul | (LUT[(c >> shift) & EIGHTBITMASK]);
        }
    }
    return answer;
}


/*-----------------------------------------------------------------------------
 *  m3D_e_sLUT_ETU32:  HELPER METHOD for ET sLUT encode
 *
 *  Parameters: const uint32_t x, const uint32_t y, const uint32_t z 
 *   
 *
 *  Return: uint32_t
 *----------------------------------------------------------------------------*/
uint32_t m3D_e_sLUT_ETU32(const uint32_t x, const uint32_t y, const uint32_t z) 
{
        uint32_t answer_x = compute3D_ET_LUT_encodeU32(x, &Morton3D_encode_x_256);
        uint32_t answer_y = compute3D_ET_LUT_encodeU32(y, &Morton3D_encode_y_256);
        uint32_t answer_z = compute3D_ET_LUT_encodeU32(z, &Morton3D_encode_z_256);
        return answer_z | answer_y | answer_x;
}

/*-----------------------------------------------------------------------------
 *  m3D_e_sLUT_ETU64:  HELPER METHOD for ET sLUT encode
 *
 *  Parameters: const uint64_t x, const uint64_t y, const uint64_t z 
 *   
 *
 *  Return: uint64_t
 *----------------------------------------------------------------------------*/
uint64_t m3D_e_sLUT_ETU64(const uint64_t x, const uint64_t y, const uint64_t z) 
{
        uint64_t answer_x = (uint64_t)compute3D_ET_LUT_encodeU64(x, &Morton3D_encode_x_256);
        uint64_t answer_y = (uint64_t)compute3D_ET_LUT_encodeU64(y, &Morton3D_encode_y_256);
        uint64_t answer_z = (uint64_t)compute3D_ET_LUT_encodeU64(z, &Morton3D_encode_z_256);
        return answer_z | answer_y | answer_x;
}

/*-----------------------------------------------------------------------------
 *  m3D_e_sLUT_ET_dVectr:  HELPER METHOD for ET sLUT encode
 *
 *  Parameters: const dVectr vec 
 *   
 *
 *  Return: float64_t
 *----------------------------------------------------------------------------*/
float64_t m3D_e_sLUT_ET_dVectr(const dVectr vec) 
{
        uint32_t answer_x = compute3D_ET_LUT_encodeU32(((uint32_t)vec.x), &Morton3D_encode_x_256);
        uint32_t answer_y = compute3D_ET_LUT_encodeU32(((uint32_t)vec.y), &Morton3D_encode_y_256);
        uint32_t answer_z = compute3D_ET_LUT_encodeU32(((uint32_t)vec.z), &Morton3D_encode_z_256);        
        return ((float64_t)(answer_z | answer_y | answer_x));
}


// ENCODE 3D Morton code : LookUpTable (LUT) (Early termination version)
// This version tries to terminate early when there are no more bits to process
// Figuring this out is probably too costly in most cases.
/*-----------------------------------------------------------------------------
 *  m3D_e_LUT_ETU32:  LookUpTable (LUT) (Early termination version)
 *
 *  Parameters: const uint32_t x, const uint32_t y, const uint32_t z 
 *   
 *
 *  Return: uint32_t
 *----------------------------------------------------------------------------*/
uint32_t m3D_e_LUT_ETU32(const uint32_t x, const uint32_t y, const uint32_t z) 
{
        uint32_t answer_x = compute3D_ET_LUT_encodeU32(x, &Morton3D_encode_x_256);
        uint32_t answer_y = compute3D_ET_LUT_encodeU32(y, &Morton3D_encode_x_256);
        uint32_t answer_z = compute3D_ET_LUT_encodeU32(z, &Morton3D_encode_x_256);
        return (answer_z << 2u) | (answer_y << 1u) | answer_x;
}

/*-----------------------------------------------------------------------------
 *  m3D_e_LUT_ETU64:  LookUpTable (LUT) (Early termination version)
 *
 *  Parameters: const uint64_t x, const uint64_t y, const uint64_t z 
 *   
 *
 *  Return: uint64_t
 *----------------------------------------------------------------------------*/
uint64_t m3D_e_LUT_ETU64(const uint64_t x, const uint64_t y, const uint64_t z) 
{
        uint64_t answer_x = (uint64_t)compute3D_ET_LUT_encodeU64(x, &Morton3D_encode_x_256);
        uint64_t answer_y = (uint64_t)compute3D_ET_LUT_encodeU64(y, &Morton3D_encode_x_256);
        uint64_t answer_z = (uint64_t)compute3D_ET_LUT_encodeU64(z, &Morton3D_encode_x_256);
        return (answer_z << 2u) | (answer_y << 1u) | answer_x;
}

/*-----------------------------------------------------------------------------
 *  m3D_e_LUT_ET_dVectr:  LookUpTable (LUT) (Early termination version)
 *
 *  Parameters: const dVectr vec
 *   
 *
 *  Return: float64_t
 *----------------------------------------------------------------------------*/
float64_t m3D_e_LUT_ET_dVectr(const dVectr vec) 
{
        uint32_t answer_x = compute3D_ET_LUT_encodeU32(((uint32_t)vec.x), &Morton3D_encode_x_256);
        uint32_t answer_y = compute3D_ET_LUT_encodeU32(((uint32_t)vec.y), &Morton3D_encode_y_256);
        uint32_t answer_z = compute3D_ET_LUT_encodeU32(((uint32_t)vec.z), &Morton3D_encode_z_256);        
        return ((float64_t)((answer_z << 2) | (answer_y << 1) | answer_x));
}

// HELPER METHOD: Magic bits encoding (helper method)
/*-----------------------------------------------------------------------------
 *  morton3D_SplitBy3bitsU32:  Magic bits encoding (helper method)
 *
 *  Parameters: const uint32_t a
 *   
 *
 *  Return: uint32_t
 *----------------------------------------------------------------------------*/
uint32_t morton3D_SplitBy3bitsU32(const uint32_t a) 
{                               
        uint32_t* masks = &magicbit3D_masks32;
        uint32_t x = a;
        x = x & masks[0u];
        x = (x | x << 16u) & masks[1u];
        x = (x | x << 8u)  & masks[2u];
        x = (x | x << 4u)  & masks[3u];
        x = (x | x << 2u)  & masks[4u];
        return x;
}

/*-----------------------------------------------------------------------------
 *  morton3D_SplitBy3bitsU64:  Magic bits encoding (helper method)
 *
 *  Parameters: const uint64_t a
 *   
 *
 *  Return: uint64_t
 *----------------------------------------------------------------------------*/
uint64_t morton3D_SplitBy3bitsU64(const uint64_t a) 
{
        uint64_t* masks = &magicbit3D_masks64;
        uint64_t x = a;
        x = x & masks[0u];
        x = (x | x << 16u) & masks[1u];
        x = (x | x << 8u)  & masks[2u];
        x = (x | x << 4u)  & masks[3u];
        x = (x | x << 2u)  & masks[4u];
        return x;
}

// ENCODE 3D Morton code : Magic bits method
// This method uses certain bit patterns (magic bits) to split bits in the coordinates
/*-----------------------------------------------------------------------------
 *  m3D_e_magicbitsU32:  Magic bits method
 *
 *  Parameters: const uint32_t x, const uint32_t y, const uint32_t z
 *   
 *
 *  Return: uint32_t
 *----------------------------------------------------------------------------*/
uint32_t m3D_e_magicbitsU32(const uint32_t x, const uint32_t y, const uint32_t z)
{
        return morton3D_SplitBy3bitsU32(x) | (morton3D_SplitBy3bitsU32(y) << 1u) | (morton3D_SplitBy3bitsU32(z) << 2u);
}

/*-----------------------------------------------------------------------------
 *  m3D_e_magicbitsU64:  Magic bits method
 *
 *  Parameters: const uint64_t x, const uint64_t y, const uint64_t z
 *   
 *
 *  Return: uint64_t
 *----------------------------------------------------------------------------*/
uint64_t m3D_e_magicbitsU64(const uint64_t x, const uint64_t y, const uint64_t z)
{
        return morton3D_SplitBy3bitsU64(x) | (morton3D_SplitBy3bitsU64(y) << 1u) | (morton3D_SplitBy3bitsU64(z) << 2u);
}

/*-----------------------------------------------------------------------------
 *  m3D_e_magicbitsMortonVec:  Magic bits method
 *
 *  Parameters: const MORTON_3DVectr_t mVec, MORTON3DVectr_Type_e Typ
 *   
 *
 *  Return: uint64_t
 *----------------------------------------------------------------------------*/
uint64_t m3D_e_magicbitsMortonVec(const MORTON_3DVectr_t mVec, MORTON3DVectr_Type_e Typ)
{
        return (Typ == MORTON_3DVectr_U64) ? (morton3D_SplitBy3bitsU64(mVec.x.u64_morton) | (morton3D_SplitBy3bitsU64(mVec.y.u64_morton) << 1u) | (morton3D_SplitBy3bitsU64(mVec.z.u64_morton) << 2u)) : (morton3D_SplitBy3bitsU32(mVec.x.u32_morton) | (morton3D_SplitBy3bitsU32(mVec.y.u32_morton) << 1u) | (morton3D_SplitBy3bitsU32(mVec.z.u32_morton) << 2u));
}

/*-----------------------------------------------------------------------------
 *  m3D_e_forU32:  morton 3d
 *
 *  Parameters: const uint32_t x, const uint32_t y, const uint32_t z
 *   
 *
 *  Return: uint32_t
 *----------------------------------------------------------------------------*/
uint32_t m3D_e_forU32(const uint32_t x, const uint32_t y, const uint32_t z)
{
    uint32_t answer = 0ul;
    uint32_t checkbits = (uint32_t)(floor((sizeof(uint32_t) * 8.0f / 3.0f)));
    uint32_t i;
    uint32_t mshifted,shift;
        
    for (i = 0ul; i < checkbits; ++i) 
    {
                mshifted= (uint32_t)(1ul) << i; // Here we need to cast 0x1 to 64bits, otherwise there is a bug when morton code is larger than 32 bits
                shift = 2ul * i; // because you have to shift back i and forth 3*i
                answer |= ((x & mshifted) << shift)        | ((y & mshifted) << (shift + 1ul))        | ((z & mshifted) << (shift + 2ul));
    }
    return answer;
}

/*-----------------------------------------------------------------------------
 *  m3D_e_forU64:  morton 3d
 *
 *  Parameters: const uint64_t x, const uint64_t y, const uint64_t z
 *   
 *
 *  Return: uint64_t
 *----------------------------------------------------------------------------*/
uint64_t m3D_e_forU64(const uint64_t x, const uint64_t y, const uint64_t z)
{
    uint64_t answer = 0ull;
    uint64_t checkbits = (uint64_t)(floor((sizeof(uint64_t) * 8.0f / 3.0f)));
    uint64_t i;
    uint64_t mshifted,shift;
        
    for (i = 0ull; i < checkbits; ++i) 
    {
                mshifted= (uint64_t)(1ull) << i;                                // Here we need to cast 0x1 to 64bits, otherwise there is a bug when morton code is larger than 32 bits
                shift = 2ull * i;                                               // because you have to shift back i and forth 3*i
                answer |= ((x & mshifted) << shift) | ((y & mshifted) << (shift + 1ull)) | ((z & mshifted) << (shift + 2ull));
    }
    return answer;
}

/*-----------------------------------------------------------------------------
 *  m3D_e_fordVectr:  morton 3d
 *
 *  Parameters: const dVectr vec
 *   
 *
 *  Return: uint64_t
 *----------------------------------------------------------------------------*/
uint64_t m3D_e_fordVectr(const dVectr vec)
{
    uint64_t answer = 0ull;
    uint64_t checkbits = (uint64_t)(floor((sizeof(uint64_t) * 8.0f / 3.0f)));
    uint64_t i;
    uint64_t mshifted,shift;
        
    for (i = 0ull; i < checkbits; ++i) 
    {
                mshifted= (uint64_t)(1ull) << i; // Here we need to cast 0x1 to 64bits, otherwise there is a bug when morton code is larger than 32 bits
                shift = 2ull * i; // because you have to shift back i and forth 3*i
                answer |= ((((uint64_t) vec.x) & mshifted) << shift)        | ((((uint64_t)vec.y) & mshifted) << (shift + 1ull))        | ((((uint64_t)vec.z) & mshifted) << (shift + 2ull));
     }
     return answer;
}

/*-----------------------------------------------------------------------------
 *  m3D_e_forMortonVec:  morton 3d
 *
 *  Parameters: const MORTON_3DVectr_t mVec, MORTON3DVectr_Type_e Typ
 *   
 *
 *  Return: uint64_t
 *----------------------------------------------------------------------------*/
uint64_t m3D_e_forMortonVec(const MORTON_3DVectr_t mVec, MORTON3DVectr_Type_e Typ)
{
        return (Typ == MORTON_3DVectr_U64) ? m3D_e_forU64(mVec.x.u64_morton,mVec.y.u64_morton,mVec.z.u64_morton) : m3D_e_forU32(mVec.x.u32_morton,mVec.y.u32_morton,mVec.z.u32_morton);
}

// ENCODE 3D Morton code : For loop (Early termination version)
// In case of the for loop, figuring out when to stop early has huge benefits.
/*-----------------------------------------------------------------------------
 *  m3D_e_for_ET_U64:  3D Morton code : For loop (Early termination version)
 *
 *  Parameters: const uint64_t x, const uint64_t y, const uint64_t z
 *   
 *
 *  Return: uint64_t
 *----------------------------------------------------------------------------*/
uint64_t m3D_e_for_ET_U64(const uint64_t x, const uint64_t y, const uint64_t z) 
{
    uint64_t answer = 0ull;
    int8_t x_max = 0ull, y_max = 0ull, z_max = 0ull;
    uint64_t checkbits = (uint64_t)(floor((sizeof(uint64_t) * 8.0f / 3.0f)));
    uint64_t i;
    uint64_t m_shifted, shift;
        
    x_max = BitScanReverseU64(x, MORTON_FROM_RHS);
    y_max = BitScanReverseU64(y, MORTON_FROM_RHS);
    z_max = BitScanReverseU64(z, MORTON_FROM_RHS);
    if ((x_max < 0) || ((y_max < 0) || (z_max < 0)))
    {
        checkbits = ((uint64_t)FMIN(checkbits, (uint64_t) (FMAX(z_max, FMAX(x_max, y_max)) + (uint64_t) 1ull)));
        for (i = 0; i < checkbits; ++i) 
        {
                    m_shifted = (uint64_t)(1ull) << i;                          // Here we need to cast 0x1 to 64bits, otherwise there is a bug when morton code is larger than 32 bits
                    shift = 2ull * i;
                    answer |= ((x & m_shifted) << shift) | ((y & m_shifted) << (shift + 1ull)) | ((z & m_shifted) << (shift + 2ull));
         }
    }
    return answer;
}

// ENCODE 3D Morton code : For loop (Early termination version)
// In case of the for loop, figuring out when to stop early has huge benefits.
/*-----------------------------------------------------------------------------
 *  m3D_e_for_ET_U32:  3D Morton code : For loop (Early termination version)
 *
 *  Parameters: const uint32_t x, const uint32_t y, const uint32_t z
 *   
 *
 *  Return: uint32_t
 *----------------------------------------------------------------------------*/
uint32_t m3D_e_for_ET_U32(const uint32_t x, const uint32_t y, const uint32_t z) 
{
    uint32_t answer = 0ul;
    int8_t x_max = 0ull, y_max = 0ull, z_max = 0ull;
    uint32_t checkbits = (uint32_t)(floor((sizeof(uint32_t) * 8.0f / 3.0f)));
    uint32_t i;
    uint32_t m_shifted, shift;
        
    x_max = BitScanReverseU32(x, MORTON_FROM_RHS);
    y_max = BitScanReverseU32(y, MORTON_FROM_RHS);
    z_max = BitScanReverseU32(z, MORTON_FROM_RHS);
    if ((x_max < 0) || ((y_max < 0) || (z_max < 0)))
    {
        checkbits = ((uint32_t)FMIN(checkbits, (uint32_t) (FMAX(z_max, FMAX(x_max, y_max)) + (uint32_t) 1ul)));
        for (i = 0; i < checkbits; ++i) 
        {
                    m_shifted = (uint32_t)(1ul) << i; // Here we need to cast 0x1 to 64bits, otherwise there is a bug when morton code is larger than 32 bits
                    shift = 2ul * i;
                    answer |= ((x & m_shifted) << shift) | ((y & m_shifted) << (shift + 1ul)) | ((z & m_shifted) << (shift + 2ul));
         }
    }
    return answer;
}

// ENCODE 3D Morton code : For loop (Early termination version)
// In case of the for loop, figuring out when to stop early has huge benefits.
/*-----------------------------------------------------------------------------
 *  m3D_e_for_ET_dVectr:  3D Morton code : For loop (Early termination version)
 *
 *  Parameters: const dVectr vec
 *   
 *
 *  Return: uint64_t
 *----------------------------------------------------------------------------*/
uint64_t m3D_e_for_ET_dVectr(const dVectr vec) 
{
    uint64_t answer = 0ull;
    int8_t x_max = 0ull, y_max = 0ull, z_max = 0ull;
    uint64_t checkbits = (uint64_t)(floor((sizeof(uint64_t) * 8.0f / 3.0f)));
    uint64_t i;
    uint64_t m_shifted, shift;
        
    x_max = BitScanReverseU64(((uint64_t)vec.x), MORTON_FROM_RHS);
    y_max = BitScanReverseU64(((uint64_t)vec.y), MORTON_FROM_RHS);
    z_max = BitScanReverseU64(((uint64_t)vec.z), MORTON_FROM_RHS);
    if ((x_max < 0) || ((y_max < 0) || (z_max < 0)))
    {
        checkbits = ((uint64_t)FMIN(checkbits, (uint64_t) (FMAX(z_max, FMAX(x_max, y_max)) + (uint64_t) 1ull)));
        for (i = 0; i < checkbits; ++i) 
        {
                    m_shifted = (uint64_t)(1ull) << i; // Here we need to cast 0x1 to 64bits, otherwise there is a bug when morton code is larger than 32 bits
                    shift = 2ull * i;
                    answer |= ((((uint64_t)vec.x) & m_shifted) << shift) | ((((uint64_t)vec.y) & m_shifted) << (shift + 1ull))        | ((((uint64_t)vec.z) & m_shifted) << (shift + 2ull));
         }
    }
    return answer;
}

/*-----------------------------------------------------------------------------
 *  m3D_e_for_ET_MortonVec:  3D Morton code : For loop (Early termination version)
 *
 *  Parameters: const MORTON_3DVectr_t mVec, MORTON3DVectr_Type_e Typ
 *   
 *
 *  Return: uint64_t
 *----------------------------------------------------------------------------*/
uint64_t m3D_e_for_ET_MortonVec(const MORTON_3DVectr_t mVec, MORTON3DVectr_Type_e Typ)
{
        return (Typ == MORTON_3DVectr_U64) ? m3D_e_for_ET_U64(mVec.x.u64_morton,mVec.y.u64_morton,mVec.z.u64_morton) : m3D_e_for_ET_U32(mVec.x.u32_morton,mVec.y.u32_morton,mVec.z.u32_morton);
}

/*-----------------------------------------------------------------------------
 *  morton3D_DecodeCoord_LUT256_U64:  3D Morton code : For loop (Early termination version)
 *
 *  Parameters: const uint64_t m, const uint8_t *LUT, const uint16_t startshift
 *   
 *
 *  Return: uint64_t
 *----------------------------------------------------------------------------*/
uint64_t morton3D_DecodeCoord_LUT256_U64(const uint64_t m, const uint8_t *LUT, const uint16_t startshift) 
{
    uint64_t a = 0ull;
    uint64_t NINEBITMASK = 0x000001ffull;
    uint16_t loops = 7u; // ceil for 32bit, floor for 64bit
    uint16_t i;
        
    for (i = 0u; i < loops; ++i)
    {
         a |= (LUT[(m >> ((i * 9u) + startshift)) & NINEBITMASK] << (3u * i));
    }
    return ((uint64_t)(a));
}

/*-----------------------------------------------------------------------------
 *  morton3D_DecodeCoord_LUT256_U32:  3D Morton code : For loop (Early termination version)
 *
 *  Parameters: const uint32_t m, const uint8_t *LUT, const uint16_t startshift
 *   
 *
 *  Return: uint32_t
 *----------------------------------------------------------------------------*/
uint32_t morton3D_DecodeCoord_LUT256_U32(const uint32_t m, const uint8_t *LUT, const uint16_t startshift) 
{
   uint64_t a = 0ul;
   uint64_t NINEBITMASK = 0x000001ffull;
   uint16_t loops = 7u; // ceil for 32bit, floor for 64bit
   uint16_t i;
        
    for (i = 0u; i < loops; ++i)
    {
                a |= (LUT[(m >> ((i * 9u) + startshift)) & NINEBITMASK] << (3u * i));
    }
    return ((uint32_t)(a));
}


// DECODE 3D Morton code : Shifted LUT
/*-----------------------------------------------------------------------------
 *  m3D_d_sLUT_U64:  DECODE 3D Morton code : Shifted LUT
 *
 *  Parameters: const uint64_t m, uint64_t *x, uint64_t *y, uint64_t *z
 *   
 *
 *  Return: void
 *----------------------------------------------------------------------------*/
void m3D_d_sLUT_U64(const uint64_t m, uint64_t *x, uint64_t *y, uint64_t *z) 
{
        *x = morton3D_DecodeCoord_LUT256_U64(m, &Morton3D_decode_x_512, 0);
        *y = morton3D_DecodeCoord_LUT256_U64(m, &Morton3D_decode_y_512, 0);
        *z = morton3D_DecodeCoord_LUT256_U64(m, &Morton3D_decode_z_512, 0);
}

/*-----------------------------------------------------------------------------
 *  m3D_d_sLUT_U32:  DECODE 3D Morton code : Shifted LUT
 *
 *  Parameters: const uint32_t m, uint32_t *x, uint32_t *y, uint32_t *z
 *   
 *
 *  Return: void
 *----------------------------------------------------------------------------*/
void m3D_d_sLUT_U32(const uint32_t m, uint32_t *x, uint32_t *y, uint32_t *z) 
{
        *x = morton3D_DecodeCoord_LUT256_U32(m, &Morton3D_decode_x_512, 0);
        *y = morton3D_DecodeCoord_LUT256_U32(m, &Morton3D_decode_y_512, 0);
        *z = morton3D_DecodeCoord_LUT256_U32(m, &Morton3D_decode_z_512, 0);
}

/*-----------------------------------------------------------------------------
 *  m3D_d_sLUT_dVectr:  DECODE 3D Morton code : Shifted LUT
 *
 *  Parameters: const uint64_t m, dVectr *vec
 *   
 *
 *  Return: void
 *----------------------------------------------------------------------------*/
void m3D_d_sLUT_dVectr(const uint64_t m, dVectr *vec) 
{
        vec->x = ((float64_t) morton3D_DecodeCoord_LUT256_U64(m, &Morton3D_decode_x_512, 0));
        vec->y = ((float64_t) morton3D_DecodeCoord_LUT256_U64(m, &Morton3D_decode_y_512, 0));
        vec->z = ((float64_t) morton3D_DecodeCoord_LUT256_U64(m, &Morton3D_decode_z_512, 0));
}

/*-----------------------------------------------------------------------------
 *  m3D_d_sLUT_MortonVec:  DECODE 3D Morton code : Shifted LUT
 *
 *  Parameters: const morton_type_u m, MORTON_3DVectr_t *mVec, MORTON3DVectr_Type_e Typ
 *   
 *
 *  Return: void
 *----------------------------------------------------------------------------*/
void m3D_d_sLUT_MortonVec(const morton_type_u m, MORTON_3DVectr_t *mVec, MORTON3DVectr_Type_e Typ)
{
    if (Typ == MORTON_3DVectr_U64) 
    {
       m3D_d_sLUT_U64(m.u64_morton, &mVec->x.u64_morton, &mVec->y.u64_morton, &mVec->z.u64_morton);
    }
    else
    {
       m3D_d_sLUT_U32(m.u32_morton, &mVec->x.u32_morton, &mVec->y.u32_morton, &mVec->z.u32_morton);                
    }
}

// DECODE 3D Morton code : LUT
/*-----------------------------------------------------------------------------
 *  m3D_d_LUT_U64:  DECODE 3D Morton code : LUT
 *
 *  Parameters: const uint64_t m, uint64_t *x, uint64_t *y, uint64_t *z
 *   
 *
 *  Return: void
 *----------------------------------------------------------------------------*/
void m3D_d_LUT_U64(const uint64_t m, uint64_t *x, uint64_t *y, uint64_t *z) 
{
        *x = morton3D_DecodeCoord_LUT256_U64(m, &Morton3D_decode_x_512, 0);
        *y = morton3D_DecodeCoord_LUT256_U64(m, &Morton3D_decode_x_512, 1);
        *z = morton3D_DecodeCoord_LUT256_U64(m, &Morton3D_decode_x_512, 2);
}

/*-----------------------------------------------------------------------------
 *  m3D_d_LUT_U32:  DECODE 3D Morton code : LUT
 *
 *  Parameters: const uint32_t m, uint32_t *x, uint32_t *y, uint32_t *z
 *   
 *
 *  Return: void
 *----------------------------------------------------------------------------*/
void m3D_d_LUT_U32(const uint32_t m, uint32_t *x, uint32_t *y, uint32_t *z) 
{
        *x = morton3D_DecodeCoord_LUT256_U32(m, &Morton3D_decode_x_512, 0);
        *y = morton3D_DecodeCoord_LUT256_U32(m, &Morton3D_decode_x_512, 1);
        *z = morton3D_DecodeCoord_LUT256_U32(m, &Morton3D_decode_x_512, 2);
}

/*-----------------------------------------------------------------------------
 *  m3D_d_LUT_dVectr:  DECODE 3D Morton code : LUT
 *
 *  Parameters: const uint64_t m, dVectr *vec
 *   
 *
 *  Return: void
 *----------------------------------------------------------------------------*/
void m3D_d_LUT_dVectr(const uint64_t m, dVectr *vec) 
{
        vec->x = ((float64_t) morton3D_DecodeCoord_LUT256_U64(m, &Morton3D_decode_x_512, 0));
        vec->y = ((float64_t) morton3D_DecodeCoord_LUT256_U64(m, &Morton3D_decode_y_512, 1));
        vec->z = ((float64_t) morton3D_DecodeCoord_LUT256_U64(m, &Morton3D_decode_z_512, 2));
}

/*-----------------------------------------------------------------------------
 *  m3D_d_LUT_MortonVec:  DECODE 3D Morton code : LUT
 *
 *  Parameters: const morton_type_u m, MORTON_3DVectr_t *mVec, MORTON3DVectr_Type_e Typ
 *   
 *
 *  Return: void 
 *----------------------------------------------------------------------------*/
void m3D_d_LUT_MortonVec(const morton_type_u m, MORTON_3DVectr_t *mVec, MORTON3DVectr_Type_e Typ)
{
    if (Typ == MORTON_3DVectr_U64) 
    {
       m3D_d_LUT_U64(m.u64_morton, &mVec->x.u64_morton, &mVec->y.u64_morton, &mVec->z.u64_morton);
    }
    else
    {
       m3D_d_LUT_U32(m.u32_morton, &mVec->x.u32_morton, &mVec->y.u32_morton, &mVec->z.u32_morton);                
    }
}

// DECODE 3D Morton code : Shifted LUT (Early termination version)
/*-----------------------------------------------------------------------------
 *  m3D_d_sLUT_ET_U64:  DECODE 3D Morton code : Shifted LUT (Early termination version)
 *
 *  Parameters: const uint64_t m, uint64_t *x, uint64_t *y, uint64_t *z
 *   
 *
 *  Return: int8_t 
 *----------------------------------------------------------------------------*/
int8_t m3D_d_sLUT_ET_U64(const uint64_t m, uint64_t *x, uint64_t *y, uint64_t *z)
{
    uint64_t NINEBITMASK = 0x000001ffull;
    uint64_t firstbit_location = 0ull;
    uint16_t i = 0u;
    uint16_t shiftback = 0u;
    uint64_t m_shifted;
    *x = 0ull; *y = 0ull; *z = 0ull;
        
    firstbit_location = BitScanReverseU64(m, MORTON_FROM_RHS);        
    if (firstbit_location <= -1) { return firstbit_location; }
        
    while (firstbit_location >= i) 
    {
          m_shifted = (m >> i) & NINEBITMASK;
          *x |= Morton3D_decode_x_512[m_shifted] << shiftback;
          *y |= Morton3D_decode_y_512[m_shifted] << shiftback;
          *z |= Morton3D_decode_z_512[m_shifted] << shiftback;
          shiftback += 3u;
          i += 9u;
    }
    return 1;
}

/*-----------------------------------------------------------------------------
 *  m3D_d_sLUT_ET_U32:  DECODE 3D Morton code : Shifted LUT (Early termination version)
 *
 *  Parameters: const uint32_t m, uint32_t *x, uint32_t *y, uint32_t *z
 *   
 *
 *  Return: int8_t 
 *----------------------------------------------------------------------------*/
int8_t m3D_d_sLUT_ET_U32(const uint32_t m, uint32_t *x, uint32_t *y, uint32_t *z)
{
    uint32_t NINEBITMASK = 0x000001fful;
    uint32_t firstbit_location = 0ul;
    uint16_t i = 0u;
    uint16_t shiftback = 0u;
    uint32_t m_shifted;
    *x = 0ul; *y = 0ul; *z = 0ul;
        
    firstbit_location = BitScanReverseU32(m, MORTON_FROM_RHS);        
    if (firstbit_location <= -1) { return firstbit_location; }
        
    while (firstbit_location >= i) 
    {
        m_shifted = (m >> i) & NINEBITMASK;
        *x |= Morton3D_decode_x_512[m_shifted] << shiftback;
        *y |= Morton3D_decode_y_512[m_shifted] << shiftback;
        *z |= Morton3D_decode_z_512[m_shifted] << shiftback;
        shiftback += 3u;
        i += 9u;
     }
     return 1;
}

/*-----------------------------------------------------------------------------
 *  m3D_d_sLUT_ET_dVectr:  DECODE 3D Morton code : Shifted LUT (Early termination version)
 *
 *  Parameters: const uint64_t m, dVectr *vec
 *   
 *
 *  Return: int8_t 
 *----------------------------------------------------------------------------*/
int8_t m3D_d_sLUT_ET_dVectr(const uint64_t m, dVectr *vec)
{
        uint64_t NINEBITMASK = 0x000001ffull;
        uint64_t firstbit_location = 0ull;
        uint16_t i = 0u;
        uint16_t shiftback = 0u;
        uint64_t m_shifted;
        uint64_t x = 0ull; 
        uint64_t y = 0ull; 
        uint64_t z = 0ull;
        
        firstbit_location = BitScanReverseU64(m, MORTON_FROM_RHS);        
        if (firstbit_location <= -1) { return firstbit_location; }
        
        while (firstbit_location >= i) 
        {
                m_shifted = (m >> i) & NINEBITMASK;
                x |= Morton3D_decode_x_512[m_shifted] << shiftback;
                y |= Morton3D_decode_y_512[m_shifted] << shiftback;
                z |= Morton3D_decode_z_512[m_shifted] << shiftback;
                shiftback += 3u;
                i += 9u;
        }
        vec->x = ((float64_t)x);
        vec->y = ((float64_t)y);
        vec->z = ((float64_t)z);        
        return 1;
}

/*-----------------------------------------------------------------------------
 *  m3D_d_LUT_ET_U64:  DECODE 3D Morton code : Shifted LUT (Early termination version)
 *
 *  Parameters: const uint64_t m, uint64_t *x, uint64_t *y, uint64_t *z
 *   
 *
 *  Return: int8_t 
 *----------------------------------------------------------------------------*/
int8_t m3D_d_LUT_ET_U64(const uint64_t m, uint64_t *x, uint64_t *y, uint64_t *z)
{
    uint64_t NINEBITMASK = 0x000001ffull;
    uint64_t firstbit_location = 0ull;
    uint16_t i = 0u;
    uint16_t shiftback = 0u;

    *x = 0ull; *y = 0ull; *z = 0ull;
        
    firstbit_location = BitScanReverseU64(m, MORTON_FROM_RHS);        
    if (firstbit_location <= -1) { return firstbit_location; }
        
    while (firstbit_location >= i) 
    {
                *x = *x | Morton3D_decode_x_512[(m >> i) & NINEBITMASK] << shiftback;
                *y = *y | Morton3D_decode_x_512[(m >> (i+1u)) & NINEBITMASK] << shiftback;
                *z = *z | Morton3D_decode_x_512[(m >> (i+2u)) & NINEBITMASK] << shiftback;
                i += 9u;
                shiftback += 3u;
     }
     return 1;
}

/*-----------------------------------------------------------------------------
 *  m3D_d_LUT_ET_U32:  DECODE 3D Morton code : Shifted LUT (Early termination version)
 *
 *  Parameters: const uint32_t m, uint32_t *x, uint32_t *y, uint32_t *z
 *   
 *
 *  Return: int8_t 
 *----------------------------------------------------------------------------*/
int8_t m3D_d_LUT_ET_U32(const uint32_t m, uint32_t *x, uint32_t *y, uint32_t *z)
{
    uint32_t NINEBITMASK = 0x000001fful;
    uint32_t firstbit_location = 0ul;
    uint16_t i = 0u;
    uint16_t shiftback = 0u;

    *x = 0ul; *y = 0ul; *z = 0ul;
        
    firstbit_location = BitScanReverseU32(m, MORTON_FROM_RHS);        
    if (firstbit_location <= -1) { return firstbit_location; }
        
    while (firstbit_location >= i) 
    {
                *x = *x | Morton3D_decode_x_512[(m >> i) & NINEBITMASK] << shiftback;
                *y = *y | Morton3D_decode_x_512[(m >> (i+1u)) & NINEBITMASK] << shiftback;
                *z = *z | Morton3D_decode_x_512[(m >> (i+2u)) & NINEBITMASK] << shiftback;
                i += 9u;
                shiftback += 3u;
    }
    return 1;
}

// DECODE 3D Morton code : LUT (Early termination version)
/*-----------------------------------------------------------------------------
 *  m3D_d_LUT_ET_dVectr:  DECODE 3D Morton code : LUT (Early termination version)
 *
 *  Parameters: const uint64_t m, dVectr *vec
 *   
 *
 *  Return: int8_t 
 *----------------------------------------------------------------------------*/
int8_t m3D_d_LUT_ET_dVectr(const uint64_t m, dVectr *vec)
{
        uint64_t NINEBITMASK = 0x000001ffull;
        uint64_t firstbit_location = 0ull;
        uint16_t i = 0u;
        uint16_t shiftback = 0u;
        uint64_t x = 0ull; 
        uint64_t y = 0ull; 
        uint64_t z = 0ull;
        
        if (vec == NULL) return -1;
        firstbit_location = BitScanReverseU64(m, MORTON_FROM_RHS);        
        if (firstbit_location <= -1) { return firstbit_location; }
        
        while (i <= firstbit_location) 
        {
                x = x | Morton3D_decode_x_512[(m >> i) & NINEBITMASK] << shiftback;
                y = y | Morton3D_decode_x_512[(m >> (i+1u)) & NINEBITMASK] << shiftback;
                z = z | Morton3D_decode_x_512[(m >> (i+2u)) & NINEBITMASK] << shiftback;
                i += 9u;
                shiftback += 3u;
        }
        vec->x = ((float64_t)x);
        vec->y = ((float64_t)y);
        vec->z = ((float64_t)z);
        return 1;
}

// HELPER METHOD for Magic bits decoding
/*-----------------------------------------------------------------------------
 *  morton3D_GetThirdBitsU32:  HELPER METHOD for Magic bits decoding
 *
 *  Parameters: const uint32_t m
 *   
 *
 *  Return: uint32_t 
 *----------------------------------------------------------------------------*/
uint32_t morton3D_GetThirdBitsU32(const uint32_t m) 
{
        uint32_t* masks = &magicbit3D_masks32; 
        uint32_t x = m & masks[4u];
        x = (x ^ (x >> 2u)) & masks[3u];
        x = (x ^ (x >> 4u)) & masks[2u];
        x = (x ^ (x >> 8u)) & masks[1u];
        x = (x ^ (x >> 16u)) & masks[0u];
        return x;
}

/*-----------------------------------------------------------------------------
 *  morton3D_GetThirdBitsU64:  HELPER METHOD for Magic bits decoding
 *
 *  Parameters: const uint32_t m
 *   
 *
 *  Return: uint64_t 
 *----------------------------------------------------------------------------*/
uint64_t morton3D_GetThirdBitsU64(const uint64_t m) 
{
        uint64_t* masks = &magicbit3D_masks64; 
        uint64_t x = m & masks[4u];
        x = (x ^ (x >> 2u)) & masks[3u];
        x = (x ^ (x >> 4u)) & masks[2u];
        x = (x ^ (x >> 8u)) & masks[1u];
        x = (x ^ (x >> 16u)) & masks[0u];
        return x;
}

// DECODE 3D Morton code : Magic bits
// This method splits the morton codes bits by using certain patterns (magic bits)
/*-----------------------------------------------------------------------------
 *  m3D_d_magicbitsU64: DECODE 3D Morton code : Magic bits
 *
 *  Parameters: const uint64_t m, uint64_t *x, uint64_t *y, uint64_t *z
 *   
 *
 *  Return: void
 *----------------------------------------------------------------------------*/
void m3D_d_magicbitsU64(const uint64_t m, uint64_t *x, uint64_t *y, uint64_t *z)
{
        *x = morton3D_GetThirdBitsU64(m);
        *y = morton3D_GetThirdBitsU64(m >> 1u);
        *z = morton3D_GetThirdBitsU64(m >> 2u);
}

/*-----------------------------------------------------------------------------
 *  m3D_d_magicbitsU32: DECODE 3D Morton code : Magic bits
 *
 *  Parameters: const uint32_t m, uint32_t *x, uint32_t *y, uint32_t *z
 *   
 *
 *  Return: void
 *----------------------------------------------------------------------------*/
void m3D_d_magicbitsU32(const uint32_t m, uint32_t *x, uint32_t *y, uint32_t *z)
{
        *x = morton3D_GetThirdBitsU32(m);
        *y = morton3D_GetThirdBitsU32(m >> 1u);
        *z = morton3D_GetThirdBitsU32(m >> 2u);
}

/*-----------------------------------------------------------------------------
 *  m3D_d_magicbitsdVectr: DECODE 3D Morton code : Magic bits
 *
 *  Parameters: const uint64_t m, dVectr *vec
 *   
 *
 *  Return: void
 *----------------------------------------------------------------------------*/
void m3D_d_magicbitsdVectr(const uint64_t m, dVectr *vec)
{
        vec->x = ((float64_t) morton3D_GetThirdBitsU64(m));
        vec->y = ((float64_t) morton3D_GetThirdBitsU64(m >> 1u));
        vec->z = ((float64_t) morton3D_GetThirdBitsU64(m >> 2u));
}

// DECODE 3D Morton code : For loop
/*-----------------------------------------------------------------------------
 *  m3D_d_forU64: DECODE 3D Morton code : For loop
 *
 *  Parameters: const uint64_t m, uint64_t *x, uint64_t *y, uint64_t *z
 *   
 *
 *  Return: void
 *----------------------------------------------------------------------------*/
void m3D_d_forU64(const uint64_t m, uint64_t *x, uint64_t *y, uint64_t *z)
{
    uint16_t checkbits = ((uint16_t)(floor((sizeof(m) * 8.0f / 3.0f))));
    uint16_t i,shift_selector,shiftback;
    uint64_t selector;
        
    *x = 0ull; *y = 0ull; *z = 0ull;
        
    for (i = 0u; i <= checkbits; ++i) 
    {
          selector = 1ull;
          shift_selector = 3u * i;
          shiftback = 2u * i;
          *x |= (m & (selector << shift_selector)) >> (shiftback);
          *y |= (m & (selector << (shift_selector + 1u))) >> (shiftback + 1u);
          *z |= (m & (selector << (shift_selector + 2u))) >> (shiftback + 2u);
     }
}

/*-----------------------------------------------------------------------------
 *  m3D_d_forU32: DECODE 3D Morton code : For loop
 *
 *  Parameters: const uint32_t m, uint32_t *x, uint32_t *y, uint32_t *z
 *   
 *
 *  Return: void
 *----------------------------------------------------------------------------*/
void m3D_d_forU32(const uint32_t m, uint32_t *x, uint32_t *y, uint32_t *z)
{
    uint16_t checkbits = ((uint16_t)(floor((sizeof(m) * 8.0f / 3.0f))));
    uint16_t i,shift_selector,shiftback;
    uint32_t selector;
        
    *x = 0ul; *y = 0ul; *z = 0ul;
        
    for (i = 0u; i <= checkbits; ++i) 
    {
         selector = 1ull;
         shift_selector = 3u * i;
         shiftback = 2u * i;
         *x |= (m & (selector << shift_selector)) >> (shiftback);
         *y |= (m & (selector << (shift_selector + 1u))) >> (shiftback + 1u);
         *z |= (m & (selector << (shift_selector + 2u))) >> (shiftback + 2u);
    }
}

// DECODE 3D Morton code : For loop (Early termination version)
/*-----------------------------------------------------------------------------
 *  m3D_d_for_ETU32: DECODE 3D Morton code : For loop (Early termination version)
 *
 *  Parameters: const uint32_t m, uint32_t *x, uint32_t *y, uint32_t *z
 *   
 *
 *  Return: int8_t
 *----------------------------------------------------------------------------*/
int8_t m3D_d_for_ETU32(const uint32_t m, uint32_t *x, uint32_t *y, uint32_t *z) 
{
    float32_t defaultbits = floor((sizeof(uint32_t) * 8.0f / 3.0f));
    uint32_t firstbit_location = 0ul;
    uint16_t checkbits,i,shift_selector,shiftback;
    uint32_t selector;
        
    *x = 0ul; *y = 0ul; *z = 0ul;
        
    firstbit_location = BitScanReverseU64(m, MORTON_FROM_RHS);        
    if (firstbit_location <= -1) { return firstbit_location; }
        
    checkbits = ((uint16_t) (FMIN(defaultbits, (firstbit_location / 3.0f))));
    for (i = 0u; i <= checkbits; ++i) 
    {
         selector = 1ul;
         shift_selector = 3u * i;
         shiftback = 2u * i;
         *x |= (m & (selector << shift_selector)) >> (shiftback);
         *y |= (m & (selector << (shift_selector + 1u))) >> (shiftback + 1u);
         *z |= (m & (selector << (shift_selector + 2u))) >> (shiftback + 2u);
    }
    return 1;
}

/*-----------------------------------------------------------------------------
 *  m3D_d_for_ETU64: DECODE 3D Morton code : For loop (Early termination version)
 *
 *  Parameters: const uint64_t m, uint64_t *x, uint64_t *y, uint64_t *z
 *   
 *
 *  Return: int8_t
 *----------------------------------------------------------------------------*/
int8_t m3D_d_for_ETU64(const uint64_t m, uint64_t *x, uint64_t *y, uint64_t *z) 
{
    float64_t defaultbits = floor((sizeof(uint64_t) * 8.0f / 3.0f));
    uint64_t firstbit_location = 0ul;
    uint32_t checkbits,i,shift_selector,shiftback;
    uint64_t selector;
        
    *x = 0ul; *y = 0ul; *z = 0ul;
        
    firstbit_location = BitScanReverseU64(m, MORTON_FROM_RHS);        
    if (firstbit_location <= -1) { return firstbit_location; }
        
    checkbits = ((uint32_t) (FMIN(defaultbits, (firstbit_location / 3.0f))));
    for (i = 0u; i <= checkbits; ++i) 
    {
          selector = 1ul;
          shift_selector = 3u * i;
          shiftback = 2u * i;
          *x |= (m & (selector << shift_selector)) >> (shiftback);
          *y |= (m & (selector << (shift_selector + 1u))) >> (shiftback + 1u);
          *z |= (m & (selector << (shift_selector + 2u))) >> (shiftback + 2u);
    }
    return 1;
}

/*-----------------------------------------------------------------------------
 *  m3D_d_for_ETUdVectr: DECODE 3D Morton code : For loop (Early termination version)
 *
 *  Parameters: const uint64_t m, dVectr *rec
 *   
 *
 *  Return: int8_t
 *----------------------------------------------------------------------------*/
int8_t m3D_d_for_ETUdVectr(const uint64_t m, dVectr *rec) 
{
    float64_t defaultbits = floor((sizeof(uint64_t) * 8.0f / 3.0f));
    uint64_t firstbit_location = 0ul;
    uint32_t checkbits,i,shift_selector,shiftback;
    uint64_t selector;
        
    uint64_t x = 0ul; 
    uint64_t y = 0ul; 
    uint64_t z = 0ul;
        
    firstbit_location = BitScanReverseU64(m, MORTON_FROM_RHS);        
    if (firstbit_location <= -1) { return firstbit_location; }
        
    checkbits = ((uint32_t) (FMIN(defaultbits, (firstbit_location / 3.0f))));
    for (i = 0u; i <= checkbits; ++i) 
    {
        selector = 1ul;
        shift_selector = 3u * i;
        shiftback = 2u * i;
        x |= (m & (selector << shift_selector)) >> (shiftback);
        y |= (m & (selector << (shift_selector + 1u))) >> (shiftback + 1u);
        z |= (m & (selector << (shift_selector + 2u))) >> (shiftback + 2u);
    }
    rec->x = ((float64_t)x);
    rec->y = ((float64_t)y);
    rec->z = ((float64_t)z);    
    return 1;
}

#ifdef __cplusplus
}
#endif

#endif
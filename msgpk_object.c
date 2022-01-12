#include "definitions.h"

#if defined(USE_MSGPACK)                                                        /* if we included this in defintions */

#include <stdint.h>
#include "msgpk_object.h"
#include "msgpk_pack_template.h"

int16_t msgpack_pack_object(msgpack_packer* pk, msgpack_object d)
{
 /*   int16_t ret=1;
    msgpack_object_kv* kv;
    msgpack_object_kv* kvend;   */
    
    switch(d.type) {
    case MSGPACK_OBJECT_NIL:
    msgpack_pack_inline_func_nil(pk);
    return MSGPACK_OBJECT_NIL;
    /* return msgpack_pack_nil(pk); */
    break;

    case MSGPACK_OBJECT_BOOLEAN:
    if(d.via.boolean) 
    {
       msgpack_pack_inline_func_true(pk);
       /* return msgpack_pack_true(pk); */
    } 
    else 
    {
       msgpack_pack_inline_func_false(pk);
       /* return msgpack_pack_false(pk); */
    }
    return MSGPACK_OBJECT_BOOLEAN;
    break;
        
    case MSGPACK_OBJECT_POSITIVE_INTEGER:
    msgpack_pack_inline_func_fixint_uint64(pk, d.via.u64);
    return MSGPACK_OBJECT_POSITIVE_INTEGER;
    /* return msgpack_pack_uint64(pk, d.via.u64); */
    break;

    case MSGPACK_OBJECT_NEGATIVE_INTEGER:
    msgpack_pack_inline_func_fixint_int64(pk, d.via.i64);
    return MSGPACK_OBJECT_NEGATIVE_INTEGER;
    /* return msgpack_pack_int64(pk, d.via.i64); */
    break;

    case MSGPACK_OBJECT_FLOAT32:
    msgpack_pack_inline_func_float32(pk, (float)d.via.f64);
    return MSGPACK_OBJECT_FLOAT32;
    /* return msgpack_pack_float(pk, (float)d.via.f64); */
    break;

    case MSGPACK_OBJECT_FLOAT64:
    msgpack_pack_inline_func_float32(pk, (float)d.via.f64);
    return MSGPACK_OBJECT_FLOAT64;
    /* return msgpack_pack_double(pk, d.via.f64); */
    break;
    
    case MSGPACK_OBJECT_STR:
    {
        msgpack_pack_inline_func_str(pk, d.via.str.size);
        msgpack_pack_inline_func_str_body(pk, d.via.str.ptr, d.via.str.size);
        return MSGPACK_OBJECT_STR;
    }
    break;

    case MSGPACK_OBJECT_BIN:
    {
        msgpack_pack_inline_func_bin(pk, d.via.bin.size);
        msgpack_pack_inline_func_bin_body(pk, d.via.bin.ptr, d.via.bin.size);
        return MSGPACK_OBJECT_BIN;
     }
     break;

    case MSGPACK_OBJECT_EXT:
    {
         msgpack_pack_inline_func_ext(pk, d.via.ext.size, d.via.ext.type);
         msgpack_pack_inline_func_ext_body(pk, d.via.ext.ptr, d.via.ext.size);
         return MSGPACK_OBJECT_EXT;
     }

    case MSGPACK_OBJECT_ARRAY:
    {
        msgpack_pack_inline_func_array(pk, d.via.array.size);
        return MSGPACK_OBJECT_ARRAY;
            /* if(ret < 0) {
                return ret;
            }
            else {
                msgpack_object* o = d.via.array.ptr;
                msgpack_object* const oend = d.via.array.ptr + d.via.array.size;
                for(; o != oend; ++o) {
                    ret = msgpack_pack_object(pk, *o);
                    if(ret < 0) { return ret; }
                }

                return 0;
            }  */
     }
     break;

    case MSGPACK_OBJECT_MAP:
    {
         msgpack_pack_inline_func_map(pk, d.via.map.size);
/*         if(ret < 0)
         {
            return ret;
         }
         else
         {
             kv =  &d.via.map.ptr;
             kvend = kv + d.via.map.size;
             for(; kv != kvend; ++kv)
             {
                ret = msgpack_pack_object(pk, kv->key);
                if(ret < 0) { return ret; }
                ret = msgpack_pack_object(pk, kv->val);
                if(ret < 0) { return ret; }
             }
             return 0;
          }       */
          return MSGPACK_OBJECT_MAP;
    }
    break;
    
    default:
    break;
    
    }
    return -1;
}

int16_t msgpack_pack_object1(msgpack_packer* pk, msgpack_object *d)
{
    int16_t ret=1;
    msgpack_object_kv* kv;
    msgpack_object_kv* kvend;
    msgpack_object* o;
    msgpack_object* const oend;

    switch(d->type) {
    case MSGPACK_OBJECT_NIL:
    msgpack_pack_inline_func_nil(pk);
    return MSGPACK_OBJECT_NIL;
    /* return msgpack_pack_nil(pk); */
    break;

    case MSGPACK_OBJECT_BOOLEAN:
    if(d->via.boolean)
    {
       msgpack_pack_inline_func_true(pk);
       /* return msgpack_pack_true(pk); */
    }
    else
    {
       msgpack_pack_inline_func_false(pk);
       /* return msgpack_pack_false(pk); */
    }
    return MSGPACK_OBJECT_BOOLEAN;
    break;

    case MSGPACK_OBJECT_POSITIVE_INTEGER:
    msgpack_pack_inline_func_fixint_uint64(pk, d->via.u64);
    return MSGPACK_OBJECT_POSITIVE_INTEGER;
    /* return msgpack_pack_uint64(pk, d.via.u64); */
    break;

    case MSGPACK_OBJECT_NEGATIVE_INTEGER:
    msgpack_pack_inline_func_fixint_int64(pk, d->via.i64);
    return MSGPACK_OBJECT_NEGATIVE_INTEGER;
    /* return msgpack_pack_int64(pk, d.via.i64); */
    break;

    case MSGPACK_OBJECT_FLOAT32:
    msgpack_pack_inline_func_float32(pk, (float32_t)d->via.f64);
    return MSGPACK_OBJECT_FLOAT32;
    /* return msgpack_pack_float(pk, (float)d.via.f64); */
    break;

    case MSGPACK_OBJECT_FLOAT64:
    msgpack_pack_inline_func_float64(pk, (float64_t)d->via.f64);
    return MSGPACK_OBJECT_FLOAT64;
    /* return msgpack_pack_double(pk, d.via.f64); */
    break;

    case MSGPACK_OBJECT_STR:
    {
        msgpack_pack_inline_func_str(pk, d->via.str.size);
        msgpack_pack_inline_func_str_body(pk, d->via.str.ptr, d->via.str.size);
        return MSGPACK_OBJECT_STR;
    }
    break;

    case MSGPACK_OBJECT_BIN:
    {
        msgpack_pack_inline_func_bin(pk, d->via.bin.size);
        msgpack_pack_inline_func_bin_body(pk, d->via.bin.ptr, d->via.bin.size);
        return MSGPACK_OBJECT_BIN;
     }
     break;

    case MSGPACK_OBJECT_EXT:
    {
         msgpack_pack_inline_func_ext(pk, d->via.ext.size, d->via.ext.type);
         msgpack_pack_inline_func_ext_body(pk, d->via.ext.ptr, d->via.ext.size);
         return MSGPACK_OBJECT_EXT;
     }

    case MSGPACK_OBJECT_ARRAY:
    {
        msgpack_pack_inline_func_array(pk, d->via.array.size);
        if(ret < 0)
        {
            return ret;
        }
        else 
        {
            o = d->via.array.ptr;
            oend = d->via.array.ptr + d->via.array.size;
            for(; o != oend; ++o) 
            {
                ret = msgpack_pack_object(pk, *o);
                if(ret < 0) { return ret; }
            }
            return 0;
        }
        return MSGPACK_OBJECT_ARRAY;
     }
     break;

    case MSGPACK_OBJECT_MAP:
    {
         msgpack_pack_inline_func_map(pk, d->via.map.size);
         if(ret < 0)
         {
            return ret;
         }
         else
         {
             kv =  d->via.map.ptr;
             kvend = kv + d->via.map.size;
             for(; kv != kvend; ++kv)
             {
                ret = msgpack_pack_object(pk, kv->key);
                if(ret < 0) { return ret; }
                ret = msgpack_pack_object(pk, kv->val);
                if(ret < 0) { return ret; }
             }
             return 0;
          }
          return MSGPACK_OBJECT_MAP;
    }
    break;

    default:
    break;

    }
    return -1;
}
#endif                                                                          /* end message pack include */
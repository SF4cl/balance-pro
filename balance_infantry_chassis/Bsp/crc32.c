#include "crc32.h"

static uint32_t table[256];
 
 
//位逆转
static uint32_t bitrev(uint32_t input, int bw)
{
    int i;
    uint32_t var;
    var = 0;
    for(i=0; i<bw; i++)
    {
        if(input & 0x01)
        {
            var |= 1<<(bw - 1 - i);
        }
        input >>= 1;
    }
    return var;
}
 
 
//码表生成
void crc32_init(uint32_t poly)
{
    int i;
    int j;
    uint32_t c;
 
 
    poly = bitrev(poly, 32);
    for(i=0; i<256; i++)
    {
        c = i;
        for (j=0; j<8; j++)
        {
            c = (c & 1) ? (poly ^ (c >> 1)) : (c >> 1);
        }
        table[i] = c;
    }
}
 
 
//计算CRC
uint32_t crc32(void* input, int len)
{
    int i;
    uint8_t index;
    uint8_t *p;
		uint32_t crc=0xFFFFFFFF;
    p = (uint8_t*)input;
    for(i=0; i<len; i++)
    {
        index = (*p ^ crc);
        crc = (crc >> 8) ^ table[index];
        p++;
    }
    return crc;
}



uint32_t crc32_core(uint32_t* ptr, uint32_t len)
{
		uint32_t xbit=0,bits;
		uint32_t data = 0;
		uint32_t CRC32 = 0xFFFFFFFF;
   // const 
		uint32_t dwPolynomial = 0x04c11db7;
    for (uint32_t i = 0; i < len; i++)
    {
        
        data=ptr[i];bits=0;
        for(bits=0;bits<32;bits++)
        {
						xbit = 1<<(31-bits);
            if (CRC32 & 0x80000000)
            {
                CRC32 <<= 1;
                CRC32 ^= dwPolynomial;
            }
            else
                CRC32 <<= 1;
            if (data & xbit)
                CRC32 ^= dwPolynomial;
        }
    }

    return CRC32;
}

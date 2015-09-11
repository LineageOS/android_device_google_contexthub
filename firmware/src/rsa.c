#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <rsa.h>


static void biMod(uint32_t *num, const uint32_t *denum, uint32_t *tmp) //num %= denum where num is RSA_LEN * 2 and denum is RSA_LEN and tmp is RSA_LEN + limb_sz
{
    uint32_t bitsh = 32, limbsh = RSA_LIMBS - 1;
    int64_t t;
    int32_t i;

    //initially set it up left shifted as far as possible
    memcpy(tmp + 1, denum, RSA_BYTES);
    tmp[0] = 0;
    bitsh = 32;

    while (!(tmp[RSA_LIMBS] & 0x80000000)) {
        for (i = RSA_LIMBS; i > 0; i--) {
            tmp[i] <<= 1;
            if (tmp[i - 1] & 0x80000000)
                tmp[i]++;
        }
        //no need to adjust tmp[0] as it is still zero
        bitsh++;
    }

    while (1) {

        //check if we should subtract (uses less space than subtracting and unroling it later)
        for (i = RSA_LIMBS; i >= 0; i--) {
            if (num[limbsh + i] < tmp[i])
                goto dont_subtract;
            if (num[limbsh + i] > tmp[i])
                break;
        }

        //subtract
        t = 0;
        for (i = 0; i <= RSA_LIMBS; i++) {
            t += (uint64_t)num[limbsh + i];
            t -= (uint64_t)tmp[i];
            num[limbsh + i] = t;
            t >>= 32;
        }

        //carry the subtraction's carry to the end
        for (i = RSA_LIMBS + limbsh + 1; i < RSA_LIMBS * 2; i++) {
            t += (uint64_t)num[i];
            num[i] = t;
            t >>= 32;
        }

dont_subtract:
        //handle bitshifts/refills
        if (!bitsh) {                          // tmp = denum << 32
            if (!limbsh)
                break;

            memcpy(tmp + 1, denum, RSA_BYTES);
            tmp[0] = 0;
            bitsh = 32;
            limbsh--;
        }
        else {                                 // tmp >>= 1
            for (i = 0; i < RSA_LIMBS; i++) {
                tmp[i] >>= 1;
                if (tmp[i + 1] & 1)
                    tmp[i] += 0x80000000;
            }
            tmp[i] >>= 1;
            bitsh--;
        }
    }
}

static void biMul(uint32_t *ret, const uint32_t *a, const uint32_t *b) //ret = a * b
{
    uint32_t i, j, c;
    uint64_t r;

    //zero the result
    memset(ret, 0, RSA_BYTES * 2);

    for (i = 0; i < RSA_LIMBS; i++) {

        //produce a partial sum & add it in
        c = 0;
        for (j = 0; j < RSA_LIMBS; j++) {
            r = (uint64_t)a[i] * b[j] + c + ret[i + j];
            ret[i + j] = r;
            c = r >> 32;
        }

        //carry the carry to the end
        for (j = i + RSA_LIMBS; j < RSA_LIMBS * 2; j++) {
            r = (uint64_t)ret[j] + c;
            ret[j] = r;
            c = r >> 32;
        }
    }
}

const uint32_t* rsaPubOp(struct RsaState* state, const uint32_t *a, const uint32_t *c)
{
    uint32_t i;

    //calculate a ^ 65536 mod c into state->tmpB
    memcpy(state->tmpB, a, RSA_BYTES);
    for (i = 0; i < 16; i++) {
        biMul(state->tmpA, state->tmpB, state->tmpB);
        biMod(state->tmpA, c, state->tmpB);
        memcpy(state->tmpB, state->tmpA, RSA_BYTES);
    }

    //calculate a ^ 65537 mod c into state->tmpA [ at this point this means do state->tmpA = (state->tmpB * a) % c ]
    biMul(state->tmpA, state->tmpB, a);
    biMod(state->tmpA, c, state->tmpB);

    //return result
    return state->tmpA;
}







#include "SequenceGuard.h"

bool sequenceIsNewer(uint32_t candidate, uint32_t reference) {
    // uint32_t subtraction already wraps modulo 2^32, which is exactly the
    // sequence space (HX_SEQUENCE_BITS == 32) -- no explicit modulus
    // operation needed. This is the same comparison as the Python side's
    // (candidate - reference) % SEQUENCE_MODULUS, just relying on the
    // language's native unsigned wraparound instead of spelling it out.
    uint32_t diff = candidate - reference;
    return diff != 0 && diff < 0x80000000u;  // 0 < diff < half the modulus
}

// Mirrors transport/protocol.py's sequence_is_newer exactly -- RFC
// 1982-style wraparound-safe comparison in the 2^32 sequence space. Equal
// counts as not-newer (a duplicate, not an update). See docs/protocol.md
// Section 4.
#pragma once

#include <cstdint>

bool sequenceIsNewer(uint32_t candidate, uint32_t reference);

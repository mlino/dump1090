// dump1090, a Mode S messages decoder for RTLSDR devices.
//
// crc.c, an implementation of the Mode S checksum.
//
// Copyright (C) 2012 by Salvatore Sanfilippo <antirez@gmail.com>
// Copyright (C) 2015 by Oliver Jowett <oliver@mutability.co.uk>
//
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
//  *  Redistributions of source code must retain the above copyright
//     notice, this list of conditions and the following disclaimer.
//
//  *  Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//

#include "dump1090.h"

#include <assert.h>

//
// Parity table for MODE S Messages.
// The table contains 112 elements, every element corresponds to a bit set
// in the message, starting from the first bit of actual data after the
// preamble.
//
// For messages of 112 bit, the whole table is used.
// For messages of 56 bits only the last 56 elements are used.
//
// The algorithm is as simple as xoring all the elements in this table
// for which the corresponding bit on the message is set to 1.
//
// The latest 24 elements in this table are set to 0 as the checksum at the
// end of the message should not affect the computation.
//
// Note: this function can be used with DF11 and DF17, other modes have
// the CRC xored with the sender address as they are reply to interrogations,
// but a casual listener can't split the address from the checksum.
//
uint32_t modes_checksum_table[112] = {
0x3935ea, 0x1c9af5, 0xf1b77e, 0x78dbbf, 0xc397db, 0x9e31e9, 0xb0e2f0, 0x587178,
0x2c38bc, 0x161c5e, 0x0b0e2f, 0xfa7d13, 0x82c48d, 0xbe9842, 0x5f4c21, 0xd05c14,
0x682e0a, 0x341705, 0xe5f186, 0x72f8c3, 0xc68665, 0x9cb936, 0x4e5c9b, 0xd8d449,
0x939020, 0x49c810, 0x24e408, 0x127204, 0x093902, 0x049c81, 0xfdb444, 0x7eda22,
0x3f6d11, 0xe04c8c, 0x702646, 0x381323, 0xe3f395, 0x8e03ce, 0x4701e7, 0xdc7af7,
0x91c77f, 0xb719bb, 0xa476d9, 0xadc168, 0x56e0b4, 0x2b705a, 0x15b82d, 0xf52612,
0x7a9309, 0xc2b380, 0x6159c0, 0x30ace0, 0x185670, 0x0c2b38, 0x06159c, 0x030ace,
0x018567, 0xff38b7, 0x80665f, 0xbfc92b, 0xa01e91, 0xaff54c, 0x57faa6, 0x2bfd53,
0xea04ad, 0x8af852, 0x457c29, 0xdd4410, 0x6ea208, 0x375104, 0x1ba882, 0x0dd441,
0xf91024, 0x7c8812, 0x3e4409, 0xe0d800, 0x706c00, 0x383600, 0x1c1b00, 0x0e0d80,
0x0706c0, 0x038360, 0x01c1b0, 0x00e0d8, 0x00706c, 0x003836, 0x001c1b, 0xfff409,
0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000,
0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000,
0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000
};

uint32_t modesChecksum(unsigned char *msg, int bits) {
    uint32_t   crc = 0;
    uint32_t   rem = 0;
    int        offset = (bits == 112) ? 0 : (112-56);
    uint32_t * pCRCTable = &modes_checksum_table[offset];
    int j;

    // We don't really need to include the checksum itself
    bits -= 24;
    for (j = 0; j < bits; j += 8, pCRCTable += 8) {
        uint8_t theByte = *msg++;

        // If bit is set, xor with corresponding table entry.
        if (theByte & 0x80) crc ^= pCRCTable[0];
        if (theByte & 0x40) crc ^= pCRCTable[1];
        if (theByte & 0x20) crc ^= pCRCTable[2];
        if (theByte & 0x10) crc ^= pCRCTable[3];
        if (theByte & 0x08) crc ^= pCRCTable[4];
        if (theByte & 0x04) crc ^= pCRCTable[5];
        if (theByte & 0x02) crc ^= pCRCTable[6];
        if (theByte & 0x01) crc ^= pCRCTable[7];
    }

    rem = (msg[0] << 16) | (msg[1] << 8) | msg[2]; // message checksum
    return ((crc ^ rem) & 0x00FFFFFF); // 24 bit checksum syndrome.
}

//
//=========================================================================
//
// Code for introducing a less CPU-intensive method of correcting
// single bit errors.
//
// Makes use of the fact that the crc checksum is linear with respect to
// the bitwise xor operation, i.e.
//      crc(m^e) = (crc(m)^crc(e)
// where m and e are the message resp. error bit vectors.
//
// Call crc(e) the syndrome.
//
// The code below works by precomputing a table of (crc(e), e) for all
// possible error vectors e (here only single bit and double bit errors),
// search for the syndrome in the table, and correct the then known error.
// The error vector e is represented by one or two bit positions that are
// changed. If a second bit position is not used, it is -1.
//
// Run-time is binary search in a sorted table, plus some constant overhead,
// instead of running through all possible bit positions (resp. pairs of
// bit positions).
//

static errorinfo *bitErrorTable_short;
static int bitErrorTableSize_short;

static errorinfo *bitErrorTable_long;
static int bitErrorTableSize_long;

static int prepareSubtable(errorinfo *table, int n, int maxsize, int offset, int startbit, int endbit, errorinfo *base_entry, int error_bit)
{
    int i = 0;

    if (error_bit >= Modes.nfix_crc)
        return n;

    for (i = startbit; i < endbit; ++i) {
        assert(n < maxsize);

        table[n] = *base_entry;
        if (endbit - i <= 24) {
            // trailing 24 bits are checksum bits
            table[n].syndrome ^= 1 << (endbit - i - 1);
        } else {
            // data bits
            table[n].syndrome ^= modes_checksum_table[i + offset];
        }
        table[n].errors = error_bit+1;
        table[n].bit[error_bit] = i;
        
        ++n;
        n = prepareSubtable(table, n, maxsize, offset, i + 1, endbit, &table[n-1], error_bit + 1);
    }

    return n;
}

// Compare function as needed for stdlib's qsort and bsearch functions
static int cmpErrorInfo(const void *p0, const void *p1) {
    errorinfo *e0 = (errorinfo*)p0;
    errorinfo *e1 = (errorinfo*)p1;
    if (e0->syndrome == e1->syndrome) {
        return 0;
    } else if (e0->syndrome < e1->syndrome) {
        return -1;
    } else {
        return 1;
    }
}

// (n k), the number of ways of selecting k distinct items from a set of n items
static int combinations(int n, int k) {
    int result = 1, i;

    if (k == 0 || k == n)
        return 1;

    if (k > n)
        return 0;

    for (i = 1; i <= k; ++i) {
        result = result * n / i;
        n = n - 1;
    }

    return result;
}

static errorinfo *prepareErrorTable(int bits, int *size_out)
{
    int maxsize, usedsize;
    errorinfo *table;
    errorinfo base_entry;
    uint32_t last;
    int i;

    assert (bits >= 0 && bits <= 112);
    assert (Modes.nfix_crc >=0 && Modes.nfix_crc <= MODES_MAX_BITERRORS);

    if (!Modes.nfix_crc) {
        *size_out = 0;
        return NULL;
    }

    maxsize = 0;
    for (i = 1; i <= Modes.nfix_crc; ++i) {
        maxsize += combinations(bits - 5, i); // space needed for all i-bit errors
    }

#ifdef CRCDEBUG    
    fprintf(stderr, "Preparing syndrome table for up to %d-bit errors in a %d-bit message (max %d entries)\n", Modes.nfix_crc, bits, maxsize);
#endif

    table = malloc(maxsize * sizeof(errorinfo));
    base_entry.syndrome = 0;
    base_entry.errors = 0;
    for (i = 0; i < MODES_MAX_BITERRORS; ++i)
        base_entry.bit[i] = -1;

    // ignore the first 5 bits (DF type)
    usedsize = prepareSubtable(table, 0, maxsize, 112 - bits, 5, bits, &base_entry, 0);

#ifdef CRCDEBUG    
    fprintf(stderr, "Sorting syndromes..\n");
#endif

    qsort(table, usedsize, sizeof(errorinfo), cmpErrorInfo);

    // Handle ambiguous cases, where there is more than one possible error pattern
    // that produces a given syndrome (this happens with >2 bit errors). The ambiguous
    // cases are set to 0xffffffff which sorts above all valid syndromes, so we can
    // re-sort the table to move them to the end, then shrink the table to remove them.

#ifdef CRCDEBUG    
    fprintf(stderr, "Finding collisions..\n");
#endif
    last = table[0].syndrome;
    for (i = 1; i < usedsize; ++i) {
        if (table[i].syndrome == last) {
            table[i-1].syndrome = 0xffffffff;
            table[i].syndrome = 0xffffffff;
        } else {
            last = table[i].syndrome;
        }
    }

    // Resort the table and trim it down
#ifdef CRCDEBUG    
    fprintf(stderr, "Sorting syndromes again..\n");
#endif
    qsort(table, usedsize, sizeof(errorinfo), cmpErrorInfo);

#ifdef CRCDEBUG    
    fprintf(stderr, "Trimming table..\n");
#endif
    for (i = usedsize; i > 0; --i) {
        if (table[i-1].syndrome != 0xffffffff)
            break;
    }

    if (i < usedsize) {
#ifdef CRCDEBUG
        fprintf(stderr, "Discarding %d collisions..\n", usedsize - i);
#endif
        table = realloc(table, i * sizeof(errorinfo));
        usedsize = i;
    }
    
    *size_out = usedsize;
    
#ifdef CRCDEBUG
    {
        // Check the table.
        unsigned char *msg = malloc(bits/8);

        for (i = 0; i < usedsize; ++i) {
            int j;
            errorinfo *ei;
            uint32_t result;

            memset(msg, 0, bits/8);
            ei = &table[i];
            for (j = 0; j < ei->errors; ++j) {
                msg[ei->bit[j] >> 3] ^= 1 << (7 - (ei->bit[j]&7));
            }

            result = modesChecksum(msg, bits);
            if (result != ei->syndrome) {
                fprintf(stderr, "PROBLEM: entry %6d/%6d  syndrome %06x  errors %d  bits ", i, usedsize, ei->syndrome, ei->errors);
                for (j = 0; j < ei->errors; ++j)
                    fprintf(stderr, "%3d ", ei->bit[j]);
                fprintf(stderr, " checksum %06x\n", result);
            }
        }

        free(msg);
    }        
    
    {
        // Show the table stats
        fprintf(stderr, "Syndrome table summary:\n");
        for (i = 1; i <= Modes.nfix_crc; ++i) {
            int j, count, possible;
            
            count = 0;
            for (j = 0; j < usedsize; ++j) 
                if (table[j].errors == i)
                    ++count;

            possible = combinations(bits-5, i);
            fprintf(stderr, "  %d entries for %d-bit errors (%d possible, %d%% coverage)\n", count, i, possible, 100 * count / possible);
        }

        fprintf(stderr, "  %d entries total\n", usedsize);
    }
#endif

    return table;
}

//
//=========================================================================
//
// Compute the table of all syndromes for 1-bit and 2-bit error vectors
void modesChecksumInit() {
    if (!Modes.nfix_crc) {
        bitErrorTable_short = bitErrorTable_long = NULL;
        bitErrorTableSize_short = bitErrorTableSize_long = 0;
        return;
    }

    bitErrorTable_short = prepareErrorTable(MODES_SHORT_MSG_BITS, &bitErrorTableSize_short);
    bitErrorTable_long = prepareErrorTable(MODES_LONG_MSG_BITS, &bitErrorTableSize_long);
}

errorinfo *modesChecksumDiagnose(uint32_t syndrome, int bitlen)
{
    errorinfo *table;
    int tablesize;

    errorinfo ei;

    if (syndrome == 0)
        return NULL; // no errors

    if (!Modes.nfix_crc)
        return NULL; // CRC fixes not enabled

    assert (bitlen == 56 || bitlen == 112);
    if (bitlen == 56) { table = bitErrorTable_short; tablesize = bitErrorTableSize_short; }
    else { table = bitErrorTable_long; tablesize = bitErrorTableSize_long; }

    ei.syndrome = syndrome;
    return bsearch(&ei, table, tablesize, sizeof(errorinfo), cmpErrorInfo);
}

void modesChecksumFix(uint8_t *msg, errorinfo *info)
{
    int i;

    if (!info)
        return;

    for (i = 0; i < info->errors; ++i)
        msg[info->bit[i] >> 3] ^= 1 << (7 - (info->bit[i] & 7));
}

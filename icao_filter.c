// dump1090, a Mode S messages decoder for RTLSDR devices.
//
// icao_filter.c, a simple hashtable for ICAO addresses
//
// Copyright (C) 2015 Oliver Jowett <oliver@mutability.co.uk>
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

// Open-addressed hash table with linear probing.
// We hash each entry in two places to handle Comm-B Data/Parity

// Maintain two tables and switch between them to age out entries.

// power of two:
#define ICAO_FILTER_SIZE 4096

static uint32_t icao_filter_a[ICAO_FILTER_SIZE];
static uint32_t icao_filter_b[ICAO_FILTER_SIZE];
static uint32_t *icao_filter_active;

static uint32_t icaoHash(uint32_t a) {
    // The following three rounds wil make sure that every bit affects
    // every output bit with ~ 50% of probability.
    a = ((a >> 16) ^ a) * 0x45d9f3b;
    a = ((a >> 16) ^ a) * 0x45d9f3b;
    a = ((a >> 16) ^ a);
    return a & (ICAO_FILTER_SIZE-1);
}

void icaoFilterInit() {
    memset(icao_filter_a, 0, sizeof(icao_filter_a));
    memset(icao_filter_b, 0, sizeof(icao_filter_b));
    icao_filter_active = icao_filter_a;
}

void icaoFilterAdd(uint32_t addr) {
    uint32_t h = icaoHash(addr);
    while (icao_filter_active[h] && icao_filter_active[h] != addr)
        h = (h+1) & (ICAO_FILTER_SIZE-1);
    if (!icao_filter_active[h])
        icao_filter_active[h] = addr;

    // also add with a zeroed top byte, for handling DF20/21 with Data Parity
    h = icaoHash(addr & 0x00ffff);
    while (icao_filter_active[h] && (icao_filter_active[h] & 0x00ffff) != (addr & 0x00ffff))
        h = (h+1) & (ICAO_FILTER_SIZE-1);
    if (!icao_filter_active[h])
        icao_filter_active[h] = addr;
}

int icaoFilterTest(uint32_t addr) {
    uint32_t h, h0;

    h0 = h = icaoHash(addr);
    while (icao_filter_a[h] && icao_filter_a[h] != addr)
        h = (h+1) & (ICAO_FILTER_SIZE-1);
    if (icao_filter_a[h])
        return 1;

    h = h0;
    while (icao_filter_b[h] && icao_filter_b[h] != addr)
        h = (h+1) & (ICAO_FILTER_SIZE-1);
    if (icao_filter_b[h])
        return 1;

    return 0;
}

uint32_t icaoFilterTestFuzzy(uint32_t partial) {
    uint32_t h, h0;

    partial &= 0x00ffff;
    h0 = h = icaoHash(partial);
    while (icao_filter_a[h] && (icao_filter_a[h] & 0x00ffff) != partial)
        h = (h+1) & (ICAO_FILTER_SIZE-1);
    if (icao_filter_a[h])
        return icao_filter_a[h];

    h = h0;
    while (icao_filter_b[h] && (icao_filter_b[h] & 0x00ffff) != partial)
        h = (h+1) & (ICAO_FILTER_SIZE-1);
    if (icao_filter_b[h])
        return icao_filter_b[h];

    return 0;
}

// call this periodically:
void icaoFilterExpire() {
    static time_t next_flip = 0;
    time_t now = time(NULL);

    if (now >= next_flip) {
        if (icao_filter_active == icao_filter_a) {
            memset(icao_filter_b, 0, sizeof(icao_filter_b));
            icao_filter_active = icao_filter_b;
        } else {
            memset(icao_filter_a, 0, sizeof(icao_filter_a));
            icao_filter_active = icao_filter_a;
        }
        next_flip = now + MODES_ICAO_CACHE_TTL;
    }
}

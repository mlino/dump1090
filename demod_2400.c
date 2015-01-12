// dump1090, a Mode S messages decoder for RTLSDR devices.
//
// demod_2400.c - 2.4MHz Mode S demodulator.
//
// Copyright (C) 2014,2015 Oliver Jowett <oliver@mutability.co.uk>
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

// 2.4MHz sampling rate version
//
// When sampling at 2.4MHz we have exactly 6 samples per 5 symbols.
// Each symbol is 500ns wide, each sample is 416.7ns wide
//
// We maintain a phase offset that is expressed in units of 1/5 of a sample i.e. 1/6 of a symbol, 83.333ns
// Each symbol we process advances the phase offset by 6 i.e. 6/5 of a sample, 500ns
//
// The correlation functions below correlate a 1-0 pair of symbols (i.e. manchester encoded 1 bit)
// starting at the given sample, and assuming that the symbol starts at a fixed 0-5 phase offset within
// m[0]. They return a correlation value, generally interpreted as >0 = 1 bit, <0 = 0 bit

// TODO check if there are better (or more balanced) correlation functions to use here

// nb: the correlation functions sum to zero, so we do not need to adjust for the DC offset in the input signal
// (adding any constant value to all of m[0..3] does not change the result)

static inline int slice_phase0(uint16_t *m) {
    return 5 * m[0] - 3 * m[1] - 2 * m[2];
}
static inline int slice_phase1(uint16_t *m) {
    return 4 * m[0] - m[1] - 3 * m[2];
}
static inline int slice_phase2(uint16_t *m) {
    return 3 * m[0] + m[1] - 4 * m[2];
}
static inline int slice_phase3(uint16_t *m) {
    return 2 * m[0] + 3 * m[1] - 5 * m[2];
}
static inline int slice_phase4(uint16_t *m) {
    return m[0] + 5 * m[1] - 5 * m[2] - m[3];
}

static inline int correlate_phase0(uint16_t *m) {
    return slice_phase0(m) * 26;
}
static inline int correlate_phase1(uint16_t *m) {
    return slice_phase1(m) * 38;
}
static inline int correlate_phase2(uint16_t *m) {
    return slice_phase2(m) * 38;
}
static inline int correlate_phase3(uint16_t *m) {
    return slice_phase3(m) * 26;
}
static inline int correlate_phase4(uint16_t *m) {
    return slice_phase4(m) * 19;
}

//
// These functions work out the correlation quality for the 10 symbols (5 bits) starting at m[0] + given phase offset.
// This is used to find the right phase offset to use for decoding.
//

static inline int correlate_check_0(uint16_t *m) {
    return
        abs(correlate_phase0(&m[0])) +
        abs(correlate_phase2(&m[2])) +
        abs(correlate_phase4(&m[4])) +
        abs(correlate_phase1(&m[7])) +
        abs(correlate_phase3(&m[9]));
}

static inline int correlate_check_1(uint16_t *m) {
    return
        abs(correlate_phase1(&m[0])) +
        abs(correlate_phase3(&m[2])) +
        abs(correlate_phase0(&m[5])) +
        abs(correlate_phase2(&m[7])) +
        abs(correlate_phase4(&m[9]));
}

static inline int correlate_check_2(uint16_t *m) {
    return
        abs(correlate_phase2(&m[0])) +
        abs(correlate_phase4(&m[2])) +
        abs(correlate_phase1(&m[5])) +
        abs(correlate_phase3(&m[7])) +
        abs(correlate_phase0(&m[10]));
}

static inline int correlate_check_3(uint16_t *m) {
    return
        abs(correlate_phase3(&m[0])) +
        abs(correlate_phase0(&m[3])) +
        abs(correlate_phase2(&m[5])) +
        abs(correlate_phase4(&m[7])) +
        abs(correlate_phase1(&m[10]));
}

static inline int correlate_check_4(uint16_t *m) {
    return
        abs(correlate_phase4(&m[0])) +
        abs(correlate_phase1(&m[3])) +
        abs(correlate_phase3(&m[5])) +
        abs(correlate_phase0(&m[8])) +
        abs(correlate_phase2(&m[10]));
}

// Work out the best phase offset to use for the given message.
static int best_phase(uint16_t *m) {
    int test;
    int best = -1;
    int bestval = (m[0] + m[1] + m[2] + m[3] + m[4] + m[5]); // minimum correlation quality we will accept

    // empirical testing suggests that 4..8 is the best range to test for here
    // (testing a wider range runs the danger of picking the wrong phase for
    // a message that would otherwise be successfully decoded - the correlation
    // functions can match well with a one symbol / half bit offset)

    // this is consistent with the peak detection which should produce
    // the first data symbol with phase offset 4..8

    test = correlate_check_4(&m[0]);
    if (test > bestval) { bestval = test; best = 4; }
    test = correlate_check_0(&m[1]);
    if (test > bestval) { bestval = test; best = 5; }
    test = correlate_check_1(&m[1]);
    if (test > bestval) { bestval = test; best = 6; }
    test = correlate_check_2(&m[1]);
    if (test > bestval) { bestval = test; best = 7; }
    test = correlate_check_3(&m[1]);
    if (test > bestval) { bestval = test; best = 8; }
    return best;
}

//
// Given 'mlen' magnitude samples in 'm', sampled at 2.4MHz,
// try to demodulate some Mode S messages.
//
void detectModeS_oversample(uint16_t *m, int mlen) {
    struct modesMessage mm;
    unsigned char msg1[MODES_LONG_MSG_BYTES], msg2[MODES_LONG_MSG_BYTES], *msg;
    int j;
    int last_message_end = -1;

    unsigned char *bestmsg;
    int bestscore, bestphase, bestsnr;

    // make these static as a bit of a hack to preserve state between calls

    // noise floor:
    uint32_t noise_count = 0;
    uint64_t noise_power = 0;

    memset(&mm, 0, sizeof(mm));
    msg = msg1;

    for (j = 0; j < mlen; j++) {
        uint16_t *preamble = &m[j];
        int high;
        uint32_t base_signal, base_noise;
        int initial_phase, first_phase, last_phase, try_phase;
        int msglen;

        // update noise for all samples that aren't part of a message
        // (we don't know if m[j] is or not, yet, so work one sample
        // in arrears)
        if ((j-1) > last_message_end) {
            // There seems to be a weird compiler bug I'm hitting here..
            // if you compute the square directly, it occasionally gets mangled.
            uint32_t s = m[j-1];
            noise_power += s * s;
            noise_count++;
        }

        // Look for a message starting at around sample 0 with phase offset 3..7

        // Ideal sample values for preambles with different phase
        // Xn is the first data symbol with phase offset N
        //
        // sample#: 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0
        // phase 3: 2/4\0/5\1 0 0 0 0/5\1/3 3\0 0 0 0 0 0 X4
        // phase 4: 1/5\0/4\2 0 0 0 0/4\2 2/4\0 0 0 0 0 0 0 X0
        // phase 5: 0/5\1/3 3\0 0 0 0/3 3\1/5\0 0 0 0 0 0 0 X1
        // phase 6: 0/4\2 2/4\0 0 0 0 2/4\0/5\1 0 0 0 0 0 0 X2
        // phase 7: 0/3 3\1/5\0 0 0 0 1/5\0/4\2 0 0 0 0 0 0 X3
        //
        
        // quick check: we must have a rising edge 0->1 and a falling edge 12->13
        if (! (preamble[0] < preamble[1] && preamble[12] > preamble[13]) )
           continue;

        if (preamble[1] > preamble[2] &&                                       // 1
            preamble[2] < preamble[3] && preamble[3] > preamble[4] &&          // 3
            preamble[8] < preamble[9] && preamble[9] > preamble[10] &&         // 9
            preamble[10] < preamble[11]) {                                     // 11-12
            // peaks at 1,3,9,11-12: phase 3
            high = (preamble[1] + preamble[3] + preamble[9] + preamble[11] + preamble[12]) / 4;
            base_signal = preamble[1] + preamble[3] + preamble[9];
            base_noise = preamble[5] + preamble[6] + preamble[7];
        } else if (preamble[1] > preamble[2] &&                                // 1
                   preamble[2] < preamble[3] && preamble[3] > preamble[4] &&   // 3
                   preamble[8] < preamble[9] && preamble[9] > preamble[10] &&  // 9
                   preamble[11] < preamble[12]) {                              // 12
            // peaks at 1,3,9,12: phase 4
            high = (preamble[1] + preamble[3] + preamble[9] + preamble[12]) / 4;
            base_signal = preamble[1] + preamble[3] + preamble[9] + preamble[12];
            base_noise = preamble[5] + preamble[6] + preamble[7] + preamble[8];
        } else if (preamble[1] > preamble[2] &&                                // 1
                   preamble[2] < preamble[3] && preamble[4] > preamble[5] &&   // 3-4
                   preamble[8] < preamble[9] && preamble[10] > preamble[11] && // 9-10
                   preamble[11] < preamble[12]) {                              // 12
            // peaks at 1,3-4,9-10,12: phase 5
            high = (preamble[1] + preamble[3] + preamble[4] + preamble[9] + preamble[10] + preamble[12]) / 4;
            base_signal = preamble[1] + preamble[12];
            base_noise = preamble[6] + preamble[7];
        } else if (preamble[1] > preamble[2] &&                                 // 1
                   preamble[3] < preamble[4] && preamble[4] > preamble[5] &&    // 4
                   preamble[9] < preamble[10] && preamble[10] > preamble[11] && // 10
                   preamble[11] < preamble[12]) {                               // 12
            // peaks at 1,4,10,12: phase 6
            high = (preamble[1] + preamble[4] + preamble[10] + preamble[12]) / 4;
            base_signal = preamble[1] + preamble[4] + preamble[10] + preamble[12];
            base_noise = preamble[5] + preamble[6] + preamble[7] + preamble[8];
        } else if (preamble[2] > preamble[3] &&                                 // 1-2
                   preamble[3] < preamble[4] && preamble[4] > preamble[5] &&    // 4
                   preamble[9] < preamble[10] && preamble[10] > preamble[11] && // 10
                   preamble[11] < preamble[12]) {                               // 12
            // peaks at 1-2,4,10,12: phase 7
            high = (preamble[1] + preamble[2] + preamble[4] + preamble[10] + preamble[12]) / 4;
            base_signal = preamble[4] + preamble[10] + preamble[12];
            base_noise = preamble[6] + preamble[7] + preamble[8];
        } else {
            // no suitable peaks
            continue;
        }

        // Check for enough signal
        if (base_signal * 2 < 3 * base_noise) // about 3.5dB SNR
            continue;

        // Check that the "quiet" bits 6,7,15,16,17 are actually quiet
        if (preamble[5] >= high ||
            preamble[6] >= high ||
            preamble[7] >= high ||
            preamble[8] >= high ||
            preamble[14] >= high ||
            preamble[15] >= high ||
            preamble[16] >= high ||
            preamble[17] >= high ||
            preamble[18] >= high) {
            ++Modes.stat_preamble_not_quiet;
            continue;
        }

        if (Modes.phase_enhance) {
            first_phase = 4;
            last_phase = 8;           // try all phases
        } else {
            // Crosscorrelate against the first few bits to find a likely phase offset
            initial_phase = best_phase(&preamble[19]);
            if (initial_phase < 0) {
                ++Modes.stat_preamble_no_correlation;
                continue; // nothing satisfactory
            }
            
            Modes.stat_preamble_phase[initial_phase%MODES_MAX_PHASE_STATS]++;            
            first_phase = last_phase = initial_phase;  // try only the phase we think it is
        }

        Modes.stat_valid_preamble++;
        bestmsg = NULL; bestscore = -1; bestphase = -1; bestsnr = -1;
        for (try_phase = first_phase; try_phase <= last_phase; ++try_phase) {
            int sigLevel = base_signal, noiseLevel = base_noise;
            uint8_t theByte;
            uint16_t *pPtr;
            unsigned char *pMsg;
            int phase, errors, i;

            // Decode all the next 112 bits, regardless of the actual message
            // size. We'll check the actual message type later
            
            pMsg = &msg[0];
            pPtr = &m[j+19] + (try_phase/5);
            phase = try_phase % 5;
            theByte = 0;
            errors  = 0;

            for (i = 0; i < MODES_LONG_MSG_BITS && errors < MODES_MSG_ENCODER_ERRS; i++) {
                int test;
                
                switch (phase) {
                case 0:
                    test = slice_phase0(pPtr);
                    phase = 2;
                    pPtr += 2;
                    break;
                    
                case 1:
                    test = slice_phase1(pPtr);
                    phase = 3;
                    pPtr += 2;
                    break;
                    
                case 2:
                    test = slice_phase2(pPtr);
                    phase = 4;
                    pPtr += 2;
                    break;
                    
                case 3:
                    test = slice_phase3(pPtr);
                    phase = 0;
                    pPtr += 3;
                    break;
                    
                case 4:
                    test = slice_phase4(pPtr);
                    
                    // A phase-4 bit exactly straddles a sample boundary.
                    // Here's what a 1-0 bit with phase 4 looks like:
                    //
                    //     |SYM 1|
                    //  xxx|     |     |xxx
                    //           |SYM 2|
                    //
                    // 012340123401234012340  <-- sample phase
                    // | 0  | 1  | 2  | 3  |  <-- sample boundaries
                    //
                    // Samples 1 and 2 only have power from symbols 1 and 2.
                    // So we can use this to extract signal/noise values
                    // as one of the two symbols is high (signal) and the
                    // other is low (noise)
                    //
                    // This also gives us an equal number of signal and noise
                    // samples, which is convenient. Using the first half of
                    // a phase 0 bit, or the second half of a phase 3 bit, would
                    // also work, but we have no guarantees about how many signal
                    // or noise bits we'd see in those phases.
                    
                    if (test < 0) {   // 0 1
                        noiseLevel += pPtr[1];
                        sigLevel += pPtr[2];
                    } else {          // 1 0
                        sigLevel += pPtr[1];
                        noiseLevel += pPtr[2];
                    }
                    phase = 1;
                    pPtr += 3;
                    break;
                    
                default:
                    test = 0;
                    break;
                }

                if (test > 0)
                    theByte |= 1;
                /* else if (test < 0) theByte |= 0; */
                else if (test == 0) {
                    ++errors;
                }
                
                if ((i & 7) == 7)
                    *pMsg++ = theByte;
                
                theByte = theByte << 1;
            }

            // See what length this message should be, and check that we got enough
            // bits before giving up for that to work.

            if (i < 8) {
                // Didn't even make it past the first byte
                continue;
            }

            msglen = modesMessageLenByType(msg[0] >> 3);
            if (i < msglen) {
                // Bailed out because of too many errors before we got the full message.
                continue;
            }
            
            // Score the mode S message and see if it's any good.
            {
                int snr;
                int score = scoreModesMessage(msg);
#if 0
                {
                    int k;
                    fprintf(stdout, "%d+%d scores %d: ", j, try_phase, score);
                    for (k = 0; k < msglen; k += 8)
                        fprintf(stdout, "%02x", msg[k/8]);
                    fprintf(stdout, "\n");
                }
#endif
                if (score < 0)
                    continue; // can't decode


                // apply the SNR to the score, so less noisy decodes are better,
                // all things being equal

                // snr = 5 * 20log10(sigLevel / noiseLevel)         (in units of 0.2dB)
                //     = 100log10(sigLevel) - 100log10(noiseLevel)                    
                while (sigLevel > 65535 || noiseLevel > 65535) {
                    sigLevel >>= 1;
                    noiseLevel >>= 1;
                }

                snr = Modes.log10lut[sigLevel] - Modes.log10lut[noiseLevel];
                score += snr;

                if (score > bestscore) {
                    // new high score!
                    bestmsg = msg;
                    bestscore = score;
                    bestphase = try_phase;
                    bestsnr = snr;

                    // swap to using the other buffer so we don't clobber our demodulated data
                    // (if we find a better result then we'll swap back, but that's OK because
                    // we no longer need this copy if we found a better one)
                    msg = (msg == msg1) ? msg2 : msg1;
                }
            }
        }

        // Do we have a candidate?
        if (!bestmsg) {
            Modes.stat_demod.badcrc++;
            continue; // nope.
        }

        msglen = modesMessageLenByType(bestmsg[0] >> 3);

        // Set initial mm structure details
        mm.timestampMsg = Modes.timestampBlk + (j*5) + bestphase;
        mm.signalLevel = (bestsnr > 255 ? 255 : (uint8_t)bestsnr);
        mm.score = bestscore;
        mm.bFlags = mm.correctedbits   = 0;

#if 0
        fprintf(stdout, "best: %d+%d score %d\n", j, bestphase, bestscore);
#endif

        // Decode the received message
        if (decodeModesMessage(&mm, bestmsg) < 0) {
            int k;
            fprintf(stderr, "oops - nonzero message score (%d), but we couldn't decode the message: ", bestscore);
            for (k = 0; k < msglen; k += 8)
                fprintf(stderr, "%02x", bestmsg[k/8]);
            fprintf(stderr, "\n");
            continue;
        }

        // Update statistics
        if (Modes.stats) {
            if (mm.correctedbits == 0) {
                Modes.stat_demod.goodcrc++;
                Modes.stat_demod.goodcrc_byphase[bestphase%MODES_MAX_PHASE_STATS]++;
            } else {
                Modes.stat_demod.badcrc++;
                Modes.stat_demod.fixed++;
                if (mm.correctedbits)
                    Modes.stat_demod.bit_fix[mm.correctedbits-1]++;
            }
        }

        // Skip over the message:
        // (we actually skip to 8 bits before the end of the message,
        //  because we can often decode two messages that *almost* collide,
        //  where the preamble of the second message clobbered the last
        //  few bits of the first message, but the message bits didn't
        //  overlap)
        last_message_end = (8 + msglen)*12/5;
        j += (8 + msglen - 8)*12/5 - 1;
#if 0
        fprintf(stdout, "skipped to %d", j);
#endif
            
        // Pass data to the next layer
        useModesMessage(&mm);
    }

    Modes.stat_noise_power += noise_power;
    Modes.stat_noise_count += noise_count;

    // avoid overflow
    while (Modes.stat_noise_power > ((uint64_t)1<<60) || Modes.stat_noise_count > ((uint32_t)1<<30)) {
        Modes.stat_noise_power >>= 1;
        Modes.stat_noise_count >>= 1;
    }
}


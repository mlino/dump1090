// dump1090, a Mode S messages decoder for RTLSDR devices.
//
// Copyright (C) 2012 by Salvatore Sanfilippo <antirez@gmail.com>
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
//
// ===================== Mode S detection and decoding  ===================
//
//=========================================================================
//
// Given the Downlink Format (DF) of the message, return the message length in bits.
//
// All known DF's 16 or greater are long. All known DF's 15 or less are short. 
// There are lots of unused codes in both category, so we can assume ICAO will stick to 
// these rules, meaning that the most significant bit of the DF indicates the length.
//
int modesMessageLenByType(int type) {
    return (type & 0x10) ? MODES_LONG_MSG_BITS : MODES_SHORT_MSG_BITS ;
}

//
// ============================== Debugging =================================
//
// Helper function for dumpMagnitudeVector().
// It prints a single bar used to display raw signals.
//
// Since every magnitude sample is between 0-255, the function uses
// up to 63 characters for every bar. Every character represents
// a length of 4, 3, 2, 1, specifically:
//
// "O" is 4
// "o" is 3
// "-" is 2
// "." is 1
//
void dumpMagnitudeBar(int index, int magnitude) {
    char *set = " .-o";
    char buf[256];
    int div = magnitude / 256 / 4;
    int rem = magnitude / 256 % 4;

    memset(buf,'O',div);
    buf[div] = set[rem];
    buf[div+1] = '\0';

    if (index >= 0)
        printf("[%.3d] |%-66s 0x%04X\n", index, buf, magnitude);
    else
        printf("[%.2d] |%-66s 0x%04X\n", index, buf, magnitude);
}
//
//=========================================================================
//
// Display an ASCII-art alike graphical representation of the undecoded
// message as a magnitude signal.
//
// The message starts at the specified offset in the "m" buffer.
// The function will display enough data to cover a short 56 bit message.
//
// If possible a few samples before the start of the messsage are included
// for context.
//
void dumpMagnitudeVector(uint16_t *m, uint32_t offset) {
    uint32_t padding = 5; // Show a few samples before the actual start.
    uint32_t start = (offset < padding) ? 0 : offset-padding;
    uint32_t end = offset + (MODES_PREAMBLE_SAMPLES)+(MODES_SHORT_MSG_SAMPLES) - 1;
    uint32_t j;

    for (j = start; j <= end; j++) {
        dumpMagnitudeBar(j-offset, m[j]);
    }
}

//
//=========================================================================
//
// This is a wrapper for dumpMagnitudeVector() that also show the message
// in hex format with an additional description.
//
// descr  is the additional message to show to describe the dump.
// msg    points to the decoded message
// m      is the original magnitude vector
// offset is the offset where the message starts
//
// The function also produces the Javascript file used by debug.html to
// display packets in a graphical format if the Javascript output was
// enabled.
//
void dumpRawMessage(char *descr, unsigned char *msg, uint16_t *m, uint32_t offset) {
    int  j;
    int  msgtype = msg[0] >> 3;
    int  fixable = 0;

    if (msgtype == 17) {
        int bits = modesMessageLenByType(msgtype);
        int crc = modesChecksum(msg, bits);
        errorinfo *ei = modesChecksumDiagnose(crc, bits);
        fixable = (ei == NULL ? 0 : ei->errors);
    }

    printf("\n--- %s\n    ", descr);
    for (j = 0; j < MODES_LONG_MSG_BYTES; j++) {
        printf("%02x",msg[j]);
        if (j == MODES_SHORT_MSG_BYTES-1) printf(" ... ");
    }
    printf(" (DF %d, Fixable: %d)\n", msgtype, fixable);
    dumpMagnitudeVector(m,offset);
    printf("---\n\n");
}

//
//=========================================================================
//
// In the squawk (identity) field bits are interleaved as follows in
// (message bit 20 to bit 32):
//
// C1-A1-C2-A2-C4-A4-ZERO-B1-D1-B2-D2-B4-D4
//
// So every group of three bits A, B, C, D represent an integer from 0 to 7.
//
// The actual meaning is just 4 octal numbers, but we convert it into a hex 
// number tha happens to represent the four octal numbers.
//
// For more info: http://en.wikipedia.org/wiki/Gillham_code
//
int decodeID13Field(int ID13Field) {
    int hexGillham = 0;

    if (ID13Field & 0x1000) {hexGillham |= 0x0010;} // Bit 12 = C1
    if (ID13Field & 0x0800) {hexGillham |= 0x1000;} // Bit 11 = A1
    if (ID13Field & 0x0400) {hexGillham |= 0x0020;} // Bit 10 = C2
    if (ID13Field & 0x0200) {hexGillham |= 0x2000;} // Bit  9 = A2
    if (ID13Field & 0x0100) {hexGillham |= 0x0040;} // Bit  8 = C4
    if (ID13Field & 0x0080) {hexGillham |= 0x4000;} // Bit  7 = A4
  //if (ID13Field & 0x0040) {hexGillham |= 0x0800;} // Bit  6 = X  or M 
    if (ID13Field & 0x0020) {hexGillham |= 0x0100;} // Bit  5 = B1 
    if (ID13Field & 0x0010) {hexGillham |= 0x0001;} // Bit  4 = D1 or Q
    if (ID13Field & 0x0008) {hexGillham |= 0x0200;} // Bit  3 = B2
    if (ID13Field & 0x0004) {hexGillham |= 0x0002;} // Bit  2 = D2
    if (ID13Field & 0x0002) {hexGillham |= 0x0400;} // Bit  1 = B4
    if (ID13Field & 0x0001) {hexGillham |= 0x0004;} // Bit  0 = D4

    return (hexGillham);
    }
//
//=========================================================================
//
// Decode the 13 bit AC altitude field (in DF 20 and others).
// Returns the altitude, and set 'unit' to either MODES_UNIT_METERS or MDOES_UNIT_FEETS.
//
int decodeAC13Field(int AC13Field, int *unit) {
    int m_bit  = AC13Field & 0x0040; // set = meters, clear = feet
    int q_bit  = AC13Field & 0x0010; // set = 25 ft encoding, clear = Gillham Mode C encoding

    if (!m_bit) {
        *unit = MODES_UNIT_FEET;
        if (q_bit) {
            // N is the 11 bit integer resulting from the removal of bit Q and M
            int n = ((AC13Field & 0x1F80) >> 2) |
                    ((AC13Field & 0x0020) >> 1) |
                     (AC13Field & 0x000F);
            // The final altitude is resulting number multiplied by 25, minus 1000.
            return ((n * 25) - 1000);
        } else {
            // N is an 11 bit Gillham coded altitude
            int n = ModeAToModeC(decodeID13Field(AC13Field));
            if (n < -12) {n = 0;}

            return (100 * n);
        }
    } else {
        *unit = MODES_UNIT_METERS;
        // TODO: Implement altitude when meter unit is selected
    }
    return 0;
}
//
//=========================================================================
//
// Decode the 12 bit AC altitude field (in DF 17 and others).
//
int decodeAC12Field(int AC12Field, int *unit) {
    int q_bit  = AC12Field & 0x10; // Bit 48 = Q

    *unit = MODES_UNIT_FEET;
    if (q_bit) {
        /// N is the 11 bit integer resulting from the removal of bit Q at bit 4
        int n = ((AC12Field & 0x0FE0) >> 1) | 
                 (AC12Field & 0x000F);
        // The final altitude is the resulting number multiplied by 25, minus 1000.
        return ((n * 25) - 1000);
    } else {
        // Make N a 13 bit Gillham coded altitude by inserting M=0 at bit 6
        int n = ((AC12Field & 0x0FC0) << 1) | 
                 (AC12Field & 0x003F);
        n = ModeAToModeC(decodeID13Field(n));
        if (n < -12) {n = 0;}

        return (100 * n);
    }
}
//
//=========================================================================
//
// Decode the 7 bit ground movement field PWL exponential style scale
//
int decodeMovementField(int movement) {
    int gspeed;

    // Note : movement codes 0,125,126,127 are all invalid, but they are 
    //        trapped for before this function is called.

    if      (movement  > 123) gspeed = 199; // > 175kt
    else if (movement  > 108) gspeed = ((movement - 108)  * 5) + 100;
    else if (movement  >  93) gspeed = ((movement -  93)  * 2) +  70;
    else if (movement  >  38) gspeed = ((movement -  38)     ) +  15;
    else if (movement  >  12) gspeed = ((movement -  11) >> 1) +   2;
    else if (movement  >   8) gspeed = ((movement -   6) >> 2) +   1;
    else                      gspeed = 0;

    return (gspeed);
}
//
//=========================================================================
//
// Capability table
char *ca_str[8] = {
    /* 0 */ "Level 1",
    /* 1 */ "reserved",
    /* 2 */ "reserved",
    /* 3 */ "reserved",
    /* 4 */ "Level 2+, ground",
    /* 5 */ "Level 2+, airborne",
    /* 6 */ "Level 2+",
    /* 7 */ "DR/Alert/SPI active"
};

// DF 18 Control field table.
char *cf_str[8] = {
    /* 0 */ "ADS-B ES/NT device with ICAO 24-bit address",
    /* 1 */ "ADS-B ES/NT device with other address",
    /* 2 */ "Fine format TIS-B",
    /* 3 */ "Coarse format TIS-B",
    /* 4 */ "TIS-B management message",
    /* 5 */ "TIS-B relay of ADS-B message with other address",
    /* 6 */ "ADS-B rebroadcast using DF-17 message format",
    /* 7 */ "Reserved"
};

// Flight status table
char *fs_str[8] = {
    /* 0 */ "Normal, Airborne",
    /* 1 */ "Normal, On the ground",
    /* 2 */ "ALERT,  Airborne",
    /* 3 */ "ALERT,  On the ground",
    /* 4 */ "ALERT & Special Position Identification. Airborne or Ground",
    /* 5 */ "Special Position Identification. Airborne or Ground",
    /* 6 */ "Reserved",
    /* 7 */ "Not assigned"
};

// Emergency state table
// from https://www.ll.mit.edu/mission/aviation/publications/publication-files/atc-reports/Grappel_2007_ATC-334_WW-15318.pdf
// and 1090-DO-260B_FRAC
char *es_str[8] = {
    /* 0 */ "No emergency",
    /* 1 */ "General emergency (squawk 7700)",
    /* 2 */ "Lifeguard/Medical",
    /* 3 */ "Minimum fuel",
    /* 4 */ "No communications (squawk 7600)",
    /* 5 */ "Unlawful interference (squawk 7500)",
    /* 6 */ "Reserved",
    /* 7 */ "Reserved"
};
//
//=========================================================================
//
char *getMEDescription(int metype, int mesub) {
    char *mename = "Unknown";

    if (metype >= 1 && metype <= 4)
        mename = "Aircraft Identification and Category";
    else if (metype >= 5 && metype <= 8)
        mename = "Surface Position";
    else if (metype >= 9 && metype <= 18)
        mename = "Airborne Position (Baro Altitude)";
    else if (metype == 19 && mesub >=1 && mesub <= 4)
        mename = "Airborne Velocity";
    else if (metype >= 20 && metype <= 22)
        mename = "Airborne Position (GNSS Height)";
    else if (metype == 23 && mesub == 0)
        mename = "Test Message";
    else if (metype == 23 && mesub == 7)
        mename = "Test Message -- Squawk";
    else if (metype == 24 && mesub == 1)
        mename = "Surface System Status";
    else if (metype == 28 && mesub == 1)
        mename = "Extended Squitter Aircraft Status (Emergency)";
    else if (metype == 28 && mesub == 2)
        mename = "Extended Squitter Aircraft Status (1090ES TCAS RA)";
    else if (metype == 29 && (mesub == 0 || mesub == 1))
        mename = "Target State and Status Message";
    else if (metype == 31 && (mesub == 0 || mesub == 1))
        mename = "Aircraft Operational Status Message";
    return mename;
}

// Correct an Address Announced field (bits 8-31) if it
// is affected by the given error syndrome. Updates *addr
// and returns 1 if changed, 0 if it was unaffected.
static int correct_aa_field(int *addr, errorinfo *ei) 
{
    int has_addr_errors = 0;

    if (!ei)
        return 0;

    for (i = 0; i < ei->errors; ++i) {
        if (ei->bit[i] >= 8 && ei->bit[i] <= 31) {
            *addr ^= 1 << (31 - ei->bit[i]);
            has_addr_errors = 1;
        }
    }

    return has_addr_errors;
}

// Score how plausible this ModeS message looks.
//
// <0        : decoding would fail
// positive  : decoding should work
//
// the more positive, the more reliable the message is
//

int scoreModesMessage(unsigned char *msg) {
    int msgtype = msg[0] >> 3; // Downlink Format
    int msgbits = modesMessageLenByType(msgtype);
    int crc     = modesChecksum(msg, msgbits);

    //fprintf(stderr, "type %d len %d crc %04x\n", msgtype, msgbits, crc);

    switch (msgtype) {
    case 0: // short air-air surveillance
    case 4: // surveillance, altitude reply
    case 5: // surveillance, altitude reply
    case 16: // long air-air surveillance
    case 24: // Comm-D (ELM)
        return icaoFilterTest(crc) ? 1000 : -1;

    case 11: { // All-call reply
        errorinfo *ei;
        int i, addr, iid;

        if (!crc)
            return 2000; // perfect case: IID=0, correct CRC

        iid = crc & 0x7f;
        crc = crc & 0xffff80;

        addr = (msg[1] << 16) | (msg[2] << 8) | (msg[3]);

        if (!crc) { // IID != 0 but CRC is OK otherwise
            if (!icaoFilterTest(addr))
                return -1;            
            return 1500;
        }

        ei = modesChecksumDiagnose(crc, msgbits);
        if (!ei)
            return -1; // can't correct errors

        // fix any errors in the address
        correct_addr_errors(&addr, ei);

        // validate address
        if (!icaoFilterTest(addr))
            return -1;
        else if (ei->errors >= 2)
            return 1000 / ei->errors;
        else if (iid != 0)
            return 750;
        else
            return 1000;
    }
        
    case 17:   // Extended squitter
    case 18: { // Extended squitter/non-transponder
        errorinfo *ei;
        int has_addr_errors, i;

        if (crc == 0)
            return 3000;

        ei = modesChecksumDiagnose(crc, msgbits);
        if (!ei)
            return -1; // can't correct errors

        int addr = (msg[1] << 16) | (msg[2] << 8) | (msg[3]);
        if (correct_addr_errors(&addr, ei)) {
            // Revalidate if it changed
            if (!icaoFilterTest(addr))
                return -1;
        }

        return 2000 / ei->errors;
    }

    case 20:   // Comm-B, altitude reply
    case 21:   // Comm-B, identity reply
        if (icaoFilterTest(crc))
            return 1000; // Address/Parity

#if 0
        if (icaoFilterTestFuzzy(crc))
            return 600;  // Data/Parity
#endif

        return -1;

    default:
        // unknown message type
        return -1;
    }
}

//
//=========================================================================
//
// Decode a raw Mode S message demodulated as a stream of bytes by detectModeS(), 
// and split it into fields populating a modesMessage structure.
//

static void decodeExtendedSquitter(struct modesMessage *mm);
static void decodeCommB(struct modesMessage *mm);
static char *ais_charset = "@ABCDEFGHIJKLMNOPQRSTUVWXYZ[\\]^_ !\"#$%&'()*+,-./0123456789:;<=>?";

int decodeModesMessage(struct modesMessage *mm, unsigned char *msg) {

    // Work on our local copy
    memcpy(mm->msg, msg, MODES_LONG_MSG_BYTES);
    msg = mm->msg;

    // Get the message type ASAP as other operations depend on this
    mm->msgtype         = msg[0] >> 3; // Downlink Format
    mm->msgbits         = modesMessageLenByType(mm->msgtype);
    mm->crc             = modesChecksum(msg, mm->msgbits);
    mm->correctedbits   = 0;

    // Do checksum work and set fields that depend on the CRC

    switch (mm->msgtype) {
    case 0: // short air-air surveillance
    case 4: // surveillance, altitude reply
    case 5: // surveillance, altitude reply
    case 16: // long air-air surveillance
    case 24: // Comm-D (ELM)
        // These message types use Address/Parity, i.e. our CRC syndrome is the sender's ICAO address.
        // We can't tell if the CRC is correct or not as we don't know the correct address.
        // Accept the message if it appears to be from a previously-seen aircraft
        if (!icaoFilterTest(mm->crc)) {
            fprintf(stderr, "reject: AP doesn't match known ICAO\n");
            return -1;
        }
        mm->addr = mm->crc;
        break;

    case 11: // All-call reply
        // This message type uses Parity/Interrogator, i.e. our CRC syndrome is CL + IC from the uplink message
        // which we can't see. So we don't know if the CRC is correct or not.
        //
        // however! CL + IC only occupy the lower 7 bits of the CRC. So if we ignore those bits when testing
        // the CRC we can still try to detect/correct errors.

        mm->iid   =  mm->crc & 0x7f;
        if (mm->crc & 0xffff80) {
            int addr;
            errorinfo *ei = modesChecksumDiagnose(mm->crc & 0xffff80, mm->msgbits);
            if (!ei) {
                fprintf(stderr, "reject: DF11 uncorrectable CRC error\n");
                return -1; // couldn't fix it
            }
            mm->correctedbits = ei->errors;
            modesChecksumFix(msg, ei);

            // check whether the corrected message looks sensible
            addr = (msg[1] << 16) | (msg[2] << 8) | (msg[3]); 
            if (!icaoFilterTest(addr)) {
                fprintf(stderr, "reject: DF11 CRC error, repaired address doesn't match known ICAO\n");
                return -1;
            }
        }
        break;

    case 17:   // Extended squitter
    case 18: { // Extended squitter/non-transponder
        errorinfo *ei;
        int addr1, addr2;

        // These message types use Parity/Interrogator, but are specified to set II=0

        if (mm->crc == 0)
            break;  // all good

        ei = modesChecksumDiagnose(mm->crc, mm->msgbits);
        if (!ei) {
            fprintf(stderr, "reject: DF17/18 uncorrectable CRC error\n");
            return -1; // couldn't fix it
        }
        mm->correctedbits = ei->errors;

        addr1 = (msg[1] << 16) | (msg[2] << 8) | (msg[3]); 
        modesChecksumFix(msg, ei);
        addr2 = (msg[1] << 16) | (msg[2] << 8) | (msg[3]); 
        
        // if the corrections touched the address, better validate it
        if (addr1 != addr2 && !icaoFilterTest(addr2)) {
            fprintf(stderr, "reject: DF17/18 CRC corrected address, repaired address doesn't match known ICAO\n");
            return -1;
        }

        break;
        
    case 20: // Comm-B, altitude reply
    case 21: // Comm-B, identity reply
        // These message types either use Address/Parity (see DF0 etc)
        // or Data Parity where the requested BDS is also xored into the top byte.
        // So not only do we not know whether the CRC is right, we also don't know if
        // the ICAO is right! Ow.

        // Try an exact match
        if (icaoFilterTest(mm->crc)) {
            // OK.
            mm->addr = mm->crc;
            mm->bds = 0; // unknown
            break;
        }

#if 0
        // Try a fuzzy match
        if ( (mm->addr = icaoFilterTestFuzzy(mm->crc)) != 0) {
            // We have an address that would match, assume it's correct
            mm->bds = (mm->crc ^ mm->addr) >> 16; // derive the BDS value based on what we think the address is
            break;
        }
#endif

        fprintf(stderr, "reject: DF20/21 address doesn't match known ICAO\n");
        return -1; // no good

    default:
        // All other message types, we don't know how to handle their CRCs, give up
        return -1;
    }

    // decode the bulk of the message

    mm->bFlags = 0;

    // AA (Address announced)
    if (mm->msgtype == 11 || mm->msgtype == 17 || mm->msgtype == 18) {
        mm->addr  = (msg[1] << 16) | (msg[2] << 8) | (msg[3]);
        if (!mm->correctedbits && (mm->msgtype != 11 || mm->iid == 0)) {            
            // No CRC errors seen, and either it was an DF17/18 extended squitter
            // or a DF11 acquisition squitter with II = 0. We probably have the right address.

            // NB this is the only place that adds addresses!
            icaoFilterAdd(mm->addr);
        }
    }

    // AC (Altitude Code)
    if (mm->msgtype == 0 || mm->msgtype == 4 || mm->msgtype == 16 || mm->msgtype == 20) {
        int AC13Field = ((msg[2] << 8) | msg[3]) & 0x1FFF; 
        if (AC13Field) { // Only attempt to decode if a valid (non zero) altitude is present
            mm->bFlags  |= MODES_ACFLAGS_ALTITUDE_VALID;
            mm->altitude = decodeAC13Field(AC13Field, &mm->unit);
        }
    }

    // AF (DF19 Application Field) not decoded

    // CA (Capability)
    if (mm->msgtype == 11 || mm->msgtype == 17) {
        mm->ca    = (msg[0] & 0x07);
        if (mm->ca == 4) {
            mm->bFlags |= MODES_ACFLAGS_AOG_VALID | MODES_ACFLAGS_AOG;
        } else if (mm->ca == 5) {
            mm->bFlags |= MODES_ACFLAGS_AOG_VALID;
        }
    }

    // CC (Cross-link capability) not decoded

    // CF (Control field)
    if (mm->msgtype == 18) {
        mm->cf = msg[0] & 7;
    }

    // DR (Downlink Request) not decoded

    // FS (Flight Status)
    if (mm->msgtype == 4 || mm->msgtype == 5 || mm->msgtype == 20 || mm->msgtype == 21) {
        mm->bFlags  |= MODES_ACFLAGS_FS_VALID;
        mm->fs = msg[0] & 7;
        if (mm->fs <= 3) {
            mm->bFlags |= MODES_ACFLAGS_AOG_VALID;
            if (mm->fs & 1)
                mm->bFlags |= MODES_ACFLAGS_AOG;
        }
    }

    // ID (Identity)
    if (mm->msgtype == 5  || mm->msgtype == 21) {
        // Gillham encoded Squawk
        int ID13Field = ((msg[2] << 8) | msg[3]) & 0x1FFF; 
        if (ID13Field) {
            mm->bFlags |= MODES_ACFLAGS_SQUAWK_VALID;
            mm->modeA   = decodeID13Field(ID13Field);
        }
    }

    // KE (Control, ELM) not decoded

    // MB (messsage, Comm-B)
    if (mm->msgtype == 20 || mm->msgtype == 21) {
        decodeCommB(mm);
    }

    // MD (message, Comm-D) not decoded

    // ME (message, extended squitter)
    if (mm->msgtype == 17 ||   //  Extended squitter
        (mm->msgtype == 18 &&  //  Extended squitter/non-transponder:
         (mm->cf == 0 ||       //   ADS-B ES/NT devices that report the ICAO 24-bit address in the AA field
          mm->cf == 1 ||       //   Reserved for ADS-B for ES/NT devices that use other addressing techniques in the AA field
          mm->cf == 6))) {     //   ADS-B rebroadcast using the same type codes and message formats as defined for DF = 17 ADS-B messages
            decodeExtendedSquitter(mm);
    }

    // MV (message, ACAS) not decoded
    // ND (number of D-segment) not decoded
    // RI (Reply information) not decoded
    // SL (Sensitivity level, ACAS) not decoded
    // UM (Utility Message) not decoded

    // VS (Vertical Status)
    if (mm->msgtype == 0 || mm->msgtype == 16) {
        mm->bFlags |= MODES_ACFLAGS_AOG_VALID;        
        if (msg[0] & 0x04)
            mm->bFlags |= MODES_ACFLAGS_AOG;
    }

    // all done
    return 0;
}

static void decodeExtendedSquitter(struct modesMessage *mm)
{    
    unsigned char *msg = mm->msg;
    int metype = mm->metype = msg[4] >> 3;   // Extended squitter message type
    int mesub  = mm->mesub  = (metype == 29 ? ((msg[4]&6)>>1) : (msg[4]  & 7));   // Extended squitter message subtype

    switch (metype) {
    case 1: case 2: case 3: case 4: {
        // Aircraft Identification and Category
        uint32_t chars;

        mm->bFlags |= MODES_ACFLAGS_CALLSIGN_VALID;

        chars = (msg[5] << 16) | (msg[6] << 8) | (msg[7]);
        mm->flight[3] = ais_charset[chars & 0x3F]; chars = chars >> 6;
        mm->flight[2] = ais_charset[chars & 0x3F]; chars = chars >> 6;
        mm->flight[1] = ais_charset[chars & 0x3F]; chars = chars >> 6;
        mm->flight[0] = ais_charset[chars & 0x3F];
        
        chars = (msg[8] << 16) | (msg[9] << 8) | (msg[10]);
        mm->flight[7] = ais_charset[chars & 0x3F]; chars = chars >> 6;
        mm->flight[6] = ais_charset[chars & 0x3F]; chars = chars >> 6;
        mm->flight[5] = ais_charset[chars & 0x3F]; chars = chars >> 6;
        mm->flight[4] = ais_charset[chars & 0x3F];
        
        mm->flight[8] = '\0';
        break;
    }

    case 19: { // Airborne Velocity Message        
        // Presumably airborne if we get an Airborne Velocity Message
        mm->bFlags |= MODES_ACFLAGS_AOG_VALID; 
        
        if ( (mesub >= 1) && (mesub <= 4) ) {
            int vert_rate = ((msg[8] & 0x07) << 6) | (msg[9] >> 2);
            if (vert_rate) {
                --vert_rate;
                if (msg[8] & 0x08) 
                    {vert_rate = 0 - vert_rate;}
                mm->vert_rate =  vert_rate * 64;
                mm->bFlags   |= MODES_ACFLAGS_VERTRATE_VALID;
            }
        }

        if ((mesub == 1) || (mesub == 2)) {
            int ew_raw = ((msg[5] & 0x03) << 8) |  msg[6];
            int ew_vel = ew_raw - 1;
            int ns_raw = ((msg[7] & 0x7F) << 3) | (msg[8] >> 5);
            int ns_vel = ns_raw - 1;
            
            if (mesub == 2) { // If (supersonic) unit is 4 kts
                ns_vel = ns_vel << 2;
                ew_vel = ew_vel << 2;
            }
            
            if (ew_raw) { // Do East/West  
                mm->bFlags |= MODES_ACFLAGS_EWSPEED_VALID;
                if (msg[5] & 0x04)
                    {ew_vel = 0 - ew_vel;}                   
                mm->ew_velocity = ew_vel;
            }
            
            if (ns_raw) { // Do North/South
                mm->bFlags |= MODES_ACFLAGS_NSSPEED_VALID;
                if (msg[7] & 0x80)
                    {ns_vel = 0 - ns_vel;}                   
                mm->ns_velocity = ns_vel;
            }
            
            if (ew_raw && ns_raw) {
                // Compute velocity and angle from the two speed components
                mm->bFlags |= (MODES_ACFLAGS_SPEED_VALID | MODES_ACFLAGS_HEADING_VALID | MODES_ACFLAGS_NSEWSPD_VALID);
                mm->velocity = (int) sqrt((ns_vel * ns_vel) + (ew_vel * ew_vel));
                
                if (mm->velocity) {
                    mm->heading = (int) (atan2(ew_vel, ns_vel) * 180.0 / M_PI);
                    // We don't want negative values but a 0-360 scale
                    if (mm->heading < 0) mm->heading += 360;
                }
            }
            
        } else if (mesub == 3 || mesub == 4) {
            int airspeed = ((msg[7] & 0x7f) << 3) | (msg[8] >> 5);
            if (airspeed) {
                mm->bFlags |= MODES_ACFLAGS_SPEED_VALID;
                --airspeed;
                if (mesub == 4)  // If (supersonic) unit is 4 kts
                    {airspeed = airspeed << 2;}
                mm->velocity =  airspeed;
            }
            
            if (msg[5] & 0x04) {
                mm->bFlags |= MODES_ACFLAGS_HEADING_VALID;
                mm->heading = ((((msg[5] & 0x03) << 8) | msg[6]) * 45) >> 7;
            }
        }

        break;
    }
        
    case 5: case 6: case 7: case 8: {
        // Ground position
        int movement;

        mm->bFlags |= MODES_ACFLAGS_AOG_VALID | MODES_ACFLAGS_AOG;
        mm->raw_latitude  = ((msg[6] & 3) << 15) | (msg[7] << 7) | (msg[8] >> 1);
        mm->raw_longitude = ((msg[8] & 1) << 16) | (msg[9] << 8) | (msg[10]);
        mm->bFlags       |= (mm->msg[6] & 0x04) ? MODES_ACFLAGS_LLODD_VALID 
            : MODES_ACFLAGS_LLEVEN_VALID;

        movement = ((msg[4] << 4) | (msg[5] >> 4)) & 0x007F;
        if ((movement) && (movement < 125)) {
            mm->bFlags |= MODES_ACFLAGS_SPEED_VALID;
            mm->velocity = decodeMovementField(movement);
        }

        if (msg[5] & 0x08) {
            mm->bFlags |= MODES_ACFLAGS_HEADING_VALID;
            mm->heading = ((((msg[5] << 4) | (msg[6] >> 4)) & 0x007F) * 45) >> 4;
        }
        
        break;
    }

    case 0: // Airborne position, baro altitude only
    case 9: case 10: case 11: case 12: case 13: case 14: case 15: case 16: case 17: case 18: // Airborne position, baro
    case 20: case 21: case 22: { // Airborne position, GNSS HAE       
        int AC12Field;

        mm->bFlags |= MODES_ACFLAGS_AOG_VALID;

        if (metype != 0) {
            mm->raw_latitude  = ((msg[6] & 3) << 15) | (msg[7] << 7) | (msg[8] >> 1);
            mm->raw_longitude = ((msg[8] & 1) << 16) | (msg[9] << 8) | (msg[10]);
            mm->bFlags       |= (mm->msg[6] & 0x04) ? MODES_ACFLAGS_LLODD_VALID 
                : MODES_ACFLAGS_LLEVEN_VALID;
        }

        AC12Field = ((msg[5] << 4) | (msg[6] >> 4)) & 0x0FFF;
        if (AC12Field) {// Only attempt to decode if a valid (non zero) altitude is present
            mm->bFlags |= MODES_ACFLAGS_ALTITUDE_VALID;
            mm->altitude = decodeAC12Field(AC12Field, &mm->unit);
        }
        
        break;
    }

    case 23: { // Test message
        if (mesub == 7) {		// (see 1090-WP-15-20)
            int ID13Field = (((msg[5] << 8) | msg[6]) & 0xFFF1)>>3;
            if (ID13Field) {
                mm->bFlags |= MODES_ACFLAGS_SQUAWK_VALID;
                mm->modeA   = decodeID13Field(ID13Field);
            }
        }
        break;
    }

    case 24: // Reserved for Surface System Status
        break;

    case 28: { // Extended Squitter Aircraft Status
        if (mesub == 1) {      // Emergency status squawk field
            int ID13Field = (((msg[5] << 8) | msg[6]) & 0x1FFF);
            if (ID13Field) {
                mm->bFlags |= MODES_ACFLAGS_SQUAWK_VALID;
                mm->modeA   = decodeID13Field(ID13Field);
            }
        }
        break;
    }

    case 29: // Aircraft Trajectory Intent
        break;

    case 30: // Aircraft Operational Coordination
        break;

    case 31: // Aircraft Operational Status
        break;

    default: 
        break;
    }
}

static void decodeCommB(struct modesMessage *mm)
{    
    unsigned char *msg = mm->msg;

    // This is a bit hairy as we don't know what the requested register was
    if (msg[4] == 0x20) { // BDS 2,0 Aircraft Identification
        uint32_t chars;
        mm->bFlags |= MODES_ACFLAGS_CALLSIGN_VALID;
        
        chars = (msg[5] << 16) | (msg[6] << 8) | (msg[7]);
        mm->flight[3] = ais_charset[chars & 0x3F]; chars = chars >> 6;
        mm->flight[2] = ais_charset[chars & 0x3F]; chars = chars >> 6;
        mm->flight[1] = ais_charset[chars & 0x3F]; chars = chars >> 6;
        mm->flight[0] = ais_charset[chars & 0x3F];
        
        chars = (msg[8] << 16) | (msg[9] << 8) | (msg[10]);
        mm->flight[7] = ais_charset[chars & 0x3F]; chars = chars >> 6;
        mm->flight[6] = ais_charset[chars & 0x3F]; chars = chars >> 6;
        mm->flight[5] = ais_charset[chars & 0x3F]; chars = chars >> 6;
        mm->flight[4] = ais_charset[chars & 0x3F];
        
        mm->flight[8] = '\0';
    }
}
//
//=========================================================================
//
// This function gets a decoded Mode S Message and prints it on the screen
// in a human readable format.
//
void displayModesMessage(struct modesMessage *mm) {
    int j;
    unsigned char * pTimeStamp;

    // Handle only addresses mode first.
    if (Modes.onlyaddr) {
        printf("%06x\n", mm->addr);
        return;         // Enough for --onlyaddr mode
    }

    // Show the raw message.
    if (Modes.mlat && mm->timestampMsg) {
        printf("@");
        pTimeStamp = (unsigned char *) &mm->timestampMsg;
        for (j=5; j>=0;j--) {
            printf("%02X",pTimeStamp[j]);
        } 
    } else
        printf("*");

    for (j = 0; j < mm->msgbits/8; j++) printf("%02x", mm->msg[j]);
    printf(";\n");

    if (Modes.raw) {
        fflush(stdout); // Provide data to the reader ASAP
        return;         // Enough for --raw mode
    }

    if (mm->msgtype < 32)
        printf("CRC: %06x\n", (int)mm->crc);

    if (mm->correctedbits != 0)
        printf("No. of bit errors fixed: %d\n", mm->correctedbits);

    printf("SNR: %d.%d dB\n", mm->signalLevel/5, 2*(mm->signalLevel%5));
    if (mm->score) printf("Score: %d\n", mm->score);

    if (mm->timestampMsg)
        printf("Time: %.2fus (phase: %d)\n", mm->timestampMsg / 12.0, (unsigned int) (360 * (mm->timestampMsg % 6) / 6));

    if (mm->msgtype == 0) { // DF 0
        printf("DF 0: Short Air-Air Surveillance.\n");
        printf("  VS             : %s\n",  (mm->msg[0] & 0x04) ? "Ground" : "Airborne");
        printf("  CC             : %d\n", ((mm->msg[0] & 0x02) >> 1));
        printf("  SL             : %d\n", ((mm->msg[1] & 0xE0) >> 5));
        printf("  Altitude       : %d %s\n", mm->altitude,
            (mm->unit == MODES_UNIT_METERS) ? "meters" : "feet");
        printf("  ICAO Address   : %06x\n", mm->addr);

    } else if (mm->msgtype == 4 || mm->msgtype == 20) {
        printf("DF %d: %s, Altitude Reply.\n", mm->msgtype,
            (mm->msgtype == 4) ? "Surveillance" : "Comm-B");
        printf("  Flight Status  : %s\n", fs_str[mm->fs]);
        printf("  DR             : %d\n", ((mm->msg[1] >> 3) & 0x1F));
        printf("  UM             : %d\n", (((mm->msg[1]  & 7) << 3) | (mm->msg[2] >> 5)));
        printf("  Altitude       : %d %s\n", mm->altitude,
            (mm->unit == MODES_UNIT_METERS) ? "meters" : "feet");
        printf("  ICAO Address   : %06x\n", mm->addr);

        if (mm->msgtype == 20) {
            if (mm->bds != 0)
                printf("  Comm-B BDS     : %02x\n", mm->bds);

            // Decode the extended squitter message
            if        ( mm->msg[4]       == 0x20) { // BDS 2,0 Aircraft identification
                printf("    BDS 2,0 Aircraft Identification : %s\n", mm->flight);
/*
            } else if ( mm->msg[4]       == 0x10) { // BDS 1,0 Datalink Capability report
                printf("    BDS 1,0 Datalink Capability report\n");

            } else if ( mm->msg[4]       == 0x30) { // BDS 3,0 ACAS Active Resolution Advisory
                printf("    BDS 3,0 ACAS Active Resolution Advisory\n");

            } else if ((mm->msg[4] >> 3) ==   28) { // BDS 6,1 Extended Squitter Emergency/Priority Status
                printf("    BDS 6,1 Emergency/Priority Status\n");

            } else if ((mm->msg[4] >> 3) ==   29) { // BDS 6,2 Target State and Status
                printf("    BDS 6,2 Target State and Status\n");

            } else if ((mm->msg[4] >> 3) ==   31) { // BDS 6,5 Extended Squitter Aircraft Operational Status
                printf("    BDS 6,5 Aircraft Operational Status\n");
*/
            }        
        }

    } else if (mm->msgtype == 5 || mm->msgtype == 21) {
        printf("DF %d: %s, Identity Reply.\n", mm->msgtype,
            (mm->msgtype == 5) ? "Surveillance" : "Comm-B");
        printf("  Flight Status  : %s\n", fs_str[mm->fs]);
        printf("  DR             : %d\n", ((mm->msg[1] >> 3) & 0x1F));
        printf("  UM             : %d\n", (((mm->msg[1]  & 7) << 3) | (mm->msg[2] >> 5)));
        printf("  Squawk         : %04x\n", mm->modeA);
        printf("  ICAO Address   : %06x\n", mm->addr);

        if (mm->msgtype == 21) {
            if (mm->bds != 0)
                printf("  Comm-B BDS     : %02x\n", mm->bds);

            // Decode the extended squitter message
            if        ( mm->msg[4]       == 0x20) { // BDS 2,0 Aircraft identification
                printf("    BDS 2,0 Aircraft Identification : %s\n", mm->flight);
/*
            } else if ( mm->msg[4]       == 0x10) { // BDS 1,0 Datalink Capability report
                printf("    BDS 1,0 Datalink Capability report\n");

            } else if ( mm->msg[4]       == 0x30) { // BDS 3,0 ACAS Active Resolution Advisory
                printf("    BDS 3,0 ACAS Active Resolution Advisory\n");

            } else if ((mm->msg[4] >> 3) ==   28) { // BDS 6,1 Extended Squitter Emergency/Priority Status
                printf("    BDS 6,1 Emergency/Priority Status\n");

            } else if ((mm->msg[4] >> 3) ==   29) { // BDS 6,2 Target State and Status
                printf("    BDS 6,2 Target State and Status\n");

            } else if ((mm->msg[4] >> 3) ==   31) { // BDS 6,5 Extended Squitter Aircraft Operational Status
                printf("    BDS 6,5 Aircraft Operational Status\n");
*/
            }        
        }

    } else if (mm->msgtype == 11) { // DF 11
        printf("DF 11: All Call Reply.\n");
        printf("  Capability  : %d (%s)\n", mm->ca, ca_str[mm->ca]);
        printf("  ICAO Address: %06x\n", mm->addr);
        if (mm->iid > 16)
            {printf("  IID         : SI-%02d\n", mm->iid-16);}
        else
            {printf("  IID         : II-%02d\n", mm->iid);}

    } else if (mm->msgtype == 16) { // DF 16
        printf("DF 16: Long Air to Air ACAS\n");
        printf("  VS             : %s\n",  (mm->msg[0] & 0x04) ? "Ground" : "Airborne");
        printf("  CC             : %d\n", ((mm->msg[0] & 0x02) >> 1));
        printf("  SL             : %d\n", ((mm->msg[1] & 0xE0) >> 5));
        printf("  Altitude       : %d %s\n", mm->altitude,
            (mm->unit == MODES_UNIT_METERS) ? "meters" : "feet");
        printf("  ICAO Address   : %06x\n", mm->addr);

    } else if (mm->msgtype == 17) { // DF 17
        printf("DF 17: ADS-B message.\n");
        printf("  Capability     : %d (%s)\n", mm->ca, ca_str[mm->ca]);
        printf("  ICAO Address   : %06x\n", mm->addr);
        printf("  Extended Squitter  Type: %d\n", mm->metype);
        printf("  Extended Squitter  Sub : %d\n", mm->mesub);
        printf("  Extended Squitter  Name: %s\n", getMEDescription(mm->metype, mm->mesub));

        // Decode the extended squitter message
        if (mm->metype >= 1 && mm->metype <= 4) { // Aircraft identification
            printf("    Aircraft Type  : %c%d\n", ('A' + 4 - mm->metype), mm->mesub);
            printf("    Identification : %s\n", mm->flight);

        } else if (mm->metype == 19) { // Airborne Velocity
            if (mm->mesub == 1 || mm->mesub == 2) {
                printf("    EW status         : %s\n", (mm->bFlags & MODES_ACFLAGS_EWSPEED_VALID)  ? "Valid" : "Unavailable");
                printf("    EW velocity       : %d\n", mm->ew_velocity);
                printf("    NS status         : %s\n", (mm->bFlags & MODES_ACFLAGS_NSSPEED_VALID)  ? "Valid" : "Unavailable");
                printf("    NS velocity       : %d\n", mm->ns_velocity);
                printf("    Vertical status   : %s\n", (mm->bFlags & MODES_ACFLAGS_VERTRATE_VALID) ? "Valid" : "Unavailable");
                printf("    Vertical rate src : %d\n", ((mm->msg[8] >> 4) & 1));
                printf("    Vertical rate     : %d\n", mm->vert_rate);

            } else if (mm->mesub == 3 || mm->mesub == 4) {
                printf("    Heading status    : %s\n", (mm->bFlags & MODES_ACFLAGS_HEADING_VALID)  ? "Valid" : "Unavailable");
                printf("    Heading           : %d\n", mm->heading);
                printf("    Airspeed status   : %s\n", (mm->bFlags & MODES_ACFLAGS_SPEED_VALID)    ? "Valid" : "Unavailable");
                printf("    Airspeed          : %d\n", mm->velocity);
                printf("    Vertical status   : %s\n", (mm->bFlags & MODES_ACFLAGS_VERTRATE_VALID) ? "Valid" : "Unavailable");
                printf("    Vertical rate src : %d\n", ((mm->msg[8] >> 4) & 1));
                printf("    Vertical rate     : %d\n", mm->vert_rate);

            } else {
                printf("    Unrecognized ME subtype: %d subtype: %d\n", mm->metype, mm->mesub);
            }

        } else if (mm->metype >= 5 && mm->metype <= 22) { // Airborne position Baro
            printf("    F flag   : %s\n", (mm->msg[6] & 0x04) ? "odd" : "even");
            printf("    T flag   : %s\n", (mm->msg[6] & 0x08) ? "UTC" : "non-UTC");
            printf("    Altitude : %d feet\n", mm->altitude);
            if (mm->bFlags & MODES_ACFLAGS_LATLON_VALID) {
                printf("    Latitude : %f\n", mm->fLat);
                printf("    Longitude: %f\n", mm->fLon);
            } else {
                printf("    Latitude : %d (not decoded)\n", mm->raw_latitude);
                printf("    Longitude: %d (not decoded)\n", mm->raw_longitude);
            }

        } else if (mm->metype == 28) { // Extended Squitter Aircraft Status
            if (mm->mesub == 1) {
				printf("    Emergency State: %s\n", es_str[(mm->msg[5] & 0xE0) >> 5]);
				printf("    Squawk: %04x\n", mm->modeA);
            } else {
                printf("    Unrecognized ME subtype: %d subtype: %d\n", mm->metype, mm->mesub);
            }

        } else if (mm->metype == 23) { // Test Message
			if (mm->mesub == 7) {
				printf("    Squawk: %04x\n", mm->modeA);
            } else {
                printf("    Unrecognized ME subtype: %d subtype: %d\n", mm->metype, mm->mesub);
			}
        } else {
            printf("    Unrecognized ME type: %d subtype: %d\n", mm->metype, mm->mesub);
        }

    } else if (mm->msgtype == 18) { // DF 18 
        printf("DF 18: Extended Squitter.\n");
        printf("  Control Field : %d (%s)\n", mm->ca, cf_str[mm->ca]);
        if ((mm->ca == 0) || (mm->ca == 1) || (mm->ca == 6)) {
            if (mm->ca == 1) {
                printf("  Other Address : %06x\n", mm->addr);
            } else {
                printf("  ICAO Address  : %06x\n", mm->addr);
            }
            printf("  Extended Squitter  Type: %d\n", mm->metype);
            printf("  Extended Squitter  Sub : %d\n", mm->mesub);
            printf("  Extended Squitter  Name: %s\n", getMEDescription(mm->metype, mm->mesub));

            // Decode the extended squitter message
            if (mm->metype >= 1 && mm->metype <= 4) { // Aircraft identification
                printf("    Aircraft Type  : %c%d\n", ('A' + 4 - mm->metype), mm->mesub);
                printf("    Identification : %s\n", mm->flight);

            } else if (mm->metype == 19) { // Airborne Velocity
                if (mm->mesub == 1 || mm->mesub == 2) {
                    printf("    EW status         : %s\n", (mm->bFlags & MODES_ACFLAGS_EWSPEED_VALID)  ? "Valid" : "Unavailable");
                    printf("    EW velocity       : %d\n", mm->ew_velocity);
                    printf("    NS status         : %s\n", (mm->bFlags & MODES_ACFLAGS_NSSPEED_VALID)  ? "Valid" : "Unavailable");
                    printf("    NS velocity       : %d\n", mm->ns_velocity);
                    printf("    Vertical status   : %s\n", (mm->bFlags & MODES_ACFLAGS_VERTRATE_VALID) ? "Valid" : "Unavailable");
                    printf("    Vertical rate src : %d\n", ((mm->msg[8] >> 4) & 1));
                    printf("    Vertical rate     : %d\n", mm->vert_rate);

                } else if (mm->mesub == 3 || mm->mesub == 4) {
                    printf("    Heading status    : %s\n", (mm->bFlags & MODES_ACFLAGS_HEADING_VALID)  ? "Valid" : "Unavailable");
                    printf("    Heading           : %d\n", mm->heading);
                    printf("    Airspeed status   : %s\n", (mm->bFlags & MODES_ACFLAGS_SPEED_VALID)    ? "Valid" : "Unavailable");
                    printf("    Airspeed          : %d\n", mm->velocity);
                    printf("    Vertical status   : %s\n", (mm->bFlags & MODES_ACFLAGS_VERTRATE_VALID) ? "Valid" : "Unavailable");
                    printf("    Vertical rate src : %d\n", ((mm->msg[8] >> 4) & 1));
                    printf("    Vertical rate     : %d\n", mm->vert_rate);

                } else {
                    printf("    Unrecognized ME subtype: %d subtype: %d\n", mm->metype, mm->mesub);
                }

            } else if (mm->metype >= 5 && mm->metype <= 22) { // Ground or Airborne position, Baro or GNSS
                printf("    F flag   : %s\n", (mm->msg[6] & 0x04) ? "odd" : "even");
                printf("    T flag   : %s\n", (mm->msg[6] & 0x08) ? "UTC" : "non-UTC");
                printf("    Altitude : %d feet\n", mm->altitude);
                if (mm->bFlags & MODES_ACFLAGS_LATLON_VALID) {
                    printf("    Latitude : %f\n", mm->fLat);
                    printf("    Longitude: %f\n", mm->fLon);
                } else {
                    printf("    Latitude : %d (not decoded)\n", mm->raw_latitude);
                    printf("    Longitude: %d (not decoded)\n", mm->raw_longitude);
                }

            } else {
                printf("    Unrecognized ME type: %d subtype: %d\n", mm->metype, mm->mesub);
            }
        }             

    } else if (mm->msgtype == 19) { // DF 19
        printf("DF 19: Military Extended Squitter.\n");

    } else if (mm->msgtype == 22) { // DF 22
        printf("DF 22: Military Use.\n");

    } else if (mm->msgtype == 24) { // DF 24
        printf("DF 24: Comm D Extended Length Message.\n");

    } else if (mm->msgtype == 32) { // DF 32 is special code we use for Mode A/C
        printf("SSR : Mode A/C Reply.\n");
        if (mm->fs & 0x0080) {
            printf("  Mode A : %04x IDENT\n", mm->modeA);
        } else {
            printf("  Mode A : %04x\n", mm->modeA);
            if (mm->bFlags & MODES_ACFLAGS_ALTITUDE_VALID)
                {printf("  Mode C : %d feet\n", mm->altitude);}
        }

    } else {
        printf("DF %d: Unknown DF Format.\n", mm->msgtype);
    }

    printf("\n");
}
//
//=========================================================================
//
// Turn I/Q samples pointed by Modes.data into the magnitude vector
// pointed by Modes.magnitude.
//
void computeMagnitudeVector(uint16_t *p) {
    uint16_t *m = &Modes.magnitude[Modes.trailing_space];
    uint32_t j;

    memcpy(Modes.magnitude,&Modes.magnitude[MODES_ASYNC_BUF_SAMPLES], Modes.trailing_space);

    // Compute the magnitudo vector. It's just SQRT(I^2 + Q^2), but
    // we rescale to the 0-255 range to exploit the full resolution.
    for (j = 0; j < MODES_ASYNC_BUF_SAMPLES; j ++) {
        *m++ = Modes.maglut[*p++];
    }
}

//
//=========================================================================
//
// Return -1 if the message is out of fase left-side
// Return  1 if the message is out of fase right-size
// Return  0 if the message is not particularly out of phase.
//
// Note: this function will access pPreamble[-1], so the caller should make sure to
// call it only if we are not at the start of the current buffer
//
int detectOutOfPhase(uint16_t *pPreamble) {
    if (pPreamble[ 3] > pPreamble[2]/3) return  1;
    if (pPreamble[10] > pPreamble[9]/3) return  1;
    if (pPreamble[ 6] > pPreamble[7]/3) return -1;
    if (pPreamble[-1] > pPreamble[1]/3) return -1;
    return 0;
}


uint16_t clamped_scale(uint16_t v, uint16_t scale) {
    uint32_t scaled = (uint32_t)v * scale / 16384;
    if (scaled > 65535) return 65535;
    return (uint16_t) scaled;
}
// This function decides whether we are sampling early or late,
// and by approximately how much, by looking at the energy in
// preamble bits before and after the expected pulse locations.
//
// It then deals with one sample pair at a time, comparing samples
// to make a decision about the bit value. Based on this decision it
// modifies the sample value of the *adjacent* sample which will
// contain some of the energy from the bit we just inspected.
//
// pPayload[0] should be the start of the preamble,
// pPayload[-1 .. MODES_PREAMBLE_SAMPLES + MODES_LONG_MSG_SAMPLES - 1] should be accessible.
// pPayload[MODES_PREAMBLE_SAMPLES .. MODES_PREAMBLE_SAMPLES + MODES_LONG_MSG_SAMPLES - 1] will be updated.
void applyPhaseCorrection(uint16_t *pPayload) {
    int j;

    // we expect 1 bits at 0, 2, 7, 9
    // and 0 bits at -1, 1, 3, 4, 5, 6, 8, 10, 11, 12, 13, 14
    // use bits -1,6 for early detection (bit 0/7 arrived a little early, our sample period starts after the bit phase so we include some of the next bit)
    // use bits 3,10 for late detection (bit 2/9 arrived a little late, our sample period starts before the bit phase so we include some of the last bit)

    uint32_t onTime = (pPayload[0] + pPayload[2] + pPayload[7] + pPayload[9]);
    uint32_t early = (pPayload[-1] + pPayload[6]) << 1;
    uint32_t late = (pPayload[3] + pPayload[10]) << 1;

    if (early > late) {
        // Our sample period starts late and so includes some of the next bit.

        uint16_t scaleUp = 16384 + 16384 * early / (early + onTime);   // 1 + early / (early+onTime)
        uint16_t scaleDown = 16384 - 16384 * early / (early + onTime); // 1 - early / (early+onTime)

        // trailing bits are 0; final data sample will be a bit low.
        pPayload[MODES_PREAMBLE_SAMPLES + MODES_LONG_MSG_SAMPLES - 1] =
            clamped_scale(pPayload[MODES_PREAMBLE_SAMPLES + MODES_LONG_MSG_SAMPLES - 1],  scaleUp);
        for (j = MODES_PREAMBLE_SAMPLES + MODES_LONG_MSG_SAMPLES - 2; j > MODES_PREAMBLE_SAMPLES; j -= 2) {
            if (pPayload[j] > pPayload[j+1]) {
                // x [1 0] y
                // x overlapped with the "1" bit and is slightly high
                pPayload[j-1] = clamped_scale(pPayload[j-1], scaleDown);
            } else {
                // x [0 1] y
                // x overlapped with the "0" bit and is slightly low
                pPayload[j-1] = clamped_scale(pPayload[j-1], scaleUp);
            }
        }
    } else {
        // Our sample period starts early and so includes some of the previous bit.

        uint16_t scaleUp = 16384 + 16384 * late / (late + onTime);   // 1 + late / (late+onTime)
        uint16_t scaleDown = 16384 - 16384 * late / (late + onTime); // 1 - late / (late+onTime)

        // leading bits are 0; first data sample will be a bit low.
        pPayload[MODES_PREAMBLE_SAMPLES] = clamped_scale(pPayload[MODES_PREAMBLE_SAMPLES], scaleUp);
        for (j = MODES_PREAMBLE_SAMPLES; j < MODES_PREAMBLE_SAMPLES + MODES_LONG_MSG_SAMPLES - 2; j += 2) {
            if (pPayload[j] > pPayload[j+1]) {
                // x [1 0] y
                // y overlapped with the "0" bit and is slightly low
                pPayload[j+2] = clamped_scale(pPayload[j+2], scaleUp);
            } else {
                // x [0 1] y
                // y overlapped with the "1" bit and is slightly high
                pPayload[j+2] = clamped_scale(pPayload[j+2], scaleDown);
            }
        }
    }
}
//
//=========================================================================
//
// Detect a Mode S messages inside the magnitude buffer pointed by 'm' and of
// size 'mlen' bytes. Every detected Mode S message is convert it into a
// stream of bits and passed to the function to display it.
//
void detectModeS(uint16_t *m, uint32_t mlen) {
    struct modesMessage mm;
    unsigned char msg[MODES_LONG_MSG_BYTES], *pMsg;
    uint16_t aux[MODES_PREAMBLE_SAMPLES+MODES_LONG_MSG_SAMPLES+1];
    uint32_t j;
    int use_correction = 0;

    memset(&mm, 0, sizeof(mm));

    // The Mode S preamble is made of impulses of 0.5 microseconds at
    // the following time offsets:
    //
    // 0   - 0.5 usec: first impulse.
    // 1.0 - 1.5 usec: second impulse.
    // 3.5 - 4   usec: third impulse.
    // 4.5 - 5   usec: last impulse.
    // 
    // Since we are sampling at 2 Mhz every sample in our magnitude vector
    // is 0.5 usec, so the preamble will look like this, assuming there is
    // an impulse at offset 0 in the array:
    //
    // 0   -----------------
    // 1   -
    // 2   ------------------
    // 3   --
    // 4   -
    // 5   --
    // 6   -
    // 7   ------------------
    // 8   --
    // 9   -------------------
    //
    for (j = 0; j < mlen; j++) {
        int high, i, errors, errors56, errorsTy; 
        uint16_t *pPreamble, *pPayload, *pPtr;
        uint8_t  theByte, theErrs;
        int msglen, scanlen;
        uint32_t sigLevel, noiseLevel;
        uint16_t snr;
        int message_ok;

        pPreamble = &m[j];
        pPayload  = &m[j+MODES_PREAMBLE_SAMPLES];

        // Rather than clear the whole mm structure, just clear the parts which are required. The clear
        // is required for every bit of the input stream, and we don't want to be memset-ing the whole
        // modesMessage structure two million times per second if we don't have to..
        mm.bFlags          =
        mm.correctedbits   = 0;

        if (!use_correction)  // This is not a re-try with phase correction
            {                 // so try to find a new preamble

            if (Modes.mode_ac) 
                {
                int ModeA = detectModeA(pPreamble, &mm);

                if (ModeA) // We have found a valid ModeA/C in the data                    
                    {
                    mm.timestampMsg = Modes.timestampBlk + ((j+1) * 6);

                    // Decode the received message
                    decodeModeAMessage(&mm, ModeA);

                    // Pass data to the next layer
                    useModesMessage(&mm);

                    j += MODEAC_MSG_SAMPLES;
                    Modes.stat_ModeAC++;
                    continue;
                    }
                }

            // First check of relations between the first 10 samples
            // representing a valid preamble. We don't even investigate further
            // if this simple test is not passed
            if (!(pPreamble[0] > pPreamble[1] &&
                  pPreamble[1] < pPreamble[2] &&
                  pPreamble[2] > pPreamble[3] &&
                  pPreamble[3] < pPreamble[0] &&
                  pPreamble[4] < pPreamble[0] &&
                  pPreamble[5] < pPreamble[0] &&
                  pPreamble[6] < pPreamble[0] &&
                  pPreamble[7] > pPreamble[8] &&
                  pPreamble[8] < pPreamble[9] &&
                  pPreamble[9] > pPreamble[6]))
            {
                if (Modes.debug & MODES_DEBUG_NOPREAMBLE &&
                    *pPreamble  > MODES_DEBUG_NOPREAMBLE_LEVEL)
                    dumpRawMessage("Unexpected ratio among first 10 samples", msg, m, j);
                continue;
            }

            // The samples between the two spikes must be < than the average
            // of the high spikes level. We don't test bits too near to
            // the high levels as signals can be out of phase so part of the
            // energy can be in the near samples
            high = (pPreamble[0] + pPreamble[2] + pPreamble[7] + pPreamble[9]) / 6;
            if (pPreamble[4] >= high ||
                pPreamble[5] >= high)
            {
                if (Modes.debug & MODES_DEBUG_NOPREAMBLE &&
                    *pPreamble  > MODES_DEBUG_NOPREAMBLE_LEVEL)
                    dumpRawMessage("Too high level in samples between 3 and 6", msg, m, j);
                continue;
            }

            // Similarly samples in the range 11-14 must be low, as it is the
            // space between the preamble and real data. Again we don't test
            // bits too near to high levels, see above
            if (pPreamble[11] >= high ||
                pPreamble[12] >= high ||
                pPreamble[13] >= high ||
                pPreamble[14] >= high)
            {
                if (Modes.debug & MODES_DEBUG_NOPREAMBLE &&
                    *pPreamble  > MODES_DEBUG_NOPREAMBLE_LEVEL)
                    dumpRawMessage("Too high level in samples between 10 and 15", msg, m, j);
                continue;
            }
            Modes.stat_valid_preamble++;
        } 

        else {
            // If the previous attempt with this message failed, retry using
            // magnitude correction
            // Make a copy of the Payload, and phase correct the copy
            memcpy(aux, &pPreamble[-1], sizeof(aux));
            applyPhaseCorrection(&aux[1]);
            Modes.stat_out_of_phase++;
            pPayload = &aux[1 + MODES_PREAMBLE_SAMPLES];
            // TODO ... apply other kind of corrections
            }

        // Decode all the next 112 bits, regardless of the actual message
        // size. We'll check the actual message type later
        pMsg    = &msg[0];
        pPtr    = pPayload;
        theByte = 0;
        theErrs = 0; errorsTy = 0;
        errors  = 0; errors56 = 0;

        // We should have 4 'bits' of 0/1 and 1/0 samples in the preamble, 
        // so include these in the signal strength 
        sigLevel = pPreamble[0] + pPreamble[2] + pPreamble[7] + pPreamble[9];
        noiseLevel = pPreamble[1] + pPreamble[3] + pPreamble[4] + pPreamble[6] + pPreamble[8];

        msglen = scanlen = MODES_LONG_MSG_BITS;
        for (i = 0; i < scanlen; i++) {
            uint32_t a = *pPtr++;
            uint32_t b = *pPtr++;

            if      (a > b) 
                {theByte |= 1; if (i < 56) { sigLevel += a; noiseLevel += b; }}
            else if (a < b) 
                {/*theByte |= 0;*/ if (i < 56) { sigLevel += b; noiseLevel += a; }}
            else {
                if (i < 56) { sigLevel += a; noiseLevel += a; }
                if (i >= MODES_SHORT_MSG_BITS) //(a == b), and we're in the long part of a frame
                    {errors++;  /*theByte |= 0;*/}
                else if (i >= 5)                    //(a == b), and we're in the short part of a frame
                    {scanlen = MODES_LONG_MSG_BITS; errors56 = ++errors;/*theByte |= 0;*/}
                else if (i)                         //(a == b), and we're in the message type part of a frame
                    {errorsTy = errors56 = ++errors; theErrs |= 1; /*theByte |= 0;*/}
                else                                //(a == b), and we're in the first bit of the message type part of a frame
                    {errorsTy = errors56 = ++errors; theErrs |= 1; theByte |= 1;}
            }

            if ((i & 7) == 7) 
              {*pMsg++ = theByte;}
            else if (i == 4) {
              msglen  = modesMessageLenByType(theByte);
              if (errors == 0)
                  {scanlen = msglen;}
            }

            theByte = theByte << 1;
            if (i < 7)
              {theErrs = theErrs << 1;}

            // If we've exceeded the permissible number of encoding errors, abandon ship now
            if (errors > MODES_MSG_ENCODER_ERRS) {

                if        (i < MODES_SHORT_MSG_BITS) {
                    msglen = 0;

                } else if ((errorsTy == 1) && (theErrs == 0x80)) {
                    // If we only saw one error in the first bit of the byte of the frame, then it's possible 
                    // we guessed wrongly about the value of the bit. We may be able to correct it by guessing
                    // the other way.
                    //
                    // We guessed a '1' at bit 7, which is the DF length bit == 112 Bits.
                    // Inverting bit 7 will change the message type from a long to a short. 
                    // Invert the bit, cross your fingers and carry on.
                    msglen  = MODES_SHORT_MSG_BITS;
                    msg[0] ^= theErrs; errorsTy = 0;
                    errors  = errors56; // revert to the number of errors prior to bit 56
                    Modes.stat_DF_Len_Corrected++;

                } else if (i < MODES_LONG_MSG_BITS) {
                    msglen = MODES_SHORT_MSG_BITS;
                    errors = errors56;

                } else {
                    msglen = MODES_LONG_MSG_BITS;
                }

            break;
            }
        }

        // Ensure msglen is consistent with the DF type
        i = modesMessageLenByType(msg[0] >> 3);
        if      (msglen > i) {msglen = i;}
        else if (msglen < i) {msglen = 0;}

        //
        // If we guessed at any of the bits in the DF type field, then look to see if our guess was sensible.
        // Do this by looking to see if the original guess results in the DF type being one of the ICAO defined
        // message types. If it isn't then toggle the guessed bit and see if this new value is ICAO defined.
        // if the new value is ICAO defined, then update it in our message.
        if ((msglen) && (errorsTy == 1) && (theErrs & 0x78)) {
            // We guessed at one (and only one) of the message type bits. See if our guess is "likely" 
            // to be correct by comparing the DF against a list of known good DF's
            int      thisDF      = ((theByte = msg[0]) >> 3) & 0x1f;
            uint32_t validDFbits = 0x017F0831;   // One bit per 32 possible DF's. Set bits 0,4,5,11,16.17.18.19,20,21,22,24
            uint32_t thisDFbit   = (1 << thisDF);
            if (0 == (validDFbits & thisDFbit)) {
                // The current DF is not ICAO defined, so is probably an errors. 
                // Toggle the bit we guessed at and see if the resultant DF is more likely
                theByte  ^= theErrs;
                thisDF    = (theByte >> 3) & 0x1f;
                thisDFbit = (1 << thisDF);
                // if this DF any more likely?
                if (validDFbits & thisDFbit) {
                    // Yep, more likely, so update the main message 
                    msg[0] = theByte;
                    Modes.stat_DF_Type_Corrected++;
                    errors--; // decrease the error count so we attempt to use the modified DF.
                }
            }
        }

        // snr = 5 * 20log10(sigLevel / noiseLevel)         (in units of 0.2dB)
        //     = 100log10(sigLevel) - 100log10(noiseLevel)

        while (sigLevel > 65535 || noiseLevel > 65535) {
            sigLevel >>= 1;
            noiseLevel >>= 1;
        }
        snr = Modes.log10lut[sigLevel] - Modes.log10lut[noiseLevel];

        // When we reach this point, if error is small, and the signal strength is large enough
        // we may have a Mode S message on our hands. It may still be broken and the CRC may not 
        // be correct, but this can be handled by the next layer.
        if ( (msglen) 
          && ((2 * snr) > (int) (MODES_MSG_SQUELCH_DB * 10))
          && (errors      <= MODES_MSG_ENCODER_ERRS) ) {
            // Set initial mm structure details
            mm.timestampMsg = Modes.timestampBlk + (j*6);
            mm.signalLevel = (snr > 255 ? 255 : (uint8_t)snr);
            mm.phase_corrected = use_correction;

            // Decode the received message
            message_ok = (decodeModesMessage(&mm, msg) == 0);

            // Update statistics
            if (Modes.stats) {
                struct demod_stats *dstats = (use_correction ? &Modes.stat_demod_phasecorrected : &Modes.stat_demod);

                switch (errors) {
                case 0:  dstats->demodulated0++; break;
                case 1:  dstats->demodulated1++; break;
                case 2:  dstats->demodulated2++; break;
                default: dstats->demodulated3++; break;
                }
                
                if (!message_ok) {
                    dstats->badcrc++;
                } else if (mm.correctedbits == 0) {
                    dstats->goodcrc++;
                    dstats->goodcrc_byphase[0]++;
                } else {
                    dstats->badcrc++;                    
                    dstats->fixed++;
                    if (mm.correctedbits <= MODES_MAX_BITERRORS)
                        dstats->bit_fix[mm.correctedbits-1] += 1;
                }
            }

            // Output debug mode info if needed
            if (use_correction) {
                if (Modes.debug & MODES_DEBUG_DEMOD)
                    dumpRawMessage("Demodulated with 0 errors", msg, m, j);
                else if (Modes.debug & MODES_DEBUG_BADCRC && mm.correctedbits != 0)
                    dumpRawMessage("Decoded with corrected CRC", msg, m, j);
                else if (Modes.debug & MODES_DEBUG_GOODCRC && mm.correctedbits == 0)
                    dumpRawMessage("Decoded with good CRC", msg, m, j);
            }

            // Skip this message if we are sure it's fine
            if (message_ok) {
                j += (MODES_PREAMBLE_US+msglen)*2 - 1;

                // Pass data to the next layer
                useModesMessage(&mm);
            }

        } else {
            message_ok = 0;
            if (Modes.debug & MODES_DEBUG_DEMODERR && use_correction) {
                printf("The following message has %d demod errors\n", errors);
                dumpRawMessage("Demodulated with errors", msg, m, j);
            }
        }

        // Retry with phase correction if enabled, necessary and possible.
        if (Modes.phase_enhance && (!message_ok || mm.correctedbits > 0) && !use_correction && j && detectOutOfPhase(pPreamble)) {
            use_correction = 1; j--;
        } else {
            use_correction = 0; 
        }
    }
}

//
//=========================================================================
//
// When a new message is available, because it was decoded from the RTL device, 
// file, or received in the TCP input port, or any other way we can receive a 
// decoded message, we call this function in order to use the message.
//
// Basically this function passes a raw message to the upper layers for further
// processing and visualization
//
void useModesMessage(struct modesMessage *mm) {
    // If we are decoding, track aircraft
    interactiveReceiveData(mm);
    
    // In non-interactive non-quiet mode, display messages on standard output
    if (!Modes.interactive && !Modes.quiet) {
        displayModesMessage(mm);
    }
    
    // Feed output clients
    if (Modes.net) {modesQueueOutput(mm);}
}
//
//=========================================================================
//
// Always positive MOD operation, used for CPR decoding.
//
int cprModFunction(int a, int b) {
    int res = a % b;
    if (res < 0) res += b;
    return res;
}
//
//=========================================================================
//
// The NL function uses the precomputed table from 1090-WP-9-14
//
int cprNLFunction(double lat) {
    if (lat < 0) lat = -lat; // Table is simmetric about the equator
    if (lat < 10.47047130) return 59;
    if (lat < 14.82817437) return 58;
    if (lat < 18.18626357) return 57;
    if (lat < 21.02939493) return 56;
    if (lat < 23.54504487) return 55;
    if (lat < 25.82924707) return 54;
    if (lat < 27.93898710) return 53;
    if (lat < 29.91135686) return 52;
    if (lat < 31.77209708) return 51;
    if (lat < 33.53993436) return 50;
    if (lat < 35.22899598) return 49;
    if (lat < 36.85025108) return 48;
    if (lat < 38.41241892) return 47;
    if (lat < 39.92256684) return 46;
    if (lat < 41.38651832) return 45;
    if (lat < 42.80914012) return 44;
    if (lat < 44.19454951) return 43;
    if (lat < 45.54626723) return 42;
    if (lat < 46.86733252) return 41;
    if (lat < 48.16039128) return 40;
    if (lat < 49.42776439) return 39;
    if (lat < 50.67150166) return 38;
    if (lat < 51.89342469) return 37;
    if (lat < 53.09516153) return 36;
    if (lat < 54.27817472) return 35;
    if (lat < 55.44378444) return 34;
    if (lat < 56.59318756) return 33;
    if (lat < 57.72747354) return 32;
    if (lat < 58.84763776) return 31;
    if (lat < 59.95459277) return 30;
    if (lat < 61.04917774) return 29;
    if (lat < 62.13216659) return 28;
    if (lat < 63.20427479) return 27;
    if (lat < 64.26616523) return 26;
    if (lat < 65.31845310) return 25;
    if (lat < 66.36171008) return 24;
    if (lat < 67.39646774) return 23;
    if (lat < 68.42322022) return 22;
    if (lat < 69.44242631) return 21;
    if (lat < 70.45451075) return 20;
    if (lat < 71.45986473) return 19;
    if (lat < 72.45884545) return 18;
    if (lat < 73.45177442) return 17;
    if (lat < 74.43893416) return 16;
    if (lat < 75.42056257) return 15;
    if (lat < 76.39684391) return 14;
    if (lat < 77.36789461) return 13;
    if (lat < 78.33374083) return 12;
    if (lat < 79.29428225) return 11;
    if (lat < 80.24923213) return 10;
    if (lat < 81.19801349) return 9;
    if (lat < 82.13956981) return 8;
    if (lat < 83.07199445) return 7;
    if (lat < 83.99173563) return 6;
    if (lat < 84.89166191) return 5;
    if (lat < 85.75541621) return 4;
    if (lat < 86.53536998) return 3;
    if (lat < 87.00000000) return 2;
    else return 1;
}
//
//=========================================================================
//
int cprNFunction(double lat, int fflag) {
    int nl = cprNLFunction(lat) - (fflag ? 1 : 0);
    if (nl < 1) nl = 1;
    return nl;
}
//
//=========================================================================
//
double cprDlonFunction(double lat, int fflag, int surface) {
    return (surface ? 90.0 : 360.0) / cprNFunction(lat, fflag);
}
//
//=========================================================================
//
// This algorithm comes from:
// http://www.lll.lu/~edward/edward/adsb/DecodingADSBposition.html.
//
// A few remarks:
// 1) 131072 is 2^17 since CPR latitude and longitude are encoded in 17 bits.
//
int decodeCPR(struct aircraft *a, int fflag, int surface) {
    double AirDlat0 = (surface ? 90.0 : 360.0) / 60.0;
    double AirDlat1 = (surface ? 90.0 : 360.0) / 59.0;
    double lat0 = a->even_cprlat;
    double lat1 = a->odd_cprlat;
    double lon0 = a->even_cprlon;
    double lon1 = a->odd_cprlon;

    // Compute the Latitude Index "j"
    int    j     = (int) floor(((59*lat0 - 60*lat1) / 131072) + 0.5);
    double rlat0 = AirDlat0 * (cprModFunction(j,60) + lat0 / 131072);
    double rlat1 = AirDlat1 * (cprModFunction(j,59) + lat1 / 131072);

    time_t now = time(NULL);
    double surface_rlat = MODES_USER_LATITUDE_DFLT;
    double surface_rlon = MODES_USER_LONGITUDE_DFLT;

    if (surface) {
        // If we're on the ground, make sure we have a (likely) valid Lat/Lon
        if ((a->bFlags & MODES_ACFLAGS_LATLON_VALID) && (((int)(now - a->seenLatLon)) < Modes.interactive_display_ttl)) {
            surface_rlat = a->lat;
            surface_rlon = a->lon;
        } else if (Modes.bUserFlags & MODES_USER_LATLON_VALID) {
            surface_rlat = Modes.fUserLat;
            surface_rlon = Modes.fUserLon;
        } else {
            // No local reference, give up
            return (-1);
        }
        rlat0 += floor(surface_rlat / 90.0) * 90.0;  // Move from 1st quadrant to our quadrant
        rlat1 += floor(surface_rlat / 90.0) * 90.0;
    } else {
        if (rlat0 >= 270) rlat0 -= 360;
        if (rlat1 >= 270) rlat1 -= 360;
    }

    // Check to see that the latitude is in range: -90 .. +90
    if (rlat0 < -90 || rlat0 > 90 || rlat1 < -90 || rlat1 > 90)
        return (-1);

    // Check that both are in the same latitude zone, or abort.
    if (cprNLFunction(rlat0) != cprNLFunction(rlat1))
        return (-1);

    // Compute ni and the Longitude Index "m"
    if (fflag) { // Use odd packet.
        int ni = cprNFunction(rlat1,1);
        int m = (int) floor((((lon0 * (cprNLFunction(rlat1)-1)) -
                              (lon1 * cprNLFunction(rlat1))) / 131072.0) + 0.5);
        a->lon = cprDlonFunction(rlat1, 1, surface) * (cprModFunction(m, ni)+lon1/131072);
        a->lat = rlat1;
    } else {     // Use even packet.
        int ni = cprNFunction(rlat0,0);
        int m = (int) floor((((lon0 * (cprNLFunction(rlat0)-1)) -
                              (lon1 * cprNLFunction(rlat0))) / 131072) + 0.5);
        a->lon = cprDlonFunction(rlat0, 0, surface) * (cprModFunction(m, ni)+lon0/131072);
        a->lat = rlat0;
    }

    if (surface) {
        a->lon += floor(surface_rlon / 90.0) * 90.0;  // Move from 1st quadrant to our quadrant
    } else if (a->lon > 180) {
        a->lon -= 360;
    }

    a->seenLatLon      = a->seen;
    a->timestampLatLon = a->timestamp;
    a->bFlags         |= (MODES_ACFLAGS_LATLON_VALID | MODES_ACFLAGS_LATLON_REL_OK);

    return 0;
}
//
//=========================================================================
//
// This algorithm comes from:
// 1090-WP29-07-Draft_CPR101 (which also defines decodeCPR() )
//
// There is an error in this document related to CPR relative decode.
// Should use trunc() rather than the floor() function in Eq 38 and related for deltaZI.
// floor() returns integer less than argument
// trunc() returns integer closer to zero than argument.
// Note:   text of document describes trunc() functionality for deltaZI calculation
//         but the formulae use floor().
//
int decodeCPRrelative(struct aircraft *a, int fflag, int surface) {
    double AirDlat;
    double AirDlon;
    double lat;
    double lon;
    double lonr, latr;
    double rlon, rlat;
    int j,m;

    if (a->bFlags & MODES_ACFLAGS_LATLON_REL_OK) { // Ok to try aircraft relative first
        latr = a->lat;
        lonr = a->lon;
    } else if (Modes.bUserFlags & MODES_USER_LATLON_VALID) { // Try ground station relative next
        latr = Modes.fUserLat;
        lonr = Modes.fUserLon;
    } else {
        return (-1); // Exit with error - can't do relative if we don't have ref.
    }

    if (fflag) { // odd
        AirDlat = (surface ? 90.0 : 360.0) / 59.0;
        lat = a->odd_cprlat;
        lon = a->odd_cprlon;
    } else {    // even
        AirDlat = (surface ? 90.0 : 360.0) / 60.0;
        lat = a->even_cprlat;
        lon = a->even_cprlon;
    }

    // Compute the Latitude Index "j"
    j = (int) (floor(latr/AirDlat) +
               trunc(0.5 + cprModFunction((int)latr, (int)AirDlat)/AirDlat - lat/131072));
    rlat = AirDlat * (j + lat/131072);
    if (rlat >= 270) rlat -= 360;

    // Check to see that the latitude is in range: -90 .. +90
    if (rlat < -90 || rlat > 90) {
        a->bFlags &= ~MODES_ACFLAGS_LATLON_REL_OK; // This will cause a quick exit next time if no global has been done
        return (-1);                               // Time to give up - Latitude error
    }

    // Check to see that answer is reasonable - ie no more than 1/2 cell away 
    if (fabs(rlat - a->lat) > (AirDlat/2)) {
        a->bFlags &= ~MODES_ACFLAGS_LATLON_REL_OK; // This will cause a quick exit next time if no global has been done
        return (-1);                               // Time to give up - Latitude error 
    }

    // Compute the Longitude Index "m"
    AirDlon = cprDlonFunction(rlat, fflag, surface);
    m = (int) (floor(lonr/AirDlon) +
               trunc(0.5 + cprModFunction((int)lonr, (int)AirDlon)/AirDlon - lon/131072));
    rlon = AirDlon * (m + lon/131072);
    if (rlon > 180) rlon -= 360;

    // Check to see that answer is reasonable - ie no more than 1/2 cell away
    if (fabs(rlon - a->lon) > (AirDlon/2)) {
        a->bFlags &= ~MODES_ACFLAGS_LATLON_REL_OK; // This will cause a quick exit next time if no global has been done
        return (-1);                               // Time to give up - Longitude error
    }

    a->lat = rlat;
    a->lon = rlon;

    a->seenLatLon      = a->seen;
    a->timestampLatLon = a->timestamp;
    a->bFlags         |= (MODES_ACFLAGS_LATLON_VALID | MODES_ACFLAGS_LATLON_REL_OK);
    return (0);
}
//
// ===================== Mode S detection and decoding  ===================
//

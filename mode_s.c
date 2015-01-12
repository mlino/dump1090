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
    int has_addr_errors = 0, i;

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
        int addr, iid;

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
        correct_aa_field(&addr, ei);

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
        int addr;

        if (crc == 0)
            return 3000;

        ei = modesChecksumDiagnose(crc, msgbits);
        if (!ei)
            return -1; // can't correct errors

        addr = (msg[1] << 16) | (msg[2] << 8) | (msg[3]);
        if (correct_aa_field(&addr, ei)) {
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
            //fprintf(stderr, "reject: AP doesn't match known ICAO\n");
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
                //fprintf(stderr, "reject: DF11 uncorrectable CRC error\n");
                return -1; // couldn't fix it
            }
            mm->correctedbits = ei->errors;
            modesChecksumFix(msg, ei);

            // check whether the corrected message looks sensible
            addr = (msg[1] << 16) | (msg[2] << 8) | (msg[3]); 
            if (!icaoFilterTest(addr)) {
                //fprintf(stderr, "reject: DF11 CRC error, repaired address doesn't match known ICAO\n");
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
            //fprintf(stderr, "reject: DF17/18 uncorrectable CRC error\n");
            return -1; // couldn't fix it
        }
        mm->correctedbits = ei->errors;

        addr1 = (msg[1] << 16) | (msg[2] << 8) | (msg[3]); 
        modesChecksumFix(msg, ei);
        addr2 = (msg[1] << 16) | (msg[2] << 8) | (msg[3]); 
        
        // if the corrections touched the address, better validate it
        if (addr1 != addr2 && !icaoFilterTest(addr2)) {
            //fprintf(stderr, "reject: DF17/18 CRC corrected address, repaired address doesn't match known ICAO\n");
            return -1;
        }

        break;
    }
        
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

        //fprintf(stderr, "reject: DF20/21 address doesn't match known ICAO\n");
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
    uint16_t *m = &Modes.magnitude[Modes.trailing_samples];
    uint32_t j;

    memcpy(Modes.magnitude,&Modes.magnitude[MODES_ASYNC_BUF_SAMPLES], Modes.trailing_samples * 2);

    // Compute the magnitudo vector. It's just SQRT(I^2 + Q^2), but
    // we rescale to the 0-255 range to exploit the full resolution.
    for (j = 0; j < MODES_ASYNC_BUF_SAMPLES; j ++) {
        *m++ = Modes.maglut[*p++];
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
// ===================== Mode S detection and decoding  ===================
//

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
#ifndef __DUMP1090_H
#define __DUMP1090_H

// File Version number
// ====================
// Format is : MajorVer.MinorVer.DayMonth.Year"
// MajorVer changes only with significant changes
// MinorVer changes when additional features are added, but not for bug fixes (range 00-99)
// DayDate & Year changes for all changes, including for bug fixes. It represent the release date of the update
//
#ifndef MODES_DUMP1090_VERSION
# define MODES_DUMP1090_VERSION     "1.10.3010.14+mu"
#endif

#ifndef MODES_DUMP1090_VARIANT
# define MODES_DUMP1090_VARIANT     "dump1090-mutability"
#endif

// ============================= Include files ==========================

#ifndef _WIN32
    #include <stdio.h>
    #include <string.h>
    #include <stdlib.h>
    #include <pthread.h>
    #include <stdint.h>
    #include <errno.h>
    #include <unistd.h>
    #include <math.h>
    #include <sys/time.h>
    #include <sys/timeb.h>
    #include <signal.h>
    #include <fcntl.h>
    #include <ctype.h>
    #include <sys/stat.h>
    #include <sys/ioctl.h>
    #include <time.h>
    #include <limits.h>
    #include "rtl-sdr.h"
    #include "anet.h"
#else
    #include "winstubs.h" //Put everything Windows specific in here
    #include "rtl-sdr.h"
    #include "anet.h"
#endif

// ============================= #defines ===============================

#define MODES_DEFAULT_PPM          52
#define MODES_DEFAULT_RATE         2000000
#define MODES_OVERSAMPLE_RATE      2400000
#define MODES_DEFAULT_FREQ         1090000000
#define MODES_DEFAULT_WIDTH        1000
#define MODES_DEFAULT_HEIGHT       700
#define MODES_ASYNC_BUF_NUMBER     16
#define MODES_ASYNC_BUF_SIZE       (16*16384)                 // 256k
#define MODES_ASYNC_BUF_SAMPLES    (MODES_ASYNC_BUF_SIZE / 2) // Each sample is 2 bytes
#define MODES_AUTO_GAIN            -100                       // Use automatic gain
#define MODES_MAX_GAIN             999999                     // Use max available gain
#define MODES_MSG_SQUELCH_DB       4.0                        // Minimum SNR, in dB
#define MODES_MSG_ENCODER_ERRS     3                          // Maximum number of encoding errors

// When changing, change also fixBitErrors() and modesInitErrorTable() !!
#define MODES_MAX_BITERRORS        6                          // Global max for fixable bit erros

#define MODES_MAX_PHASE_STATS      10

#define MODEAC_MSG_SAMPLES       (25 * 2)                     // include up to the SPI bit
#define MODEAC_MSG_BYTES          2
#define MODEAC_MSG_SQUELCH_LEVEL  0x07FF                      // Average signal strength limit
#define MODEAC_MSG_FLAG          (1<<0)
#define MODEAC_MSG_MODES_HIT     (1<<1)
#define MODEAC_MSG_MODEA_HIT     (1<<2)
#define MODEAC_MSG_MODEC_HIT     (1<<3)
#define MODEAC_MSG_MODEA_ONLY    (1<<4)
#define MODEAC_MSG_MODEC_OLD     (1<<5)

#define MODES_PREAMBLE_US        8              // microseconds = bits
#define MODES_PREAMBLE_SAMPLES  (MODES_PREAMBLE_US       * 2)
#define MODES_PREAMBLE_SIZE     (MODES_PREAMBLE_SAMPLES  * sizeof(uint16_t))
#define MODES_LONG_MSG_BYTES     14
#define MODES_SHORT_MSG_BYTES    7
#define MODES_LONG_MSG_BITS     (MODES_LONG_MSG_BYTES    * 8)
#define MODES_SHORT_MSG_BITS    (MODES_SHORT_MSG_BYTES   * 8)
#define MODES_LONG_MSG_SAMPLES  (MODES_LONG_MSG_BITS     * 2)
#define MODES_SHORT_MSG_SAMPLES (MODES_SHORT_MSG_BITS    * 2)
#define MODES_LONG_MSG_SIZE     (MODES_LONG_MSG_SAMPLES  * sizeof(uint16_t))
#define MODES_SHORT_MSG_SIZE    (MODES_SHORT_MSG_SAMPLES * sizeof(uint16_t))

#define MODES_OS_PREAMBLE_SAMPLES  (20)
#define MODES_OS_PREAMBLE_SIZE     (MODES_OS_PREAMBLE_SAMPLES  * sizeof(uint16_t))
#define MODES_OS_LONG_MSG_SAMPLES  (268)
#define MODES_OS_SHORT_MSG_SAMPLES (135)
#define MODES_OS_LONG_MSG_SIZE     (MODES_LONG_MSG_SAMPLES  * sizeof(uint16_t))
#define MODES_OS_SHORT_MSG_SIZE    (MODES_SHORT_MSG_SAMPLES * sizeof(uint16_t))

#define MODES_OUT_BUF_SIZE         (1500)
#define MODES_OUT_FLUSH_SIZE       (MODES_OUT_BUF_SIZE - 256)
#define MODES_OUT_FLUSH_INTERVAL   (60)

#define MODES_ICAO_CACHE_LEN 1024 // Power of two required
#define MODES_ICAO_CACHE_TTL 60   // Time to live of cached addresses
#define MODES_UNIT_FEET 0
#define MODES_UNIT_METERS 1

#define MODES_USER_LATLON_VALID (1<<0)

#define MODES_ACFLAGS_LATLON_VALID   (1<<0)  // Aircraft Lat/Lon is decoded
#define MODES_ACFLAGS_ALTITUDE_VALID (1<<1)  // Aircraft altitude is known
#define MODES_ACFLAGS_HEADING_VALID  (1<<2)  // Aircraft heading is known
#define MODES_ACFLAGS_SPEED_VALID    (1<<3)  // Aircraft speed is known
#define MODES_ACFLAGS_VERTRATE_VALID (1<<4)  // Aircraft vertical rate is known
#define MODES_ACFLAGS_SQUAWK_VALID   (1<<5)  // Aircraft Mode A Squawk is known
#define MODES_ACFLAGS_CALLSIGN_VALID (1<<6)  // Aircraft Callsign Identity
#define MODES_ACFLAGS_EWSPEED_VALID  (1<<7)  // Aircraft East West Speed is known
#define MODES_ACFLAGS_NSSPEED_VALID  (1<<8)  // Aircraft North South Speed is known
#define MODES_ACFLAGS_AOG            (1<<9)  // Aircraft is On the Ground
#define MODES_ACFLAGS_LLEVEN_VALID   (1<<10) // Aircraft Even Lot/Lon is known
#define MODES_ACFLAGS_LLODD_VALID    (1<<11) // Aircraft Odd Lot/Lon is known
#define MODES_ACFLAGS_AOG_VALID      (1<<12) // MODES_ACFLAGS_AOG is valid
#define MODES_ACFLAGS_FS_VALID       (1<<13) // Aircraft Flight Status is known
#define MODES_ACFLAGS_NSEWSPD_VALID  (1<<14) // Aircraft EW and NS Speed is known
#define MODES_ACFLAGS_LATLON_REL_OK  (1<<15) // Indicates it's OK to do a relative CPR
#define MODES_ACFLAGS_REL_CPR_USED   (1<<16) // (on message) relative CPR was used

#define MODES_ACFLAGS_LLEITHER_VALID (MODES_ACFLAGS_LLEVEN_VALID | MODES_ACFLAGS_LLODD_VALID)
#define MODES_ACFLAGS_LLBOTH_VALID   (MODES_ACFLAGS_LLEVEN_VALID | MODES_ACFLAGS_LLODD_VALID)
#define MODES_ACFLAGS_AOG_GROUND     (MODES_ACFLAGS_AOG_VALID    | MODES_ACFLAGS_AOG)

#define MODES_DEBUG_DEMOD (1<<0)
#define MODES_DEBUG_DEMODERR (1<<1)
#define MODES_DEBUG_BADCRC (1<<2)
#define MODES_DEBUG_GOODCRC (1<<3)
#define MODES_DEBUG_NOPREAMBLE (1<<4)
#define MODES_DEBUG_NET (1<<5)
#define MODES_DEBUG_JS (1<<6)

// When debug is set to MODES_DEBUG_NOPREAMBLE, the first sample must be
// at least greater than a given level for us to dump the signal.
#define MODES_DEBUG_NOPREAMBLE_LEVEL 25

#define MODES_INTERACTIVE_REFRESH_TIME 250      // Milliseconds
#define MODES_INTERACTIVE_ROWS          22      // Rows on screen
#define MODES_INTERACTIVE_DELETE_TTL   300      // Delete from the list after 300 seconds
#define MODES_INTERACTIVE_DISPLAY_TTL   60      // Delete from display after 60 seconds

#define MODES_NET_HEARTBEAT_INTERVAL    60      // seconds

#define MODES_NET_SERVICES_NUM          7
#define MODES_NET_INPUT_RAW_PORT    30001
#define MODES_NET_OUTPUT_RAW_PORT   30002
#define MODES_NET_OUTPUT_SBS_PORT   30003
#define MODES_NET_INPUT_BEAST_PORT  30004
#define MODES_NET_OUTPUT_BEAST_PORT 30005
#define MODES_NET_HTTP_PORT          8080
#define MODES_NET_OUTPUT_FA_TSV_PORT 10001
#define MODES_CLIENT_BUF_SIZE  1024
#define MODES_NET_SNDBUF_SIZE (1024*64)
#define MODES_NET_SNDBUF_MAX  (7)

#ifndef HTMLPATH
#define HTMLPATH   "./public_html"      // default path for gmap.html etc
#endif

#define MODES_NOTUSED(V) ((void) V)

//======================== structure declarations =========================

// Structure used to describe a networking client
struct client {
    struct client*  next;                // Pointer to next client
    int    fd;                           // File descriptor
    int    service;                      // TCP port the client is connected to
    int    buflen;                       // Amount of data on buffer
    char   buf[MODES_CLIENT_BUF_SIZE+1]; // Read buffer
};

// Structure used to describe an aircraft in iteractive mode
struct aircraft {
    uint32_t      addr;           // ICAO address
    char          flight[16];     // Flight number
    unsigned char signalLevel[8]; // Last 8 Signal Amplitudes
    int           altitude;       // Altitude
    int           speed;          // Velocity
    int           track;          // Angle of flight
    int           vert_rate;      // Vertical rate.
    time_t        seen;           // Time at which the last packet was received
    time_t        seenLatLon;     // Time at which the last lat long was calculated
    uint64_t      timestamp;      // Timestamp at which the last packet was received
    uint64_t      timestampLatLon;// Timestamp at which the last lat long was calculated
    long          messages;       // Number of Mode S messages received
    int           modeA;          // Squawk
    int           modeC;          // Altitude
    long          modeAcount;     // Mode A Squawk hit Count
    long          modeCcount;     // Mode C Altitude hit Count
    int           modeACflags;    // Flags for mode A/C recognition

    int           fatsv_emitted_altitude;  // last FA emitted altitude
    int           fatsv_emitted_track;     // last FA emitted angle of flight
    time_t        fatsv_last_emitted;      // time aircraft was last FA emitted

    // Encoded latitude and longitude as extracted by odd and even CPR encoded messages
    int           odd_cprlat;
    int           odd_cprlon;
    int           even_cprlat;
    int           even_cprlon;
    uint64_t      odd_cprtime;
    uint64_t      even_cprtime;
    double        lat, lon;       // Coordinated obtained from CPR encoded data
    int           bFlags;         // Flags related to valid fields in this structure
    struct aircraft *next;        // Next aircraft in our linked list
};

// Common stats for non-phase-corrected vs phase-corrected cases
struct demod_stats {
    unsigned int demodulated0;
    unsigned int demodulated1;
    unsigned int demodulated2;
    unsigned int demodulated3;
    unsigned int goodcrc;
    unsigned int goodcrc_byphase[MODES_MAX_PHASE_STATS];
    unsigned int badcrc;
    unsigned int fixed;

    // Histogram of fixed bit errors: index 0 for single bit erros,
    // index 1 for double bit errors etc.
    unsigned int bit_fix[MODES_MAX_BITERRORS];
};

// Common writer state for all output sockets of one type
struct net_writer {
    int socket;          // listening socket FD, used to identify the owning service
    int connections;     // number of active clients
    void *data;          // shared write buffer, sized MODES_OUT_BUF_SIZE
    int dataUsed;        // number of bytes of write buffer currently used
    time_t lastWrite;    // time of last write to clients
};

// Program global state
struct {                             // Internal state
    pthread_t       reader_thread;

    pthread_mutex_t data_mutex;      // Mutex to synchronize buffer access
    pthread_cond_t  data_cond;       // Conditional variable associated
    uint16_t       *pData          [MODES_ASYNC_BUF_NUMBER]; // Raw IQ sample buffers from RTL
    struct timeb    stSystemTimeRTL[MODES_ASYNC_BUF_NUMBER]; // System time when RTL passed us this block
    int             iDataIn;         // Fifo input pointer
    int             iDataOut;        // Fifo output pointer
    int             iDataReady;      // Fifo content count
    int             iDataLost;       // Count of missed buffers

    int             trailing_samples;// extra trailing samples in magnitude buffer

    uint16_t       *pFileData;       // Raw IQ samples buffer (from a File)
    uint16_t       *magnitude;       // Magnitude vector
    uint64_t        timestampBlk;    // Timestamp of the start of the current block
    struct timeb    stSystemTimeBlk; // System time when RTL passed us currently processing this block
    int             fd;              // --ifile option file descriptor
    uint32_t       *icao_cache;      // Recently seen ICAO addresses cache
    uint16_t       *maglut;          // I/Q -> Magnitude lookup table
    uint16_t       *log10lut;        // Magnitude -> log10 lookup table
    int             exit;            // Exit from the main loop when true

    // RTLSDR
    char *        dev_name;
    int           gain;
    int           enable_agc;
    rtlsdr_dev_t *dev;
    int           freq;
    int           ppm_error;

    // Networking
    char           aneterr[ANET_ERR_LEN];
    struct client *clients;          // Our clients
    int            ris;              // Raw input listening socket
    int            bis;              // Beast input listening socket
    int            https;            // HTTP listening socket

    struct net_writer raw_out;       // Raw output
    struct net_writer beast_out;     // Beast-format output
    struct net_writer sbs_out;       // SBS-format output
    struct net_writer fatsv_out;     // FATSV-format output

#ifdef _WIN32
    WSADATA        wsaData;          // Windows socket initialisation
#endif

    // Configuration
    char *filename;                  // Input form file, --ifile option
    int   oversample;
    int   phase_enhance;             // Enable phase enhancement if true
    int   nfix_crc;                  // Number of crc bit error(s) to correct
    int   check_crc;                 // Only display messages with good CRC
    int   raw;                       // Raw output format
    int   beast;                     // Beast binary format output
    int   mode_ac;                   // Enable decoding of SSR Modes A & C
    int   debug;                     // Debugging mode
    int   net;                       // Enable networking
    int   net_only;                  // Enable just networking
    int   net_heartbeat_interval;    // TCP heartbeat interval (seconds)
    int   net_output_sbs_port;       // SBS output TCP port
    int   net_output_flush_size;     // Minimum Size of output data
    int   net_output_flush_interval; // Maximum interval (in seconds) between outputwrites
    int   net_output_raw_port;       // Raw output TCP port
    int   net_input_raw_port;        // Raw input TCP port
    int   net_output_beast_port;     // Beast output TCP port
    int   net_input_beast_port;      // Beast input TCP port
    char  *net_bind_address;         // Bind address
    int   net_http_port;             // HTTP port
    int   net_fatsv_port;            // FlightAware TSV port
    int   net_sndbuf_size;           // TCP output buffer size (64Kb * 2^n)
    int   quiet;                     // Suppress stdout
    int   interactive;               // Interactive mode
    int   interactive_rows;          // Interactive mode: max number of rows
    int   interactive_display_ttl;   // Interactive mode: TTL display
    int   interactive_delete_ttl;    // Interactive mode: TTL before deletion
    int   stats;                     // Print stats at exit in --ifile mode
    int   onlyaddr;                  // Print only ICAO addresses
    int   metric;                    // Use metric units
    int   mlat;                      // Use Beast ascii format for raw data output, i.e. @...; iso *...;
    int   interactive_rtl1090;       // flight table in interactive mode is formatted like RTL1090
    char *json_aircraft_path;        // Path to json aircraft file to write, or NULL not to.
    char *json_metadata_path;        // Path to json metadata file to write, or NULL not to.
    int   json_interval;             // Interval between rewriting the json aircraft file
    int   json_location_accuracy;    // Accuracy of location metadata: 0=none, 1=approx, 2=exact

    // User details
    double fUserLat;                // Users receiver/antenna lat/lon needed for initial surface location
    double fUserLon;                // Users receiver/antenna lat/lon needed for initial surface location
    int    bUserFlags;              // Flags relating to the user details

    // Interactive mode
    struct aircraft *aircrafts;
    uint64_t         interactive_last_update; // Last screen update in milliseconds
    time_t           last_cleanup_time;       // Last cleanup time in seconds

    // Statistics
    unsigned int stat_preamble_no_correlation;
    unsigned int stat_preamble_not_quiet;
    unsigned int stat_valid_preamble;
    unsigned int stat_preamble_phase[MODES_MAX_PHASE_STATS];

    struct demod_stats stat_demod;
    struct demod_stats stat_demod_phasecorrected;

    unsigned int stat_http_requests;
    unsigned int stat_out_of_phase;

    unsigned int stat_DF_Len_Corrected;
    unsigned int stat_DF_Type_Corrected;
    unsigned int stat_ModeAC;

    unsigned int stat_blocks_processed;
    unsigned int stat_blocks_dropped;

    struct timespec stat_cputime;

    // remote messages:
    unsigned int stat_remote_accepted;
    unsigned int stat_remote_rejected;

    // total messages:
    unsigned int stat_messages_total;

    // noise floor:
    uint64_t stat_noise_power;
    uint32_t stat_noise_count;

    // CPR decoding:
    unsigned int stat_cpr_global_ok;
    unsigned int stat_cpr_global_bad;
    unsigned int stat_cpr_global_skipped;
    unsigned int stat_cpr_local_ok;
    unsigned int stat_cpr_local_skipped;
    unsigned int stat_cpr_filtered;
} Modes;

// The struct we use to store information about a decoded message.
struct modesMessage {
    // Generic fields
    unsigned char msg[MODES_LONG_MSG_BYTES];      // Binary message.
    int           msgbits;                        // Number of bits in message 
    int           msgtype;                        // Downlink format #
    uint32_t      crc;                            // Message CRC
    int           correctedbits;                  // No. of bits corrected 
    uint32_t      addr;                           // ICAO Address from bytes 1 2 and 3
    int           phase_corrected;                // True if phase correction was applied
    uint64_t      timestampMsg;                   // Timestamp of the message
    int           remote;                         // If set this message is from a remote station
    unsigned char signalLevel;                    // Signal Amplitude
    int           score;                          // Message quality score

    // DF 11,17
    int  ca;                    // Responder capabilities
    int  iid;

    // DF 18
    int  cf;                    // Control field

    // DF 17, DF 18
    int    metype;              // Extended squitter message type.
    int    mesub;               // Extended squitter message subtype.
    int    heading;             // Reported by aircraft, or computed from from EW and NS velocity
    int    raw_latitude;        // Non decoded latitude.
    int    raw_longitude;       // Non decoded longitude.
    double fLat;                // Coordinates obtained from CPR encoded data if/when decoded
    double fLon;                // Coordinates obtained from CPR encoded data if/when decoded
    char   flight[16];          // 8 chars flight number.
    int    ew_velocity;         // E/W velocity.
    int    ns_velocity;         // N/S velocity.
    int    vert_rate;           // Vertical rate.
    int    velocity;            // Reported by aircraft, or computed from from EW and NS velocity

    // DF4, DF5, DF20, DF21
    int  fs;                    // Flight status for DF4,5,20,21
    int  modeA;                 // 13 bits identity (Squawk).

    // DF20/21 (sometimes)
    int  bds;

    // Fields used by multiple message types.
    int  altitude;
    int  unit; 
    int  bFlags;                // Flags related to fields in this structure
};

// ======================== function declarations =========================

#ifdef __cplusplus
extern "C" {
#endif

//
// Functions exported from mode_ac.c
//
int  detectModeA       (uint16_t *m, struct modesMessage *mm);
void decodeModeAMessage(struct modesMessage *mm, int ModeA);
int  ModeAToModeC      (unsigned int ModeA);

//
// Functions exported from mode_s.c
//
void detectModeS        (uint16_t *m, uint32_t mlen);
void detectModeS_oversample (uint16_t *m, int mlen);
int scoreModesMessage (unsigned char *msg);
int decodeModesMessage (struct modesMessage *mm, unsigned char *msg);
void displayModesMessage(struct modesMessage *mm);
void useModesMessage    (struct modesMessage *mm);
void computeMagnitudeVector(uint16_t *pData);
int decodeCPRsurface(double reflat, double reflon,
                     int even_cprlat, int even_cprlon,
                     int odd_cprlat, int odd_cprlon,
                     int fflag,
                     double *out_lat, double *out_lon);
int decodeCPRairborne(int even_cprlat, int even_cprlon,
                      int odd_cprlat, int odd_cprlon,
                      int fflag,
                      double *out_lat, double *out_lon);
int  decodeCPRrelative(double reflat, double reflon,
                       int cprlat, int cprlon,
                       int fflag, int surface,
                       double range_limit,
                       double *out_lat, double *out_lon);
int modesMessageLenByType(int type);

// From crc.c:
typedef struct {
    uint32_t syndrome;                 // CRC syndrome
    int      errors;                   // number of errors
    int8_t   bit[MODES_MAX_BITERRORS]; // bit positions to fix (-1 = no bit)
} errorinfo;

void modesChecksumInit();
uint32_t modesChecksum(uint8_t *msg, int bitlen);
errorinfo *modesChecksumDiagnose(uint32_t syndrome, int bitlen);
void modesChecksumFix(uint8_t *msg, errorinfo *info);

// From icao_bloom.c:
void icaoFilterInit();
void icaoFilterAdd(uint32_t addr);
int  icaoFilterTest(uint32_t addr);
uint32_t icaoFilterTestFuzzy(uint32_t partial);
void icaoFilterExpire();

//
// Functions exported from interactive.c
//
struct aircraft* interactiveReceiveData(struct modesMessage *mm);
void  interactiveShowData(void);
void  interactiveRemoveStaleAircrafts(void);
int   decodeBinMessage   (struct client *c, char *p);
struct aircraft *interactiveFindAircraft(uint32_t addr);

//
// Functions exported from net_io.c
//
void modesInitNet         (void);
void modesQueueOutput     (struct modesMessage *mm);
void modesReadFromClient(struct client *c, char *sep, int(*handler)(struct client *, char *));
void modesNetPeriodicWork (void);

void writeJsonToFile(const char *path, char * (*generator) (int*));
char *generateAircraftJson(int *len);
char *generateReceiverJson(int *len);

#ifdef __cplusplus
}
#endif

#endif // __DUMP1090_H

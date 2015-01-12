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

static int verbose_device_search(char *s);

//
// ============================= Utility functions ==========================
//
void sigintHandler(int dummy) {
    MODES_NOTUSED(dummy);
    signal(SIGINT, SIG_DFL);  // reset signal handler - bit extra safety
    Modes.exit = 1;           // Signal to threads that we are done
}
//
// =============================== Terminal handling ========================
//
#ifndef _WIN32
// Get the number of rows after the terminal changes size.
int getTermRows() { 
    struct winsize w; 
    ioctl(STDOUT_FILENO, TIOCGWINSZ, &w); 
    return (w.ws_row); 
} 

// Handle resizing terminal
void sigWinchCallback() {
    signal(SIGWINCH, SIG_IGN);
    Modes.interactive_rows = getTermRows();
    interactiveShowData();
    signal(SIGWINCH, sigWinchCallback); 
}
#else 
int getTermRows() { return MODES_INTERACTIVE_ROWS;}
#endif
//
// =============================== Initialization ===========================
//
void modesInitConfig(void) {
    // Default everything to zero/NULL
    memset(&Modes, 0, sizeof(Modes));

    // Now initialise things that should not be 0/NULL to their defaults
    Modes.gain                    = MODES_MAX_GAIN;
    Modes.freq                    = MODES_DEFAULT_FREQ;
    Modes.ppm_error               = MODES_DEFAULT_PPM;
    Modes.check_crc               = 1;
    Modes.net_heartbeat_interval  = MODES_NET_HEARTBEAT_INTERVAL;
    Modes.net_output_sbs_port     = MODES_NET_OUTPUT_SBS_PORT;
    Modes.net_output_raw_port     = MODES_NET_OUTPUT_RAW_PORT;
    Modes.net_input_raw_port      = MODES_NET_INPUT_RAW_PORT;
    Modes.net_output_beast_port   = MODES_NET_OUTPUT_BEAST_PORT;
    Modes.net_input_beast_port    = MODES_NET_INPUT_BEAST_PORT;
    Modes.net_http_port           = MODES_NET_HTTP_PORT;
    Modes.net_fatsv_port          = MODES_NET_OUTPUT_FA_TSV_PORT;
    Modes.interactive_rows        = getTermRows();
    Modes.interactive_delete_ttl  = MODES_INTERACTIVE_DELETE_TTL;
    Modes.interactive_display_ttl = MODES_INTERACTIVE_DISPLAY_TTL;
    Modes.json_interval           = 1;
    Modes.json_location_accuracy  = 1;
}
//
//=========================================================================
//
void modesInit(void) {
    int i, q;

    pthread_mutex_init(&Modes.data_mutex,NULL);
    pthread_cond_init(&Modes.data_cond,NULL);

    // Allocate the various buffers used by Modes
    Modes.trailing_samples = (Modes.oversample ? (MODES_OS_PREAMBLE_SAMPLES + MODES_OS_LONG_MSG_SAMPLES) : (MODES_PREAMBLE_SAMPLES + MODES_LONG_MSG_SAMPLES)) + 16;
    if ( ((Modes.icao_cache = (uint32_t *) malloc(sizeof(uint32_t) * MODES_ICAO_CACHE_LEN * 2)                  ) == NULL) ||
         ((Modes.pFileData  = (uint16_t *) malloc(MODES_ASYNC_BUF_SIZE)                                         ) == NULL) ||
         ((Modes.magnitude  = (uint16_t *) calloc(MODES_ASYNC_BUF_SAMPLES+Modes.trailing_samples, 2)            ) == NULL) ||
         ((Modes.maglut     = (uint16_t *) malloc(sizeof(uint16_t) * 256 * 256)                                 ) == NULL) ||
         ((Modes.log10lut   = (uint16_t *) malloc(sizeof(uint16_t) * 256 * 256)                                 ) == NULL) )
    {
        fprintf(stderr, "Out of memory allocating data buffer.\n");
        exit(1);
    }

    // Clear the buffers that have just been allocated, just in-case
    memset(Modes.icao_cache, 0,   sizeof(uint32_t) * MODES_ICAO_CACHE_LEN * 2);
    memset(Modes.pFileData,127,   MODES_ASYNC_BUF_SIZE);

    // Validate the users Lat/Lon home location inputs
    if ( (Modes.fUserLat >   90.0)  // Latitude must be -90 to +90
      || (Modes.fUserLat <  -90.0)  // and 
      || (Modes.fUserLon >  360.0)  // Longitude must be -180 to +360
      || (Modes.fUserLon < -180.0) ) {
        Modes.fUserLat = Modes.fUserLon = 0.0;
    } else if (Modes.fUserLon > 180.0) { // If Longitude is +180 to +360, make it -180 to 0
        Modes.fUserLon -= 360.0;
    }
    // If both Lat and Lon are 0.0 then the users location is either invalid/not-set, or (s)he's in the 
    // Atlantic ocean off the west coast of Africa. This is unlikely to be correct. 
    // Set the user LatLon valid flag only if either Lat or Lon are non zero. Note the Greenwich meridian 
    // is at 0.0 Lon,so we must check for either fLat or fLon being non zero not both. 
    // Testing the flag at runtime will be much quicker than ((fLon != 0.0) || (fLat != 0.0))
    Modes.bUserFlags &= ~MODES_USER_LATLON_VALID;
    if ((Modes.fUserLat != 0.0) || (Modes.fUserLon != 0.0)) {
        Modes.bUserFlags |= MODES_USER_LATLON_VALID;
    }

    // Limit the maximum requested raw output size to less than one Ethernet Block 
    if (Modes.net_output_flush_size > (MODES_OUT_FLUSH_SIZE))
      {Modes.net_output_flush_size = MODES_OUT_FLUSH_SIZE;}
    if (Modes.net_output_flush_interval > (MODES_OUT_FLUSH_INTERVAL))
      {Modes.net_output_flush_interval = MODES_OUT_FLUSH_INTERVAL;}
    if (Modes.net_sndbuf_size > (MODES_NET_SNDBUF_MAX))
      {Modes.net_sndbuf_size = MODES_NET_SNDBUF_MAX;}

    // Initialise the Block Timers to something half sensible
    ftime(&Modes.stSystemTimeBlk);
    for (i = 0; i < MODES_ASYNC_BUF_NUMBER; i++)
      {Modes.stSystemTimeRTL[i] = Modes.stSystemTimeBlk;}

    // Each I and Q value varies from 0 to 255, which represents a range from -1 to +1. To get from the 
    // unsigned (0-255) range you therefore subtract 127 (or 128 or 127.5) from each I and Q, giving you 
    // a range from -127 to +128 (or -128 to +127, or -127.5 to +127.5)..
    //
    // To decode the AM signal, you need the magnitude of the waveform, which is given by sqrt((I^2)+(Q^2))
    // The most this could be is if I&Q are both 128 (or 127 or 127.5), so you could end up with a magnitude 
    // of 181.019 (or 179.605, or 180.312)
    //
    // However, in reality the magnitude of the signal should never exceed the range -1 to +1, because the 
    // values are I = rCos(w) and Q = rSin(w). Therefore the integer computed magnitude should (can?) never 
    // exceed 128 (or 127, or 127.5 or whatever)
    //
    // If we scale up the results so that they range from 0 to 65535 (16 bits) then we need to multiply 
    // by 511.99, (or 516.02 or 514). antirez's original code multiplies by 360, presumably because he's 
    // assuming the maximim calculated amplitude is 181.019, and (181.019 * 360) = 65166.
    //
    // So lets see if we can improve things by subtracting 127.5, Well in integer arithmatic we can't
    // subtract half, so, we'll double everything up and subtract one, and then compensate for the doubling 
    // in the multiplier at the end.
    //
    // If we do this we can never have I or Q equal to 0 - they can only be as small as +/- 1.
    // This gives us a minimum magnitude of root 2 (0.707), so the dynamic range becomes (1.414-255). This 
    // also affects our scaling value, which is now 65535/(255 - 1.414), or 258.433254
    //
    // The sums then become mag = 258.433254 * (sqrt((I*2-255)^2 + (Q*2-255)^2) - 1.414)
    //                   or mag = (258.433254 * sqrt((I*2-255)^2 + (Q*2-255)^2)) - 365.4798
    //
    // We also need to clip mag just incaes any rogue I/Q values somehow do have a magnitude greater than 255.
    //

    for (i = 0; i <= 255; i++) {
        for (q = 0; q <= 255; q++) {
            int mag, mag_i, mag_q;

            mag_i = (i * 2) - 255;
            mag_q = (q * 2) - 255;

            mag = (int) round((sqrt((mag_i*mag_i)+(mag_q*mag_q)) * 258.433254) - 365.4798);

            Modes.maglut[(i*256)+q] = (uint16_t) ((mag < 65535) ? mag : 65535);
        }
    }

    // Prepare the log10 lookup table.
    // This maps from a magnitude value x (scaled as above) to 100log10(x)
    for (i = 0; i <= 65535; i++) {
        int l10 = (int) round(100 * log10( (i + 365.4798) / 258.433254) );
        Modes.log10lut[i] = (uint16_t) ((l10 < 65535 ? l10 : 65535));
    }

    // Prepare error correction tables
    modesChecksumInit();
    icaoFilterInit();
}
//
// =============================== RTLSDR handling ==========================
//
int modesInitRTLSDR(void) {
    int j;
    int device_count, dev_index = 0;
    char vendor[256], product[256], serial[256];

    if (Modes.dev_name) {
        if ( (dev_index = verbose_device_search(Modes.dev_name)) < 0 )
            return -1;
    }

    device_count = rtlsdr_get_device_count();
    if (!device_count) {
        fprintf(stderr, "No supported RTLSDR devices found.\n");
        return -1;
    }

    fprintf(stderr, "Found %d device(s):\n", device_count);
    for (j = 0; j < device_count; j++) {
        rtlsdr_get_device_usb_strings(j, vendor, product, serial);
        fprintf(stderr, "%d: %s, %s, SN: %s %s\n", j, vendor, product, serial,
            (j == dev_index) ? "(currently selected)" : "");
    }

    if (rtlsdr_open(&Modes.dev, dev_index) < 0) {
        fprintf(stderr, "Error opening the RTLSDR device: %s\n",
            strerror(errno));
        return -1;
    }

    // Set gain, frequency, sample rate, and reset the device
    rtlsdr_set_tuner_gain_mode(Modes.dev,
        (Modes.gain == MODES_AUTO_GAIN) ? 0 : 1);
    if (Modes.gain != MODES_AUTO_GAIN) {
        int *gains;
        int numgains;

        numgains = rtlsdr_get_tuner_gains(Modes.dev, NULL);
        if (numgains <= 0) {
            fprintf(stderr, "Error getting tuner gains\n");
            return -1;
        }

        gains = malloc(numgains * sizeof(int));
        if (rtlsdr_get_tuner_gains(Modes.dev, gains) != numgains) {
            fprintf(stderr, "Error getting tuner gains\n");
            free(gains);
            return -1;
        }
        
        if (Modes.gain == MODES_MAX_GAIN) {
            int highest = -1;
            int i;

            for (i = 0; i < numgains; ++i) {
                if (gains[i] > highest)
                    highest = gains[i];
            }

            Modes.gain = highest;
            fprintf(stderr, "Max available gain is: %.2f dB\n", Modes.gain/10.0);
        } else {
            int closest = -1;
            int i;

            for (i = 0; i < numgains; ++i) {
                if (closest == -1 || abs(gains[i] - Modes.gain) < abs(closest - Modes.gain))
                    closest = gains[i];
            }

            if (closest != Modes.gain) {
                Modes.gain = closest;
                fprintf(stderr, "Closest available gain: %.2f dB\n", Modes.gain/10.0);
            }
        }

        free(gains);

        fprintf(stderr, "Setting gain to: %.2f dB\n", Modes.gain/10.0);
        if (rtlsdr_set_tuner_gain(Modes.dev, Modes.gain) < 0) {
            fprintf(stderr, "Error setting tuner gains\n");
            return -1;
        }
    } else {
        fprintf(stderr, "Using automatic gain control.\n");
    }
    rtlsdr_set_freq_correction(Modes.dev, Modes.ppm_error);
    if (Modes.enable_agc) rtlsdr_set_agc_mode(Modes.dev, 1);
    rtlsdr_set_center_freq(Modes.dev, Modes.freq);
    rtlsdr_set_sample_rate(Modes.dev, Modes.oversample ? MODES_OVERSAMPLE_RATE : MODES_DEFAULT_RATE);

    rtlsdr_reset_buffer(Modes.dev);
    fprintf(stderr, "Gain reported by device: %.2f dB\n",
        rtlsdr_get_tuner_gain(Modes.dev)/10.0);

    return 0;
}
//
//=========================================================================
//
// We use a thread reading data in background, while the main thread
// handles decoding and visualization of data to the user.
//
// The reading thread calls the RTLSDR API to read data asynchronously, and
// uses a callback to populate the data buffer.
//
// A Mutex is used to avoid races with the decoding thread.
//
void rtlsdrCallback(unsigned char *buf, uint32_t len, void *ctx) {

    MODES_NOTUSED(ctx);

    // Lock the data buffer variables before accessing them
    pthread_mutex_lock(&Modes.data_mutex);

    Modes.iDataIn &= (MODES_ASYNC_BUF_NUMBER-1); // Just incase!!!

    // Get the system time for this block
    ftime(&Modes.stSystemTimeRTL[Modes.iDataIn]);

    if (len > MODES_ASYNC_BUF_SIZE) {len = MODES_ASYNC_BUF_SIZE;}

    // Queue the new data
    Modes.pData[Modes.iDataIn] = (uint16_t *) buf;
    Modes.iDataIn    = (MODES_ASYNC_BUF_NUMBER-1) & (Modes.iDataIn + 1);
    Modes.iDataReady = (MODES_ASYNC_BUF_NUMBER-1) & (Modes.iDataIn - Modes.iDataOut);   

    if (Modes.iDataReady == 0) {
      // Ooooops. We've just received the MODES_ASYNC_BUF_NUMBER'th outstanding buffer
      // This means that RTLSDR is currently overwriting the MODES_ASYNC_BUF_NUMBER+1
      // buffer, but we havent yet processed it, so we're going to lose it. There
      // isn't much we can do to recover the lost data, but we can correct things to
      // avoid any additional problems.
      Modes.iDataOut   = (MODES_ASYNC_BUF_NUMBER-1) & (Modes.iDataOut+1);
      Modes.iDataReady = (MODES_ASYNC_BUF_NUMBER-1);   
      Modes.iDataLost++;
    }
 
    // Signal to the other thread that new data is ready, and unlock
    pthread_cond_signal(&Modes.data_cond);
    pthread_mutex_unlock(&Modes.data_mutex);
}
//
//=========================================================================
//
// This is used when --ifile is specified in order to read data from file
// instead of using an RTLSDR device
//
void readDataFromFile(void) {
    pthread_mutex_lock(&Modes.data_mutex);
    while(Modes.exit == 0) {
        ssize_t nread, toread;
        unsigned char *p;

        if (Modes.iDataReady) {
            pthread_cond_wait(&Modes.data_cond, &Modes.data_mutex);
            continue;
        }

        if (Modes.interactive) {
            // When --ifile and --interactive are used together, slow down
            // playing at the natural rate of the RTLSDR received.
            pthread_mutex_unlock(&Modes.data_mutex);
            usleep(64000);
            pthread_mutex_lock(&Modes.data_mutex);
        }

        toread = MODES_ASYNC_BUF_SIZE;
        p = (unsigned char *) Modes.pFileData;
        while(toread) {
            nread = read(Modes.fd, p, toread);
            if (nread <= 0) {
                // Done.
                Modes.exit = 1; // Signal the other threads to exit.
                goto OUT;
            }
            p += nread;
            toread -= nread;
        }
        if (toread) {
            // Not enough data on file to fill the buffer? Pad with no signal.
            memset(p,127,toread);
        }

        Modes.iDataIn &= (MODES_ASYNC_BUF_NUMBER-1); // Just incase!!!

        // Get the system time for this block
        ftime(&Modes.stSystemTimeRTL[Modes.iDataIn]);

        // Queue the new data
        Modes.pData[Modes.iDataIn] = Modes.pFileData;
        Modes.iDataIn    = (MODES_ASYNC_BUF_NUMBER-1) & (Modes.iDataIn + 1);
        Modes.iDataReady = (MODES_ASYNC_BUF_NUMBER-1) & (Modes.iDataIn - Modes.iDataOut);   

        // Signal to the other thread that new data is ready
        pthread_cond_signal(&Modes.data_cond);
    }

 OUT: 
    pthread_mutex_unlock(&Modes.data_mutex);
}
//
//=========================================================================
//
// We read data using a thread, so the main thread only handles decoding
// without caring about data acquisition
//

void *readerThreadEntryPoint(void *arg) {
    MODES_NOTUSED(arg);

    if (Modes.filename == NULL) {
        while (!Modes.exit) {
            rtlsdr_read_async(Modes.dev, rtlsdrCallback, NULL,
                              MODES_ASYNC_BUF_NUMBER,
                              MODES_ASYNC_BUF_SIZE);

            if (!Modes.exit) {
                fprintf(stderr, "Warning: lost the connection to the RTLSDR device.\n");
                rtlsdr_close(Modes.dev);

                do {
                    sleep(5);
                    fprintf(stderr, "Trying to reconnect to the RTLSDR device..\n");
                } while (!Modes.exit && modesInitRTLSDR() < 0);
            }
        }
    } else {
        readDataFromFile();
    }

    // Wake the main thread (if it's still waiting)
    pthread_mutex_lock(&Modes.data_mutex);
    Modes.exit = 1; // just in case
    pthread_cond_signal(&Modes.data_cond);
    pthread_mutex_unlock(&Modes.data_mutex);

#ifndef _WIN32
    pthread_exit(NULL);
#else
    return NULL;
#endif
}
//
// ============================== Snip mode =================================
//
// Get raw IQ samples and filter everything is < than the specified level
// for more than 256 samples in order to reduce example file size
//
void snipMode(int level) {
    int i, q;
    uint64_t c = 0;

    while ((i = getchar()) != EOF && (q = getchar()) != EOF) {
        if (abs(i-127) < level && abs(q-127) < level) {
            c++;
            if (c > MODES_PREAMBLE_SIZE) continue;
        } else {
            c = 0;
        }
        putchar(i);
        putchar(q);
    }
}
//
// ================================ Main ====================================
//
void showHelp(void) {
    printf(
"-----------------------------------------------------------------------------\n"
"| dump1090 ModeS Receiver     %45s |\n"
"-----------------------------------------------------------------------------\n"
"--device-index <index>   Select RTL device (default: 0)\n"
"--gain <db>              Set gain (default: max gain. Use -10 for auto-gain)\n"
"--enable-agc             Enable the Automatic Gain Control (default: off)\n"
"--freq <hz>              Set frequency (default: 1090 Mhz)\n"
"--ifile <filename>       Read data from file (use '-' for stdin)\n"
"--interactive            Interactive mode refreshing data on screen\n"
"--interactive-rows <num> Max number of rows in interactive mode (default: 15)\n"
"--interactive-ttl <sec>  Remove from list if idle for <sec> (default: 60)\n"
"--interactive-rtl1090    Display flight table in RTL1090 format\n"
"--raw                    Show only messages hex values\n"
"--net                    Enable networking\n"
"--modeac                 Enable decoding of SSR Modes 3/A & 3/C\n"
"--net-only               Enable just networking, no RTL device or file used\n"
"--net-bind-address <ip>  IP address to bind to (default: Any; Use 127.0.0.1 for private)\n"
"--net-http-port <port>   HTTP server port (default: 8080)\n"
"--net-ri-port <port>     TCP raw input listen port  (default: 30001)\n"
"--net-ro-port <port>     TCP raw output listen port (default: 30002)\n"
"--net-sbs-port <port>    TCP BaseStation output listen port (default: 30003)\n"
"--net-bi-port <port>     TCP Beast input listen port  (default: 30004)\n"
"--net-bo-port <port>     TCP Beast output listen port (default: 30005)\n"
"--net-fatsv-port <port>  FlightAware TSV output port (default: 10001)\n"
"--net-ro-size <size>     TCP output minimum size (default: 0)\n"
"--net-ro-interval <rate> TCP output memory flush rate in seconds (default: 0)\n"
"--net-heartbeat <rate>   TCP heartbeat rate in seconds (default: 60 sec; 0 to disable)\n"
"--net-buffer <n>         TCP buffer size 64Kb * (2^n) (default: n=0, 64Kb)\n"
"--lat <latitude>         Reference/receiver latitude for surface posn (opt)\n"
"--lon <longitude>        Reference/receiver longitude for surface posn (opt)\n"
"--fix                    Enable single-bits error correction using CRC\n"
"--no-fix                 Disable single-bits error correction using CRC\n"
"--no-crc-check           Disable messages with broken CRC (discouraged)\n"
"--phase-enhance          Enable phase enhancement\n"
"--aggressive             More CPU for more messages (two bits fixes, ...)\n"
"--mlat                   display raw messages in Beast ascii mode\n"
"--stats                  With --ifile print stats at exit. No other output\n"
"--stats-every <seconds>  Show and reset stats every <seconds> seconds\n"
"--onlyaddr               Show only ICAO addresses (testing purposes)\n"
"--metric                 Use metric units (meters, km/h, ...)\n"
"--snip <level>           Strip IQ file removing samples < level\n"
"--debug <flags>          Debug mode (verbose), see README for details\n"
"--quiet                  Disable output to stdout. Use for daemon applications\n"
"--ppm <error>            Set receiver error in parts per million (default 0)\n"
"--no-decode              Don't decode the message contents beyond the minimum necessary\n"
"--write-json <dir>       Periodically write json output to <dir> (for serving by a separate webserver)\n"
"--write-json-every <t>   Write json output every t seconds (default 1)\n"
"--json-location-accuracy <n>  Accuracy of receiver location in json metadata: 0=no location, 1=approximate, 2=exact\n"
"--help                   Show this help\n"
"\n"
"Debug mode flags: d = Log frames decoded with errors\n"
"                  D = Log frames decoded with zero errors\n"
"                  c = Log frames with bad CRC\n"
"                  C = Log frames with good CRC\n"
"                  p = Log frames with bad preamble\n"
"                  n = Log network debugging info\n"
"                  j = Log frames to frames.js, loadable by debug.html\n",
MODES_DUMP1090_VARIANT " " MODES_DUMP1090_VERSION
    );
}

#ifdef _WIN32
void showCopyright(void) {
    uint64_t llTime = time(NULL) + 1;

    printf(
"-----------------------------------------------------------------------------\n"
"|                        dump1090 ModeS Receiver         Ver : " MODES_DUMP1090_VERSION " |\n"
"-----------------------------------------------------------------------------\n"
"\n"
" Copyright (C) 2012 by Salvatore Sanfilippo <antirez@gmail.com>\n"
" Copyright (C) 2014 by Malcolm Robb <support@attavionics.com>\n"
"\n"
" All rights reserved.\n"
"\n"
" THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS\n"
" ""AS IS"" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT\n"
" LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR\n"
" A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT\n"
" HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,\n"
" SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT\n"
" LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,\n"
" DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY\n"
" THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT\n"
" (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE\n"
" OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.\n"
"\n"
" For further details refer to <https://github.com/MalcolmRobb/dump1090>\n" 
"\n"
    );

  // delay for 1 second to give the user a chance to read the copyright
  while (llTime >= time(NULL)) {}
}
#endif


static void display_demod_stats(const char *prefix, struct demod_stats *dstats) {
    int j;

    printf("%d %sdemodulated with 0 errors\n",                  dstats->demodulated0, prefix);
    printf("%d %sdemodulated with 1 error\n",                   dstats->demodulated1, prefix);
    printf("%d %sdemodulated with 2 errors\n",                  dstats->demodulated2, prefix);
    printf("%d %sdemodulated with > 2 errors\n",                dstats->demodulated3, prefix);
    printf("%d %swith good crc\n",                              dstats->goodcrc, prefix);
    for (j = 0; j < MODES_MAX_PHASE_STATS; ++j)
        if (dstats->goodcrc_byphase[j] > 0)
            printf("   %d %swith phase offset %d\n",            dstats->goodcrc_byphase[j], prefix, j);
    printf("%d %swith bad crc\n",                               dstats->badcrc, prefix);
    printf("%d %serrors corrected\n",                           dstats->fixed, prefix);

    for (j = 0;  j < Modes.nfix_crc;  j++) {
        printf("   %d %swith %d bit %s\n", dstats->bit_fix[j], prefix, j+1, (j==0)?"error":"errors");
    }
}

static void reset_demod_stats(struct demod_stats *dstats) {
    int j;

    dstats->demodulated0 = 
        dstats->demodulated1 =
        dstats->demodulated2 =
        dstats->goodcrc =
        dstats->badcrc =
        dstats->fixed = 0;

    for (j = 0;  j < MODES_MAX_BITERRORS;  j++) {
        dstats->bit_fix[j] = 0;
    }

    for (j = 0;  j < MODES_MAX_PHASE_STATS;  j++) {
        dstats->goodcrc_byphase[j] = 0;
    }
}

static void display_stats(void) {
    int j;
    time_t now = time(NULL);

    printf("\n\n");
    if (Modes.interactive)
        interactiveShowData();

    printf("Statistics as at %s", ctime(&now));

    if (!Modes.net_only) {
        printf("%d sample blocks processed\n",                    Modes.stat_blocks_processed);
        printf("%d sample blocks dropped\n",                      Modes.stat_blocks_dropped);

        if (Modes.stat_blocks_processed > 0) {
            long cpu_millis = (long)Modes.stat_cputime.tv_sec*1000L + Modes.stat_cputime.tv_nsec/1000000L;
            long sample_millis = (long) ((uint64_t)Modes.stat_blocks_processed * MODES_ASYNC_BUF_SAMPLES / (Modes.oversample ? 2400 : 2000));
            printf("%ld ms CPU time used to process %ld ms samples, %.1f%% load\n",
                   cpu_millis, sample_millis, 100.0 * cpu_millis / sample_millis);
        }

        printf("%d ModeA/C detected\n",                           Modes.stat_ModeAC);
        printf("%d Mode-S preambles with poor correlation\n",     Modes.stat_preamble_no_correlation);
        printf("%d Mode-S preambles with noise in the quiet period\n", Modes.stat_preamble_not_quiet);
        printf("%d valid Mode-S preambles\n",                     Modes.stat_valid_preamble);
        for (j = 0; j < MODES_MAX_PHASE_STATS; ++j)
            if (Modes.stat_preamble_phase[j] > 0)
                printf("   %d with phase offset %d\n",                Modes.stat_preamble_phase[j], j);
        printf("%d DF-?? fields corrected for length\n",          Modes.stat_DF_Len_Corrected);
        printf("%d DF-?? fields corrected for type\n",            Modes.stat_DF_Type_Corrected);
        
        display_demod_stats("", &Modes.stat_demod);
        if (Modes.phase_enhance) {
            printf("%d phase enhancement attempts\n",                 Modes.stat_out_of_phase);
            display_demod_stats("phase enhanced ", &Modes.stat_demod_phasecorrected);
        }
        
        printf("%d total usable messages\n",
               Modes.stat_demod.goodcrc + Modes.stat_demod_phasecorrected.goodcrc +
               Modes.stat_demod.fixed + Modes.stat_demod_phasecorrected.fixed);

        if (Modes.stat_noise_count) {
            printf("Noise floor: %.1f dBFS\n",
                   10 * log10(Modes.stat_noise_power / Modes.stat_noise_count) - 96.33); // 96.33 is 10log10(65536 ** 2) (full range power)
        }
    }

    printf("%d remote messages accepted\n"
           "%d remote messages rejected\n",
           Modes.stat_remote_accepted,
           Modes.stat_remote_rejected);

    printf("%d global CPR attempts with valid positions\n"
           "%d global CPR attempts with bad data\n"
           "%d global CPR attempts with insufficient data\n"
           "%d local CPR attempts with valid positions\n"
           "%d local CPR attempts with insufficient data\n"
           "%d CPR messages that look like transponder failures filtered\n",
           Modes.stat_cpr_global_ok,
           Modes.stat_cpr_global_bad,
           Modes.stat_cpr_global_skipped,
           Modes.stat_cpr_local_ok,
           Modes.stat_cpr_local_skipped,
           Modes.stat_cpr_filtered);

    fflush(stdout);

    Modes.stat_cputime.tv_sec = 0;
    Modes.stat_cputime.tv_nsec = 0;

    Modes.stat_blocks_processed =
        Modes.stat_blocks_dropped = 0;

    Modes.stat_ModeAC =
        Modes.stat_preamble_no_correlation =
        Modes.stat_preamble_not_quiet =
        Modes.stat_valid_preamble =
        Modes.stat_DF_Len_Corrected =
        Modes.stat_DF_Type_Corrected = 
        Modes.stat_out_of_phase = 0;

    Modes.stat_remote_accepted =
        Modes.stat_remote_rejected = 0;

    Modes.stat_noise_power = 0;
    Modes.stat_noise_count = 0;

    Modes.stat_cpr_global_ok = 
        Modes.stat_cpr_global_bad =
        Modes.stat_cpr_global_skipped = 
        Modes.stat_cpr_local_ok = 
        Modes.stat_cpr_local_skipped = 
        Modes.stat_cpr_filtered = 0;

    for (j = 0;  j < MODES_MAX_PHASE_STATS;  j++) {
        Modes.stat_preamble_phase[j] = 0;
    }

    reset_demod_stats(&Modes.stat_demod);
    reset_demod_stats(&Modes.stat_demod_phasecorrected);
}


//
//=========================================================================
//
// This function is called a few times every second by main in order to
// perform tasks we need to do continuously, like accepting new clients
// from the net, refreshing the screen in interactive mode, and so forth
//
void backgroundTasks(void) {
    static time_t next_stats;
    static time_t next_json;

    icaoFilterExpire();

    if (Modes.net) {
	modesNetPeriodicWork();
    }    

    // If Modes.aircrafts is not NULL, remove any stale aircraft
    if (Modes.aircrafts) {
        interactiveRemoveStaleAircrafts();
    }

    // Refresh screen when in interactive mode
    if (Modes.interactive) {
        interactiveShowData();
    }

    if (Modes.stats > 0) {
        time_t now = time(NULL);
        if (now > next_stats) {
            if (next_stats != 0)
                display_stats();
            next_stats = now + Modes.stats;
        }
    }

    if (Modes.json_aircraft_path && Modes.json_interval > 0) {
        time_t now = time(NULL);
        if (now >= next_json) {
            writeJsonToFile(Modes.json_aircraft_path, generateAircraftJson);
            next_json = now + Modes.json_interval;
        }
    }
}

//
//=========================================================================
//
int verbose_device_search(char *s)
{
	int i, device_count, device, offset;
	char *s2;
	char vendor[256], product[256], serial[256];
	device_count = rtlsdr_get_device_count();
	if (!device_count) {
		fprintf(stderr, "No supported devices found.\n");
		return -1;
	}
	fprintf(stderr, "Found %d device(s):\n", device_count);
	for (i = 0; i < device_count; i++) {
		rtlsdr_get_device_usb_strings(i, vendor, product, serial);
		fprintf(stderr, "  %d:  %s, %s, SN: %s\n", i, vendor, product, serial);
	}
	fprintf(stderr, "\n");
	/* does string look like raw id number */
	device = (int)strtol(s, &s2, 0);
	if (s2[0] == '\0' && device >= 0 && device < device_count) {
		fprintf(stderr, "Using device %d: %s\n",
			device, rtlsdr_get_device_name((uint32_t)device));
		return device;
	}
	/* does string exact match a serial */
	for (i = 0; i < device_count; i++) {
		rtlsdr_get_device_usb_strings(i, vendor, product, serial);
		if (strcmp(s, serial) != 0) {
			continue;}
		device = i;
		fprintf(stderr, "Using device %d: %s\n",
			device, rtlsdr_get_device_name((uint32_t)device));
		return device;
	}
	/* does string prefix match a serial */
	for (i = 0; i < device_count; i++) {
		rtlsdr_get_device_usb_strings(i, vendor, product, serial);
		if (strncmp(s, serial, strlen(s)) != 0) {
			continue;}
		device = i;
		fprintf(stderr, "Using device %d: %s\n",
			device, rtlsdr_get_device_name((uint32_t)device));
		return device;
	}
	/* does string suffix match a serial */
	for (i = 0; i < device_count; i++) {
		rtlsdr_get_device_usb_strings(i, vendor, product, serial);
		offset = strlen(serial) - strlen(s);
		if (offset < 0) {
			continue;}
		if (strncmp(s, serial+offset, strlen(s)) != 0) {
			continue;}
		device = i;
		fprintf(stderr, "Using device %d: %s\n",
			device, rtlsdr_get_device_name((uint32_t)device));
		return device;
	}
	fprintf(stderr, "No matching devices found.\n");
	return -1;
}
//
//=========================================================================
//
int main(int argc, char **argv) {
    int j;

    // Set sane defaults
    modesInitConfig();
    signal(SIGINT, sigintHandler); // Define Ctrl/C handler (exit program)

    // Parse the command line options
    for (j = 1; j < argc; j++) {
        int more = j+1 < argc; // There are more arguments

        if (!strcmp(argv[j],"--device-index") && more) {
            Modes.dev_name = strdup(argv[++j]);
        } else if (!strcmp(argv[j],"--gain") && more) {
            Modes.gain = (int) (atof(argv[++j])*10); // Gain is in tens of DBs
        } else if (!strcmp(argv[j],"--enable-agc")) {
            Modes.enable_agc++;
        } else if (!strcmp(argv[j],"--freq") && more) {
            Modes.freq = (int) strtoll(argv[++j],NULL,10);
        } else if (!strcmp(argv[j],"--ifile") && more) {
            Modes.filename = strdup(argv[++j]);
        } else if (!strcmp(argv[j],"--fix")) {
            if (!Modes.nfix_crc)
                Modes.nfix_crc = 1;
        } else if (!strcmp(argv[j],"--no-fix")) {
            Modes.nfix_crc = 0;
        } else if (!strcmp(argv[j],"--no-crc-check")) {
            Modes.check_crc = 0;
        } else if (!strcmp(argv[j],"--phase-enhance")) {
            Modes.phase_enhance = 1;
        } else if (!strcmp(argv[j],"--raw")) {
            Modes.raw = 1;
        } else if (!strcmp(argv[j],"--net")) {
            Modes.net = 1;
        } else if (!strcmp(argv[j],"--modeac")) {
            Modes.mode_ac = 1;
        } else if (!strcmp(argv[j],"--net-beast")) {
            Modes.beast = 1;
        } else if (!strcmp(argv[j],"--net-only")) {
            Modes.net = 1;
            Modes.net_only = 1;
       } else if (!strcmp(argv[j],"--net-heartbeat") && more) {
            Modes.net_heartbeat_interval = atoi(argv[++j]);
       } else if (!strcmp(argv[j],"--net-ro-size") && more) {
            Modes.net_output_flush_size = atoi(argv[++j]);
        } else if (!strcmp(argv[j],"--net-ro-rate") && more) {
            Modes.net_output_flush_interval = atoi(argv[++j]) / 15; // backwards compatibility
        } else if (!strcmp(argv[j],"--net-ro-interval") && more) {
            Modes.net_output_flush_interval = atoi(argv[++j]);
        } else if (!strcmp(argv[j],"--net-ro-port") && more) {
            if (Modes.beast) // Required for legacy backward compatibility
                {Modes.net_output_beast_port = atoi(argv[++j]);;}
            else
                {Modes.net_output_raw_port = atoi(argv[++j]);}
        } else if (!strcmp(argv[j],"--net-ri-port") && more) {
            Modes.net_input_raw_port = atoi(argv[++j]);
        } else if (!strcmp(argv[j],"--net-bo-port") && more) {
            Modes.net_output_beast_port = atoi(argv[++j]);
        } else if (!strcmp(argv[j],"--net-bi-port") && more) {
            Modes.net_input_beast_port = atoi(argv[++j]);
        } else if (!strcmp(argv[j],"--net-bind-address") && more) {
            Modes.net_bind_address = strdup(argv[++j]);
        } else if (!strcmp(argv[j],"--net-http-port") && more) {
            Modes.net_http_port = atoi(argv[++j]);
        } else if (!strcmp(argv[j],"--net-fatsv-port") && more) {
            Modes.net_fatsv_port = atoi(argv[++j]);
        } else if (!strcmp(argv[j],"--net-sbs-port") && more) {
            Modes.net_output_sbs_port = atoi(argv[++j]);
        } else if (!strcmp(argv[j],"--net-buffer") && more) {
            Modes.net_sndbuf_size = atoi(argv[++j]);
        } else if (!strcmp(argv[j],"--onlyaddr")) {
            Modes.onlyaddr = 1;
        } else if (!strcmp(argv[j],"--metric")) {
            Modes.metric = 1;
        } else if (!strcmp(argv[j],"--aggressive")) {
            if (Modes.nfix_crc < 2)
                Modes.nfix_crc = 2;
            else
                Modes.nfix_crc++;

            if (Modes.nfix_crc > MODES_MAX_BITERRORS)
                Modes.nfix_crc = MODES_MAX_BITERRORS;
        } else if (!strcmp(argv[j],"--interactive")) {
            Modes.interactive = 1;
        } else if (!strcmp(argv[j],"--interactive-rows") && more) {
            Modes.interactive_rows = atoi(argv[++j]);
        } else if (!strcmp(argv[j],"--interactive-ttl") && more) {
            Modes.interactive_display_ttl = atoi(argv[++j]);
        } else if (!strcmp(argv[j],"--lat") && more) {
            Modes.fUserLat = atof(argv[++j]);
        } else if (!strcmp(argv[j],"--lon") && more) {
            Modes.fUserLon = atof(argv[++j]);
        } else if (!strcmp(argv[j],"--debug") && more) {
            char *f = argv[++j];
            while(*f) {
                switch(*f) {
                case 'D': Modes.debug |= MODES_DEBUG_DEMOD; break;
                case 'd': Modes.debug |= MODES_DEBUG_DEMODERR; break;
                case 'C': Modes.debug |= MODES_DEBUG_GOODCRC; break;
                case 'c': Modes.debug |= MODES_DEBUG_BADCRC; break;
                case 'p': Modes.debug |= MODES_DEBUG_NOPREAMBLE; break;
                case 'n': Modes.debug |= MODES_DEBUG_NET; break;
                case 'j': Modes.debug |= MODES_DEBUG_JS; break;
                default:
                    fprintf(stderr, "Unknown debugging flag: %c\n", *f);
                    exit(1);
                    break;
                }
                f++;
            }
        } else if (!strcmp(argv[j],"--stats")) {
            Modes.stats = -1;
        } else if (!strcmp(argv[j],"--stats-every") && more) {
            Modes.stats = atoi(argv[++j]);
        } else if (!strcmp(argv[j],"--snip") && more) {
            snipMode(atoi(argv[++j]));
            exit(0);
        } else if (!strcmp(argv[j],"--help")) {
            showHelp();
            exit(0);
        } else if (!strcmp(argv[j],"--ppm") && more) {
            Modes.ppm_error = atoi(argv[++j]);
        } else if (!strcmp(argv[j],"--quiet")) {
            Modes.quiet = 1;
        } else if (!strcmp(argv[j],"--mlat")) {
            Modes.mlat = 1;
        } else if (!strcmp(argv[j],"--interactive-rtl1090")) {
            Modes.interactive = 1;
            Modes.interactive_rtl1090 = 1;
        } else if (!strcmp(argv[j],"--oversample")) {
            Modes.oversample = 1;
#ifndef _WIN32
        } else if (!strcmp(argv[j], "--write-json") && more) {
            ++j;
            Modes.json_aircraft_path = malloc(strlen(argv[j]) + 15);
            strcpy(Modes.json_aircraft_path, argv[j]);
            strcat(Modes.json_aircraft_path, "/aircraft.json");

            Modes.json_metadata_path = malloc(strlen(argv[j]) + 15);
            strcpy(Modes.json_metadata_path, argv[j]);
            strcat(Modes.json_metadata_path, "/receiver.json");
        } else if (!strcmp(argv[j], "--write-json-every") && more) {
            Modes.json_interval = atoi(argv[++j]);
        } else if (!strcmp(argv[j], "--json-location-accuracy") && more) {
            Modes.json_location_accuracy = atoi(argv[++j]);
#endif
        } else {
            fprintf(stderr,
                "Unknown or not enough arguments for option '%s'.\n\n",
                argv[j]);
            showHelp();
            exit(1);
        }
    }

#ifdef _WIN32
    // Try to comply with the Copyright license conditions for binary distribution
    if (!Modes.quiet) {showCopyright();}
#endif

#ifndef _WIN32
    // Setup for SIGWINCH for handling lines
    if (Modes.interactive) {signal(SIGWINCH, sigWinchCallback);}
#endif

    if (Modes.mode_ac && Modes.oversample) {
        fprintf(stderr,
                "Warning: --modeac is currently ignored when --oversample is used;\n"
                "         no ModeA/C messages will be decoded.\n");
    }

    // Initialization
    modesInit();

    if (Modes.net_only) {
        fprintf(stderr,"Net-only mode, no RTL device or file open.\n");
    } else if (Modes.filename == NULL) {
        if (modesInitRTLSDR() < 0) {
            exit(1);
        }
    } else {
        if (Modes.filename[0] == '-' && Modes.filename[1] == '\0') {
            Modes.fd = STDIN_FILENO;
        } else if ((Modes.fd = open(Modes.filename,
#ifdef _WIN32
                                    (O_RDONLY | O_BINARY)
#else
                                    (O_RDONLY)
#endif
                                    )) == -1) {
            perror("Opening data file");
            exit(1);
        }
    }
    if (Modes.net) modesInitNet();

    if (Modes.json_metadata_path && Modes.json_interval > 0) {
        writeJsonToFile(Modes.json_metadata_path, generateReceiverJson); // once only on startup
    }

    // If the user specifies --net-only, just run in order to serve network
    // clients without reading data from the RTL device
    while (Modes.net_only) {
        if (Modes.exit) {
            display_stats();
            exit(0); // If we exit net_only nothing further in main()
        }
        backgroundTasks();
        usleep(100000);
    }

    // Create the thread that will read the data from the device.
    pthread_create(&Modes.reader_thread, NULL, readerThreadEntryPoint, NULL);
    pthread_mutex_lock(&Modes.data_mutex);

    while (Modes.exit == 0) {
        struct timespec cpu_start_time, cpu_end_time;

        if (Modes.iDataReady == 0) {
            pthread_cond_wait(&Modes.data_cond,&Modes.data_mutex); // This unlocks Modes.data_mutex, and waits for Modes.data_cond 
            continue;                                              // Once (Modes.data_cond) occurs, it locks Modes.data_mutex
        }

        // Modes.data_mutex is Locked, and (Modes.iDataReady != 0)
        if (Modes.iDataReady) { // Check we have new data, just in case!!
 
            Modes.iDataOut &= (MODES_ASYNC_BUF_NUMBER-1); // Just incase

            // Translate the next lot of I/Q samples into Modes.magnitude
            computeMagnitudeVector(Modes.pData[Modes.iDataOut]);

            Modes.stSystemTimeBlk = Modes.stSystemTimeRTL[Modes.iDataOut];

            // Update the input buffer pointer queue
            Modes.iDataOut   = (MODES_ASYNC_BUF_NUMBER-1) & (Modes.iDataOut + 1); 
            Modes.iDataReady = (MODES_ASYNC_BUF_NUMBER-1) & (Modes.iDataIn - Modes.iDataOut);   

            // If we lost some blocks, correct the timestamp
            if (Modes.iDataLost) {
                Modes.timestampBlk += (MODES_ASYNC_BUF_SAMPLES * 6 * Modes.iDataLost);
                Modes.stat_blocks_dropped += Modes.iDataLost;
                Modes.iDataLost = 0;
            }

            // It's safe to release the lock now
            pthread_cond_signal (&Modes.data_cond);
            pthread_mutex_unlock(&Modes.data_mutex);

            // Process data after releasing the lock, so that the capturing
            // thread can read data while we perform computationally expensive
            // stuff at the same time.

            clock_gettime(CLOCK_THREAD_CPUTIME_ID, &cpu_start_time);
            
            if (Modes.oversample)
                detectModeS_oversample(Modes.magnitude, MODES_ASYNC_BUF_SAMPLES);
            else
                detectModeS(Modes.magnitude, MODES_ASYNC_BUF_SAMPLES);

            clock_gettime(CLOCK_THREAD_CPUTIME_ID, &cpu_end_time);
            Modes.stat_cputime.tv_sec += (cpu_end_time.tv_sec - cpu_start_time.tv_sec);
            Modes.stat_cputime.tv_nsec += (cpu_end_time.tv_nsec - cpu_start_time.tv_nsec);
            if (Modes.stat_cputime.tv_nsec < 0) {
                Modes.stat_cputime.tv_nsec += 1000000000L;
                Modes.stat_cputime.tv_sec--;
            } else if (Modes.stat_cputime.tv_nsec > 1000000000L) {
                Modes.stat_cputime.tv_nsec -= 1000000000L;
                Modes.stat_cputime.tv_sec++;
            }

            // Update the timestamp ready for the next block
            if (Modes.oversample)
                Modes.timestampBlk += (MODES_ASYNC_BUF_SAMPLES*5);
            else
                Modes.timestampBlk += (MODES_ASYNC_BUF_SAMPLES*6);
            Modes.stat_blocks_processed++;
        } else {
            pthread_cond_signal (&Modes.data_cond);
            pthread_mutex_unlock(&Modes.data_mutex);
        }

        backgroundTasks();
        pthread_mutex_lock(&Modes.data_mutex);
    }

    // If --stats were given, print statistics
    if (Modes.stats) {
        display_stats();
    }

    if (Modes.filename == NULL) {
        rtlsdr_cancel_async(Modes.dev);  // Cancel rtlsdr_read_async will cause data input thread to terminate cleanly
        rtlsdr_close(Modes.dev);
    }

    pthread_join(Modes.reader_thread,NULL);     // Wait on reader thread exit
    pthread_cond_destroy(&Modes.data_cond);     // Thread cleanup - only after the reader thread is dead!
    pthread_mutex_destroy(&Modes.data_mutex);

#ifndef _WIN32
    pthread_exit(0);
#else
    return (0);
#endif
}
//
//=========================================================================
//

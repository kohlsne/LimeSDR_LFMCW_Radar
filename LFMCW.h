#ifndef LFMCW_H
#define LFMCW_H

#include "LimeSuite.h" // The Library for talking to the LimeSDR
#include <iostream>
#include <chrono>
#include <math.h>
#include <fftw3.h> //http://fftw.org/
#include <algorithm>// std::rotate
#include <stdint.h>

// LMS_FMT_I12 LMS_FMT_I16 LMS_FMT_F32

#define REAL 0
#define IMAG 1
#define CHANNEL0 0
#define CHANNEL1 1
#define c (299792458 / 3 * 2)


#define IF_OFFSET 0

//#define IF_OFFSET 39000

#define LMS_DATA_TYPE_F32 0 //RX rate: 32.1782 MB/s TX rate: 5.37395 MB/s
#define LMS_DATA_TYPE_I16 1 //RX rate: 32.1782 MB/s TX rate: 5.37395 MB/s
#define LMS_DATA_TYPE_I12 2 //RX rate: 24.15 MB/s  TX rate: 4.05917 MB/s

#define LMS_DATA_TYPE LMS_DATA_TYPE_I12
#define INT12_MAX 0x7ff
//#undef INT16_MAX
//#define INT16_MAX 0x7ff0

//Valid oversampling values are 1, 2, 4, 8, 16, 32 or 0 (use device default oversampling value).
#define OVERSAMPLE_SCALER 1 //Find out more about this

#define TRUE 1
#define FALSE 0

#define PRINT TRUE
#define PLOT FALSE
#define PLOT_RANGE FALSE
#define SAVE TRUE
#define SIG_PROCESS FALSE


#if PLOT
#include "gnuPlotPipe.h" //Used to plot
#endif

//Device structure, should be initialize to NULL
extern lms_device_t* device;

//Helper function for errors
int error();

//Init the LimeSDR
void initDevice();

void printDeviceOptions();

void printDeviceConfig();

void printStreamStatus(lms_stream_t *stream, bool RxOrTx);

void checkStreamStatus(lms_stream_t *RXstream,lms_stream_t *TXstream);

void enableChannels();

void setCenterFreq(float freqRX, float freqTX);

void setGain(float gainRX, float gainTX);

void setSampleRate(float Fs);

void setLPF(float B);

void calibrate(float B);

void setupSteams(lms_stream_t &rx_streams, lms_stream_t &tx_streams);

void closeDevice();

void fftShift(float *data, uint32_t N);

void findPeak(float *outMag, uint32_t Fs, uint32_t N, float &max_x, float &max_y);


void seconds2Samples(float startDelay_sec, float Fs, uint32_t sizeOfChirp, uint32_t numOfChirps, uint32_t &startDelayChirps, uint32_t &startDelaySamps);

//Use metadata for additional control over sample function behavior
void setMetaData(lms_stream_meta_t &rx_metadata,  lms_stream_meta_t &tx_metadata);


void mixer(fftwf_complex *rx, fftwf_complex *tx, uint32_t sizeOfChirp, uint32_t numOfChirps);

void normalizeRX(fftwf_complex *rx, uint32_t N);

void complex2Magnitude(float *outMag, fftwf_complex *out,  uint32_t N);


void makeChirp(int16_t *signal, uint32_t sizeOfChirp, float Tc, float B, float Fc_start);

void makeChirp(fftwf_complex *signal, uint32_t sizeOfChirp, float Tc, float B, float Fc_start);






#if LMS_DATA_TYPE == LMS_DATA_TYPE_I12 || LMS_DATA_TYPE == LMS_DATA_TYPE_I16
    void syncTimeStamps(float wait,int16_t *bufferRX, int16_t *txSignal, lms_stream_t &rx_stream, lms_stream_t &tx_stream,lms_stream_meta_t &rx_metadata, lms_stream_meta_t &tx_metadata, uint32_t timeoutmsRX,  uint32_t timeoutmsTX, uint32_t N, uint32_t sizeOfChirp, uint32_t numOfChirps, float Fs);


    void convert2Float(int16_t *in, fftwf_complex *out, uint32_t N);

    #if SIG_PROCESS
        void processData(int16_t *bufferRX, fftwf_complex *txSignal, float *rxSigMag, float Fs, uint32_t N, uint32_t sizeOfChirp, uint32_t numOfChirps, float &max_x, float &max_y, float S);
    #endif

#else //if LMS_DATA_TYPE == LMS_DATA_TYPE_F32
    void syncTimeStamps(float wait,fftwf_complex *bufferRX, fftwf_complex *txSignal, lms_stream_t &rx_stream, lms_stream_t &tx_stream,lms_stream_meta_t &rx_metadata, lms_stream_meta_t &tx_metadata, uint32_t timeoutmsRX,  uint32_t timeoutmsTX, uint32_t N, uint32_t sizeOfChirp, uint32_t numOfChirps, float Fs);
    #if SIG_PROCESS
        void processData(fftwf_complex *bufferRX, fftwf_complex *txSignal, float *rxSigMag, float Fs, uint32_t N, uint32_t sizeOfChirp, uint32_t numOfChirps, float &max_x, float &max_y, float S);
    #endif

//void makeChirp(fftwf_complex *signal, uint32_t sizeOfChirp, float Tc, float B, float Fc_start);
#endif


#if PLOT
void plotDFT(float *outMag, uint32_t Fs, uint32_t N, float peak, float S);
#endif

#endif

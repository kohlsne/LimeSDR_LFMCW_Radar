
/**
 @file   LFMCW.cpp
 @author Nicholas Kohls
 @brief  LFMCW
 */

//API: https://docs.myriadrf.org/LMS_API/index.html
// g++ -o LFMCW LFMCW.cpp -lLimeSuite -lfftw3 -lfftw3f && ./LFMCW

#include "LFMCW.h"



//(625 MB/s) usb3 megabyte per second
using namespace std;

//Helper function for errors
int error(){
    cout<<"ERROR:"<<LMS_GetLastErrorMessage() << endl;
    if (device != NULL)
        LMS_Close(device);
    exit(-1);
}

//Init the LimeSDR
void initDevice(){
    //Find devices
    int n;
    lms_info_str_t list[8]; //should be large enough to hold all detected devices
    if ((n = LMS_GetDeviceList(list)) < 0) //NULL can be passed to only get number of devices
        error();
    
    if(PRINT) cout << "Devices found: " << n << endl; //print number of devices
    if (n < 1)
        exit(-1);
    
    //open the first device
    if (LMS_Open(&device, list[0], NULL))
        error();
    //Initialize device with default configuration
    //Do not use if you want to keep existing configuration
//    //Use LMS_LoadConfig(device, "/path/to/file.ini") to load config from INI
    if (LMS_Init(device) != 0)
        error();
}


void enableChannels(){
    if (LMS_EnableChannel(device, LMS_CH_RX, CHANNEL0, true) != 0) error();
    if (LMS_EnableChannel(device, LMS_CH_TX, CHANNEL0, true) != 0) error();
}

void setCenterFreq(float freqRX, float freqTX){
    if (LMS_SetLOFrequency(device, LMS_CH_RX, CHANNEL0, freqRX) != 0) error();
    if (LMS_SetLOFrequency(device, LMS_CH_TX, CHANNEL0, freqTX) != 0) error();
    //Automatically selects antenna port
}
//Set the combined gain value This function computes and sets the optimal gain values of various amplifiers that are present in the device based on desired normalized gain value.
//actual gain depends on LO frequency and analog LPF configuration and resulting output signal level may be different when those values are changed
//Desired gain, range [0, 1.0], where 1.0 represents the maximum gain
void setGain(float gainRX, float gainTX){
    if (LMS_SetNormalizedGain(device, LMS_CH_RX, CHANNEL0, gainRX) != 0) error();
    if (LMS_SetNormalizedGain(device, LMS_CH_TX, CHANNEL0, gainTX) != 0) error();
}
//This only implements channel 0, I havent made everything generic yet
void setupSteams(lms_stream_t &rx_streams, lms_stream_t &tx_streams){
    //Initialize streams
    //All streams setups should be done before starting streams. New streams cannot be set-up if at least stream is runniing.
    rx_streams.channel = CHANNEL0; //channel number
    tx_streams.channel = CHANNEL0; //channel number
    rx_streams.fifoSize =  uint32_t(4080 * 4096);  //fifo size in samples
//    rx_streams.fifoSize =  (UINT32_MAX & 0x1FFFFFE0);  //fifo size in samples
    tx_streams.fifoSize = uint32_t(4080 * 4096); //fifo size in samples
    rx_streams.throughputVsLatency = 0.5; //some middle ground
    tx_streams.throughputVsLatency = 0.5; //some middle ground -- Look into this
    rx_streams.isTx = false; //RX channel
    tx_streams.isTx = true; //TX channel
    #if LMS_DATA_TYPE == LMS_DATA_TYPE_I12
    rx_streams.dataFmt = lms_stream_t::LMS_FMT_I12; //12-bit integers
    tx_streams.dataFmt = lms_stream_t::LMS_FMT_I12; //12-bit integers
    if(PRINT) cout << "LMS_FMT_I12\n";
    #elif LMS_DATA_TYPE == LMS_DATA_TYPE_I16
    rx_streams.dataFmt = lms_stream_t::LMS_FMT_I16; //16-bit integers
    tx_streams.dataFmt = lms_stream_t::LMS_FMT_I16; //16-bit integers
    if(PRINT) cout << "LMS_FMT_I16\n";
    #else
    rx_streams.dataFmt = lms_stream_t::LMS_FMT_F32; //32-bit floating point
    tx_streams.dataFmt = lms_stream_t::LMS_FMT_F32; //32-bit floating point
    if(PRINT) cout << "LMS_FMT_F32\n";
    #endif

    if (LMS_SetupStream(device, &rx_streams) != 0) error();
    if (LMS_SetupStream(device, &tx_streams) != 0) error();

    //FIFO size (in samples) used by stream.
    //    Set the size of FIFO buffer to ~1 Msample (fifosize = 1024*1024). This sets the sizeof LMS API buffer for this stream on host PC. The actual size may not be exactly asrequested.
    //    Parameter for controlling configuration bias toward low latency or high data throughput range [0,1.0]. 0 - lowest latency, usually results in lower throughput 1 - higher throughput, usually results in higher latency, It affects the size of data transfers from hardware
    //Data between hardware and PC is transferred using 8 bytes (I(32-bit)+Q(32-bit)) per sample.
}
 
//converts the fftwf_complex values to magnitude
void complex2Magnitude(float *outMag, fftwf_complex *out, uint32_t N){
    for(uint32_t i = 0; i<N;i++){
        outMag[i] = 10 * log10(sqrt(out[i][REAL]*out[i][REAL] + out[i][IMAG]*out[i][IMAG]));
//        outMag[i] = sqrt(out[i][REAL]*out[i][REAL] + out[i][IMAG]*out[i][IMAG]);
    }
}


void closeDevice(){LMS_Close(device);}

//Set sampling rate for all RX/TX channels. Sample rate is in complex samples (1 sample = I + Q). The function sets sampling rate that is used for data exchange with the host. It also allows to specify higher sampling rate to be used in RF by setting oversampling ratio.
//t also allows to specify oversampling that should beused in hardware. In this case oversampling is 2, which means that hardware will besampling RF signal at 16 MHz rate and downsampling it to 8 MHz before sendingsamples to PC.
void setSampleRate(float Fs){
    if (LMS_SetSampleRate(device, Fs, OVERSAMPLE_SCALER) != 0)
        error(); //What is the OVERSAMPLE_SCALER? look into API
}

//fft shift so you can plot it and it will make sense
void fftShift(float *data, uint32_t N){
    if (N % 2 == 0)
        rotate(&data[0],&data[N>>1],&data[N]);
    else
        rotate(&data[0],&data[(N>>1)+1],&data[N]);
}

#if PLOT
void plotDFT(float *outMag, uint32_t Fs, uint32_t N, float peak,float S){
    GNUPlotPipe gp;
    float X,Y;
    float df=float(Fs)/N;

#if PLOT_RANGE //plot with range as x axis
//        float temp = (IF_OFFSET - 50) * c / (2 * S);
//        float temp1 = (IF_OFFSET + 800) * c / (2 * S);
    float temp = -50;
    float temp1 = 500;
    gp.writef("set size square\n set xrange[%f:%f]\n set yrange[-1:%f]\n set title 'Time Domain'\n set xlabel 'Range'\n set ylabel 'dB'\n", temp, temp1, peak * 1.1);
    gp.write("plot '-' w l\n");
    float const0 = -IF_OFFSET - Fs/2;
    float const1 = c / (2 * S);
    for (uint32_t i = 0; i < N; i++){
        X = (i * df + const0) * const1;
        Y = outMag[i];
        gp.writef("%f %f\n", X, Y);
    }
#else //plot dft with freq as x axis
    float offset = float(Fs/2);
    gp.writef("set size square\n set xrange[%f:%f]\n set yrange[-1:%f]\n set title 'Time Domain'\n set xlabel 'Freq'\n set ylabel 'dB'\n", -offset, offset, peak * 1.1);
//    gp.writef("set size square\n set xrange[%f:%f]\n set yrange[-1:%f]\n set title 'Time Domain'\n set xlabel 'Freq'\n set ylabel 'Amplitude'\n", IF_OFFSET - 4000, IF_OFFSET + 3500, peak * 1.1);
    gp.write("plot '-' w l\n");
    for (uint32_t i = 0; i < N; i++){
        X = i * df - Fs/2;
        Y = outMag[i];
        gp.writef("%f %f\n", X, Y);
    }
#endif

    gp.write("e\n");
    gp.flush();
}
#endif

//"\nBin Range(m):\t"<<float(Fs)/N * c / (1 * S

//Find the max peak
void findPeak(float *outMag, uint32_t Fs, uint32_t N, float &max_x, float &max_y){
    max_x = 0;
    max_y = 0;
    float X,Y;
    float df=float(Fs)/float(N);
    float offset = -float(Fs)/2;
    for (uint32_t i = 0; i < N; i++){
        X = i * df + offset;
        Y = outMag[i];
        if (max_y < Y){
            max_y = Y;
            max_x = X;
        }
//        if (Y > 3 )
//            if(PRINT)
//                cout << "Max: "<< X << " " << Y << endl;
        
    }
//    if(PRINT) cout << "Max: "<< max_x << " " << max_y << endl<< endl;
    cout << "Max: "<< max_x << " " << max_y << endl;

}



////This makes the chirp if the data type is fftwf_complex
//void makeRepeatedChirp(fftwf_complex *signal, uint32_t sizeOfChirp, uint32_t numOfChirps, float Tc, float B, float Fc_start){
//    float temp0 = M_PI*Tc/sizeOfChirp;
//    float temp1 = B/sizeOfChirp;
//    float temp2 = 2*Fc_start;
//    uint32_t index = 0;
//    for (uint32_t i = 0; i < numOfChirps; i++){
//        for (uint32_t j = 0; j < sizeOfChirp; j++){
//            float temp3 = j*temp0*(j*temp1+temp2);
//            //This is the chrip
//            signal[index][REAL] = cos(temp3);
//            signal[index][IMAG] = sin(temp3);
//            index++;
//            //This is just a single complex frequency, it is just a postive freq
//    //        signal[i][REAL] = cos(M_PI * 2 * Tc*i/N * Fc_start); // real number
//    //        signal[i][IMAG] = sin(M_PI * 2 * Tc*i/N * Fc_start); //imaginary number
//            //      float t = Tc*i/N; //ignore this
//        }
//    }
//}

//Use metadata for additional control over sample function behavior
void setMetaData(lms_stream_meta_t &rx_metadata,  lms_stream_meta_t &tx_metadata){
    rx_metadata.flushPartialPacket = false; //currently has no effect in RX
    rx_metadata.waitForTimestamp = false; //currently has no effect in RX
    //In TX: send samples to HW even if packet is not completely filled (end TX burst).
    tx_metadata.flushPartialPacket = true; //do not force sending of incomplete packet
    tx_metadata.waitForTimestamp = true; //Enable synchronization to HW timestamp
}

//void mixer(fftwf_complex *rx, fftwf_complex *tx, uint32_t N){
//    for (uint32_t i = 0; i < N; i++){
////      (A - iB)(C + iD) = (AC + BD) + i(AD-BC)
//        float mixReal = rx[i][REAL] * tx[i][REAL] + rx[i][IMAG]*tx[i][IMAG];
//        float mixImag = rx[i][REAL] * tx[i][IMAG] - rx[i][IMAG]*tx[i][REAL];
//        rx[i][REAL] = mixReal;
//        rx[i][IMAG] = mixImag;
//    }
//}

void mixer(fftwf_complex *rx, fftwf_complex *tx, uint32_t sizeOfChirp, uint32_t numOfChirps){
    uint32_t index = 0;
    for (uint32_t j = 0; j < numOfChirps; j++){
        for (uint32_t i = 0; i < sizeOfChirp; i++){
    //      (A - iB)(C + iD) = (AC + BD) + i(AD-BC)
            float mixReal = rx[index][REAL] * tx[i][REAL] + rx[index][IMAG]*tx[i][IMAG];
            float mixImag = rx[index][REAL] * tx[i][IMAG] - rx[index][IMAG]*tx[i][REAL];
            rx[index][REAL] = mixReal;
            rx[index][IMAG] = mixImag;
            index++;
        }
    }
}


void normalizeSignal(fftwf_complex *rx, uint32_t N){
    float avgReal = 0;
    float avgImag = 0;
    for (uint32_t i = 0; i < N; i++){
        avgReal += rx[i][REAL];
        avgImag += rx[i][IMAG];
    }
    avgReal / N;
    avgImag / N;
    for (uint32_t i = 0; i < N; i++){
        rx[i][REAL] - avgReal;
        rx[i][IMAG] - avgImag;
    }
    
}


void setLPF(float B){
    if(LMS_SetLPFBW(device,LMS_CH_RX,CHANNEL0,B)!=0)//low pass filter
        error();
    if(LMS_SetLPFBW(device,LMS_CH_TX,CHANNEL0,B)!=0)//low pass filter
        error();
}


void calibrate(float B){
    uint32_t flags = 0;
    if(LMS_Calibrate(device,LMS_CH_RX,CHANNEL0,B,flags)!=0)
        error();
    if(LMS_Calibrate(device,LMS_CH_TX,CHANNEL0,B,flags)!=0)
        error();
}


void seconds2Samples(float startDelay_sec, float Fs, uint32_t sizeOfChirp, uint32_t numOfChirps, uint32_t &startDelayNChirps, uint32_t &startDelaySamps){
    startDelaySamps = uint32_t(startDelay_sec*Fs);
    startDelayNChirps = startDelaySamps / (sizeOfChirp * numOfChirps);
    if (startDelaySamps % (sizeOfChirp * numOfChirps) != 0)
        startDelayNChirps += 1;
    uint32_t startDelayChirps = startDelayNChirps * numOfChirps;
    startDelaySamps = startDelayChirps * sizeOfChirp;
    if(PRINT) cout << "Time actually delayed:\t" << startDelaySamps / Fs << endl;
}



void printDeviceOptions(){
    //Get number of channels
    uint32_t n;
    if ((n = LMS_GetNumChannels(device, LMS_CH_RX)) < 0)
        error();
    cout << "Number of RX channels: " << n << endl;
    if ((n = LMS_GetNumChannels(device, LMS_CH_TX)) < 0)
        error();
    cout << "Number of TX channels: " << n << endl;
    //Print Freq ranges
    lms_range_t range;
    if ((n =LMS_GetLOFrequencyRange(device,LMS_CH_RX, &range)) < 0)
        error();
    cout << "Freq Range RX:\t" << range.min <<" - "<<range.max<<"\tStep: "<<range.step<< endl;
    if ((n =LMS_GetLOFrequencyRange(device,LMS_CH_TX, &range)) < 0)
        error();
    cout << "Freq Range TX:\t" << range.min <<" - "<<range.max<<"\tStep: "<<range.step<< endl;
    
    //print antenna options
    lms_name_t antenna_list[10];//large enough list for antenna names.//Alternatively, NULL can be passed to LMS_GetAntennaList() to obtain number of antennae
    if((n=LMS_GetAntennaList(device,LMS_CH_RX,CHANNEL0,antenna_list))<0) error();
    cout<<"Available antennae RX:\n";//print available antennae names
    for(int i=0;i<n;i++)
        cout<<i<<": "<<antenna_list[i]<<endl;
    if((n=LMS_GetAntennaList(device,LMS_CH_TX,CHANNEL0,antenna_list))<0) error();
    cout<<"Available antennae TX:\n";//print available antennae names
    for(int i=0;i<n;i++)
        cout<<i<<": "<<antenna_list[i]<<endl;
    
    //Get allowed LPF bandwidth range
    if(LMS_GetLPFBWRange(device,LMS_CH_RX,&range)!=0) error();
    cout<<"RX LPF bandwitdh range: "<<range.min/1e6<<" - "<<range.max/1e6<<" MHz\n";
    if(LMS_GetLPFBWRange(device,LMS_CH_TX,&range)!=0) error();
    cout<<"TX LPF bandwitdh range: "<<range.min/1e6<<" - "<<range.max/1e6<<" MHz\n\n";

}



void printDeviceConfig(){
    //print clock frequencies
    int n;
    float_type freq;
    if ((n =LMS_GetClockFreq(device,LMS_CLOCK_REF, &freq)) < 0) error();
    cout << "LMS_CLOCK_REF:\t" << freq <<endl; //Chip reference clock.
    if ((n =LMS_GetClockFreq(device,LMS_CLOCK_CGEN, &freq)) < 0) error();
    cout << "LMS_CLOCK_CGEN:\t" << freq <<endl; //Chip reference clock.
    if ((n =LMS_GetClockFreq(device,LMS_CLOCK_SXR , &freq)) < 0) error();
    cout << "LMS_CLOCK_SXR :\t" << freq <<endl; //Chip reference clock.
    if ((n =LMS_GetClockFreq(device,LMS_CLOCK_SXT , &freq)) < 0) error();
    cout << "LMS_CLOCK_SXT:\t" << freq <<endl; //Chip reference clock.
    //print center frequency
    if(LMS_GetLOFrequency(device,LMS_CH_RX,0,&freq)!=0) error();
    cout<<"Center frequency:\t"<<freq/1e6<<" MHz\n";
    //print antenna
    lms_name_t antenna_list[10];//large enough list for antenna names.//Alternatively, NULL can be passed to LMS_GetAntennaList() to obtain number of antennae
    if((n=LMS_GetAntennaList(device,LMS_CH_RX,CHANNEL0,antenna_list))<0) error();
    if((n=LMS_GetAntenna(device,LMS_CH_RX,CHANNEL0))<0) error();
    cout<<"Automatically selected RX antenna: "<<n<<": "<<antenna_list[n]<<endl;
    if((n=LMS_GetAntennaList(device,LMS_CH_TX,CHANNEL0,antenna_list))<0) error();
    if((n=LMS_GetAntenna(device,LMS_CH_TX,CHANNEL0))<0) error();
    cout<<"Automatically selected TX antenna: "<<n<<": "<<antenna_list[n]<<endl;
    //print sample rate
    float_type rate, rf_rate;
    if(LMS_GetSampleRate(device,LMS_CH_RX,CHANNEL0,&rate,&rf_rate)!=0) error();
    cout<<"\nHost interface sample rate: "<<rate/1e6<<" MHz\nRF ADC sample rate: "<<rf_rate/1e6<<" MHz\n"<<"OVERSAMPLE_SCALER: "<<OVERSAMPLE_SCALER<<"\n\n";
    //print gain
    float_type gain;//normalized gain
    if(LMS_GetNormalizedGain(device,LMS_CH_RX,CHANNEL0,&gain)!=0) error();
    cout<<"Normalized RX Gain: "<<gain<<endl;
    uint32_t gaindB;//gain in dB
    if(LMS_GetGaindB(device,LMS_CH_RX,CHANNEL0,&gaindB)!=0) error();
    cout<<"RX Gain: "<<gaindB<<" dB"<<endl;
    if(LMS_GetNormalizedGain(device,LMS_CH_TX,CHANNEL0,&gain)!=0) error();
    cout<<"Normalized TX Gain: "<<gain<<endl;
    if(LMS_GetGaindB(device,LMS_CH_TX,CHANNEL0,&gaindB)!=0) error();
    cout<<"TX Gain: "<<gaindB<<" dB\n\n";
}

void printStreamStatus(lms_stream_t *stream, bool RxOrTx){
    lms_stream_status_t status;
    LMS_GetStreamStatus(stream, &status); //Obtain RX stream stats
    if (RxOrTx == LMS_CH_RX){
        cout << "RX FIFO Fill: " << status.fifoFilledCount << endl;
        cout << "RX FIFO Size: " << status.fifoSize << endl;
        cout << "RX 0 FIFO: " << 100 * status.fifoFilledCount / status.fifoSize << "%" << endl; //percentage of RX 0 fifo filled
        cout << "RX FIFO under: " << status.underrun << endl;
        cout << "RX FIFO over: " << status.overrun << endl;
        cout << "RX FIFO droped: " << status.droppedPackets << endl;
        cout << "RX rate: " << status.linkRate / 1e6 << " MB/s\n"; //link data rate (both channels))
        cout << "RX Timestamp: " << status.timestamp << endl;
    }
    else{
        cout << "TX FIFO Fill: " << status.fifoFilledCount << endl;
        cout << "TX FIFO Size: " << status.fifoSize << endl;
        cout << "TX 0 FIFO: " << 100 * status.fifoFilledCount / status.fifoSize << "%" << endl; //percentage of RX 0 fifo filled
        cout << "TX FIFO under: " << status.underrun << endl;
        cout << "TX FIFO over: " << status.overrun << endl;
        cout << "TX FIFO droped: " << status.droppedPackets << endl;
        cout << "TX rate: " << status.linkRate / 1e6 << " MB/s\n"; //link data rate (both channels))
        cout << "TX Timestamp: " << status.timestamp << endl;
    }
}

void checkStreamStatus(lms_stream_t *RXstream, lms_stream_t *TXstream){
    lms_stream_status_t status;
    
    LMS_GetStreamStatus(RXstream, &status); //Obtain RX stream stats
    float percentFilledRX = (float)status.fifoFilledCount / status.fifoSize;
    uint32_t underrunRX = status.underrun;
    uint32_t overrunRX = status.overrun;
    uint32_t droppedPacketsRX = status.droppedPackets;
    
    LMS_GetStreamStatus(TXstream, &status); //Obtain RX stream stats
    float percentFilledTX = (float)status.fifoFilledCount / status.fifoSize;
    uint32_t underrunTX = status.underrun;
    uint32_t overrunTX = status.overrun;
    uint32_t droppedPacketsTX = status.droppedPackets;
    
    bool flag = false;
    
    if(percentFilledRX > .90){
        if (PRINT) cout << "ERROR: RX Buffer Overflow\n";
        flag = true;
    }
    if(underrunRX){
        if (PRINT) cout << "ERROR: underrunRX\n";
        flag = true;
    }
    if(overrunRX){
        if (PRINT) cout << "ERROR: overrunRX\n";
        flag = true;
    }
    if(droppedPacketsRX){
        if (PRINT) cout << "ERROR: droppedPacketsRX\n";
    }
    
    if(percentFilledTX < .1){
        if (PRINT) cout << "ERROR: TX Buffer Empty\n";
        flag = true;
    }
    if(underrunTX){
        if (PRINT) cout << "ERROR: underrunTX\n";
        flag = true;
    }
    if(overrunTX){
        if (PRINT) cout << "ERROR: overrunTX\n";
        flag = true;
    }
    if(droppedPacketsTX){
        if (PRINT) cout << "ERROR: droppedPacketsTX\n";
    }
    if (flag){
        printStreamStatus(RXstream, LMS_CH_RX);
        printStreamStatus(TXstream, LMS_CH_TX);
        error();
    }
}




#if LMS_DATA_TYPE == LMS_DATA_TYPE_I12 || LMS_DATA_TYPE == LMS_DATA_TYPE_I16
void syncTimeStamps(float wait, int16_t *bufferRX, int16_t *txSignal, lms_stream_t &rx_stream, lms_stream_t &tx_stream,lms_stream_meta_t &rx_metadata, lms_stream_meta_t &tx_metadata, uint32_t timeoutmsRX,  uint32_t timeoutmsTX, uint32_t N, uint32_t sizeOfChirp, uint32_t numOfChirps, float Fs){
    lms_stream_status_t status; // variable to get the status
//    N = N*2;
    LMS_GetStreamStatus(&tx_stream, &status);
    uint32_t TX_fifo_fill =  status.fifoSize/4 * 3;
    //Start transmitting in the future by this much
    uint32_t startDelayNChirps, startDelaySamps;
    seconds2Samples(wait, Fs, sizeOfChirp, numOfChirps, startDelayNChirps, startDelaySamps); //finds the amount of samples to delay the transmit
    
    //Receive data from the RX stream twice - to get the correct timestamp offset
    int samplesRead = LMS_RecvStream(&rx_stream, bufferRX, N, &rx_metadata, timeoutmsRX);
    samplesRead = LMS_RecvStream(&rx_stream, bufferRX, N, &rx_metadata, timeoutmsRX);
    

    tx_metadata.timestamp = uint64_t(startDelaySamps) + rx_metadata.timestamp;//Set time stamp for future trasnmit
    
////    Actually transmit in the future
//    for (uint8_t i = 0; i < numOfChirps * 2; i++){
////    while(status.fifoFilledCount < status.fifoSize/4 * 3){
//        int sampleSent = LMS_SendStream(&tx_stream, txSignal, sizeOfChirp, &tx_metadata, timeoutmsTX);
//        tx_metadata.timestamp += sizeOfChirp;
//    }
//
    //Throw awasy receive samples -- invalid data until catches up with trasnmit
    for (uint32_t i = 0; i < startDelayNChirps; i++){
        samplesRead = LMS_RecvStream(&rx_stream, bufferRX, N, &rx_metadata, timeoutmsRX);//timeoutms);
        //fill tx fifo
        if (status.fifoFilledCount < TX_fifo_fill){
            for (uint32_t j = 0; j < numOfChirps; j++){
                LMS_SendStream(&tx_stream, txSignal, sizeOfChirp, &tx_metadata, timeoutmsTX);
                tx_metadata.timestamp += sizeOfChirp;
                LMS_GetStreamStatus(&tx_stream, &status); //Obtain TX stream stats
                if (status.fifoFilledCount > TX_fifo_fill){break;}
            }
        }
    }
    printf("Percent FIfo filled TX: %f\n",status.fifoFilledCount / float(status.fifoSize));

}
#else //if LMS_DATA_TYPE == LMS_DATA_TYPE_F32
void syncTimeStamps(float wait, fftwf_complex *bufferRX, fftwf_complex *txSignal, lms_stream_t &rx_stream, lms_stream_t &tx_stream,lms_stream_meta_t &rx_metadata, lms_stream_meta_t &tx_metadata, uint32_t timeoutmsRX,  uint32_t timeoutmsTX, uint32_t N, uint32_t sizeOfChirp, uint32_t numOfChirps, float Fs){
    lms_stream_status_t status; // variable to get the status
//    N = N*2;
    //Receive data from the RX stream twice - to get the correct timestamp offset
    int samplesRead = LMS_RecvStream(&rx_stream, bufferRX, N, &rx_metadata, timeoutmsRX);
    samplesRead = LMS_RecvStream(&rx_stream, bufferRX, N, &rx_metadata, timeoutmsRX);
    
    //Start transmitting in the future by this much
    uint32_t startDelayNChirps, startDelaySamps;
    seconds2Samples(wait, Fs, sizeOfChirp, numOfChirps, startDelayNChirps, startDelaySamps); //finds the amount of samples to delay the transmit
    tx_metadata.timestamp = uint64_t(startDelaySamps) + rx_metadata.timestamp;//Set time stamp for future trasnmit
    
    //Actually transmit in the future
    for (uint8_t i = 0; i < numOfChirps * 2; i++){
        int sampleSent = LMS_SendStream(&tx_stream, txSignal, sizeOfChirp, &tx_metadata, timeoutmsTX);
        tx_metadata.timestamp += sizeOfChirp;
    }
    //Throw awasy receive samples -- invalid data until catches up with trasnmit
    for (uint32_t i = 0; i < startDelayNChirps; i++){
        samplesRead = LMS_RecvStream(&rx_stream, bufferRX, N, &rx_metadata, timeoutmsRX);//timeoutms);
    }
}
#endif

////This makes the chirp if the data type is fftwf_complex
//void makeChirp(fftwf_complex *signal, uint32_t sizeOfChirp, float Tc, float B, float Fc_start){
//    float temp0 = M_PI*Tc/sizeOfChirp;
//    float temp1 = B/sizeOfChirp;
//    float temp2 = 2*Fc_start;
//    for (uint32_t i = 0; i < sizeOfChirp; i++){
//        float temp3 = i*temp0*(i*temp1+temp2);
//        //This is the chrip
//        signal[i][REAL] = cos(temp3);
//        signal[i][IMAG] = sin(temp3);
//        //This is just a single complex frequency, it is just a postive freq
////        signal[i][REAL] = cos(M_PI * 2 * Tc*i/N * Fc_start); // real number
////        signal[i][IMAG] = sin(M_PI * 2 * Tc*i/N * Fc_start); //imaginary number
//        //      float t = Tc*i/N; //ignore this
//    }
//}

#if SIG_PROCESS
void processData(fftwf_complex *bufferRX, fftwf_complex *txSignal, float *rxSigMag, float Fs, uint32_t N, uint32_t sizeOfChirp, uint32_t numOfChirps, float &max_x, float &max_y, float S){
    fftwf_plan p;
    fftwf_complex rxSigFFT[N];//seg fault here -- raspberry pi out of memory?
    normalizeSignal(bufferRX, N);
    //mix -- find difference in frequency of RX and TX
    mixer(bufferRX, txSignal, sizeOfChirp, numOfChirps);
    p = fftwf_plan_dft_1d(N, bufferRX, rxSigFFT, FFTW_FORWARD, FFTW_ESTIMATE);
    fftwf_execute(p);
    //Get the magnitude of Rx signal
    complex2Magnitude(rxSigMag, rxSigFFT,N);
    //Free up memory from FFT
    fftwf_destroy_plan(p);
    fftwf_cleanup();
    //fft shift the signal for plotting
    fftShift(rxSigMag,N);
    //Find the peak
    findPeak(rxSigMag,Fs,N,max_x,max_y);
    
    cout << "Range\t" << (max_x-IF_OFFSET) * c / (2 * S) << endl;
}
#endif

//if LMS_DATA_TYPE == LMS_DATA_TYPE_F32
void makeChirp(fftwf_complex *signal, uint32_t sizeOfChirp, float Tc, float B, float Fc_start){
    float temp0 = M_PI*Tc/sizeOfChirp;
    float temp1 = B/sizeOfChirp;
    float temp2 = 2*Fc_start;
    for (uint32_t i = 0; i < sizeOfChirp; i++){
        float temp3 = i*temp0*(i*temp1+temp2);
        //This is the chrip
        signal[i][REAL] = cos(temp3);
        signal[i][IMAG] = sin(temp3);
        //This is just a single complex frequency, it is just a postive freq
//        signal[i][REAL] = cos(M_PI * 2 * Tc*i/N * Fc_start); // real number
//        signal[i][IMAG] = sin(M_PI * 2 * Tc*i/N * Fc_start); //imaginary number
        //      float t = Tc*i/N; //ignore this
    }
}

void makeChirp(int16_t *signal, uint32_t sizeOfChirp, float Tc, float B, float Fc_start){
    float temp0 = M_PI*Tc/sizeOfChirp;
    float temp1 = B/sizeOfChirp;
    float temp2 = 2*Fc_start;
    for (uint32_t i = 0; i < sizeOfChirp; i++){
        float temp3 = i*temp0*(i*temp1+temp2);
        #if LMS_DATA_TYPE == LMS_DATA_TYPE_I12
        signal[2*i] = int16_t(INT12_MAX * cos(temp3));
        signal[2*i+1] = int16_t(INT12_MAX * sin(temp3));
        #else //if LMS_DATA_TYPE == LMS_DATA_TYPE_I16
//        signal[2*i] = int16_t(INT12_MAX * cos(temp3)) << 4;
//        signal[2*i+1] = int16_t(INT12_MAX * sin(temp3)) << 4;
        signal[2*i] = int16_t(INT16_MAX * cos(temp3));
        signal[2*i+1] = int16_t(INT16_MAX * sin(temp3));
        #endif
    }
}




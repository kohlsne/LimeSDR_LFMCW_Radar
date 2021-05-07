/*
 * A LFMCW radar system with the LimeSDR
 * Functions declarations found in LFMCW.h
 * Function definitions found in LFMCW.cpp
 */

#include "LFMCW.h"

using namespace std;

lms_device_t* device = NULL;

int main(int argc, char**argv){
    initDevice();//find LimeSDR
    if(PRINT) printDeviceOptions();
    
//    Parameters
    float Fs = 36.0e6;
    float Tc = 0.1e-3;//Time in seconds the signal/ chirp
    float Fc_start = 0e3; //starting frequency of chirp
    float B = 16.0e6;
    float centerFreqRX =694e6;
    float centerFreqTX =694e6;
    float gainRX = 0.2;
    float gainTX = 0.4;
    float S = B / Tc; //slope of chirp
    uint32_t numOfChirps = 100; //number of chirps to be received at a time
    uint32_t repeat = 10;
    float delayStart = 3.0;

    uint32_t sizeOfChirp =  uint32_t(Fs * Tc); //number of samples in a chirp = samples sent
    uint32_t N = sizeOfChirp * numOfChirps; //number of samples to be received at a time
    //Timeouts for receive and transmits
    uint32_t timeoutmsRX = uint32_t(1e3 * Tc * numOfChirps +0.1);
    uint32_t timeoutmsTX = uint32_t(Tc * 1e3);
    
    if (PRINT) cout<<"Fs:\t\t"<<Fs<<"\nTc:\t\t"<<Tc<<"\nFc_start:\t"<<Fc_start<<"\nB:\t\t"<<B<<"\nCarrierRX:\t"<<centerFreqRX<<"\nCarrierTX:\t"<<centerFreqTX<<"\ngainRX:\t\t"<<gainRX<<"\ngainTX:\t\t"<<gainTX<<"\nsizeChirp:\t"<<sizeOfChirp<<"\nnumOfChirps:\t"<<numOfChirps<<"\nN:\t\t"<<N<<"\nS:\t\t"<<S<<"\nChripFreq:\t"<<1/Tc<<"\nChirpFreqRange:\t"<<1/Tc/(1*S)*c<<"\nbinSize(Hz):\t"<<float(Fs)/N<<"\nBin Range(m):\t"<<float(Fs)/N *c/(1*S)<<"\n1 IF(m) =\t"<<c/(1*S)<<"\nCable(m):\t"<<146<<"\nCable(Hz):\t"<<146*S/c<<endl<<endl;
    //To improve accuracy of IF 1 signal -> increase S = increase B decrease Tc
    //To minimize frequency bin size -> Minimize Fs, Maximize num of chirps
    
    //Setup hardware
    enableChannels();//LMS_EnableChannel just channel 0
    setCenterFreq(centerFreqRX, centerFreqTX);//LMS_SetLOFrequency both Rx and Tx
    setSampleRate(Fs);// LMS_SetSampleRate
    setGain(gainRX,gainTX);//LMS_SetNormalizedGain both RX and Tx
    setLPF(2*B); //LMS_SetLPFBW both Rx and Tx
//    calibrate(2*B); //LMS_Calibrate both Rx and TX
    if(PRINT) printDeviceConfig();
    
    //Streaming Setup
    lms_stream_t rx_stream, tx_stream;
    setupSteams(rx_stream, tx_stream);
    //Initialize data buffers
    
    //fftwf_complex bufferRX[N] = float bufferRX[N][2] = float bufferRX[2*N]
#if LMS_DATA_TYPE == LMS_DATA_TYPE_F32
    fftwf_complex bufferRX[N], txSignal[sizeOfChirp];//Note the buffers are different sizes
#elif LMS_DATA_TYPE == LMS_DATA_TYPE_I12 || LMS_DATA_TYPE == LMS_DATA_TYPE_I16
    int16_t bufferRX[N*2], txSignal[sizeOfChirp*2];
#endif
    //txSignal trasnmitts one chirps at a time
    //bufferRX receives numOfChirps chrips at a time
    makeChirp(txSignal,sizeOfChirp,Tc,B,Fc_start); // make signal for transmitting
    
    
#if SAVE
    FILE *pFileRX;
    pFileRX = fopen (argv[1],"wb");
    if (pFileRX==NULL){
        cout << "file error\n";
        return(1);
    }
//    FILE *pFileTX;
//    pFileTX = fopen ("TX.bin","wb");
//    if (pFileTX==NULL){
//        cout << "file error\n";
//        return(1);
//    }
//    #if LMS_DATA_TYPE == LMS_DATA_TYPE_F32
//        #if SIG_PROCESS
//            File *pFileMix;
//            pFileMix = fopen ("Mix.bin","wb");
//            if (pFileMix==NULL){
//                cout << "file error\n";
//                return(1);
//            }
//        #endif
//        fwrite(txSignal,sizeof(fftwf_complex),sizeOfChirp,pFileTX); //LMS_DATA_TYPE_F32
//    #elif LMS_DATA_TYPE == LMS_DATA_TYPE_I12 || LMS_DATA_TYPE == LMS_DATA_TYPE_I16
//        fwrite(txSignal,sizeof(int16_t),sizeOfChirp*2,pFileTX); //LMS_DATA_TYPE_I12/16
//    #endif
//        //    cout <<"Bytes TX: "<<sizeof(txSignal) << endl;
//    fclose (pFileTX);
#endif

    
    
    //Start streaming
    LMS_StartStream(&rx_stream);
    LMS_StartStream(&tx_stream);
    
    lms_stream_meta_t rx_metadata ,tx_metadata;
    setMetaData(rx_metadata, tx_metadata); //tx_metadata.waitForTimestamp = true; everything else is false
    
    syncTimeStamps(delayStart,bufferRX, txSignal, rx_stream, tx_stream, rx_metadata, tx_metadata, timeoutmsRX, timeoutmsTX, N, sizeOfChirp, numOfChirps, Fs);
    //Now receiving valid data
    //Things are lined up here and the data is valid as long as the Rx fifo doesnt overflow
    float max_x, max_y; //used for finding stronges frequency in FFT
    float rxSigMag[N]; //Holds the magnitude of the complex signal of the receive
    lms_stream_status_t status; // variable to get the status
    LMS_GetStreamStatus(&tx_stream, &status);
    uint32_t TX_fifo_fill =  status.fifoSize/4 * 3;
    for (uint32_t i = 0; i < repeat; i++){
        cout<<i<<endl;
        //Fill up Tx buffer until it is at 3/4 half full
        LMS_GetStreamStatus(&tx_stream, &status); //Obtain TX stream stats
        while(status.fifoFilledCount < TX_fifo_fill){
            int sampleSent = LMS_SendStream(&tx_stream, txSignal, sizeOfChirp, &tx_metadata, timeoutmsTX);
            tx_metadata.timestamp += sizeOfChirp;
            LMS_GetStreamStatus(&tx_stream, &status); //Obtain TX stream stats
        }
        //Receive
        int samplesRead = LMS_RecvStream(&rx_stream, bufferRX, N, &rx_metadata, timeoutmsRX);//timeoutms);
        
        
#if SAVE // can make it so it saves with the 12 bit format and not 16
    #if LMS_DATA_TYPE == LMS_DATA_TYPE_I12 || LMS_DATA_TYPE == LMS_DATA_TYPE_I16
        fwrite(bufferRX,sizeof(int16_t),N*2,pFileRX); // change save
    #endif
    #if LMS_DATA_TYPE == LMS_DATA_TYPE_F32
        fwrite(bufferRX,sizeof(fftwf_complex),N,pFileRX); // change save
        #if SIG_PROCESS
            mixer(bufferRX, txSignal, sizeOfChirp, numOfChirps);
            fwrite(bufferRX,sizeof(fftwf_complex),N,pFileMix); //this is the fft I believe
        #endif
    #endif
#endif

        
        
        
        //Signal Processing
//#if SIG_PROCESS
//        if (PRINT) cout <<endl << i << endl;
//        processData(bufferRX, txSignal, rxSigMag, Fs, N, sizeOfChirp, numOfChirps, max_x, max_y, S);
        
//#endif
        if (PRINT) printStreamStatus(&rx_stream, LMS_CH_RX);
        cout << "RX Timestamp: " << rx_metadata.timestamp << endl;
        if (PRINT) printStreamStatus(&tx_stream, LMS_CH_TX);
        checkStreamStatus(&rx_stream,&tx_stream);
        if (PRINT) cout<<endl;
    }
#if SAVE // can make it so it saves with the 12 bit format and not 16
    #if SIG_PROCESS && LMS_DATA_TYPE == LMS_DATA_TYPE_I32
        fclose(pFileMix);
    #endif
    fclose (pFileRX);
#endif
    //Close everything
    LMS_StopStream(&rx_stream); //stream is stopped but can be started again with LMS_StartStream()
    LMS_StopStream(&tx_stream);
    LMS_DestroyStream(device, &rx_stream); //stream is deallocated and can no longer be used
    LMS_DestroyStream(device, &tx_stream);
    closeDevice();
    
//#if SAVE // can make it so it saves with the 12 bit format and not 16
//    fwrite(rxSigMag,sizeof(float),N,pFileMix); //this is the fft I believe
//    fclose(pFileMix);
//#endif

    
    
//#if PLOT
//    plotDFT(rxSigMag,Fs,N,max_y,S);

    
//    //Plot the receive
////    N = sizeOfChirp;
//    fftwf_plan p;
//    fftwf_complex rxSigFFT[N];//seg fault here -- raspberry pi out of memory?
////    mixer(bufferRX_complex, txSignal, sizeOfChirp, numOfChirps);
//    p = fftwf_plan_dft_1d(N, bufferRX, rxSigFFT, FFTW_FORWARD, FFTW_ESTIMATE);
//    fftwf_execute(p);
//    //Get the magnitude of Rx signal
//    complex2Magnitude(rxSigMag, rxSigFFT,N);
//    //Free up memory from FFT
//    fftwf_destroy_plan(p);
//    fftwf_cleanup();
//    //fft shift the signal for plotting
//    fftShift(rxSigMag,N);
//    plotDFT(rxSigMag,Fs,N,5,S); // plots just the receive signal


//#endif
    cout<<"Success\n";
    return 0;
}

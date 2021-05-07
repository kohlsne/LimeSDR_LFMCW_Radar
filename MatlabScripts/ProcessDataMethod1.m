clear all; close all;
%This script used the feed-though nulling techinque estimating the
%resulting mixed freqeucny from bleed-though signal and substracting it from the mixed signal

% It is currently configuered to work with a coaxial cable and not
% antennas, the differnces is the constant c and in equations calculating
% the range. line 19, 25, 183
fileName = './DataCoaxialCable/t0.bin';
DFT_LenthInChirps = 4;
numOfChirps = 10;

B = 16.0e6; % Bandwidth of chirp
Fs = 36.0e6;       % Sample Frequecy (samples per second), the bigger = more accurate
Fc_start = 0;        % Start Frequency of chirp
Tc = 0.1e-3;                 % The length of the chirp (seconds)

Fc_end = Fc_start + B;  % End Frequency of chirp
S = B / Tc;             % Slope of FT graph of chirp
c = 299792458 / 3 * 2;          % Speed of light (m/sec)
sizeOfChirp = cast(Fs * Tc,'int64');
N = cast(sizeOfChirp * numOfChirps,'int64');
df = Fs/sizeOfChirp;
Freq_range = -Fs/2:df:Fs/2-df;
DFT_Length = sizeOfChirp * DFT_LenthInChirps;
RangeBin = cast(Fs/DFT_Length,'double') * c / (S);
figNum = 1;

% Make chirp in matlab
dt = 1/Fs;              % Sample Period (seconds per sample)
t = (0:dt:Tc-dt)';      % Time array of chirp seconds
TX = exp(2*pi*1i*(S./2.* t.^2 + Fc_start.*t));

% Read complex file RX
fileID = fopen(fileName);
RX = fread(fileID, [2 N], '*int16');
fclose(fileID);
RX = cast(RX,'double'); RX = RX ./ cast(0x7FF,'double');
%Substract Mean
% RX(1, :) = RX(1, :) - mean(RX(1, :)); RX(2, :) = RX(2, :) - mean(RX(2, :));
RX = complex(RX(1, :), RX(2,:)); RX = RX(:); %Convert to complex type

figure(figNum);figNum = figNum+1;
plot(t,real(RX(1:sizeOfChirp)));
xlabel('time'); ylabel('amp'); title('Method 2: RX');

%Average the chirps together
RX_Avg = zeros(sizeOfChirp,1);
for i = 1:numOfChirps
    k = i-1;
    RX_Avg = RX_Avg + RX(k*sizeOfChirp+1:i*sizeOfChirp);
end
RX = (RX_Avg/numOfChirps);
clear RX_Avg;

%Normalize the chirps
Max_real = max(real(RX(201:400)));
Max_imag = max(imag(RX(201:400)));
real = real(RX)./Max_real;
imag = imag(RX)./Max_imag;
RX = complex(real, imag); RX = RX(:); %Convert to complex type
clear real imag;

figure(figNum);figNum = figNum+1;
plot(t,real(RX(1:sizeOfChirp)));
xlabel('time'); ylabel('amp'); title('Method 2: RX');

Mix = zeros(DFT_Length,1);
Mix(1:sizeOfChirp) = TX .* conj(RX);
Z = 10*log10(abs(fftshift(fft(Mix))));
[Power_Normal,indmax] = max(Z);
df = Fs/DFT_Length;
Freq_range = -Fs/2:df:Fs/2-df;
IF_Bleed = Freq_range(indmax);
IF_Bleed = cast(IF_Bleed,'double');
Start_IF_Sample = cast(floor(Fs/S*IF_Bleed+1),'int64');

%Plot Bleedthough Signal
Freq_range = (-Fs/2:df:Fs/2-df); 
figure(figNum);figNum = figNum+1;
plot(Freq_range,Z); xlabel('Hz'); ylabel('dB'); title('Method 2: Mixed Signal');
%Plot Bleedthough signal - zeroing our useless info
Mix(1:Start_IF_Sample) = 0;
Z = 10*log10(abs(fftshift(fft(Mix))));
figure(figNum);figNum = figNum+1;
plot(Freq_range,Z); xlabel('Hz'); ylabel('dB'); title('Method 2: Mixed Signal');

TX2 = -circshift(TX,Start_IF_Sample) ;

figure(figNum);figNum = figNum+1;
plot(t,real(RX)); hold on;
plot(t,real(TX2));
xlabel('Time (s)'); ylabel('Amplitude'); title('Method 2: The Receive Signal And The Synthesized Receive Signal');
legend("Receive Signal", "Synthesized Receive Signal");
axis([1e-5 2e-5 -1.1 1.1]); grid on;
%saveas(gcf,'Null2_f_zi.png');

figure(figNum);figNum = figNum+1;
plot(t,imag(RX)); hold on;
plot(t,imag(TX2));
xlabel('time'); ylabel('amp'); title('Method 2: imag-TimeDelayFix');
%Shift Find Phi

phi = RX ./ TX2;
phi1= mean(phi(201:400));%smooth version of phi
phase = atan2(imag(phi1),real(phi1));
TX2 = TX2 * exp(1i*phase);
% TX2 = TX2 .* phi1;
figure(figNum);figNum = figNum+1;
plot(t,real(RX)); hold on;
plot(t,real(TX2));
xlabel('Time (s)'); ylabel('Amplitude'); title('Method 2: The Receive Signal And The Synthesized Receive Signal');
legend("Receive Signal", "Synthesized Receive Signal");
%saveas(gcf,'Null2_fp_zo.png');
axis([1e-5 2e-5 -1.1 1.1]); grid on;
%saveas(gcf,'Null2_fp_zi.png');

%Find Diff of Magnitude Average
Ratio = RX ./ TX2;
windowSize = 100; 
b = (1/windowSize)*ones(1,windowSize);
a = 1;
y = filter(b,a,Ratio);
%Plot Filter
figure(figNum);figNum = figNum+1;
plot(t,y); hold on;
xlabel('Time (s)'); ylabel('A_0(t)'); title('Method 2: Amplitude Moving Average Filter');
%saveas(gcf,'Null2_AvgFilter.png');

TX2 = TX2 .* y;

figure(figNum);figNum = figNum+1;
plot(t,real(RX)); hold on;
plot(t,real(TX2));
xlabel('Time (s)'); ylabel('Amplitude'); title('Method 2: The Receive Signal And The Synthesized Receive Signal');
legend("Receive Signal", "Synthesized Receive Signal");
axis([1e-5 2e-5 -1.1 1.1]); grid on;
%saveas(gcf,'Null2_fpa_zo.png');

RX2 = RX - TX2;
figure(figNum);figNum = figNum+1;
plot(t,real(RX2));
xlabel('Time (s)'); ylabel('Amplitude');  title('Method 2: The Mixed Signal With Feed-through Nulling');
ylim([-1.1 1.1]); grid on;
%saveas(gcf,'Null2_fpad_zo.png');

figure(figNum);figNum = figNum+1;
plot(t,imag(RX2));
xlabel('Time (s)'); ylabel('Amplitude');  title('Method 2: The Mixed Signal With Feed-through Nulling');
ylim([-1.1 1.1]); grid on;

RX2(1:150) = 0;
IND = find(abs(real(RX2)) > 0.15 | abs(imag(RX2)) > 0.15);
RX2(IND) = 0;
RX2 = RX2 .* hamming(sizeOfChirp);

figure(figNum);figNum = figNum+1;
plot(t,RX2); hold on;
xlabel('Time (s)'); ylabel('Amplitude');  title('Method 2: The Mixed Signal With Feed-through Nulling');
% axis([1e-5 2e-5 -1.1 1.1]); grid on;
ylim([-1.1 1.1]); grid on;
%saveas(gcf,'Null2_fpadh_zo.png');

figure(figNum);figNum = figNum+1;
plot(t,imag(RX2)); hold on;
xlabel('Time (s)'); ylabel('Amplitude');  title('Method 2: The Mixed Signal With Feed-through Nulling');
% axis([1e-5 2e-5 -1.1 1.1]); grid on;
ylim([-1.1 1.1]); grid on;


Mix = zeros(DFT_Length,1);
Mix(1:sizeOfChirp) = TX .* conj(RX2);
Z = 10*log10(abs(fftshift(fft(Mix))));
df = Fs/DFT_Length;
Freq_range = -Fs/2:df:Fs/2-df;
figure(figNum);figNum = figNum+1;
plot(Freq_range,Z); xlabel('Hz'); ylabel('dB');

Mix(1:Start_IF_Sample) = 0;
Z = fftshift(fft(Mix));
Z = 10*log10(abs(Z));
Power_Null = Z(indmax);
[Power_Peak,indmax] = max(Z);
Freq_range = ((-Fs/2:df:Fs/2-df) - IF_Bleed) *c/S;%/2;
figure(figNum);figNum = figNum+1;
plot(Freq_range,Z); xlabel('Range (meters)'); ylabel('dB');
title('Method 2: Range Plot Of The Mixed Signal With Feed-through Nulling');
axis([0 300 0 20]); grid on;
%saveas(gcf,'Null2_dft.png');

Power_Diff = Power_Normal - Power_Null;
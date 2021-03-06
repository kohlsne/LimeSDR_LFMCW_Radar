clear all; close all;
%This script used the feed-though nulling techinque estimating the
%bleed-though signal and substracting it from the receive signal

% It is currently configuered to work with a coaxial cable and not
% antennas, the differnces is the constant c and in equations calculating
% the range. line 19, 25, 184

fileName = './DataCoaxialCable/t0.bin';
DFT_LenthInChirps = 4;
numOfChirps = 10;

B = 16.0e6; % Bandwidth of chirp
Fs = 36.0e6;       % Sample Frequecy (samples per second), the bigger = more accurate
Fc_start = 0;        % Start Frequency of chirp
Tc = 0.1e-3;                 % The length of the chirp (seconds)

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
%Chirp Function & Figure
TX = exp(2*pi*1i*(S./2.* t.^2 + Fc_start.*t));

% Read complex file RX
fileID = fopen(fileName);
RX = fread(fileID, [2 N], '*int16');
fclose(fileID);
RX = cast(RX,'double'); RX = RX ./ cast(0x7FF,'double');
%Substract Mean
RX(1, :) = RX(1, :) - mean(RX(1, :)); RX(2, :) = RX(2, :) - mean(RX(2, :));
RX = complex(RX(1, :), RX(2,:)); RX = RX(:); %Convert to complex type

%Average the chirps together
RX_Avg = zeros(sizeOfChirp,1);
for i = 1:numOfChirps
k = i-1;
RX_Avg = RX_Avg + RX(k*sizeOfChirp+1:i*sizeOfChirp);
end
RX = (RX_Avg/numOfChirps);
clear RX_Avg;

%Normalize the chirps
Max_real = max(real(RX));
Max_imag = max(imag(RX));
real = real(RX)./Max_real;
imag = imag(RX)./Max_imag;
RX = complex(real, imag); RX = RX(:); %Convert to complex type
clear real imag;

%Mix
Mix = zeros(DFT_Length,1);
Mix(1:sizeOfChirp) = TX .* conj(RX);

Z = 10*log10(abs(fftshift(fft(Mix))));
[Power_Normal,indmax] = max(Z);
df = Fs/DFT_Length;
Freq_range = -Fs/2:df:Fs/2-df;
IF_Bleed = Freq_range(indmax);
IF_Bleed = cast(IF_Bleed,'double');
Start_IF_Sample = cast(floor(Fs/S*IF_Bleed+1),'int64');

Freq_range = ((-Fs/2:df:Fs/2-df) - IF_Bleed) *c/S;
figure(figNum);figNum = figNum+1; 
plot(Freq_range,Z); xlabel('Range (meters)'); ylabel('dB'); title('Range Plot Of The Mixed Signal');
axis([0 300 0 35]); grid on;
%saveas(gcf,'Null_No_Mixed_Range.png');

Freq_range = -Fs/2:df:Fs/2-df;

%Plot sythetic signal
IF_FPGA_Signal = exp(1i*(2*pi*IF_Bleed*t));
figure(figNum);figNum = figNum+1;
plot(t,real(Mix(1:sizeOfChirp))); hold on;
plot(t,real(IF_FPGA_Signal));
xlabel('Time (s)'); ylabel('Amplitude'); title('Method 1: The Mixed Signal And The Synthesized Mixed Signal');
legend("Mixed Signal", "Synthesized $f_{FPGA}(t)$",'interpreter', 'latex');
axis([1e-5 2e-5 -1.1 1.1]);
%saveas(gcf,'Null1_f_zi.png');

%Find the phase of the signal
Mix = Mix(1:sizeOfChirp);
phi = atan2(imag(Mix),real(Mix)) - 2 * pi * IF_Bleed* t;
k = 1;
while size(k)~=0
k = find(phi<0);
phi(phi<0) = phi(phi<0) + 2*pi;
end
phi = mod(phi,2*pi);
phi = mean(phi(100:400));
%Make Signal to cancel out
IF_FPGA_Signal = exp(1i*(2*pi*IF_Bleed*t + phi));

figure(figNum);figNum = figNum+1;
plot(t,real(Mix)); hold on;
plot(t,real(IF_FPGA_Signal));
xlabel('Time (s)'); ylabel('Amplitude'); title('Method 1: The Mixed Signal And The Synthesized Mixed Signal');
legend("Mixed Signal", "Synthesized $f_{FPGA}(t)$",'interpreter', 'latex');
%saveas(gcf,'Null1_fp_zo.png');
axis([1e-5 2e-5 -1.1 1.1]); 
%saveas(gcf,'Null1_fp_zi.png');

% figure(figNum);figNum = figNum+1;
% plot(t,imag(Mix)); hold on;
% plot(t,imag(IF_FPGA_Signal));
% xlabel('time'); ylabel('amp'); title('Method 1: Imag');

%Make moving filter
Ratio = Mix ./ IF_FPGA_Signal;
windowSize = 100; 
b = (1/windowSize)*ones(1,windowSize);
a = 1;
y = filter(b,a,Ratio);
%Plot Filter
figure(figNum);figNum = figNum+1;
plot(t,y); hold on;
xlabel('Time (s)'); ylabel('A_0(t)'); title('Method 1: Amplitude Moving Average Filter');
%saveas(gcf,'Null1_AvgFilter.png');
%apply moving filter
IF_FPGA_Signal = IF_FPGA_Signal .* y;

figure(figNum);figNum = figNum+1;
plot(t,real(Mix)); hold on;
plot(t,real(IF_FPGA_Signal));
xlabel('Time (s)'); ylabel('Amplitude'); title('Method 1: The Mixed Signal And The Synthesized Mixed Signal');
legend("Mixed Signal", "Synthesized $f_{FPGA}(t)$",'interpreter', 'latex');
%saveas(gcf,'Null1_fpa_zo.png');

figure(figNum);figNum = figNum+1;
plot(t,imag(Mix)); hold on;
plot(t,imag(IF_FPGA_Signal));
xlabel('time'); ylabel('amp'); title('Method 1: Imag')

Mix_New = zeros(DFT_Length,1);
Mix_New(1:sizeOfChirp) = Mix - IF_FPGA_Signal;
Mix_New(1:Start_IF_Sample) = 0;

figure(figNum);figNum = figNum+1;
plot(t,real(Mix_New(1:sizeOfChirp)));
xlabel('Time (s)'); ylabel('Amplitude'); title('Method 1: The Mixed Signal With Feed-through Nulling');
ylim([-1.1 1.1]);
%saveas(gcf,'Null1_fpad_zo.png');


figure(figNum);figNum = figNum+1;
plot(t,imag(Mix_New(1:sizeOfChirp)));
xlabel('Time (s)'); ylabel('Amplitude'); title('Method 1: The Mixed Signal With Feed-through Nulling');
ylim([-1.1 1.1]);

Mix_New(1:150) = 0;
IND = find(abs(real(Mix_New)) > 0.15 | abs(imag(Mix_New)) > 0.15);
Mix_New(IND) = 0;
Mix_New(1:sizeOfChirp) = Mix_New(1:sizeOfChirp) .* hamming(sizeOfChirp);

figure(figNum);figNum = figNum+1;
plot(t,real(Mix_New(1:sizeOfChirp)));
xlabel('Time (s)'); ylabel('Amplitude'); title('Method 1: The Mixed Signal With Feed-through Nulling');
ylim([-1.1 1.1]);
%saveas(gcf,'Null1_fpadh_zo.png');

figure(figNum);figNum = figNum+1;
plot(t,imag(Mix_New(1:sizeOfChirp)));
xlabel('Time (s)'); ylabel('Amplitude'); title('Method 1: The Mixed Signal With Feed-through Nulling');
ylim([-1.1 1.1]);

%Plot Mixed Signal with feedthrough nulling
Z = 10*log10(abs(fftshift(fft(Mix_New))));
% [Power_Null,indmax] = max(Z);
Power_Null = Z(indmax);
[Power_Peak,indmax] = max(Z);
figure(figNum);figNum = figNum+1; grid on;
plot(Freq_range,Z); xlabel('Hz'); ylabel('dB'); title('Method 1: Mix Nulling Applied');


Freq_range = ((-Fs/2:df:Fs/2-df) - IF_Bleed) *c/S;
figure(figNum);figNum = figNum+1; 
plot(Freq_range,Z); xlabel('Range (meters)'); ylabel('dB'); title('Method 1: Range Plot Of The Mixed Signal With Feed-through Nulling');
axis([0 300 0 20]); grid on;
%saveas(gcf,'Null1_dft.png');

Power_Diff = Power_Normal - Power_Null;
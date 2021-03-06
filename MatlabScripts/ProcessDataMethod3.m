clear all; close all;
%This script used the feed-though nulling techinque of capturing the signal
%without the target and then capturing the signal with the target

% It is currently configuered to work with a coaxial cable and not
% antennas, the differnces is the constant c and in equations calculating
% the range. line 20, 27, 146

fileName1 = './DataCoaxialCable/t1.bin';
fileName2 = './DataCoaxialCable/s2.bin'
DFT_LenthInChirps = 4;
numOfChirps = 10;

B = 16.0e6; % Bandwidth of chirp
Fs = 36.0e6;% Sample Frequecy (samples per second), the bigger = more accurate
Fc_start = 0; % Start Frequency of chirp
Tc = 0.1e-3; % The length of the chirp (seconds)

S = B / Tc;  % Slope of FT graph of chirp
c = 299792458 /3 * 2; % Speed of light in a wire(m/sec)
% c = 299792456; % Speed of light in free space(m/sec)
sizeOfChirp = cast(Fs * Tc,'int64'); %number of samples in in a singular chirp
N = cast(sizeOfChirp * numOfChirps,'int64'); %number of samples in mulitple chirps
df = Fs/sizeOfChirp;
Freq_range = -Fs/2:df:Fs/2-df;
DFT_Length = sizeOfChirp * DFT_LenthInChirps;
RangeResolution= cast(Fs/DFT_Length,'double') * c / (S);%/2;
figNum = 1;

% Make chirp in matlab
dt = 1/Fs;              % Sample Period (seconds per sample)
t = (0:dt:Tc-dt)';      % Time array of chirp seconds
TX = exp(2*pi*1i*(S./2.* t.^2 + Fc_start.*t));

% Read complex file RX
fileID = fopen(fileName1);
RX = fread(fileID, [2 N], '*int16');
fclose(fileID);
RX = cast(RX,'double'); RX = RX ./ cast(0x7FF,'double');;
RX = complex(RX(1, :), RX(2,:)); RX = RX(:); %Convert to complex type
RX_1 = RX(1:sizeOfChirp);

Mix = zeros(DFT_Length,1);
Mix(1:sizeOfChirp) = TX .* conj(RX(1:sizeOfChirp));
Mix_test = Mix;
Mix_test(1:sizeOfChirp) = Mix_test(1:sizeOfChirp) .* hamming(sizeOfChirp);
Z = 10*log10(abs(fftshift(fft(Mix))));
[Power_Normal,indmax] = max(Z);
df = Fs/DFT_Length;
Freq_range = -Fs/2:df:Fs/2-df;
IF_Bleed = Freq_range(indmax);
IF_Bleed = cast(IF_Bleed,'double');
Start_IF_Sample = cast(floor(Fs/S*IF_Bleed+1),'int64');
df = Fs/sizeOfChirp;
figure(figNum);figNum = figNum+1;
plot(Freq_range,Z); xlabel('Hz'); ylabel('dB'); title('Method 3: Mix ');


fileID = fopen(fileName2);
ZX = fread(fileID, [2 N], '*int16');
fclose(fileID);
ZX = cast(ZX,'double'); ZX = ZX ./ cast(0x7FF,'double');
%Substract Mean
% RX(1, :) = RX(1, :) - mean(RX(1, :)); RX(2, :) = RX(2, :) - mean(RX(2, :));
ZX = complex(ZX(1, :), ZX(2,:)); ZX = ZX(:); %Convert to complex type
ZX_1 = ZX(1:sizeOfChirp);


Ratio = RX ./ ZX;
windowSize = 100; 
b = (1/windowSize)*ones(1,windowSize);
a = 1;
y = filter(b,a,Ratio);
ZX = ZX .* y;
ZX_1 = ZX(1:sizeOfChirp);

t = (0:dt:Tc-dt)';      % Time array of chirp seconds
figure(figNum);figNum = figNum+1;
plot(t,real(RX_1)); hold on;
plot(t,real(ZX_1));
xlabel('time'); ylabel('amp'); title('Method 3: Real');

figure(figNum);figNum = figNum+1;
plot(t,imag(RX_1)); hold on;
plot(t,imag(ZX_1));
xlabel('time'); ylabel('amp'); title('Method 3: Imag');

RX = RX - ZX;
RX_1 = RX(1:sizeOfChirp);

figure(figNum);figNum = figNum+1;
plot(t,real(RX_1));
xlabel('time'); ylabel('amp'); title('Method 3: Real');

figure(figNum);figNum = figNum+1;
plot(t,imag(RX_1));
xlabel('time'); ylabel('amp'); title('Method 3: Imag');

%Average the chirps together
RX_Avg = zeros(sizeOfChirp,1);
for i = 1:numOfChirps
    k = i-1;
    RX_Avg = RX_Avg + RX(k*sizeOfChirp+1:i*sizeOfChirp);
end
RX = (RX_Avg/numOfChirps);
clear RX_Avg;

%Get rid of mean
meanR = mean(real(RX));
meanI = mean(imag(RX));
R_sample = real(RX) - meanR;
I_sample = imag(RX) - meanI;
RX = complex(R_sample, I_sample); RX = RX(:); %Convert to complex type

RX(1:150) = 0;
IND = find(abs(real(RX)) > 0.15 | abs(imag(RX)) > 0.15);
RX(IND) = 0;
RX = RX .* hamming(sizeOfChirp);

figure(figNum);figNum = figNum+1;
plot(t,real(RX));
xlabel('time'); ylabel('amp'); title('Method 3: Real Fix');

figure(figNum);figNum = figNum+1;
plot(t,imag(RX));
xlabel('time'); ylabel('amp'); title('Method 3: Imag Fix');

Z = 10*log10(abs(fftshift(fft(RX))));
Freq_range = (-Fs/2:df:Fs/2-df);
figure(figNum);figNum = figNum+1;
plot(Freq_range,Z); xlabel('Hz'); ylabel('dB'); title('Method 3: RX');

Mix = zeros(DFT_Length,1);
Mix(1:sizeOfChirp) = TX .* conj(RX);
Z = 10*log10(abs(fftshift(fft(Mix))));
Power_Null = Z(indmax);
[Power_Peak,indmax2] = max(Z);
df = Fs/DFT_Length;
Freq_range = -Fs/2:df:Fs/2-df;
IF_Bleed = Freq_range(indmax);
IF_Bleed = cast(IF_Bleed,'double');
Start_IF_Sample = cast(floor(Fs/S*IF_Bleed+1),'int64');
figure(figNum);figNum = figNum+1;
plot(Freq_range,Z); xlabel('Hz'); ylabel('dB'); title('Method 3: Mix ');

Freq_range = ((-Fs/2:df:Fs/2-df) - IF_Bleed) *c/S;%/2;
figure(figNum);figNum = figNum+1;
plot(Freq_range,Z);  xlabel('Range (meters)'); ylabel('dB');
title('Method 3: Range Plot Of The Mixed Signal With Feed-through Nulling');
axis([0 300 0 20]); grid on;
% saveas(gcf,'Null3_dft.png');

Power_Diff = Power_Normal - Power_Null;
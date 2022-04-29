pkg load signal;
pkg load control;
clear all;
Fs = 8000;
Ts=1/Fs;
%Frecuencias del SI y NO
[si, Fs]=audioread('si.wav');
F1 = fft(si);
L =length(si);

P2 = abs(F1/L);
P1 = P2(1:(L/2+1)); %se toma la mitad ya que es simetria par
P1(2:end-1) = P1(2:end-1); %se multiplica por 2 para tener su amplitud original

[no, Fs]=audioread('no2.wav');
F2 = fft(no);
L2 =length(no);

P3 = abs(F2/L2);
P4 = P3(1:(L2/2+1)); %se toma la mitad ya que es simetria par
P4(2:end-1) = P4(2:end-1); %se multiplica por 2 para tener su amplitud original

figure(1)
f = Fs*(0:L/2)/L;
f2 = Fs*(0:L2/2)/L2;
subplot(2,1,1)
plot(f,P1),grid;
title('Espectro de magnitud (SI)');
ylabel('Abs');
xlabel('frecuency (Hz)');
subplot(2,1,2)
plot(f2,P4),grid;
title('Espectro de magnitud (NO)');
ylabel('Abs');
xlabel('frecuency (Hz)');

%Filtro resonador
z=tf('z',Ts)
r = 0.98;
Fr = 523;
W0 = Fr*pi/4000; %Se normaliza la señal (ya queda como w0)
b0 = (1-r)*sqrt(1+r*r-2*r*cos(2*W0));
G=(b0/(1-2*r*cos(W0)*z^(-1)+r*r*z^(-2)));
G
[num,den]=tfdata(G);
figure(2)
freqz(num{1},den{1},[],Fs);title('Resonador 523Hz');
figure(3)
w=linspace(0,2*pi,100);
circsx = cos(w); 
circsy = sin(w);
plot(circsx,circsy);hold;pzmap(G);title('Resonador 523Hz');
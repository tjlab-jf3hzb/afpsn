%--------------------------------------------------------------------
% To generate "H_filter.h",
%    execute this script on MATLAB/GNU Octave.
%
%      July 19, 2018  by T. Uebo
%--------------------------------------------------------------------


clear;


%-------Modify this section if you need -----------------------------
% Define Pass Band: f1(Hz) to f2(Hz)
f1=300;
f2=3000;

%Set a Window function
%wf=ones(Ndat,1);
%wf=hanning(Ndat);
%wf=hamming(Ndat);
wf=blackman(Ndat);
%wf=bartlett(Ndat);
%--------------------------------------------------------------------


%Sampling frequency
fs=10.55e3;
%Number of Taps
Ndat=512;




%---------- Calc. complex FIR coeff. ---------------------
if (f1<f2)
  fa=f1+fs/2;
  fb=f2+fs/2;
else
  fa=f2+fs/2;
  fb=f1+fs/2;
end

ka=round( fa/(fs/Ndat) ); kb=round( fb/(fs/Ndat) );
fres=[ zeros(1,ka-1) ones(1,kb+1-ka) zeros(1,Ndat-kb)];

fres=[fres(Ndat/2+1:end) fres(1:Ndat/2)];
ipr=ifft(fres);

Ich=[real(ipr(Ndat/2+1:end)) real(ipr(1:Ndat/2))];
Qch=[imag(ipr(Ndat/2+1:end)) imag(ipr(1:Ndat/2))];
Ich=Ich.*wf.'; Qch=Qch.*wf.';

figure(1); plot(Ich); ylabel('FIR coeff. (Real Part)');
figure(2); plot(Qch); ylabel('FIR coeff. (Imaginary Part)');
figure(3);
[hr,fr]=freqz(Ich + sqrt(-1).*Qch,[1], Ndat*16,'whole',fs);
fr=fr-fs/2;
hr=[hr(Ndat*8+1:end); hr(1:Ndat*8)];
plot(fr, 20.*log10( abs(hr)) );
xlabel('Frequency(Hz)');
ylabel('Response(dB)');
grid on;



figure(4)
subplot(1,2,1)
plot(fr, 20.*log10( abs(hr)) );
xlabel('Frequency(Hz)');
ylabel('Response(dB)');
grid on;
axis([f1-200, f1+200, -80 10]);

subplot(1,2,2)
plot(fr, 20.*log10( abs(hr)) );
xlabel('Frequency(Hz)');
ylabel('Response(dB)');
grid on;
axis([f2-200, f2+200, -80 10]);




%--- Convert format to Q1.15 & Create header file -------------
Icoeff=round(Ich*32767*1.8);
Qcoeff=round(Qch*32767*1.8);

Icoeff=Icoeff+(Icoeff<0)*65536;
Qcoeff=Qcoeff+(Qcoeff<0)*65536;

FID=fopen('H_filter.h', 'w');
fprintf(FID, 'const fractional H_Re[%d]={\n' , Ndat);

for p=[1:Ndat-1]
fprintf(FID,'0x%04x,\n', Icoeff(p));  
end;
fprintf(FID,'0x%04x\n', Icoeff(Ndat) );
fprintf(FID,'};\n');

fprintf(FID, 'const fractional H_Im[%d]={\n' , Ndat);
for p=[1:Ndat-1]
fprintf(FID,'0x%04x,\n', Qcoeff(p) );  
end;
fprintf(FID,'0x%04x\n',  Qcoeff(Ndat) );
fprintf(FID,'};\n');

fclose(FID);


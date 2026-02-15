function [autocross_mean,frequencies]=autocross(data1,data2,fsamp,N_win,N_OL,Win)
% data1, data2: time histories input
% fsamp: sampling frequency
% N_win: number of points in each subrecords that it has been used to divide the time history
% N_OL: number of points of overlap between two consequent subrecords
% Win: time window used to weight the data


N=length(data1); % Number of points of the data (data1 and data2 must have the same size)
df=fsamp/N_win; % The frequency resolution is due to the number of points of the window


if (N_win/2)==(floor(N_win/2))
    frequencies=0:df:(N_win/2*df);
else
    frequencies=0:df:((N_win-1)/2)*df;
end

NF=length(frequencies);

num_records=fix((N-N_OL)/(N_win-N_OL));

autocross=zeros(NF,num_records);

counter=1;
finalPoint_nextIT=0; % Index of the final point at next iteration (initialized at 0)

while finalPoint_nextIT <= N

    start_p=(counter-1)*(N_win-N_OL)+1;

    finish_p=start_p+(N_win-1);


    sp1=fft(Win.*data1(start_p:finish_p));
    sp1=sp1./N_win;

    sp2=fft(Win.*data2(start_p:finish_p));
    sp2=sp2./N_win;

    autocross(:,counter)=conj(sp1(1:NF)).*sp2(1:NF); % I'm saving the result only for the positive frequency WITHOUT COMPENSATING FOR THE MODULE AT THIS STAGE

    counter=counter+1;

    finalPoint_nextIT=finish_p+N_win-N_OL;

end

if (N_win/2)==(floor(N_win/2)) % I compensate for the module since I represent only the positive frequencies
    autocross(2:end-1,:)=autocross(2:end-1,:)*2;
else
    autocross(2:end,:)=autocross(2:end,:)*2;
end


autocross_mean=mean(autocross(1:NF,:),2);
end
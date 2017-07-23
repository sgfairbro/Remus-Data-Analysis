%Reading in gps/acomms data from 10-26-15 in-water tests
clear all
close all
clc
%Use either data from 10-26 (file_id = 26) run or 10-27 run (file_id = 27)
file_id = 27;
%fid = fopen('gps-acomms-2015-10-26.txt');
%NMEA GPS message example
%  $GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47
% 
% Where:
% msgHead          GGA          Global Positioning System Fix Data
% timeStamp        123519       Fix taken at 12:35:19 UTC
% lat              4807.038,N   Latitude 48 deg 07.038' N
% lon              01131.000,E  Longitude 11 deg 31.000' E
% fixq     1            Fix quality: 0 = invalid
%                                1 = GPS fix (SPS)
%                                2 = DGPS fix
%                                3 = PPS fix
%                                4 = Real Time Kinematic
%                                5 = Float RTK
%                                6 = estimated (dead reckoning) (2.3 feature)
% 			                     7 = Manual input mode
% 			                     8 = Simulation mode
% no_sat           08           Number of satellites being tracked
% horz_dil         0.9          Horizontal dilution of position
% alt              545.4,M      Altitude, Meters, above mean sea level
% geoid_h          46.9,M       Height of geoid (mean sea level) above WGS84
%                       ellipsoid
%      (empty field) time in seconds since last DGPS update
%      (empty field) DGPS station ID number
%      *47          the checksum data, always begins with *

%tsPos_tx = [];
tsPos_rx = [];
sig_str = []; %Record signal strength
dist_vec_rx = [];
veh_pos = [];
% %Use first while loop to find the indices of all the requests sent from
% %TOPSIDE (indicated by cycle init, $CACYC)
% while (~feof(fid))                              % For each block:
%     tline = fgetl(fid); %Retrieve line
%     f_ind = f_ind + 1;
%     InputText = strsplit(tline,','); %Parse line
%     msgHead = InputText{1}; %Extract message header
%     if(strcmp(msgHead,'$CACYC')) %Indicates that a cycle init (CACYC) was recently triggered
%         init_ind = [init_ind f_ind];
%     end
% end
%fclose(fid)
depthVec = [];
switch(file_id)
    case 26
        fid = fopen('gps-acomms-2015-10-26.txt');
    case 27
        fid = fopen('gps-acomms-2015-10-27.txt');
    case 28
        fid = fopen('gps-acomms-2015-10-26_27_combo.txt');
    otherwise
end
counter = 0;
notTrue = 0;
%Define system state vector as follows
%[]
%Repeat search for specific messages using indices of CACYC messages
while(~feof(fid))
    tline = fgetl(fid);%counter = counter + 1
    InputText = strsplit(tline,','); %Parse line
    msgHead = InputText{1}; %Extract message header
    %Begin search for next GPS message following CACYC message
    if (strcmp(msgHead,'$CACYC'))
        %Search for receive strength
        while( ~(strcmp(msgHead,'$CADQF')) && ~feof(fid))
        %while( ~(strcmp(msgHead,'$CARXD') || strcmp(msgHead,'$CAMSG')) && ~feof(fid))
            tline = fgetl(fid);%counter = counter + 1
            InputText = strsplit(tline,','); %Parse line
            msgHead = InputText{1}; %Extract message header
        end
        %Record signal strength
        if(strcmp(msgHead,'$CADQF')) % Double-check to make sure previous 
                                     % while loop exited because of 
                                     % signal strength message and not
                                     % EOF
            sig_str = str2num(InputText{2}); %Save for inclusion in veh_pos(:,4)
            %Get next message (assumed to be either $CARXD or $CAMSG)
            tline = fgetl(fid);
            InputText = strsplit(tline,','); %Parse line
            msgHead = InputText{1}; %Extract message header
        else
            break;
        end
        %Get next message (either $CARXD or $CAMSG)
        %%tline = fgetl(fid);
        %%if(tline ~= -1)
            %InputText = strsplit(tline,','); %Parse line
            %msgHead = InputText{1}; %Extract message header
        %%end
        %Evaluate which message was identified
        switch(msgHead)
            case '$CAMSG' %Indicates an error message
                %Re-use previous vehicle pose as nearest estimate
                veh_pos = [veh_pos;[0 veh_pos(end,2:4)]];
% % % % %                 %Retrieve latest GPS position for reference
% % % % %                 %This position represents the nearest location at which the
% % % % %                 %Topside vehicle receives the reply (versus its location 
% % % % %                 %when the request is sent out, commented out above).
% % % % %                 while (~strcmp(msgHead,'$GPGGA')  && ~feof(fid))
% % % % %                     tline = fgetl(fid);%counter = counter + 1
% % % % %                     InputText = strsplit(tline,','); %Parse line
% % % % %                     msgHead = InputText{1}; %Extract message header
% % % % %                 end
% % % % %                 %Secondary $GPGGA message acquired
% % % % %                 %Extract Topside POS info
% % % % %                 timeStamp = str2num(InputText{2});
% % % % %                 %Convert from Decimal-Decimal to Decimal-Degrees
% % % % %                 lat_dd = decdec2decdeg(str2num(InputText{3}));
% % % % %                 lon_dd = decdec2decdeg(str2num(InputText{5}));
% % % % %                 [x,y,u] = deg2utm(lat_dd, lon_dd); %Convert from dd to utm
% % % % %                 fix_q = str2num(InputText{7});
% % % % %                 %if(fix_q ~= 0) %If fix quality flag is not invalid (0), proceed
% % % % %                     pos_vec = [timeStamp x y];
% % % % %                     tsPos_rx = [tsPos_rx;pos_vec]; %Store TOPSIDE position upon receiving reply
% % % % %                 %end                
                
                
            case '$CARXD' %Indicates a successful message
                gg = strsplit(InputText{6});
                %Create character array to extract specific byte info
                gg = char(gg);
                %gg = '0e86FA11AD20C984f394ab8c' %Portion of 32-byte to use
                %for example conversion.
                %message from CCL for AUVs (Stokey)
                % 1) Select LAT/LON bytes
                % 2) Collapse BACK into a string
                % 3) Convert from hexidecimal to decimal
                msg_lat = hex2dec(cellstr([gg(7:8) gg(5:6) gg(3:4)]));
                msg_lon = hex2dec(cellstr([gg(13:14) gg(11:12) gg(9:10)]));
                %Decode depth
                dep = bitand(hex2dec(cellstr([gg(27:28) gg(25:26)])),hex2dec('1FFF'));
                if(dep <= 1000)
                    vz = dep*0.1;
                else if ((dep > 1000) && (dep <= 1500))
                        vz = 100 + (dep-1000)*0.2;
                    else if ((dep > 1500) && (dep <= 3100))
                            vz = 200 + (dep-1500)*0.5;
                        else if ((dep > 3100) && (dep <= 8100))
                                vz = 1000 + (dep-3100)*1.0;
                            else
                                vz = 6000;
                            end
                        end
                    end
                end
                                    
                %Decode into Decimal Degrees
                % %8388607 (dec) = 0x07FFFFF (hex) and is used to define
                % the LSB resolution
                msg_lat = msg_lat*(180/8388607);
                msg_lon = msg_lon*(180/8388607);
                [vx vy] = deg2utm(msg_lat,msg_lon);
                %Mark the vehicle pose with a "1" to indicate "success".
                veh_pos = [veh_pos;[1 vx vy vz]];
                
% %Strictly for reference, use the following to decode the 9-11 bytes
% %containing time information in a received CCL message.
%         vSec = bitand(timeStamp,15)*4;
%         vMin = 60*(bitand(bitshift(timeStamp,4),hex2dec('3F')));
%         vHour = 60*60*(bitand(bitshift(timeStamp,10),hex2dec('1F')));
%         %vDay not needed since experiment is assumed to take place on same
%         %day
%         %vDay = 24*60*60*(bitand(bitshift(timeStamp,15),hex2dec('1F')));
%         timeStamp = vSec+vMin+vHour;
%         %Bit shift to extract time stamp in 
% %         long Decode_time_date(TIME_DATE input, short *mon, short *day, short *hour, short *min, short *sec)
% %         {
% %             TIME_DATE_LONG comp;
% %             comp.as_long = 0;
% %             comp.as_time_date = input;
% %             *mon = (short)((comp.as_long >> (4 + 6 + 5 + 5)) & 0x000F);
% %             *day = (short)((comp.as_long >> (4 + 6 + 5)) & 0x001F);
% %             *hour = (short)((comp.as_long >> (4 + 6)) & 0x001F);
% %             *min = (short)((comp.as_long >> (4)) & 0x003F);
% %             *sec = (short)(((comp.as_long ) & 0x000F) * 4);
% %         }
            otherwise
        end
        
        %Retrieve latest GPS position for reference
        %This position represents the nearest location at which the
        %Topside vehicle receives the reply regardless whether the reply
        %is a success ($CARDX) or failure ($CAMSG).
        while (~strcmp(msgHead,'$GPGGA')  && ~feof(fid))
            tline = fgetl(fid);%counter = counter + 1
            InputText = strsplit(tline,','); %Parse line
            msgHead = InputText{1}; %Extract message header
        end
        %$GPGGA message acquired
        %Extract Topside POS info
        timeStamp = str2num(InputText{2});
        %Convert from Decimal-Decimal to Decimal-Degrees
        lat_dd = decdec2decdeg(str2num(InputText{3}));
        lon_dd = decdec2decdeg(str2num(InputText{5}));
        [x,y,u] = deg2utm(lat_dd, lon_dd); %Convert from dd to utm
        fix_q = str2num(InputText{7});
        %if(fix_q ~= 0) %If fix quality flag is not invalid (0), proceed
            pos_vec = [timeStamp x y sig_str];
            tsPos_rx = [tsPos_rx;pos_vec]; %Store TOPSIDE position upon receiving reply
        %end
    end
end
fclose(fid)    
%Clean up position information to exclude erroneous points and bias
%Get indices that do not fit and apply to both sets of GPS data

switch(file_id)
    case 26 %MUST REMOVE IMMEDIATELY FOLLOWING FINDING INDICES
        %FOR FILTERING 10-26 DATA
        ts_null = find((tsPos_rx(:,3) < 4.59e6) | (tsPos_rx(:,3) > 4.603e6));
        tsPos_rx(ts_null,:) = []; 
        veh_pos(ts_null,:) = [];
        veh_null = find((veh_pos(:,3) < 4.6005e6) | (veh_pos(:,3) > 4.6025e6));
        veh_pos(veh_null,:) = [];
        tsPos_rx(veh_null,:) = [];
    case 27
        %FOR FILTERING 10-27 DATA
        ts_null = find((tsPos_rx(:,3) < 4.5841e6) | (tsPos_rx(:,3) > 4.5862e6));
        tsPos_rx(ts_null,:) = [];
        veh_pos(ts_null,:) = [];
        veh_null = find((veh_pos(:,3) < 4.584e6) | (veh_pos(:,3) > 4.5865e6));        
        veh_pos(veh_null,:) = [];
        tsPos_rx(veh_null,:) = [];
    case 28
        %FOR FILTERING combination DATA
        ts_null = find((tsPos_rx(:,3) < 4.5841e6) | (tsPos_rx(:,3) > 4.603e6));
        tsPos_rx(ts_null,:) = [];
        veh_pos(ts_null,:) = [];
        veh_null = find((veh_pos(:,3) < 4.584e6) | (veh_pos(:,3) > 4.6025e6));        
        veh_pos(veh_null,:) = [];
        tsPos_rx(veh_null,:) = [];        
    otherwise
end

%tsPos_rx = tsPos_rx(((tsPos_rx(:,3) > 4.59e6) & (tsPos_rx(:,3) < 4.603e6)),:);
tsPos_rx = [tsPos_rx(:,1) (tsPos_rx(:,2) - mean(tsPos_rx(:,2))) (tsPos_rx(:,3) - mean(tsPos_rx(:,3))) tsPos_rx(:,4)];
%veh_pos = veh_pos(((veh_pos(:,2) > 4.59e6) & (veh_pos(:,2) < 4.65e6)),:);
veh_pos = [veh_pos(:,1) (veh_pos(:,2) - mean(veh_pos(:,2))) (veh_pos(:,3) - mean(veh_pos(:,3))) veh_pos(:,4)];
%Attempt to plot position of GPS fixes
figure;plot(tsPos_rx(:,2),tsPos_rx(:,3),'r*');hold on;
%%%Remove NULL vehicle position values
%%veh_pos = veh_pos((veh_pos(:,1) ~= 0),:);

switch(file_id)
    case 26
        save('10_26_data_update.mat');
    case 27
        save('10_27_data_update.mat');
end
%%
%Prepare PDF info
%load data
load 10_26_data_update.mat
plot(veh_pos(:,2),veh_pos(:,3),'g*')
%title('TOPSIDE and Remote Vehicle Trajectories');
xlabel('Easting [m]', 'fontsize',12, 'fontweight', 'bold');
ylabel('Northing [m]', 'fontsize',12, 'fontweight', 'bold');
legend('UUV trajectory','TOPSIDE trajectory')
distVec = zeros(1,length(veh_pos));
for r =1:length(distVec)
    %Adjusted to account of depth value (veh_pos(r,2:4))
    distVec(r) = norm(abs([tsPos_rx(r,2:3) 0] - veh_pos(r,2:4)));
end
binRes = 25;
binVec = 0:binRes:max(distVec); %Increase resolution of bins
dd = hist(distVec,binVec); %Total number of message counts at each distance

%Extract indices of successful transmissions (where veh_pos(:,1) == 1)
successInd = find(veh_pos(:,1) == 1);
failInd = find(veh_pos(:,1) == 0);
distSuccess = distVec(successInd);
distFail = distVec(failInd);
ss = hist(distSuccess,binVec);
ff = hist(distFail,binVec);
%Calculate p values for each distance bin
pp = ss./dd; %# of successes / # of total messages per distance bin
%Two looks at the same data to get an idea of the probability distribution
%Plot success probabilities as a function of the distance at which they occurred.
figure;
subplot(2,1,1);hist(distSuccess,binVec);title('Histogram of successes across different ranges')
subplot(2,1,2);stem(binVec,ss);
%Plot relationship between successes and distance with signal strength and
%distance
sig_set_vec_s = zeros(1,length(binVec));
sig_set_vec_f = zeros(1,length(binVec));

sig_info_s = [distSuccess' tsPos_rx(successInd,4)]; %Get all distances associated with successful comms
sig_info_f = [distFail' tsPos_rx(failInd,4)]; %Get all distances associated with failed comms

sig_info_s = sortrows(sig_info_s,1); %Sort numerically by row
sig_info_f = sortrows(sig_info_f,1); %Sort numerically by row

figure;plot(sig_info_s(:,1),sig_info_s(:,2))
figure;plot(sig_info_f(:,1),sig_info_f(:,2))

for uu = 2:length(binVec)
    %Extract all signal strength values within the distance bin for successes   
    sig_set_s = sig_info_s((sig_info_s(:,1) > binVec(uu-1))&(sig_info_s(:,1) < binVec(uu)),2);
    sig_set_vec_s(uu) = mean(sig_set_s); %Record average value
    
    %Extract all signal strength values within the distance bin for failures
    sig_set_f = sig_info_f((sig_info_f(:,1) > binVec(uu-1))&(sig_info_f(:,1) < binVec(uu)),2);
    sig_set_vec_f(uu) = mean(sig_set_f); %Record average value
end
figure;
subplot(3,1,1);plot(binVec,sig_set_vec_s,'r*');title('Plot of signal strength for successful comms across different ranges')
subplot(3,1,2);plot(binVec,sig_set_vec_f,'g*');title('Plot of signal strength for failed comms across different ranges')
subplot(3,1,3);stem(binVec,ss);
%Print statistics of signal strength
disp('Successful comm stats:')
sig_stat_s = [mean(sig_info_s(:,2)) std(sig_info_s(:,2))]
disp('Failed comm stats:')
sig_stat_f = [mean(sig_info_f(:,2)) std(sig_info_f(:,2))]
%Histogram of signal strength for all received messages (CADRX-GOOD & CAMSG-BAD)
figure;hist(tsPos_rx(:,4));title('Histogram of signal strength values (success and failure)');
%12/27/15, LTP: Parameter estimation
pars_exp = mle(distSuccess,'distribution','exp'); %Exponential
pars_gamma = mle(distSuccess,'distribution','gamma'); %Gamma
pars_norm = mle(distSuccess,'distribution','norm'); %Normal (Gaussian)
pars_rayleigh = mle(distSuccess,'distribution','rayl'); %Rayleigh
pars_rician = mle(distSuccess,'distribution','rician'); %Rician

%TODO: Create more parameter estimations and evaluate rmse or
%difference between the actual data and estimated pdf.
ss_est_exp = exppdf(binVec, pars_exp);
ss_est_exp = ss_est_exp/max(ss_est_exp); %Normalized
%ss_est_exp = (ss_est_exp/max(ss_est_exp))*max(ss)*0.01; %Normalized

ss_est_gamma = gampdf(binVec, pars_gamma(1), pars_gamma(2));
ss_est_gamma = ss_est_gamma / max(ss_est_gamma); %Normalized
%ss_est_gamma = (ss_est_gamma / max(ss_est_gamma))*max(ss)*0.01; %Normalized

ss_est_norm = normpdf(binVec,pars_norm(1), pars_norm(2)); %(X, mu, sigma)
ss_est_norm = ss_est_norm / max(ss_est_norm); %Normalized
%ss_est_norm = (ss_est_norm / max(ss_est_norm))*max(ss)*0.01; %Normalized

ss_est_rician = pdf('rician',binVec,pars_rician(1), pars_rician(2));
ss_est_rician = ss_est_rician / max(ss_est_rician); %Normalized
%ss_est_rician = (ss_est_rician / max(ss_est_rician))*max(ss)*0.01; %Normalized

%Rayleigh is a special case of Rician
%In the context of radio signal propagation fading, this occurs when there
%is no direct line of sight. Rayleigh SHOULD produce the same results.
% ss_est_rayleigh = raylpdf(binVec,pars_rayleigh(1));
% ss_est_rayleigh = ss_est_rayleigh / max(ss_est_rayleigh); %Normalized

%LTP 042116: Added smoothing feature for SMC2016 submission
alpha = 0.1;
ss(2:end-1) = alpha*ss(2:end-1) + (1-alpha)*0.5*(ss(1:end-2)+ss(3:end));
ss = ss/max(ss); %Normalize original histogram data
%ss = ss*0.01; %temporary original histogram data as percentages"
%Plot original histogram data, normalized
h_pdf=figure;plot(binVec,ss,'r-','linewidth',3);hold on;
%Plot exponential probability model, normalized
plot(binVec,ss_est_exp,'g*-');
%Plot gamma probability model, normalized
plot(binVec,ss_est_gamma,'b*-');
%Plot normal probability model, normalized
plot(binVec,ss_est_norm,'k*-');
%Plot rician probability model, normalized
plot(binVec,ss_est_rician,'c*-');
% % Plot rayleigh probability model, normalized
% plot(binVec,ss_est_rayleigh,'m*-');
xlabel('Communication Range [m]', 'fontsize',12, 'fontweight', 'bold');
ylabel('Probability of Successful Transmission, normalized', 'fontsize',12, 'fontweight', 'bold');
%h_leg = legend('Data set','Exponential','Gamma','Normal','Rician','Rayleigh')
h_leg = legend('Data set','Exponential','Gamma','Normal','Rician')
set(h_leg,'fontsize',12);
exp_rmse = sqrt(mean(power((ss - ss_est_exp),2)))
gamma_rmse = sqrt(mean(power((ss - ss_est_gamma),2)))
norm_rmse = sqrt(mean(power((ss - ss_est_norm),2)))
rician_rmse = sqrt(mean(power((ss - ss_est_rician),2)))
%rayleigh_rmse = sqrt(mean(power((ss - ss_est_rayleigh),2)))
set(h_pdf, 'Position', [1675 243 800 550]) 

% switch(file_id)
%     case 26
%         save('10_26_data_update.mat');
%     case 27
%         save('10_27_data_update.mat');
% end
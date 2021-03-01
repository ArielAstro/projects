function [sensorN, sensorW, sensorL, sensorD, score_s, config, fuselage, performance] = opt_intDBF4(TOWeight, AC_DryWeight, DRM_W, CroSec, fuselage_L, numb, max_senW, max_senL)
%% optimize_interiorDBF: Internal volume Optimization
% Optimizing for scores:
%   M2 Scoring: #containers/(time to complete 3 laps)
%   M3 Scoring: #laps*sensor_length*sensor_weight
% Note: "optintVscore.m" (from scoringPostDR.m) and "Takeoff_call.m" are required auxiliary functions

%   Code written by: Jhordan Baque
% DBF - Aerodynamics Squad - U. of Notre Dame
%% Inputs: 
%   TOWeight    : Takeoff weight (lbs)
%   AC_DryWeight: Aircraft weight including 1 battery and w/o payload (lbs)
%   DRM_W       : max DRM weight (lbs)
%   CroSec       : Max fuselage Cross-sectional width, heigh (in)
%   fuselage_L  : Available fuselage length (in)
% Optional inputs
%   numb  : next-best option can be any number > 1
%   max_senW    : Limits the weight of the sensor to the given value
%   max_senL    : Limits the length of the sensot to the given value
%   
%% Outputs:
%   sensorN    : Total number of sensors
%   sensorW    : Weight of each sensor
%   sensorL    : Max sensor length
%   sensorD    : Max sensor diameter
%   config     : Sensor arrangement (square or triang)

%% TEST
% 
% clear all
% TOWeight = 11;
% AC_DryWeight = 7.5;
% DRM_W = 0.5;
% CroSec = 4.25;
% fuselage_L = 15;
% numb = 1;
% %max_senW, max_senL % Optional parameters
%%
%
    LDRatio = 4; % min length to diameter ratio
    SD = 1.5;  % Sensor diameter set by team to 1in + 0.25in for container
    minw = 5/16;
    CroSec = 4.5:1/16:CroSec; % Initialize array for crossection from min = SD, to max = (user input CroSec)
    payload_w = TOWeight - AC_DryWeight - DRM_W;
    [n_sensor1, n_sensor2] = deal(zeros(1,length(CroSec))); % Initialize Square, Triang arrangement arrays  
    if exist('max_senW'), maxW = max_senW; % If the sensor weight is limited by the User
    else, maxW = payload_w; 
    end
    if exist('max_senL'), maxL = max_senL; % If the sensor length is limited by the user
    else, maxL = fuselage_L;
    end
    
 % Compute max number of sensors per cross-section (See sq_conf function below)
    
 % Square stacking
    for i = 1:length(CroSec), n_sensor1(i) = sq_conf(CroSec(i), SD, sq_conf(CroSec(i), SD)); end
    
 % Triangular stacking
    for i = 1:length(CroSec)
        ns = 0;
        lin = 0;
        [TRH, TRW] = deal(CroSec(i)); % TRH = total remaining heigh, TRW =TRWidth
                                      % these change within the loop as the
                                      % area is filled by sensors
        while SD*sqrt(2) <= TRH
            ns = ns + sq_conf(TRW, SD*sqrt(2)); 
            TRH = TRH - SD*sqrt(2)/2; 
            lin = lin+1;    % increase the line number, neede for staggered configuration
            if (rem(lin,2) ~= 0)
                TRW = CroSec(i)-SD*sqrt(2)/2; % for cylindrical cross section use XSDim-SD(i)/2;
                                              % shifts right every other
                                              % line of sensors
            else
                TRW = CroSec(i); 
            end
        end
        n_sensor2(i) = ns;
    end

    % Compiling n of sensors and diameter into a single matrix, and eliminating duplicates
    n_d_sensor1 = [n_sensor1(1); CroSec(1)];
    for i = 1:length(n_sensor1)-1
        if n_sensor1(i) < n_sensor1(i+1)
            n_d_sensor1 = [n_d_sensor1,[n_sensor1(i+1); CroSec(i+1)]];
        end
    end

    n_d_sensor2 = [n_sensor2(1); CroSec(1)];
    for i = 1:length(n_sensor2)-1
        if n_sensor2(i) < n_sensor2(i+1)
            n_d_sensor2 = [n_d_sensor2,[n_sensor2(i+1); CroSec(i+1)]];
        end
    end

    min_l = LDRatio*SD:1/16:fuselage_L; % Sensor length array
    
    % Sensor length calculation
    n_l_sensor = [[]]; 
    for i = 1:length(min_l)
        for j = 1:floor(fuselage_L./min_l(i)) % How many sensors of each length can fit (Same for both cases)
            if fuselage_L/j >= min_l(i), n_l_sensor(i,j,1) = j; end % Maximize the length of sensors to fill fuselage
        end
    end
    for i = 1:length(min_l)
        for j = 1:length(n_l_sensor(i,:,1)), n_l_sensor(i,j,2) = fuselage_L/n_l_sensor(i,j,1); end
    end

    % Reducing all options to only those viable and compiling into single matrix
    % of the form: number of sensors
    %              sensor weight
    %              sensor length
    %              sensor width
    %              number of sensor stacks lengthwise
    
    n_d_l_sensor1= [];
    for i = 1:size(n_d_sensor1,2)
        for j = 1:length(n_l_sensor(i,:,1))
            numbsen = n_d_sensor1(1,i)*n_l_sensor(i,j,1);
            weisens = payload_w/numbsen;
            if (n_l_sensor(i,j,2) ~= Inf || n_l_sensor(i,j,1) ~= 0)&& (weisens <= maxW) && (n_l_sensor(i,j,2) <= maxL)&& (weisens >= minw)
                n_d_l_sensor1 = [n_d_l_sensor1, [numbsen; weisens; n_l_sensor(1,j,2); n_d_sensor1(2,i);n_l_sensor(1,j,1)]];
            end
        end
    end
    
    n_d_l_sensor2= [[]];
    for i = 1:size(n_d_sensor2,2)
        for j = 1:length(n_l_sensor(i,:,1))
            numbsen = n_d_sensor2(1,i)*n_l_sensor(i,j,1);
            weisens = payload_w/numbsen;
            if (n_l_sensor(i,j,2) ~= Inf || n_l_sensor(i,j,1) ~= 0) && (weisens <= maxW) && (n_l_sensor(i,j,2) <= maxL)&& (weisens >= minw)
                n_d_l_sensor2 = [n_d_l_sensor2, [numbsen; weisens; n_l_sensor(i,j,2); n_d_sensor2(2,i);n_l_sensor(i,j,1)]];
            end
        end
    end
    
    % Comparing scores and eliminating duplicates and options that lead to an
    % unnecesarily wider fusealge crossection
    if size(n_d_l_sensor1,2) == 0
        n_d_l_sensor1 = zeros(8,1);
    elseif size(n_d_l_sensor2,2) == 0
        n_d_l_sensor2 = zeros(8,1);
    else
        for i=1:length(n_d_l_sensor1(1,:))-1
            for j = i+1:length(n_d_l_sensor1(1,:))
                if n_d_l_sensor1(1,i) == n_d_l_sensor1(1,j) && n_d_l_sensor1(4,i) > n_d_l_sensor1(4,j)
                    n_d_l_sensor1(:,i) = deal(0);
                elseif n_d_l_sensor1(1,i) == n_d_l_sensor1(1,j) && n_d_l_sensor1(4,i) < n_d_l_sensor1(4,j)
                    n_d_l_sensor1(:,j) = deal(0);
                end
            end
            for j = 1:length(n_d_l_sensor2(1,:))
                if n_d_l_sensor1(1,i) == n_d_l_sensor2(1,j) && n_d_l_sensor1(4,i) > n_d_l_sensor2(4,j)
                    n_d_l_sensor1(:,i) = deal(0);
                elseif n_d_l_sensor1(1,i) == n_d_l_sensor2(1,j) && n_d_l_sensor1(4,i) < n_d_l_sensor2(4,j)
                    n_d_l_sensor2(:,j) = deal(0);
                end
            end
        end
    end
    if size(n_d_l_sensor2,2) == 0
        n_d_l_sensor2 = zeros(8,1);
    else
        for i=1:length(n_d_l_sensor2(1,:))-1
            for j = i+1:length(n_d_l_sensor2(1,i))
                if n_d_l_sensor2(1,i) == n_d_l_sensor2(1,j) && n_d_l_sensor2(4,i) > n_d_l_sensor2(4,j)
                    n_d_l_sensor2(:,i) = deal(0);
                elseif n_d_l_sensor2(1,i) == n_d_l_sensor2(1,j) && n_d_l_sensor2(4,i) < n_d_l_sensor2(4,j)
                    n_d_l_sensor2(:,j) = deal(0);
                end
            end
        end
    end
    
  % Score Gathering and Optimization
    l1 = length(n_d_l_sensor1(1,:)); %number of square arrangement options
    l2 = length(n_d_l_sensor2(1,:)); %number of staggered arrangement options
  % Square config
    for i=1:l1
        if n_d_l_sensor1(1,i)~= 0
            [m2,m3,n_d_l_sensor1(6,i),v1m2(i),v1m3(i)] = optintVscore(n_d_l_sensor1(1,i), n_d_l_sensor1(2,i), n_d_l_sensor1(4,i),AC_DryWeight,n_d_l_sensor1(3,i),2);
            score1M2raw(i) = (n_d_l_sensor1(1,i)./m2);
            score1M3raw(i) = ((n_d_l_sensor1(3,i).*n_d_l_sensor1(2,i)).*m3);
            n_d_l_sensor1(7,i) = m2;
            n_d_l_sensor1(8,i) = m3;
        elseif n_d_l_sensor1(1,1) == 0
            score1M2raw = zeros(1,l1);
            score1M3raw = zeros(1,l1);
            v1m2(i) = 0;
            v1m3(i) = 0;
        end
    end
  % Triang config
    for i=1:l2
        if n_d_l_sensor2(1,i)~= 0
            [m2,m3, n_d_l_sensor2(6,i) v2m2(i),v2m3(i)] = optintVscore(n_d_l_sensor2(1,i), n_d_l_sensor2(2,i), n_d_l_sensor2(4,i),AC_DryWeight,n_d_l_sensor2(3,i),2);
            score2M2raw(i) = (n_d_l_sensor2(1,i)./m2);
            score2M3raw(i) = ((n_d_l_sensor2(3,i).*n_d_l_sensor2(2,i)).*m3);
            n_d_l_sensor2(7,i) = m2;
            n_d_l_sensor2(8,i) = m3;
        elseif n_d_l_sensor2(1,1) == 0
            score2M2raw = zeros(1,l2);
            score2M3raw = zeros(1,l2);
            v2m2(i) = 0;
            v2m3(i) = 0;
        end
    end
    lengthtrack = 2000 + 40*2*pi*2;
    m2best = 36*3*lengthtrack./(95);
    m3best = 36*floor((95)*600/lengthtrack);
    l1 = length(score1M2raw);
    l2 = length(score2M2raw);
    scoreM2 = [score1M2raw, score2M2raw]./m2best; %mat2gray([score1M2raw, score2M2raw]);
    scoreM3 = [score1M3raw, score2M3raw]./m3best; %mat2gray([score1M3raw, score2M3raw]);
    score1M2 = scoreM2(1:l1);
    score2M2 = scoreM2(l1+1:end);
    score1M3 = scoreM3(1:l1);
    score2M3 = scoreM3(l1+1:end);
    
        % Normalizing total scores (add the adequate 0-columns so vectors
        % match length)
    if l2 > l1
        score = [(score1M2 + score1M3),zeros(1,abs(l2-l1)) ;score2M2 + score2M3];
    elseif l2 < l1
        score = [(score1M2 + score1M3); [(score2M2 + score2M3), zeros(1,abs(l2-l1))]];
    else
        score = [(score1M2 + score1M3); (score2M2 + score2M3)];
    end
    
    
        % Index of best overal score -> in1(in2)
    [a, in1] = max(score, [],2); [~, in2] = max(a);
    
        % If the next 'nth' best score is needed this code executes
    n=1;
    if exist('numb')
        n=numb;
        for i = 1:(numb-1)
            score(in2, in1(in2)) = 0;
            [a, in1] = max(score, [],2); [~, in2] = max(a);
        end
    end
    
    % Function output
    if in2 == 1 
        sensorN = n_d_l_sensor1(1,in1(in2));
        sensorW = n_d_l_sensor1(2,in1(in2));
        sensorL = n_d_l_sensor1(3,in1(in2));
        sensorD = SD ;
        fuselage = [n_d_l_sensor1(4,in1(in2)), n_d_l_sensor1(6,in1(in2))];
        score_s = [score(1, in1(in2)), score1M2raw(in1(in2)),score1M3raw(in1(in2))];
        config = "Square";
        performance = [n_d_l_sensor1(7,in1(in2)),n_d_l_sensor1(8,in1(in2)),v1m2(in1(in2)),v1m3(in1(in2))];
    else
        sensorN = n_d_l_sensor2(1,in1(in2));
        sensorW = n_d_l_sensor2(2,in1(in2));
        sensorL = n_d_l_sensor2(3,in1(in2));
        sensorD = SD;
        fuselage = [n_d_l_sensor2(4,in1(in2)), n_d_l_sensor2(6,in1(in2))];
        score_s = [score(2, in1(in2)), score2M2raw(in1(in2)),score2M3raw(in1(in2))];
        config = "Triangle";
        performance = [n_d_l_sensor2(7,in1(in2)),n_d_l_sensor2(8,in1(in2)),v2m2(in1(in2)),v2m3(in1(in2))];
    end
    
    % Plot score distribution
    if n == 0
        figure(1)
        clf
        title("Score Distribution")
        subplot(2,2,1)
        hold on
        plot(n_d_l_sensor1(4,:),score(1,1:l1), '*', 'linewidth', 3)
        plot(n_d_l_sensor2(4,:),score(2,1:l2), '+', 'linewidth', 3)
        legend('square config', 'triang config')
        xlabel('Fuselage diameter (in)')
        ylabel('Norm. score')
        axis auto
        grid on
        subplot(2,2,2)
        hold on
        plot(n_d_l_sensor1(1,:),score(1,1:l1), '*', 'linewidth', 3)
        plot(n_d_l_sensor2(1,:),score(2,1:l2), '+', 'linewidth', 3)
        legend('square config', 'triang config')
        xlabel('Number of sensors')
        ylabel('Norm. score')
        axis auto
        grid on
        subplot(2,2,3)
        hold on
        plot(n_d_l_sensor1(2,:),score(1,1:l1), '*', 'linewidth', 3)
        plot(n_d_l_sensor2(2,:),score(2,1:l2), '+', 'linewidth', 3)
        legend('square config', 'triang config')
        xlabel('Sensor weight (lbs)')
        ylabel('Norm. score')
        axis auto
        grid on
        subplot(2,2,4)
        hold on
        plot(n_d_l_sensor1(3,:),score(1,1:l1), '*', 'linewidth', 3)
        plot(n_d_l_sensor2(3,:),score(2,1:l2), '+', 'linewidth', 3)
        legend('square config', 'triang config')
        xlabel('Sensor length (in)')
        ylabel('Norm. score')
        axis auto
        grid on
    end
end

%% Aux Functions
%
% *sq_conf()*
function [ns] = sq_conf(tot_dim, sen_dim, n_in)
%sq_conf() computes how many mxm areas can fit into a larger MxM area
    ns = floor(tot_dim/sen_dim);
    if exist('n_in')
        ns = ns*n_in;
    end
end

%%
% *optintVscore()*
function [tM2, tM3, chord, vm2,vm3] = optintVscore(nsen, wsen, f_CroSec, AC_DryWeight,len, vel_H_L)
%optinVscore returns mission 2 and mission 3 scores
% function getspeed.m directly based on 'scoringPostDR.m' ND DBF20-21

%   Code adapted by: Jhordan Baque
% DBF - Aerodynamics Squad - U. of Notre Dame
%% Inputs:
%   nsen    : number of sensors
%   wsen    : weigh of each sensor
%   f_CroSec: fuselage Cross-section
% Optional input:
%   vel_H_L : 1 to calculate score using velLow, 2 to use velHigh; default: avg velocity 
%%
%
    fusel = f_CroSec;
    lengthtrack = 2000 + 40*2*pi*2;
    WIm = AC_DryWeight+nsen*wsen+0.5; %0.5 lbs for DRM
    [vh, vl, chord] = getspeed(WIm, fusel);
    if exist('vel_H_L')
        if (vel_H_L == 1), vel = vl; elseif (vel_H_L==2), vel = vh; end
    else, vel = (vh);
    end
    tM2 = 3*lengthtrack./(vel); % Time mission 2
    vm2 = vel;

    [vh, vl] = getspeed(WIm, fusel, chord);
    if exist('vel_H_L')
        if (vel_H_L == 1), vel = vl; elseif (vel_H_L==2), vel = vh; end
    else, vel = (vh);
    end
    tM3 = floor((vel)*600/lengthtrack); %numlaps
    vm3 = vel;
end

%% Functions
%
function [vhigh, vlow, chord] = getspeed(WIm, fwide, chordin)
%Computes an approximate velocity accounting for thrust and drag conditions
%     syms v 
    RPM = 7696;%5634; 
    pitch = 8;
    diams = 16;
  
    vcf = 3.281;
    cf = 4.448;
%     q_inf = 0.5*1.225*((v./vcf)^2);
    q1 = 0.5*1.225/(vcf^2);
    coe1 = 4.392399E-8;
    coe2 = 4.23333E-4;

    if exist('chordin')
        diam = 1.25*0.0254; % Single container diameter/width
        C_d_sen = 0.2; %stay about the same
%         Drag_Sensor = q_inf*(diam^2)*(C_d_sen)/4.448;
        chord = chordin; 
        cds7 = (diam^2)*(C_d_sen)/4.448;
    else
%         Drag_Sensor = 0;
        cds7 = 0;
        chord = Takeoff_call(WIm);
    end
    
%     T = 4.392399E-8*RPM*(diams^3.5)/sqrt(pitch)*(4.23333E-4*RPM*pitch-v/3.281)/4.448;

    S_wIm = 5*chord;
    W = WIm*cf;
    S_w = S_wIm*0.0929;

    % Wing Drag
%     Cl = W/(S_w*q_inf);
    Cl = W/(S_w*q1);
    C_d_wing_0 = 0.01;
    AR = 5^2/S_wIm;
    e = 0.75;
    C_d_wing_i = (Cl^2)/(pi*AR*e);
%     Drag_wing = q_inf*S_w*(C_d_wing_i + C_d_wing_0)/4.448;
    cds1 = S_w*(C_d_wing_0)/cf;
    cdss = C_d_wing_i/cf;

    % Horizontal Tail Drag
    C_d_htail = 0.003;
    S_htailIm = 501.76/144;%1*chord;
    S_htail = S_htailIm*0.0929;
%     Drag_htail = q_inf*S_htail*(C_d_htail)/4.448;
    cds2 = S_htail*(C_d_htail)/cf;
    
    % Vertical Tail Drag
    C_d_vtail = 0.003; % from Princeton University
    S_vtailIm = 106.59/144;%0.3194*chord;
    S_vtail =   S_vtailIm*0.0929;
%     Drag_vtail = q_inf*S_vtail*(C_d_vtail)/4.448;
    cds3 = S_vtail*(C_d_vtail)/cf;
    
    %Fuselage Drag
    C_d_fuse = 0.27; %stay about the same
    S_fuse = ((fwide^2)/144)*0.0929;
%     Drag_fuse = q_inf*S_fuse*(C_d_fuse)/4.448;
    cds4 = S_fuse*(C_d_fuse)/cf;

    % Strut Drag
    S_strut = 0.002053; % [m^2]
    C_d_strut = 0.003;  % treating it as a flat plate
%     Drag_strut = q_inf*S_strut*(C_d_strut)/4.448;
    cds5 = S_strut*(C_d_strut)/cf;
    
    % Wheel Drag
    Cd_wheel = 1.2; % found from NASA of 
    S_wheel = pi*(1.625/12)^2*0.0929;
%     Drag_wheel = q_inf*S_wheel*Cd_wheel/4.448;
    cds6 = S_wheel*Cd_wheel/cf;
    
    % Total Drag
%     DragM = Drag_htail + Drag_wing + Drag_fuse + 2*Drag_wheel + Drag_vtail + Drag_strut + Drag_Sensor;
%     eqn = DragM == T;
%     Ssol = solve(eqn,v, 'ReturnConditions', true, 'Real', true);
%     velocity = vpa(Ssol.v);
    cds = cds1 + cds2 + cds3 + 2*cds4 + cds5 + cds6;
    A = cds*q1*pi*AR*e;
    B = ((coe1*RPM*diams^3.5)/(vcf*sqrt(pitch)))*pi*AR*e;
    C = -coe1*coe2*RPM^2*sqrt(pitch)*(diams^3.5)*pi*AR*e;
    D = 0;
    E = 1.225*W^2;
    p = (8*A*C-3*B^2)/(8*A^2);
    q = (B^3-4*A*B*C+8*(A^2)*D)/(8*A^3);
    del0 = C^2 -3*B*D+12*A*E;
    del1 = 2*C^3-9*B*C*D+27*E*B^2+27*A*D^2-72*A*C*E;
    Q =((del1+sqrt(del1^2 - 4*del0^3))/2)^(1/3);
    S = 0.5*sqrt(-2*p/3+1/(3*A)*(Q+del0/Q));
    %x1 = -B/(4*A)-S-0.5*sqrt(-4*S^2-2*p+q/S);
    %x2 = -B/(4*A)-S+0.5*sqrt(-4*S^2-2*p+q/S);
    vlow = real(-B/(4*A)+S-0.5*sqrt(-4*S^2-2*p-q/S));
    vhigh = real(-B/(4*A)+S+0.5*sqrt(-4*S^2-2*p-q/S));
    
  %  vlow = double(velocity(2));
 %   vhigh = double(velocity(3));
end

function [chord_final] = Takeoff_call(input)
%% initial parameters
weight=input; % weight input from Takeoff_function.m
% weight_met=weight*4.448;
% N=500;
% c=linspace(1,3,N); % chord vector [ft]
% c_met=c./3.281;
% S=5/3.281; % span [m]
% rho=1.225;
% rho_eng=0.00237;
% Smax=80;
% Tint=0.0005;
% RPM = 7696;%5634; 
% pitch = 8;
% dia = 18;
% AR=5;
% e=0.9;
% To=4.392399E-8*RPM*dia^3.5/sqrt(pitch)*(4.23333E-4*RPM*pitch); % static thrust
% To_eng=To./4.448;
% mu=0.04;
% g=32.2;
% % K=1/(pi*AR*e);
% % h=0.33;
% b=5;
% % phi=(16*h/b)^2/(1+(16*h/b)^2);
% % Kg=phi*K;
% % Clg=mu/(2*Kg);
% Clg=linspace(0.8,0.4,N);
% % velocity vectors
% Vstall=sqrt(2*weight_met./(rho.*c_met.*S.*Clg));
% Vnec=1.2.*Vstall;
% Vnec_eng=Vnec.*3.281; % m/s to ft/s
% % Vstall_eng=Vstall.*3.281; % m/s to ft/s
% n=2;
% vi=0;
% a=1;
% 
% % loops
% while vi<Vnec_eng(n-1) && n <= N
%     vi=0;
%     d=0;
%     a=1;
%     while d<Smax && a==1
%         if vi==0
%             CDg=0;
%             dvdt=g*((To_eng/weight)-mu)-((g/weight)*(0.5*rho_eng*b*c(n-1)*(CDg-mu*Clg(n-1))))*vi^2;
%             vi=vi+Tint*dvdt;
%             d=d+Tint*vi+0.5*dvdt*Tint^2;
%         elseif vi>0 && vi<Vnec_eng(n-1)+1
%             CDg=1.8824/(0.5*rho_eng*vi^2*b*c(n-1));
%             dvdt=g*((To_eng/weight)-mu)-((g/weight)*(0.5*rho_eng*b*c(n-1)*(CDg-mu*Clg(n-1))))*vi^2;
%             vi=vi+Tint*dvdt;
%             d=d+Tint*vi+0.5*dvdt*Tint^2;
%         else
%             a=0;
%         end
%     end
%     n=n+1;
% end
%     if a==1
%         chord_final=c(n-2);
%     else
%         chord_final=1;
%     end
    chord_final=2;
end 

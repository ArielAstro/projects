%% mk-2 Sensor Optimization
% DBF - Aerodynamics Squad - U. of Notre Dame
%   Code written by: Jhordan Baque
%%
%
clear all
tic
k = 11;%21;
n = 10;%2 % Best n arrangements    
m = 1;%41 % Number of weight intervals (edit table and or or remove cols if value changed)
MAXW = 18;%20 % max weight
MINW = 18;%10 %min weight
XSD = 5;%5 % Cross-section
PLL = 36; % Payload bay length
wmo = linspace(0.5,1.2,k);

for ai = 1:k
    a(:,:,:,ai) = deal(run_opt(n,m,MAXW, MINW, XSD,PLL,wmo(ai))); % run main function
end 

m2 = [];m3 = [];
for i = 1:size(a,4)
for j = 1:size(a,3)
    m2 = [m2; a(2,:,j,i)];
    m3 = [m3; a(3,:,j,i)];
end
end

%% Overall normalized scores
lengthtrack = 2000 + 40*2*pi*2;
m2best = 36*3*lengthtrack./(95);
m3best = 36*floor((95)*600/lengthtrack);


m2 = m2/m2best;%mat2gray(m2);
m3 = m3/m3best;%mat2gray(m3);

% m2 = mat2gray(m2);
% m3 = mat2gray(m3);

overall = mat2gray(m2+m3);

%%Plots
mm2 = []; mm3 = [];mm = [];ll = [];nn = [];cc = [];fw = [];
for j = 1:k
    for i = 1:m
        mm2 = [mm2, m2(i,:)];
        mm3 = [mm3, m3(i,:)];
        mm = [mm, overall(i,:)];
        ll = [ll, a(6,:,i,j)];
        fw = [fw, a(7,:,i,j)];
        cc = [cc, a(8,:,i,j)];
        nn = [nn, a(4,:,i,j)];
    end
end
figure(1)
scatter(fw,mm,[],mm,'filled','MarkerEdgeColor',[0 .4 .4], 'LineWidth',0.2)
colorbar()
grid on
hold on
figure(2)
clf, scatter3(fw,cc,mm,nn*5,mm,'filled','MarkerEdgeColor',[0 .4 .4], 'LineWidth',0.2)
xlabel('Fuselage Width (ft)')
ylabel('Wing chord (ft)')
c = colorbar;
c.Label.String = 'Score';
grid on
figure(3)
histogram2(ll,mm,[12 12],'FaceColor','flat'), colorbar()
toc

%%
% 3D surface plot
% [X,Y] = meshgrid(linspace(1,3,5082),linspace(1,5,5082));
% clf
% for k = 1:5082
% for i=1:5082
% if abs(cc(k)-X(i,i))<0.15
% for j = 1:5082
% if abs(fw(k)-Y(j,j))<0.01 && abs(cc(k)-X(i,i))<0.01
% Z(i,j) = mm(k);
% elseif abs(fw(k)-Y(j,j))<0.02 && abs(cc(k)-X(i,i))<0.02
% Z(i,j) = 0.8*mm(k);
% elseif abs(fw(k)-Y(j,j))<0.03 && abs(cc(k)-X(i,i))<0.03
% Z(i,j) = 0.7*mm(k);
% elseif abs(fw(k)-Y(j,j))<0.04 && abs(cc(k)-X(i,i))<0.04
% Z(i,j) = 0.6*mm(k);
% elseif abs(fw(k)-Y(j,j))<0.05 && abs(cc(k)-X(i,i))<0.05
% Z(i,j) = 0.5*mm(k);
% elseif abs(fw(k)-Y(j,j))<0.08 && abs(cc(k)-X(i,i))<0.08
% Z(i,j) = 0.4*mm(k);
% elseif abs(fw(k)-Y(j,j))<0.15 && abs(cc(k)-X(i,i))<0.15
% Z(i,j) = 0.3*mm(k);
% end, end, end, end,end
% surf(X,Y,Z, 'EdgeColor', 'none')
% colorbar()
% xlabel('Chord length [ft]')
% c = colorbar;
% c.Label.String = 'Score';
%%
%
function [Optimized] = run_opt(n,m,MAXW, MINW, XSD,PLL,wmo)
    s = string();
    aa = 1;
    TOW = linspace(MAXW,MINW,m);
    Optimized =zeros(12,n,m);
    wm = wmo;%linspace(0.5,2.5,aa);
    PLLv = linspace(20,PLL,aa);
    k = 1;
    for j = 1:m
        for i = 1:n
        for p = 1:aa
            [sn, sw, sl, sh, sc, t, f, perf] = opt_intDBF4(TOW(j), 8.5, 0.5, XSD, 36, i,wm);
            s = s + '  ' + t ;
            Optimized(:,k,j) = [sc(1); sc(2); sc(3);sn;sw;sl;f(1);f(2);perf(3);perf(1);perf(4);perf(2)];            
            %Plot arrangement
%             if j == 1
%                 figure(i+1);
%                 clf
%                 plot_sens(sn,sw,sl,sh,sc(1),[f(1),PLLv(p)], t);
%             end 
            k = k+1;
        end
        end
        k=1;
    end
%     for i = 1:size(Optimized,2)
%         T = table(Optimized(:,i,1));%,Optimized(:,i,2),Optimized(:,i,3),Optimized(:,i,4),Optimized(:,i,5),Optimized(:,i,6),Optimized(:,i,7),Optimized(:,i,8));%,Optimized(:,i,9),Optimized(:,i,10),Optimized(:,i,11),Optimized(:,i,12),Optimized(:,i,13),Optimized(:,i,14),Optimized(:,i,15),Optimized(:,i,16));
%         for i = 1:m
%             names(i) = string(TOW(i)) + '[lbs]';
%         end
%         T.Properties.RowNames = {'Score Norm';'M2 Raw Score';'M3 Raw Score';
%             'N sens'; 'Weight sens [lbs]'; 'Length sens';
%             'Fuslg width [in]' ; 'Chord Length [ft]';
%             'M2 Speed';'M2 time';'M3 Speed';'N laps'};
%         T.Properties.VariableNames = names
%     end
end


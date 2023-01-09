%% Lower-layer controller initialization 
load('DATA/HEV_powertrain_parameters.mat') % from the E-CoSM 2021 http://shenlab.jp/ecosm2021/program/benchmark-challenge.html
Horizon_Lowlayer = T_PreDomn-T_Apply; % units:[s]
Sampling_Lowlayer = 0.05; Ts_lower = Sampling_Lowlayer;
N_PMP_Lowlayer = Horizon_Lowlayer/Sampling_Lowlayer;
v_h_limit_max=spd_lim; % Km/h 
Initial_vhcl_acc= 0; % we-make-Initial acc [m/s2]
Initial_MG1_spd = 0; % we-make-Initial engine speed [rpm]
Initial_MG2_spd = 0; % we-make-Initial engine speed [rpm]
%% engine BSFC
rads2rpm=60/2/pi;   rpm2rads=1/rads2rpm; 
N_rpm=1000:5:5600;  T_nm=0:5:145; % Engine 
[X,Y]=meshgrid(N_rpm,T_nm); % map 
Fuel_rate_Matrix=interp2(FuelConsmp_EngSpd,FuelConsmp_EngTrq,FuelConsmp_Rate_Matrix',X,Y);
BSFC_Matrix=1000*3600*Fuel_rate_Matrix./(rpm2rads*X.*Y);%BSFC Data
%% energy OOL
Power_level=0:200:62800; % unit:[W]
Maxtrq=interp1(Maxtorque_EngSpd,Maxtorque_EngTrq,N_rpm);%1000-5600rpm Max torque
Power_ex=Power_level*rads2rpm;
N_opt=zeros(1,length(Power_ex));
T_opt=zeros(1,length(Power_ex));
Fuel_Rate_opt=zeros(1,length(Power_ex));
Index=zeros(1,length(Power_ex));
for i=1:length(Power_ex) 
    T_s=Power_ex(i)./N_rpm;
    fuel_rate_s=interp2(FuelConsmp_EngSpd,FuelConsmp_EngTrq,FuelConsmp_Rate_Matrix',N_rpm,T_s);
    fuel_rate_s(find(T_s>Maxtrq))=nan;%out of torque range is nan
    [value,ind]=min(fuel_rate_s);
    Fuel_Rate_opt(i)=value;
    Index(i)=ind;
    N_opt(i)=N_rpm(ind);
    T_opt(i)=T_s(ind);
end
Fuel_Rate_opt_nihe=Fuel_Rate_opt;%unit[g]
Power_nihe=Power_level/1000;%unit[kw]
Fuel_OOL=interp2(FuelConsmp_EngSpd,FuelConsmp_EngTrq,FuelConsmp_Rate_Matrix',N_opt,T_opt);%OOL line fuel
%% method-1:Used the conversion from LBFT
eng_opt_tau_map_spd=[1000 1600 4000 6000]; 
eng_opt_tau_max_trq=[120 150 150 100];        
eng_opt_tau_map_fuel=[0 0.1 0.2 0.3 0.4 0.5 0.6 0.7 0.8 0.9 1];
for i = 1:length(eng_opt_tau_map_spd)
    for j = 1:length(eng_opt_tau_map_fuel)
        eng_opt_tau(i,j) = (j-1)/(length(eng_opt_tau_map_fuel)-1)*eng_opt_tau_max_trq(i);
    end
end
%% Generator Control   
max_gen_spd=11000*rpm2rads;   min_gen_spd=-11000*rpm2rads; %rad/s  
LookUp_MG1_MaxMintorque_Spd=[-11250;-11000;-10750;-10500;-10250;-10000;-9750;-9500;-9250;-9000;-8750;-8500;-8250;-8000;-7750;-7500;-7250;-7000;-6750;-6500;-6250;-6000;-5750;-5500;-5250;-5000;-4750;-4500;-4250;-4000;-3750;-3500;-3250;-3000;-2750;-2500;-2250;-2000;-1750;-1500;-1250;-1000;-750;-500;-250;0;250;500;750;1000;1250;1500;1750;2000;2250;2500;2750;3000;3250;3500;3750;4000;4250;4500;4750;5000;5250;5500;5750;6000;6250;6500;6750;7000;7250;7500;7750;8000;8250;8500;8750;9000;9250;9500;9750;10000;10250;10500;10750;11000;11250];
LookUp_MG1_MaxMintorque_MaxTrq=[22.575;22.575;23.52;24.465;25.095;25.725;26.46;27.195;28.0875;28.98;29.82;30.66;31.8675;33.075;34.3875;35.7;37.17;38.64;40.11;41.58;42;42;42;42;42;42;42;42;42;42;42;42;42;42;42;42;42;42;42;42;42;42;42;42;42;42;42;42;42;42;42;42;42;42;42;42;42;42;42;42;42;42;42;42;42;42;42;42;42;42;42;41.5800000000000;40.1100000000000;38.64;37.17;35.7;34.3875;33.075;31.8675;30.66;29.82;28.98;28.0875;27.195;26.46;25.725;25.095;24.465;23.52;22.575;22.575];
%% Rule-based
target_soc=50; % try to maintain the soc-----------benchmark
soc_control=45;   % for charging mode control-------benchmark
w_engine_on = 40*1000/3600/(Rt*0.001); % (rad/s) 40 km/h Guess,1 mph= 1.609344 km/h
P_ev_mode = 50630; %(w) -----guess£ºomega*tau
eng_pwr_max=62832; %engine max power (W)
P_eng_mode =60000; %(w)
eng_idle_spd=800*rpm2rads;  % (rad/s), engine's idle speed
% gc_kp_on= 0.005;%*0.1;        % Proportional Factor Controller gains  N*m/(rad/s) 
gc_kp_on= 0.45;%*0.1;        % Proportional Factor Controller gains  N*m/(rad/s) 
% gc_ki_on=0.005;       % Integration Factor          N*m/(rad/s)/sec       
gc_ki_on=0.0001;       % Integration Factor          N*m/(rad/s)/sec   
disp('Lower-Layer Controller Initial Sucessfully!'); 
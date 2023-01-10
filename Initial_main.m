% close all
clear all
clc
%% Traffic data set up
%---We need the data of the traffic in "DATA/Traffic_Data/"
datai=2;% It need to been set as "datai=x" to correspond to the new dataset "ECOSM2021_benchmark_problem_datax.mat" 
% http://shenlab.jp/ecosm2021/program/benchmark-challenge.html
% Note:Each data name in "ECOSM2021_benchmark_problem_datax.mat" needs to be consistent with "ECOSM2021_benchmark_problem_datax.mat".
%% Step1: initialization parameter: running the function "work_initial.p" or run the function
eval(['load(''DATA/Traffic_Data/ECOSM2021_benchmark_problem_data',num2str(datai),'.mat'')']);
% Defining the unit conversion 
ras2rpm = 60/2/pi; % [rad/s -> rpm]
rpm2rads=1/ras2rpm; %[rpm -> rad/s]
mps2kmh = 3600/1000; % [m/s -> km/h]
gamma = 25; % The equivalent factor
% Index Performance 
spd_lim = 60; % maximum speed limitation on the road in Japan[km/h]
hw_min = 3; % minimum headway distance [m]
driv_rect_time = 0.5; % driver reflect time [s]
% Initial parameters
Initial_SOC = 50;          % Initial SOC [%]
Initial_headway = Sensor_FOL_dDist(2); % Initial distance headway [m]
Initial_eng_spd = 0; % Initial engine speed [rpm]
Initial_vhcl_spd = 0; % Initial ego vehicle speed [m/s]
simTs = 0.01;              % Sampling time [sec]0.01
% Slope data from the E-CoSM 2021 http://shenlab.jp/ecosm2021/program/benchmark-challenge.html
load('DATA/Route_slope_parameters.mat') % load the slope 
% Simulation time setting [sec], please set it
t_end =max(System_Time)+120;     
%% Upper layer initialization---------------------
Initial_Upper
%% Lower-layer initialization
Initial_Lower
%% -------------Data loaded sucessfully!-----------------------
disp('Data loaded sucessfully!');
%% Step2: Running the Model
% open the "Plant_Controller_2023.slx"
% click the "run"
 
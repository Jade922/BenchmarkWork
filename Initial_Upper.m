%% Upper layer initialization 
Route_Destination_Distance = 16002;  
Route_Intersection_Distance = [ 1288;2358;2572;2886;3066;3150;3422;3784;4696;
                                5377;5576;6855;7626;10162;10575;10948;11201;11537;
                                11781;12302;12587;12785;13032;13368;14364;15802];                         
%% Horizon
Ts_predict = 0.1;   Ts_upper = 0.05;  % Sampling time
T_PreDomn = 5;      T_Apply = 1.5; % [s]
m = T_PreDomn/Ts_upper; % Control domain
p = m; % Predicted domain
T = T_Apply/Ts_upper; % Solving interval
%% Constraints
hw_min_ctrl = 10; % minimum headway distance in controller [m]
driv_rect_time_ctrl = 0.6; % driver reflect time in controller [s]
d_inter_margin = 1;  d_pre_margin = 1;
VMAX_CONS = 51/3.6;  VMIN_CONS = 0;      % Velocity
ACCLMAX_CONS = 1.6;  ACCLMIN_CONS = -10; % Acc
%% Initial variables
x0 = [ 0; Initial_vhcl_spd ];% Initial state: [position; speed]
flag_pre=0;     flag_inter=0;     phase = 0;         pattern_GoThrough = 0;
N_inter_h = 0;  exitflag = 0;     Result_U_best = [];   
u0_iter = zeros(m,1);     flag_destn = 0;
u_k = 0;     x_k = x0;    y_k = [0;0];
%% Longnitude dynamics model
M_v = 1700;  % Vehicle mass [kg]
k10 = -26.2596;     k11 = 7.884;     k2 = 16670;     k3 = 107.3;
Ax = [ 1                 Ts_upper-(((Ts_upper^2)*k11)/(2*M_v)); 
       0                 1-(Ts_upper*k11/M_v)];
Bu = [ (Ts_upper^2)/2;   Ts_upper];
Bw = [-(Ts_upper^2)/2;  -Ts_upper];
Cx = [0   -k11/M_v ;
      0    k11 ]; 
Du = [1;   0]; 
Dw = [-1;  M_v]; 
%% Transfer matrice T1/T2/T3/T4/T5
% T1 (p*2p) -->>  T1 * X(k+1) --> S(k+1)
T1=zeros(p,size(Ax,1)*p);
[row,col]=size(T1);
for i=1:1:row
    T1(i,col/row*i-1)=1;
end
% T2 (p*2p) -->> T2 * X(k+1) --> V(k+1)
T2=zeros(p,size(Ax,1)*p);
[row,col]=size(T2);
for i=1:1:row
    T2(i,col/row*i)=1;
end
% T3 (2p*2p) -->> X_k = T3*X_(k+1) + T4*x_k
T3=zeros(size(Ax,1)*p,size(Ax,1)*p);
for i = 1:2*p
    for j = 1:2*p
        if i == j+2
            T3(i,j) = 1;
        end
    end
end
% T4 (2p*2) -->> T4 = zeros(size(Ax,1)*p,size(Ax,1));
T4(1,1) = 1;   T4(2,2) = 1;
% T5 (1*2p)
for i = 1:p
   T5(1,2*i-1:2*i) = [0 1];
end
T_TransSamp = zeros(p,(p*Ts_upper/Ts_predict));
if Ts_predict > Ts_upper
    for i = 1:(p*Ts_upper/Ts_predict)
        T_TransSamp(((Ts_predict/Ts_upper)*(i-1)+1):(Ts_predict/Ts_upper)*i,i)=[1;1];
    end
else
    for i = 1:p
        T_TransSamp(i,((Ts_upper/Ts_predict)*(i-1)+1):(Ts_upper/Ts_predict)*i)=...
            (Ts_predict/Ts_upper)*ones(1,Ts_upper/Ts_predict);
    end
end
%% Prediction equation
M_xx = zeros(size(Ax,1)*p,size(Ax,1));  M_xu = zeros(size(Bu,1)*p,size(Bu,2)*p);   M_xw = zeros(size(Bw,1)*p,size(Bw,2)*p);
M_yx = zeros(size(Cx,1)*p,size(Cx,1));  M_yu = zeros(size(Du,1)*p,size(Du,2)*p);   M_yw = zeros(size(Dw,1)*p,size(Dw,2)*p);
for i = 1:p  % Row
    M_xx( (size(Ax,1)*(i-1)+1:size(Ax,1)*i),: ) = Ax^i;
    M_yx( (size(Ax,1)*(i-1)+1:size(Ax,1)*i),: ) = Cx*Ax^(i-1);
    for j = 1:p  % Column
        if i >= j  % Row >= Column
            M_xu( (size(Bu,1)*(i-1)+1:size(Bu,1)*i),(size(Bu,2)*(j-1)+1:size(Bu,2)*j) )= Ax^(i-j)*Bu;
            M_xw( (size(Bw,1)*(i-1)+1:size(Bw,1)*i),(size(Bw,2)*(j-1)+1:size(Bw,2)*j) )= Ax^(i-j)*Bw;
            if i == j
                M_yu( (size(Du,1)*(i-1)+1:size(Du,1)*i),(size(Du,2)*(j-1)+1:size(Du,2)*j) )= Du;
                M_yw( (size(Dw,1)*(i-1)+1:size(Dw,1)*i),(size(Dw,2)*(j-1)+1:size(Dw,2)*j) )= Dw;
            else
                M_yu( (size(Du,1)*(i-1)+1:size(Du,1)*i),(size(Du,2)*(j-1)+1:size(Du,2)*j) )= Cx*Ax^(i-j-1)*Bu;
                M_yw( (size(Dw,1)*(i-1)+1:size(Dw,1)*i),(size(Dw,2)*(j-1)+1:size(Dw,2)*j) )= Cx*Ax^(i-j-1)*Bw;
            end
        end
    end
end

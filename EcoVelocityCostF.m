function J=EcoVelocityCostF(U)
    %% -------------------------Data load--------------------------%
    M_xx = evalin('base','M_xx'); M_xu = evalin('base','M_xu'); M_xw = evalin('base','M_xw');
    W_k = evalin('base','W_k');T1 = evalin('base','T1');     
    T2 = evalin('base','T2'); T3 = evalin('base','T3'); T4 = evalin('base','T4');     
    p  = evalin('base','p'); u_k = evalin('base','u_k'); x_k = evalin('base','x_k');
    driv_rect_time_ctrl = evalin('base','driv_rect_time_ctrl'); 
    hw_min_ctrl = evalin('base','hw_min_ctrl'); 
    d_pre_margin = evalin('base','d_pre_margin'); s_pre_p = evalin('base','s_pre_p'); 
    %% J_d：Keeping following
    X_k_1 = M_xx * x_k + M_xu * U + M_xw * W_k;
    d_satisfy = s_pre_p - (driv_rect_time_ctrl*T2+T1)*X_k_1-hw_min_ctrl * ones(p,1);
    J_d = ((d_satisfy(end) - d_pre_margin)^2)/10;
    %% J_delta_a：Softness of acceleration
    J_delta_a = norm((U - u_k),2);
    %% J_fuel_quad (最大值为4)
    V_k = T2*T4*x_k + T2*T3*X_k_1;
    p00 =      0.0603;    p10 =     0.02683;
    p01 =   7.536e-05;    p20 =   -0.001251;
    p11 =   6.772e-05;    p02 =   1.961e-08;
    J_fuel_quad = sum(p00 + p10*V_k + p01*U*1700 + p20.*V_k.^2 + p11.*V_k.*U*1700 + p02.*U.^2*1700^2)/p/4;
    %% J：Cost function
    J = J_d + J_delta_a + 7.5*J_fuel_quad;
end

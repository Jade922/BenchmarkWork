%--------------------Generator/Motor effiency map-------------------------%
%---------------------------------M/G1------------------------------------%
MG1_Elec_power=rpm2rads*MG1_MechanicalLoss_Trq*MG1_MechanicalLoss_Spd/1000;%kw
ind1_pos=find(MG1_MechanicalLoss_Trq>0);% 正转
ind1_neg=find(MG1_MechanicalLoss_Trq<0);% 反转
ind1_zero=setdiff(1:length(MG1_MechanicalLoss_Trq),[ind1_pos,ind1_neg]);

MG1_Output_power(ind1_pos,:)=MG1_Elec_power(ind1_pos,:)-...
    MG1_MechanicalLoss_Matrix(ind1_pos,:);
MG1_Output_power(ind1_neg,:)=MG1_Elec_power(ind1_neg,:)+...
    MG1_MechanicalLoss_Matrix(ind1_neg,:);
MG1_Output_power(ind1_zero,:)=0;
MG1_effiency=100*MG1_Output_power./MG1_Elec_power;%unit:[%]

MG1_MechanicalLoss_MaxTrq=interp1(MG1_MaxMintorque_Spd,...
    MG1_MaxMintorque_MaxTrq,MG1_MechanicalLoss_Spd);%扭矩上限
MG1_MechanicalLoss_MinTrq=interp1(MG1_MaxMintorque_Spd,...
    MG1_MaxMintorque_MinTrq,MG1_MechanicalLoss_Spd);%扭矩下限
for j=1:length(MG1_MechanicalLoss_Spd)
    Ind1_eff=find(MG1_MechanicalLoss_Trq<MG1_MechanicalLoss_MaxTrq(j) &...
        MG1_MechanicalLoss_Trq>MG1_MechanicalLoss_MinTrq(j));
    Ind1_ineff=setdiff(1:length(MG1_MechanicalLoss_Trq),Ind1_eff);
    MG1_effiency(Ind1_ineff,j)=nan;
end
% [~,ind2]=max(abs(MG1_effiency),[],1);
% MG1_trq_opt=MG1_MechanicalLoss_Trq(ind2);%optimal torque value


%%
%--------------------------------M/G2-------------------------------------%
MG2_Elec_power=rpm2rads*MG2_MechanicalLoss_Trq*MG2_MechanicalLoss_Spd/1000;%kw

ind2_pos=find(MG2_MechanicalLoss_Trq>0);% 正转
ind2_neg=find(MG2_MechanicalLoss_Trq<0);% 反转
ind2_zero=setdiff(1:length(MG2_MechanicalLoss_Trq),[ind2_pos,ind2_neg]);

MG2_Output_power(ind2_pos,:)=MG2_Elec_power(ind2_pos,:)-...
    MG2_MechanicalLoss_Matrix(ind2_pos,:);
MG2_Output_power(ind2_neg,:)=MG2_Elec_power(ind2_neg,:)+...
    MG2_MechanicalLoss_Matrix(ind2_neg,:);
MG2_Output_power(ind2_zero,:)=0;
MG2_effiency=100*MG2_Output_power./MG2_Elec_power;%unit:[%]
% [~,ind3]=max(abs(MG2_effiency),[],1);
% MG2_trq_opt=MG2_MechanicalLoss_Trq(ind3);%optimal torque value

MG2_MechanicalLoss_MaxTrq=interp1(MG2_MaxMintorque_Spd,...
    MG2_MaxMintorque_MaxTrq,MG2_MechanicalLoss_Spd);%扭矩上限
MG2_MechanicalLoss_MinTrq=interp1(MG2_MaxMintorque_Spd,...
    MG2_MaxMintorque_MinTrq,MG2_MechanicalLoss_Spd);%扭矩下限
for k=1:length(MG2_MechanicalLoss_Spd)
    Ind2_eff=find(MG2_MechanicalLoss_Trq<MG2_MechanicalLoss_MaxTrq(k) &...
        MG2_MechanicalLoss_Trq>MG2_MechanicalLoss_MinTrq(k));
    Ind2_ineff=setdiff(1:length(MG2_MechanicalLoss_Trq),Ind2_eff);
    MG2_effiency(Ind2_ineff,k)=nan;
end

%% 
figure('Name','Plot the MG1&2','Color','white');
subplot(2,1,1)%MG1
[C2,h4]=contour(MG1_MechanicalLoss_Spd,MG1_MechanicalLoss_Trq,...
    MG1_effiency,[70,75,80,85,88,90:1:94]);%Effiency line
clabel(C2,h4);
hold on
h5=plot(MG1_MaxMintorque_Spd,MG1_MaxMintorque_MaxTrq,'-r','linewidth',2);%Maxtorque line
% h6=plot(MG1_MaxMintorque_Spd,MG1_MaxMintorque_MinTrq,'-b','linewidth',2);%Mintorque line
% legend([h4,h5,h6],'Effiency','$T_{M/G1,max}$','$T_{M/G1,min}$');
legend([h4,h5],'Effiency','$T_{M/G1,max}$');
set(legend,'Interpreter','Latex');
xlim([0 11000]);
ylim([0 45]);
xlabel('M/G1 speed $[rpm]$','Interpreter','Latex');
ylabel('M/G1 torque $[Nm]$','Interpreter','Latex');
subplot(2,1,2)%MG2
[C3,h7]=contour(MG2_MechanicalLoss_Spd,MG2_MechanicalLoss_Trq,...
    MG2_effiency,[70,85,90:1:95]);%Effiency line
clabel(C3,h7);
hold on
h8=plot(MG2_MaxMintorque_Spd,MG2_MaxMintorque_MaxTrq,'-r','linewidth',2);%Maxtorque line
% h9=plot(MG2_MaxMintorque_Spd,MG2_MaxMintorque_MinTrq,'-b','linewidth',2);%Mintorque line
% legend([h7,h8,h9],'Effiency','$T_{M/G2,max}$','$T_{M/G2,min}$');
legend([h7,h8],'Effiency','$T_{M/G2,max}$');
set(legend,'Interpreter','Latex');
xlim([0 17500]);
ylim([0 165]);
xlabel('M/G2 speed $[rpm]$','Interpreter','Latex');
ylabel('M/G2 torque $[Nm]$','Interpreter','Latex');


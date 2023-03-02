clear 
clc

% Author: Mohamad T. Shahab
% developed with help from Obadah Wali, 12 Sep 2022
% Run on MATLAB 2022a
% The Simulink models are based, in part, on examples from UAV Toolbox Support Package for PX4 Autopilots


%% =======

rotorTorqueMax=0.078;
rotorThrustMax=5.3;
rotorTimeConst=0.005;
dragCoeffMov=0;
windVel=[0;0;0];
g=9.81;

act_up=1000;
act_lo=0;



Ixx1=7.5*10^(-3);
Iyy1=7.3*10^(-3);
Izz1=6.8*10^(-3);



inertia1=[Ixx1 0 0;
    0 Iyy1 0;
    0 0 Izz1];

mass1=0.89*1;



SampleTime=0.01;

% L=.11;
L=.355*1;
% l=.103;
l=sqrt(3)*L/6;
h=.3;
Hz=-0.08;

theta=45;
H=h*sin(rad2deg(theta));
ang=pi/3;


Ilm=L*L*mass1;


% 




%% ===== initialization
stoptime=5;


deg_d=1;

rad_d=deg2rad(deg_d);

rad_d=0.01;

init.euler=[0*rad_d;1*rad_d;0];
init.angRates=[0;0;0];
init.posNED=[0;0;2];
init.vb=[0;0;0];

des_r=0;
des_p=0;
des_y=0;

des_alt=1;

KP1x=6.5;
KP2x=0.15^1;
KIx=0.2*1;
KDx=0.003*1;

KP1y=6.5;
KP2y=0.15^1;
KIy=0.2*1;
KDy=0.003*1;

KP1z=2.8;
KP2z=0.15^1;
KIz=0.2*1;
KDz=0.003*1;


%% =====

figure(1)
% figure(2)

for nn=[0,1,2]
% nn=0;
% stoptime=5;


% if nn==0
% 
% stoptime=2;
% end

NumRotors = 4^(nn+1);

%% =======

%positions in NED (right-hand rule etc.)

P1_1=[0;0;0];
P2_1=[l;0;0];
P3_1=[-l*cos(ang);l*sin(ang);0];
P4_1=[-l*cos(ang);-l*sin(ang);0];

rotorDirX=[1 -1 -1 1];

rotorDir=rotorDirX;
% for ii=1:(NumRotors/4)-1
% rotorDir=[rotorDir 1 -1 -1 1];
% end


RDx=rotorDirX;


%% ========

rotorDirX=[1 -1 -1 1];

rotorPositions1=[P1_1 P2_1 P3_1 P4_1];

%% ====

M=[1 1 1 1;
    [P1_1(2) P2_1(2) P3_1(2) P4_1(2)];
    -1*[P1_1(1) P2_1(1) P3_1(1) P4_1(1)];
    RDx];

Mv=inv(M);



%% ======= four tetracopters

T_1p_f = [l+2*L/3;0;-H];
T_1p_c = [2*L/3;0;-H];
T_1p_r = [2*L/3-l*cos(ang);l*sin(ang);-H];
T_1p_l = [2*L/3-l*cos(ang);-l*sin(ang);-H];

T_2p_c = [0;0;0];
T_2p_f = [l;0;0];
T_2p_r = [-l*cos(ang);l*sin(ang);0];
T_2p_l = [-l*cos(ang);-l*sin(ang);0];

T_3p_f = [l-L/3;2*L/3*cos(pi/6);-H];
T_3p_c = [-L/3;2*L/3*cos(pi/6);-H];
T_3p_r = [-L/3-l*cos(ang);2*L/3*cos(pi/6)+l*sin(ang);-H];
T_3p_l = [-L/3-l*cos(ang);2*L/3*cos(pi/6)-l*sin(ang);-H];

T_4p_f = [l-L/3;-2*L/3*cos(pi/6);-H];
T_4p_c = [-L/3;-2*L/3*cos(pi/6);-H];
T_4p_r = [-L/3-l*cos(ang);-2*L/3*cos(pi/6)+l*sin(ang);-H];
T_4p_l = [-L/3-l*cos(ang);-2*L/3*cos(pi/6)-l*sin(ang);-H];


rotorPositions4=[T_1p_f,T_1p_c,T_1p_r,T_1p_l,T_2p_c,T_2p_f,T_2p_r,T_2p_l,T_3p_f,T_3p_c,T_3p_r,T_3p_l,T_4p_f,T_4p_c,T_4p_r,T_4p_l];

Ixx4=4*Ixx1+Ilm*13/12;
Iyy4=4*Iyy1+Ilm*13/12;
Izz4=4*Izz1+Ilm*1;

inertia4=diag([Ixx4,Iyy4,Izz4]);


%% sixteen tetracopters
T_11p_f = [0 l+2*L/3+4*l -3*H];
T_11p_c = [0 2*L/3+4*l -3*H];
T_11p_r = [l*sin(ang) 2*L/3-l*cos(ang)+4*l -3*H];
T_11p_l = [-l*sin(ang) 2*L/3-l*cos(ang)+4*l -3*H];

T_12p_f = [0 l+4*l -2*H];
T_12p_c = [0 0+4*l -2*H];
T_12p_r = [l*sin(ang) -l*cos(ang)+4*l -2*H];
T_12p_l = [-l*sin(ang) -l*cos(ang)+4*l -2*H];

T_13p_f = [2*L/3*cos(pi/6) l-L/3+4*l -3*H];
T_13p_c = [2*L/3*cos(pi/6) -L/3+4*l -3*H];
T_13p_r = [2*L/3*cos(pi/6)+l*sin(ang) -L/3-l*cos(ang)+4*l -3*H];
T_13p_l = [2*L/3*cos(pi/6)-l*sin(ang) -L/3-l*cos(ang)+4*l -3*H];

T_14p_f = [-2*L/3*cos(pi/6) l-L/3+4*l -3*H];
T_14p_c = [-2*L/3*cos(pi/6) -L/3+4*l -3*H];
T_14p_r = [-2*L/3*cos(pi/6)+l*sin(ang) -L/3-l*cos(ang)+4*l -3*H];
T_14p_l = [-2*L/3*cos(pi/6)-l*sin(ang) -L/3-l*cos(ang)+4*l -3*H];

T_21p_f = [0 l+2*L/3 -H];
T_21p_c = [0 2*L/3 -H];
T_21p_r = [l*sin(ang) 2*L/3-l*cos(ang) -H];
T_21p_l = [-l*sin(ang) 2*L/3-l*cos(ang) -H];

T_22p_f = [0 l 0];
T_22p_c = [0 0 0];
T_22p_r = [l*sin(ang) -l*cos(ang) 0];
T_22p_l = [-l*sin(ang) -l*cos(ang) 0];

T_23p_f = [2*L/3*cos(pi/6) l-L/3 -H];
T_23p_c = [2*L/3*cos(pi/6) -L/3 -H];
T_23p_r = [2*L/3*cos(pi/6)+l*sin(ang) -L/3-l*cos(ang) -H];
T_23p_l = [2*L/3*cos(pi/6)-l*sin(ang) -L/3-l*cos(ang) -H];

T_24p_f = [-2*L/3*cos(pi/6) l-L/3 -H];
T_24p_c = [-2*L/3*cos(pi/6) -L/3 -H];
T_24p_r = [-2*L/3*cos(pi/6)+l*sin(ang) -L/3-l*cos(ang) -H];
T_24p_l = [-2*L/3*cos(pi/6)-l*sin(ang) -L/3-l*cos(ang) -H];

T_31p_f = [0+4/3*L*sin(ang) l+2*L/3-2*l -3*H];
T_31p_c = [0 2*L/3-2*l+4/3*L*sin(ang) -3*H];
T_31p_r = [l*sin(ang)+4/3*L*sin(ang) 2*L/3-l*cos(ang)-2*l -3*H];
T_31p_l = [-l*sin(ang)+4/3*L*sin(ang) 2*L/3-l*cos(ang)-2*l -3*H];

T_32p_f = [0+4/3*L*sin(ang) l-2*l -2*H];
T_32p_c = [0+4/3*L*sin(ang) 0-2*l -2*H];
T_32p_r = [l*sin(ang)+4/3*L*sin(ang) -l*cos(ang)-2*l -2*H];
T_32p_l = [-l*sin(ang)+4/3*L*sin(ang) -l*cos(ang)-2*l -2*H];

T_33p_f = [2*L/3*cos(pi/6)+4/3*L*sin(ang) l-L/3-2*l -3*H];
T_33p_c = [2*L/3*cos(pi/6)+4/3*L*sin(ang) -L/3-2*l -3*H];
T_33p_r = [2*L/3*cos(pi/6)+l*sin(ang)+4/3*L*sin(ang) -L/3-l*cos(ang)-2*l -3*H];
T_33p_l = [2*L/3*cos(pi/6)-l*sin(ang)+4/3*L*sin(ang) -L/3-l*cos(ang)-2*l -3*H];

T_34p_f = [-2*L/3*cos(pi/6)+4/3*L*sin(ang) l-L/3-2*l -3*H];
T_34p_c = [-2*L/3*cos(pi/6)+4/3*L*sin(ang) -L/3-2*l -3*H];
T_34p_r = [-2*L/3*cos(pi/6)+l*sin(ang)+4/3*L*sin(ang) -L/3-l*cos(ang)-2*l -3*H];
T_34p_l = [-2*L/3*cos(pi/6)-l*sin(ang)+4/3*L*sin(ang) -L/3-l*cos(ang)-2*l -3*H];

T_41p_f = [0-4/3*L*sin(ang) l+2*L/3-2*l -3*H];
T_41p_c = [0-4/3*L*sin(ang) 2*L/3-2*l -3*H];
T_41p_r = [l*sin(ang)-4/3*L*sin(ang) 2*L/3-l*cos(ang)-2*l -3*H];
T_41p_l = [-l*sin(ang)-4/3*L*sin(ang) 2*L/3-l*cos(ang)-2*l -3*H];

T_42p_f = [0-4/3*L*sin(ang) l-2*l -2*H];
T_42p_c = [0-4/3*L*sin(ang) 0-2*l -2*H];
T_42p_r = [l*sin(ang)-4/3*L*sin(ang) -l*cos(ang)-2*l -2*H];
T_42p_l = [-l*sin(ang)-4/3*L*sin(ang) -l*cos(ang)-2*l -2*H];

T_43p_f = [2*L/3*cos(pi/6)-4/3*L*sin(ang) l-L/3-2*l -3*H];
T_43p_c = [2*L/3*cos(pi/6)-4/3*L*sin(ang) -L/3-2*l -3*H];
T_43p_r = [2*L/3*cos(pi/6)+l*sin(ang)-4/3*L*sin(ang) -L/3-l*cos(ang)-2*l -3*H];
T_43p_l = [2*L/3*cos(pi/6)-l*sin(ang)-4/3*L*sin(ang) -L/3-l*cos(ang)-2*l -3*H];

T_44p_f = [-2*L/3*cos(pi/6)-4/3*L*sin(ang) l-L/3-2*l -3*H];
T_44p_c = [-2*L/3*cos(pi/6)-4/3*L*sin(ang) -L/3-2*l -3*H];
T_44p_r = [-2*L/3*cos(pi/6)+l*sin(ang)-4/3*L*sin(ang) -L/3-l*cos(ang)-2*l -3*H];
T_44p_l = [-2*L/3*cos(pi/6)-l*sin(ang)-4/3*L*sin(ang) -L/3-l*cos(ang)-2*l -3*H];

RP4=[T_41p_f;T_41p_c;T_41p_r;T_41p_l;T_42p_c;T_42p_f;T_42p_r;T_42p_l;T_43p_f;T_43p_c;T_43p_r;T_43p_l;T_44p_f;T_44p_c;T_44p_r;T_44p_l];
RP3=[T_31p_f;T_31p_c;T_31p_r;T_31p_l;T_32p_c;T_32p_f;T_32p_r;T_32p_l;T_33p_f;T_33p_c;T_33p_r;T_33p_l;T_34p_f;T_34p_c;T_34p_r;T_34p_l];
RP2=[T_21p_f;T_21p_c;T_21p_r;T_21p_l;T_22p_c;T_22p_f;T_22p_r;T_22p_l;T_23p_f;T_23p_c;T_23p_r;T_23p_l;T_24p_f;T_24p_c;T_24p_r;T_24p_l];
RP1=[T_11p_f;T_11p_c;T_11p_r;T_11p_l;T_12p_c;T_12p_f;T_12p_r;T_12p_l;T_13p_f;T_13p_c;T_13p_r;T_13p_l;T_14p_f;T_14p_c;T_14p_r;T_14p_l];

RP_16=[RP1;RP2;RP3;RP4];

RP_16=[RP_16(:,2) RP_16(:,1) RP_16(:,3)]';
rotorPositions16=RP_16;

Ixx16=4*Ixx4+16*Ilm*13/12;
Iyy16=4*Iyy4+16*Ilm*13/12;
Izz16=4*Izz4+16*Ilm*1;

inertia16=diag([Ixx16,Iyy16,Izz16]);


%% ===== selection ===

if nn==0

mass=mass1;
inertia=inertia1;
rotorPositions=rotorPositions1;

model='att_control_OneModule_V6';

elseif nn==1

mass=4*mass1;
inertia=inertia4;
rotorPositions=rotorPositions4;

model='att_control_FourModules_V6';

elseif nn==2

mass=16*mass1;
inertia=inertia16;
rotorPositions=rotorPositions16;

model='att_control_SixteenModules_V6';

end





%% run simulation


load_system(model);
out=sim(model,stoptime);


%% plots

Psz=12;

figure(1)
subplot(311)
plot(out.euler.Time,out.euler.Data(:,3),'LineWidth',1);hold on
xlabel('time (s)')
ylabel('roll (rad)', 'FontSize', Psz)
aaa1 = get(gca,'XTickLabel');  
 set(gca,'XTickLabel',aaa1,'FontSize',Psz)
 xticks([0:.5:stoptime])
% axis([0 out.euler.Time(end) .9*min(out.euler.Data(:,3)) 1.1*max(out.euler.Data(:,3))])
% xbd1=stoptime-[1 0.01];
% %  xbd1=[450 490];
% pos_fig1=[0.6 0.8 0.275 0.075];
% [pp,zz]=zoomPlot_v2(out.euler.Time,out.euler.Data(:,3),xbd1,pos_fig1,[3,1]);hold on
subplot(312)
plot(out.euler.Time,out.euler.Data(:,2),'LineWidth',1);hold on
xlabel('time (s)')
ylabel('pitch (rad)', 'FontSize', Psz)
aaa1 = get(gca,'XTickLabel');  
 set(gca,'XTickLabel',aaa1,'FontSize',Psz)
% axis([0 out.euler.Time(end) .5*min(out.euler.Data(:,2)) 1.1*max(out.euler.Data(:,2))])
 xticks([0:.01:0.03])
xticks([0:.5:stoptime])
subplot(313)
plot(out.position.Time,out.position.Data(:,3),'LineWidth',1);hold on
xlabel('time (s)')
ylabel('altitude (m)', 'FontSize', Psz)
aaa1 = get(gca,'XTickLabel');  
 set(gca,'XTickLabel',aaa1,'FontSize',Psz)
xticks([0:.5:stoptime])
% axis([0 out.position.Time(end) .9*min(out.position.Data(:,3)) 1.1*max(out.position.Data(:,3))])

tsmf=50;



if nn==0

x00=out.euler.Data(:,3)-out.euler.Data(1,3);
x01=x00*(1/x00(end));

stpinfo=stepinfo(x01,out.euler.Time,1,0);
R1=stpinfo.SettlingTime

x00=out.euler.Data(:,2)-out.euler.Data(1,2);
x01=x00*(1/x00(end));

stpinfo=stepinfo(x01,out.euler.Time,1,0);
P1=stpinfo.SettlingTime

x00=out.position.Data(:,3)-out.position.Data(1,3);
x01=x00*(1/x00(end));

stpinfo=stepinfo(x01,out.euler.Time,1,0);
Z1=stpinfo.SettlingTime

elseif nn==1

x00=out.euler.Data(:,3)-out.euler.Data(1,3);
x01=x00*(1/x00(end));

stpinfo=stepinfo(x01,out.euler.Time,1,0);
R4=stpinfo.SettlingTime

x00=out.euler.Data(:,2)-out.euler.Data(1,2);
x01=x00*(1/x00(end));

stpinfo=stepinfo(x01,out.euler.Time,1,0);
P4=stpinfo.SettlingTime

x00=out.position.Data(:,3)-out.position.Data(1,3);
x01=x00*(1/x00(end));

stpinfo=stepinfo(x01,out.euler.Time,1,0);
Z4=stpinfo.SettlingTime

elseif nn==2

x00=out.euler.Data(:,3)-out.euler.Data(1,3);
x01=x00*(1/x00(end));

stpinfo=stepinfo(x01,out.euler.Time,1,0);
R16=stpinfo.SettlingTime

x00=out.euler.Data(:,2)-out.euler.Data(1,2);
x01=x00*(1/x00(end));

stpinfo=stepinfo(x01,out.euler.Time,1,0);
P16=stpinfo.SettlingTime

x00=out.position.Data(:,3)-out.position.Data(1,3);
x01=x00*(1/x00(end));

stpinfo=stepinfo(x01,out.euler.Time,1,0);
Z16=stpinfo.SettlingTime


end


end


hold off


hold off




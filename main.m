%% Cessna Ce500 Citation 1 (landing configuration)
close all; clc, clearvars;

%Aircraft Flight Condition Variables
x_cg = 0.3; % [$\bar{c}$]
W = 44675; % [N]
m = 4556; % [kg]
S = 24.2; % [m^2]
c = 2.022; % [m]
b = 13.36; % [m]

V = 51.4; % [m/s]
h = 0; % [m]
rho = 1.225; % [kg/m^3]
g = 9.807; %[m/s]

%Stability & Control Variables 
mu_c = 76;
twmu_c = 2*mu_c;
mu_b = 11;

K_X_2 = 0.012;
K_Z_2 = 0.037;
K_XZ = 0.002;
K_Y_2 = 0.98;

C_X_0 = 0;              C_Z_0 = -1.1360;        
C_X_u = -0.2173;        C_Z_u = -2.2720;        C_m_u = 0;
C_X_a = 0.4692;         C_Z_a = -5.1300;        C_m_a = -0.4000;
C_X_q = 0;              C_Z_q = -3.8400;        C_m_q = -7.3500;
C_X_d = 0;              C_Z_d = 0.6238;         C_m_d = -1.5530;
                        C_Z_da = 1.4050;        C_m_da = -3.6150;

                    
C_Y_b = -0.9896;        C_l_b = -0.0772;        C_n_b = 0.1628;
C_Y_p = -0.0870;        C_l_p = -0.3415;        C_n_p = -0.0108;
C_Y_r = 0.4300;         C_l_r = 0.2830;         C_n_r = -0.1930;
C_Y_d_a = 0;            C_l_d_a = -0.2349;      C_n_d_a = 0.0286;
C_Y_d_r = 0.3037;       C_l_d_r = 0.0286;       C_n_d_r = -0.1261;

%% Turbulence Parameters
sgm_w_g = 1; % [m/s]
L_g = 150; % [m]
sgm_u_g = sgm_w_g/V;
sgm_a_g = sgm_w_g/V;

%% Symmetric Gust Derivatives
%Assumptions
C_m_ac = 0; %C_m about the Aerodynamic Centre of the Wing plus Fuselage for C_L_w = 0
C_m_h = 0; %Contribution of the Horizontal Tailplane to C_m
lh = 0; % tailength [m] 

C_X_u_g = C_X_u;
C_X_a_g = C_X_a;

C_Z_u_g = C_Z_u;
C_Z_du_g = 2*C_m_ac;
C_Z_a_g = C_Z_a;
C_Z_da_g = C_Z_da-C_Z_q;

C_m_u_g = C_m_u;
C_m_du_g = -2*C_m_h*(lh/c);
C_m_a_g = C_m_a;
C_m_da_g = C_m_da-C_m_q;

%% Aircraft Symmetric Stability Derivatives
x_u = (V/c)*(C_X_u/twmu_c);
x_a = (V/c)*(C_X_a/twmu_c);
x_t = (V/c)*(C_Z_0/twmu_c);
x_u_g = (V/c)*(C_X_u_g/twmu_c);
x_a_g = (V/c)*(C_X_a_g/twmu_c);
x_d_e = (V/c)*(C_X_d/twmu_c);

z_u = (V/c)*(C_Z_u/(twmu_c-C_Z_da));
z_a = (V/c)*(C_Z_a/(twmu_c-C_Z_da));
z_t = (V/c)*(-C_X_0/(twmu_c-C_Z_da));
z_q = (V/c)*(twmu_c+C_Z_q)/(twmu_c-C_Z_da);
z_u_g = (V/c)*(C_Z_u_g/(twmu_c-C_Z_da));
z_du_g = (V/c)*(C_Z_du_g/(twmu_c-C_Z_da));
z_a_g = (V/c)*(C_Z_a_g/(twmu_c-C_Z_da));
z_da_g = (V/c)*(C_Z_da_g/(twmu_c-C_Z_da));
z_d_e = (V/c)*(C_Z_d/(twmu_c-C_Z_da));

m_u = (V/c)*((C_m_u+C_Z_u*C_m_da/(twmu_c-C_Z_da))/(twmu_c*K_Y_2));
m_a = (V/c)*((C_m_a+C_Z_a*C_m_da/(twmu_c-C_Z_da))/(twmu_c*K_Y_2));
m_t = (V/c)*((-C_X_0*C_m_da/(twmu_c-C_Z_da))/(twmu_c*K_Y_2));
m_q = (V/c)*(C_m_q+C_m_da*(twmu_c+C_Z_q)/(twmu_c-C_Z_da))/(twmu_c*K_Y_2);
m_u_g = (V/c)*((C_m_u_g+C_Z_u_g*C_m_da/(twmu_c-C_Z_da))/(twmu_c*K_Y_2));
m_du_g = (V/c)*((C_m_du_g+C_Z_du_g*C_m_da/(twmu_c-C_Z_da))/(twmu_c*K_Y_2));
m_a_g = (V/c)*((C_m_a_g+C_Z_a_g*C_m_da/(twmu_c-C_Z_da))/(twmu_c*K_Y_2));
m_da_g = (V/c)*((C_m_da_g+C_Z_da_g*C_m_da/(twmu_c-C_Z_da))/(twmu_c*K_Y_2));
m_d_e = (V/c)*((C_m_d+C_Z_d*C_m_da/(twmu_c-C_Z_da))/(twmu_c*K_Y_2));

%% State Space System
A = [x_u,  x_a,  x_t,  0,    x_u_g,                       x_a_g,      0;
     z_u,  z_a,  z_t,  z_q,  z_u_g-z_du_g*(V/L_g)*(c/V),  z_a_g,      z_da_g*(c/V);
     0,    0,    0,    V/c,  0,                           0,          0;
     m_u,  m_a,  m_t,  m_q,  m_u_g-m_du_g*(V/L_g)*(c/V),  m_a_g,      m_da_g*(c/V);
     0,    0,    0,    0,    -V/L_g,                      0,          0;
     0,    0,    0,    0,    0,                           0,          1;
     0,    0,    0,    0,    0,                           -V^2/L_g^2, -2*V/L_g];
     
B = [x_d_e,   0,                                   0;
     z_d_e,   z_du_g*(c/V)*sgm_u_g*sqrt(2*V/L_g),  z_da_g*(c/V)*sgm_a_g*sqrt(3*V/L_g);
     0,       0,                                   0;
     m_d_e,   m_du_g*(c/V)*sgm_u_g*sqrt(2*V/L_g),  m_da_g*(c/V)*sgm_a_g*sqrt(3*V/L_g);
     0,       sgm_u_g*sqrt(2*V/L_g),               0;
     0,       0,                                   sgm_a_g*sqrt(3*V/L_g);
     0,       0,                                   (1-2*sqrt(3))*sgm_a_g*sqrt((V/L_g)^3)];
  
C = eye(7);

D = zeros(7,3);

ssce500 = ss(A,B,C,D);

% For trimming simulink model
% sim('ce500');
% set_param('ce500/State-Space','A','A','B','B','C','C','D','D');

%% Stability Analysis

%% Pole-Zero Map
figure
pzmap(ssce500), grid on;

%Pole Analysis
[wn,zeta,poles] = damp(ssce500);
%Phugoid Poles
ph = [poles(1:2),wn(1:2),zeta(1:2)];

figure
rlocus(-ssce500(3,1)), grid on;

%% Pitch Damper Autopilot
%Gain Requiered to achieve Zeta = 0.5 for Phugoid Poles
K_t = -0.111;
K = [0,0,K_t,0,0,0,0];
A_fb = A-B(:,1)*K;
ssce500ap = ss(A_fb, B, C, D);

figure
pzmap(ssce500, ssce500ap), grid on; 
%Pole Analysis with Feedback
[wn_ap,zeta_ap,poles_ap] = damp(ssce500ap);
%Phugoid Poles with Feedback
ph_ap = [poles_ap(1:2),wn_ap(1:2),zeta_ap(1:2)];

%% Phugoid Analysis

%trim point from simulink model
% load('trimpoint.mat');
% x0 = op1.states.x;
% u0 = [op1.inputs(1).u,op1.inputs(2).u,op1.inputs(3).u];
x0 = [51.4000;-22.7643;-19.2344;0.0000;-0.0000;0.0000;0.0000];
u0 = [5.8073, 0, 0];

%Setting up time-scale
dt_ph = 0.01; %[s]
T_ph = 300; %[s]
t_ph = [0:dt_ph:T_ph];
N_ph = length(t_ph);

yph = -impulse(ssce500(:,1),t_ph);
yph_ap = -impulse(ssce500ap(:,1),t_ph);

%% Plotting Aircraft Phugoid Response
figure
sgtitle('Aircraft Phugoid Response');

subplot(2,2,1)
plot(t_ph,x0(1)+yph(:,1),'b--'), grid on, hold on;
plot(t_ph,x0(1)+yph_ap(:,1),'r')
title('Velocity');
xlabel('$t, [s]$','Interpreter','latex','FontSize',14); 
ylabel('$\hat{u}, \left[\frac{m}{s}\right]$','Interpreter','latex','FontSize',14);
legend('Ce500','Ce500 AP');

subplot(2,2,2)
plot(t_ph,-x0(2)+yph(:,2),'b--'), grid on, hold on;
plot(t_ph,-x0(2)+yph_ap(:,2),'r')
title('Angle of Attack');
xlabel('$t, [s]$','Interpreter','latex','FontSize',14); 
ylabel('$\alpha, [^\circ]$','Interpreter','latex','FontSize',14);

subplot(2,2,3)
plot(t_ph,-x0(3)+yph(:,3),'b--'), grid on, hold on;
plot(t_ph,-x0(3)+yph_ap(:,3),'r')
title('Pitch Angle');
xlabel('$t, [s]$','Interpreter','latex','FontSize',14); 
ylabel('$\theta, [^\circ]$','Interpreter','latex','FontSize',14);

subplot(2,2,4)
plot(t_ph,-x0(4)+yph(:,4),'b--'), grid on, hold on;
plot(t_ph,-x0(4)+yph_ap(:,4),'r')
title('Pitch Rate');
xlabel('$t, [s]$','Interpreter','latex','FontSize',14); 
ylabel(' $ \frac{q\bar{c}}{V}, \left[\frac{\circ}{s}\right] $','Interpreter','latex','FontSize',14);


%% Time Domain Analysis

%% Load Factor
C = [C;
    -(V/g)*z_u, -(V/g)*z_a, -(V/g)*z_t, (V/g)*((V/c)-z_q), -(V/g)*(z_u_g-z_du_g*(V/L_g)*(c/V)), -(V/g)*z_a_g, -(V/g)*z_da_g*(c/V)];
D = [D;
    z_d_e, z_du_g*(c/V)*sgm_u_g*sqrt(2*V/L_g), z_da_g*(c/V)*sgm_a_g*sqrt(3*V/L_g)];
    
ssce500 = ss(A,B,C,D);
ssce500ap = ss(A_fb, B, C, D);

%% Atmospheric Turbulence Simulation
%Setting up time-scale
dt = 0.05; %[s]
fs = 1/dt; %[Hz]
T = 200; %[s]
t = [0:dt:T]; % time axis
N = length(t); % number of samples

Nf = 500; % number of points in frequency axis 
w = logspace(-2,2,Nf); 

fres = fs/N; % frequency resolution in Hz
f = fres*[1:1:N/2]; % frequency axis , in Hz

% DEFINE FREQUENCY VECTOR FOR PLOTTING
omega = 2*pi*fs*(0:(N/2)-1)/N; % frequency axis , in rad/s

%Aircraft Inputs
d_e = zeros(1,N);
w1 = zeros(1,N);
w3 = randn(1,N)/sqrt(dt);
u = [d_e; w1; w3];

%Simulation
y = lsim(ssce500,u,t);
y_ap = lsim(ssce500ap,u,t);

uv = y(:,1); uv_ap = y_ap(:,1);
alp = y(:,2); alp_ap = y_ap(:,2);
th = y(:,3); th_ap = y_ap(:,3);
q = y(:,4); q_ap = y_ap(:,4);
nz = y(:,8); nz_ap = y_ap(:,8);

%% Ploting Simulation Results without AP
figure;
sgtitle('Aircraft Turbulence Response');

subplot(3,2,1)
plot(t,uv,'b--'); hold on;
plot(t,uv_ap,'r'); grid on;
title('Velocity');
xlabel('$t, [s]$','Interpreter','latex','FontSize',14); 
ylabel(' $ \frac{\hat{u}}{V},  [-]$','Interpreter','latex','FontSize',14);
legend('Uncontrolled Aircraft','Controlled Aircraft');

subplot(3,2,2)
plot(t,alp*180/pi,'b--'); hold on;
plot(t,alp_ap*180/pi,'r'); grid on;
title('Angle of Attack');
xlabel('$t, [s]$','Interpreter','latex','FontSize',14); 
ylabel('$\alpha, [^\circ]$','Interpreter','latex','FontSize',14);

subplot(3,2,3)
plot(t,th*180/pi,'b--'); hold on;
plot(t,th_ap*180/pi,'r'); grid on;
title('Pitch Angle');
xlabel('$t, [s]$','Interpreter','latex','FontSize',14); 
ylabel('$\theta, [^\circ]$','Interpreter','latex','FontSize',14);

subplot(3,2,4)
plot(t,q*180/pi,'b--'); hold on;
plot(t,q_ap*180/pi,'r'); grid on;
title('Pitch Rate');
xlabel('$t, [s]$','Interpreter','latex','FontSize',14); 
ylabel(' $ \frac{q\bar{c}}{V}, \left[\frac{\circ}{s}\right] $','Interpreter','latex','FontSize',14);

subplot(3,2,5)
plot(t,nz,'b--'); hold on;
plot(t,nz_ap,'r'); grid on;
title('Load Factor');
xlabel('$t, [s]$','Interpreter','latex','FontSize',14); 
ylabel('$n_{z}, [-]$','Interpreter','latex','FontSize',14);

%% Analytical Power Spectral Densities

mag = bode(A,B,C,D,3,w);

Suu = mag(:,1).^2;
Saa = mag(:,2).^2;
Stt = mag(:,3).^2;
Sqq = mag(:,4).^2;
Snn = mag(:,8).^2;

mag_ap = bode(A_fb,B,C,D,3,w);
Suu_ap = mag_ap(:,1).^2;
Saa_ap = mag_ap(:,2).^2;
Stt_ap = mag_ap(:,3).^2;
Sqq_ap = mag_ap(:,4).^2;
Snn_ap = mag_ap(:,8).^2;


%% Calculating Fast Fourier Transform

% Ce500 State Variables
U = dt*fft(uv);
ALPHA = dt*fft(alp);
THETA = dt*fft(th);
Q = dt*fft(q);
NZ = dt*fft(nz);

% Ce500 Autopilot State Variables
U_AP = dt *fft(uv_ap);
ALPHA_AP = dt*fft(alp_ap);
THETA_AP = dt*fft(th_ap);
Q_AP = dt*fft(q_ap);
NZ_AP = dt*fft(nz_ap);

%% Calculating Periodograms
% Ce500 State Variables
Pu = real((1/T)*U.*conj(U));
Palpha = real((1/T)*ALPHA.*conj(ALPHA));
Ptheta = real((1/T)*THETA.*conj(THETA));
Pq = real((1/T)*Q.*conj(Q));
Pnz = real((1/T)*NZ.*conj(NZ));

% Ce500 Autopilot State Variables
Pu_ap = real((1/T)*U_AP.*conj(U_AP));
Palpha_ap = real((1/T)*ALPHA_AP.*conj(ALPHA_AP));
Ptheta_ap = real((1/T)*THETA_AP.*conj(THETA_AP));
Pq_ap = real((1/T)*Q_AP.*conj(Q_AP));
Pnz_ap = real((1/T)*NZ_AP.*conj(NZ_AP));

%% Calculating Smooth Periodograms using Pwelch
% Ce500 State Variables
Swlch = pwelch(y,1400,200,N,fs,'onesided');
Swlch = Swlch/2; % adjust power for negative frequencies
Swlch = Swlch(2:floor(N/2+1),:); % get rid of the zero frequency, to align

% Ce500 Autopilot State Variables
Swlch_ap = pwelch(y_ap,1400,200,N,fs,'onesided');
Swlch_ap = Swlch_ap/2; Swlch_ap = Swlch_ap(2:floor(N/2+1),:); 


%% Analytical  PSD
figure;
sgtitle('Analytical Power Spectral Densities');

subplot(3,2,1);
loglog(w,Suu,'b--'); hold on;
loglog(w,Suu_ap,'r'); grid on;
title('Velocity');
axis(10.^[-2,2,-15,0]);
xlabel('$\omega, [rad/s]$','Interpreter','latex','FontSize',14); 
ylabel('$ S_{\hat{u} \hat{u}}, [ \frac{rad^2}{rad/s} ] $','Interpreter','latex','FontSize',14);
legend('Uncontrolled Aircraft','Controlled Aircraft');

subplot(3,2,2);
loglog(w,Saa,'b--'); hold on;
loglog(w,Saa_ap,'r'); grid on;
title('Angle of Attack');
axis(10.^[-2,2,-15,0]);
xlabel('$\omega, [rad/s]$','Interpreter','latex','FontSize',14); 
ylabel('$ S_{\alpha \alpha}, [ \frac{rad^2}{rad/s} ] $','Interpreter','latex','FontSize',14)

subplot(3,2,3);
loglog(w,Stt,'b--'); hold on;
loglog(w,Stt_ap,'r'); grid on;
title('Pitch Angle');
axis(10.^[-2,2,-15,0]);
xlabel('$\omega, [rad/s]$','Interpreter','latex','FontSize',14); 
ylabel('$ S_{\theta \theta}, [ \frac{rad^2}{rad/s} ] $','Interpreter','latex','FontSize',14);

subplot(3,2,4);
loglog(w,Sqq,'b--'); hold on;
loglog(w,Sqq_ap,'r'); grid on;
title('Pitch Rate');
axis(10.^[-2,2,-20,-5]);
xlabel('$\omega, [rad/s]$','Interpreter','latex','FontSize',14); 
ylabel('$ S_{qq}, [ \frac{rad^2}{rad/s} ] $','Interpreter','latex','FontSize',14);

subplot(3,2,5);
loglog(w,Snn,'b--'); hold on;
loglog(w,Snn_ap,'r'); grid on;
title('Load Factor');
axis(10.^[-2,2,-10,0]);
xlabel('$\omega, [rad/s]$','Interpreter','latex','FontSize',14); 
ylabel('$ S_{n_{z}n_{z}} $','Interpreter','latex','FontSize',14);

%% Analytical and Pwelch Periodograms
figure;

subplot(5,2,1)
loglog(w,Suu,'r'); hold on;
loglog(omega,Swlch(:,1),'b--'); grid on;
axis(10.^[-2,2,-15,0]);
title('Uncontrolled Aircraft Velocity');
xlabel('$\omega, [rad/s]$','Interpreter','latex','FontSize',14); 
ylabel('$ S_{\hat{u} \hat{u}}, [ \frac{rad^2}{rad/s} ] $','Interpreter','latex','FontSize',14);
legend('Analytical','Periodogram (pwelch)');

subplot(5,2,2)
loglog(w,Suu_ap,'r'); hold on;
loglog(omega,Swlch_ap(:,1),'b--'); grid on;
axis(10.^[-2,2,-15,0]);
title('Controlled Aircraft Velocity');
xlabel('$\omega, [rad/s]$','Interpreter','latex','FontSize',14); 
ylabel('$ S_{\hat{u} \hat{u}}, [ \frac{rad^2}{rad/s} ] $','Interpreter','latex','FontSize',14);

subplot(5,2,3);
loglog(w,Saa,'r'); hold on;
loglog(omega,Swlch(:,2),'b--'); grid on;
axis(10.^[-2,2,-15,0]);
title('Uncontrolled Aircraft Angle of Attack');
xlabel('$\omega, [rad/s]$','Interpreter','latex','FontSize',14); 
ylabel('$ S_{\alpha \alpha}, [ \frac{rad^2}{rad/s} ] $','Interpreter','latex','FontSize',14)

subplot(5,2,4);
loglog(w,Saa_ap,'r'); hold on;
loglog(omega,Swlch_ap(:,2),'b--'); grid on;
axis(10.^[-2,2,-15,0]);
title('Controlled Aircraft Angle of Attack');
xlabel('$\omega, [rad/s]$','Interpreter','latex','FontSize',14); 
ylabel('$ S_{\alpha \alpha}, [ \frac{rad^2}{rad/s} ] $','Interpreter','latex','FontSize',14)

subplot(5,2,5);
loglog(w,Stt,'r'); hold on;
loglog(omega,Swlch(:,3),'b--'); grid on;
axis(10.^[-2,2,-15,0]);
title('Uncontrolled Aircraft Pitch Angle');
xlabel('$\omega, [rad/s]$','Interpreter','latex','FontSize',14); 
ylabel('$ S_{\theta \theta}, [ \frac{rad^2}{rad/s} ] $','Interpreter','latex','FontSize',14);

subplot(5,2,6);
loglog(w,Stt_ap,'r'); hold on;
loglog(omega,Swlch_ap(:,3),'b--'); grid on;
axis(10.^[-2,2,-15,0]);
title('Controlled Aircraft Pitch Angle');
xlabel('$\omega, [rad/s]$','Interpreter','latex','FontSize',14); 
ylabel('$ S_{\theta \theta}, [ \frac{rad^2}{rad/s} ] $','Interpreter','latex','FontSize',14);

subplot(5,2,7);
loglog(w,Sqq,'r'); hold on;
loglog(omega,Swlch(:,4),'b--'); grid on;
axis(10.^[-2,2,-20,-5]);
title('Uncontrolled Aircraft Pitch Rate');
xlabel('$\omega, [rad/s]$','Interpreter','latex','FontSize',14); 
ylabel('$ S_{qq}, [ \frac{rad^2}{rad/s} ] $','Interpreter','latex','FontSize',14);

subplot(5,2,8);
loglog(w,Sqq_ap,'r'); hold on;
loglog(omega,Swlch_ap(:,4),'b--'); grid on;
title('Controlled Aircraft Pitch Rate');
axis(10.^[-2,2,-20,-5]);
xlabel('$\omega, [rad/s]$','Interpreter','latex','FontSize',14); 
ylabel('$ S_{qq}, [ \frac{rad^2}{rad/s} ] $','Interpreter','latex','FontSize',14);

subplot(5,2,9);
loglog(w,Snn,'r'); hold on;
loglog(omega,Swlch(:,8),'b--'); grid on;
title('Uncontrolled Aircraft Load Factor');
axis(10.^[-2,2,-10,0]);
xlabel('$\omega, [rad/s]$','Interpreter','latex','FontSize',14); 
ylabel('$ S_{n_{z}n_{z}} $','Interpreter','latex','FontSize',14);

subplot(5,2,10);
loglog(w,Snn_ap,'r'); hold on;
loglog(omega,Swlch_ap(:,8),'b--'); grid on;
title('Controlled Aircraft Load Factor');
axis(10.^[-2,2,-10,0]);
xlabel('$\omega, [rad/s]$','Interpreter','latex','FontSize',14); 
ylabel('$ S_{n_{z}n_{z}} $','Interpreter','latex','FontSize',14);

%% Analytical and FFT Periodograms
figure;

subplot(5,2,1)
loglog(w,Suu,'r'); hold on;
loglog(omega,Pu(1:round(N/2)-1),'b--'); grid on;
axis(10.^[-2,2,-15,0]);
title('Uncontrolled Aircraft Velocity');
xlabel('$\omega, [rad/s]$','Interpreter','latex','FontSize',14); 
ylabel('$ S_{\hat{u} \hat{u}}, [ \frac{rad^2}{rad/s} ] $','Interpreter','latex','FontSize',14);
legend('Analytical','Periodogram (fft)');

subplot(5,2,2)
loglog(w,Suu_ap,'r'); hold on;
loglog(omega,Pu_ap(1:round(N/2)-1),'b--'); grid on;
axis(10.^[-2,2,-15,0]);
title('Controlled Aircraft Velocity');
xlabel('$\omega, [rad/s]$','Interpreter','latex','FontSize',14); 
ylabel('$ S_{\hat{u} \hat{u}}, [ \frac{rad^2}{rad/s} ] $','Interpreter','latex','FontSize',14);

subplot(5,2,3);
loglog(w,Saa,'r'); hold on;
loglog(omega,Palpha(1:round(N/2)-1),'b--'); grid on;
axis(10.^[-2,2,-15,0]);
title('Uncontrolled Aircraft Angle of Attack');
xlabel('$\omega, [rad/s]$','Interpreter','latex','FontSize',14); 
ylabel('$ S_{\alpha \alpha}, [ \frac{rad^2}{rad/s} ] $','Interpreter','latex','FontSize',14)

subplot(5,2,4);
loglog(w,Saa_ap,'r'); hold on;
loglog(omega,Palpha_ap(1:round(N/2)-1),'b--'); grid on;
axis(10.^[-2,2,-15,0]);
title('Controlled Aircraft Angle of Attack');
xlabel('$\omega, [rad/s]$','Interpreter','latex','FontSize',14); 
ylabel('$ S_{\alpha \alpha}, [ \frac{rad^2}{rad/s} ] $','Interpreter','latex','FontSize',14)

subplot(5,2,5);
loglog(w,Stt,'r'); hold on;
loglog(omega,Ptheta(1:round(N/2)-1),'b--'); grid on;
axis(10.^[-2,2,-15,0]);
title('Uncontrolled Aircraft Pitch Angle');
xlabel('$\omega, [rad/s]$','Interpreter','latex','FontSize',14); 
ylabel('$ S_{\theta \theta}, [ \frac{rad^2}{rad/s} ] $','Interpreter','latex','FontSize',14);

subplot(5,2,6);
loglog(w,Stt_ap,'r'); hold on;
loglog(omega,Ptheta_ap(1:round(N/2)-1),'b--'); grid on;
axis(10.^[-2,2,-15,0]);
title('Controlled Aircraft Pitch Angle');
xlabel('$\omega, [rad/s]$','Interpreter','latex','FontSize',14); 
ylabel('$ S_{\theta \theta}, [ \frac{rad^2}{rad/s} ] $','Interpreter','latex','FontSize',14);

subplot(5,2,7);
loglog(w,Sqq,'r'); hold on;
loglog(omega,Pq(1:round(N/2)-1),'b--'); grid on;
axis(10.^[-2,2,-20,-5]);
title('Uncontrolled Aircraft Pitch Rate');
xlabel('$\omega, [rad/s]$','Interpreter','latex','FontSize',14); 
ylabel('$ S_{qq}, [ \frac{rad^2}{rad/s} ] $','Interpreter','latex','FontSize',14);

subplot(5,2,8);
loglog(w,Sqq_ap,'r'); hold on;
loglog(omega,Pq_ap(1:round(N/2)-1),'b--'); grid on;
title('Controlled Aircraft Pitch Rate');
axis(10.^[-2,2,-20,-5]);
xlabel('$\omega, [rad/s]$','Interpreter','latex','FontSize',14); 
ylabel('$ S_{qq}, [ \frac{rad^2}{rad/s} ] $','Interpreter','latex','FontSize',14);

subplot(5,2,9);
loglog(w,Snn,'r'); hold on;
loglog(omega,Pnz(1:round(N/2)-1),'b--'); grid on;
title('Uncontrolled Aircraft Load Factor');
axis(10.^[-2,2,-10,0]);
xlabel('$\omega, [rad/s]$','Interpreter','latex','FontSize',14); 
ylabel('$ S_{n_{z}n_{z}} $','Interpreter','latex','FontSize',14);

subplot(5,2,10);
loglog(w,Snn_ap,'r'); hold on;
loglog(omega,Pnz_ap(1:round(N/2)-1),'b--'); grid on;
title('Controlled Aircraft Load Factor');
axis(10.^[-2,2,-10,0]);
xlabel('$\omega, [rad/s]$','Interpreter','latex','FontSize',14); 
ylabel('$ S_{n_{z}n_{z}} $','Interpreter','latex','FontSize',14);


%% Calculating Variances of Aircraft States
% Ce500
% Using Analytical PSD
dw = diff(w)'; 

vars(1,1) = sum(dw.*Suu(1:Nf-1))/pi;
vars(2,1) = sum(dw.*Saa(1:Nf-1))/pi;
vars(3,1) = sum(dw.*Stt(1:Nf-1))/pi;
vars(4,1) = sum(dw.*Sqq(1:Nf-1))/pi;
vars(5,1) = sum(dw.*Snn(1:Nf-1))/pi;

% Using Lyapunov Equation
Wc = 1;
% Only vertical turbulence input
Bin = B(:,3);
Din = D(:,3);
% Solving Lyapunov Equation
L = lyap(A, Bin*Wc*Bin');
% Solving Cyy_ss = C*Cxx_ss*C' + DWD'
L = C*L*C' + Din*Wc*Din'; 
vars(:,2) = [diag(L(1:4,1:4));L(8,8)];

% Using Matlab Function var.m on variable time traces
vars(1,3) = var(uv);
vars(2,3) = var(alp);
vars(3,3) = var(th);
vars(4,3) = var(q);
vars(5,3) = var(nz);

% Ce500 AP
% Using Analytical PSD
dw = diff(w)'; 

vars_ap(1,1) = sum(dw.*Suu_ap(1:Nf-1))/pi;
vars_ap(2,1) = sum(dw.*Saa_ap(1:Nf-1))/pi;
vars_ap(3,1) = sum(dw.*Stt_ap(1:Nf-1))/pi;
vars_ap(4,1) = sum(dw.*Sqq_ap(1:Nf-1))/pi;
vars_ap(5,1) = sum(dw.*Snn_ap(1:Nf-1))/pi;

% Using Lyapunov Equation
% Solving Lyapunov Equation
L_ap = lyap(A_fb, Bin*Wc*Bin');
% Solving Cyy_ss = C*Cxx_ss*C' + DWD'
L_ap = C*L_ap*C' + Din*Wc*Din'; 
vars_ap(:,2) = [diag(L_ap(1:4,1:4));L_ap(8,8)];

% Using Matlab Function var.m on variable time traces
vars_ap(1,3) = var(uv_ap);
vars_ap(2,3) = var(alp_ap);
vars_ap(3,3) = var(th_ap);
vars_ap(4,3) = var(q_ap);
vars_ap(5,3) = var(nz_ap);


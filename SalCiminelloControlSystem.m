function one_magnet_height_control
%% =======================
% Single Electromagnet — 1 DOF Gap Control
% States: x (downward displacement), dx, ei (integral error)
% Gap:    d = d_nom - x   [m]  (keep d positive)
% Model:  Fmag = Cmag * I^nI / d^nD
% =======================
%% --- Environment & sim ---
g      = 9.81;          % [m/s^2]
t_end  = 5.0;           % [s]
opts   = odeset('RelTol',1e-6,'AbsTol',1e-8);
%% --- Mechanical (mass carried by this magnet) ---
m_eff    = 4;           % [kg] effective levitated mass
k_struct = 0;           % [N/m] stiffness (set >0 if needed)
c_struct = 0;           % [N*s/m] damping (set >0 if needed)
%% --- Magnet model (measured/fitted) ---
% Fmag = Cmag * I^nI / d^nD
Imax  = 15;             % [A] current supply limit
nI    = 2.0264;         % exponent on current (fit)
nD    = 0.5416;         % exponent on gap distance (fit)
Cmag  = 0.1061;         % [N*m^nD / A^nI] fitted constant
%% --- Gap reference ---
deq    = 0.004;         % [m] nominal hover gap
d1     = deq + 0.003;   % [m] initial gap (0..t_step)
d2     = deq;           % [m] target after t_step
t_step = 0.5;           % [s]
d_ref_fun    = @(t) (t < t_step).*d1 + (t >= t_step).*d2;
d_refdot_fun = @(t) 0*t;   % hold position
% Safety range (physics/sensor limits)
d_min  = 0.001;         % [m]
d_max  = 0.050;         % [m]
%% --- Controller (gap PID on d) ---
Kp  = 2e4;     % was 8e5
Kd  = 2e3;     % was 3e4
Ki  = 1e3;     % was 2e4
kaw = 200;     % anti-windup speed is fine to keep            % [1/s] anti-windup back-calc gain
%% --- Pack parameters ---
P.g  = g; P.m = m_eff; P.k = k_struct; P.c = c_struct;
P.Cmag = Cmag; P.nI = nI; P.nD = nD; P.Imax = Imax;
P.d_min = d_min; P.d_max = d_max;
P.Kp = Kp; P.Kd = Kd; P.Ki = Ki; P.kaw = kaw;
P.d_ref_fun = d_ref_fun; P.d_refdot_fun = d_refdot_fun;
P.d_nom = deq;          % x=0 ↔ d=deq
%% --- Initial conditions ---
x0  = P.d_nom - d1;     % so d(0) = d1
dx0 = 0;
ei0 = 0;
X0  = [x0; dx0; ei0];
%% --- Simulate ---
[t, X] = ode45(@(t,X) dyn_gap_pid(t,X,P), [0 t_end], X0, opts);
% Unpack
x   = X(:,1);  dx = X(:,2);  ei = X(:,3);
d   = P.d_nom - x;
dref = arrayfun(P.d_ref_fun, t);
% Postcompute currents/forces
[Fmag, I_cmd, F_cmd] = post_signals(t, x, dx, ei, P);
%% --- Plots ---
figure(1); clf
subplot(4,1,1)
plot(t, 1e3*d,'LineWidth',1.8); grid on
hold on; plot(t, 1e3*dref,'--','LineWidth',1.4)
ylabel('Gap d [mm]'); title('Single Magnet — Gap Control')
legend('d','d_{ref}','Location','best')
subplot(4,1,2)
plot(t, I_cmd,'LineWidth',1.8); grid on
yline(P.Imax,'--r'); ylabel('Current I [A]')
subplot(4,1,3)
plot(t, Fmag,'LineWidth',1.8); grid on
yline(P.m*P.g,'--'); ylabel('F_{mag} [N]')
subplot(4,1,4)
plot(t, F_cmd,'LineWidth',1.8); grid on
xlabel('Time [s]'); ylabel('F_{cmd} [N]')
end
%% ===== Dynamics with PID + anti-windup =====
function dX = dyn_gap_pid(t, X, P)
    x  = X(1);  dx = X(2);  ei = X(3);
    % Gap and rate
    d  = P.d_nom - x;
    dd = -dx;
    % Reference and errors  (keep your sign convention e = d_ref - d)
    d_ref   = P.d_ref_fun(t);
    d_ref_d = P.d_refdot_fun(t);
    e    = d_ref   - d;
    edot = d_ref_d - dd;
    % --- 1) Unsaturated force command
    F_cmd_unsat = P.m*P.g + P.Kp*e + P.Kd*edot + P.Ki*ei;
    % --- 2) Saturate FORCE, not just current
    d_eff  = min(max(d, P.d_min), P.d_max);
    F_max  = P.Cmag * (P.Imax^P.nI) / (P.d_min^P.nD);   % physical max at Imax, smallest gap
    F_cmd  = min(max(F_cmd_unsat, 0), F_max);           % clamp force command
    % --- 3) Map clamped force to current
    if F_cmd <= 0
        I_cmd = 0;
    else
        I_cmd = ((F_cmd * d_eff^P.nD) / P.Cmag)^(1/P.nI);
        I_cmd = max(0, min(P.Imax, I_cmd));
    end
    % Realized magnetic force from clamped current
    Fmag = P.Cmag * (I_cmd.^P.nI) / (d_eff.^P.nD);
    % --- 4) Proper back-calculation anti-windup using force clamp
    % Use the *difference between clamped and unclamped force* to unwind integrator
    ei_dot = e + P.kaw * (F_cmd - F_cmd_unsat);
    % Plant
    xdd = (-P.m*P.g + Fmag - P.k*x - P.c*dx)/P.m;
    dX  = [dx; xdd; ei_dot];
end
%% ===== Postcompute signals =====
function [Fmag, I_cmd, F_cmd] = post_signals(t, x, dx, ei, P)
n = numel(t);
Fmag = zeros(n,1); I_cmd = zeros(n,1); F_cmd = zeros(n,1);
for k = 1:n
    d  = P.d_nom - x(k);
    dd = -dx(k);
    e    = P.d_ref_fun(t(k)) - d;
    edot = P.d_refdot_fun(t(k)) - dd;
    F_cmd(k,1) = P.m*P.g + P.Kp*e + P.Kd*edot + P.Ki*ei(k);
    d_eff = min(max(d, P.d_min), P.d_max);
    I = ((max(F_cmd(k),0) * d_eff^P.nD) / P.Cmag)^(1/P.nI);
    I = max(0, min(P.Imax, I));
    I_cmd(k,1) = I;
    Fmag(k,1)  = P.Cmag * (I.^P.nI) / (d_eff.^P.nD);
end
end

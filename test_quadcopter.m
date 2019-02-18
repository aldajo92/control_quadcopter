clc; clear; close all

%% 

% U = 2.8U = [ w(k), w(k), w(k), w(k) ];
% 
% X_init = [0,0,10,0,0,0,0,0,0,0,0,0];
% X_0 = X_init;
% [t_emulation, x_quadcopter] = quadcopter(U, X_0, 0, 10);
% 
% plot(t_emulation, x_quadcopter(:,3))

%%

k = 1.98e-6;      % cosntante de empuje
m = 5;            % masa del quadcopter
g = 9.8;          % constante de gravedad

X_init = [0,0,10,0,0,0,0,0,0,0,0,0];
X_0 = X_init;

Tf = 80;
Ts = 10;

time = 0:Ts:Tf;

Samples = size(time, 2);
eqW = (sqrt(m*g/k)/2);
w = [ eqW, eqW+0.5, eqW-0.5, eqW, eqW-0.3, eqW+0.3, eqW+1, eqW-1, eqW, eqW];

x_quad = [];
t_total = [];
control = [];

for k = 1: Samples
    U = [ w(k), w(k), w(k), w(k) ];
    [t_emulation, x_quadcopter] = quadcopter(U, X_0, time(k), Ts);
    t_total = [t_total; t_emulation];
    x_quad = [x_quad; x_quadcopter];
    control = [control; w(k)*ones(size(t_emulation))];
    X_0 = x_quadcopter(size(x_quadcopter,1), :);
end

subplot(2,1,1)
plot(t_total, x_quad(:,3))
grid on
subplot(2,1,2)
plot(t_total, control)
grid on
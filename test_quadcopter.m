X_init = [0,0,10,0,0,0,0,0,0,0,0,0];
X_0 = X_init;
[t_emulation, x_quadcopter] = quadcopter(U, X_0, 0, 10);

plot(t_emulation, x_quadcopter(:,3))

%%

k = 2.98e-6;      % cosntante de empuje
m = 1.8;    % masa del quadcopter
g = 9.8;    % constante de gravedad

X_init = [0,0,10,0,0,0,0,0,0,0,0,0];
X_0 = X_init;

Tf = 60;
Ts = 10;

time = 0:Ts:Tf;

Samples = size(time, 2);
eqW = (sqrt(m*g/k)/2)
w = [ eqW, eqW+0.5, eqW-0.5, eqW, eqW-0.08, eqW+0.08, eqW, eqW, eqW, eqW];

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

% x_abs = (ones(size(x_quad,1),1) .* X_init) + x_quad;
subplot(2,1,1)
plot(t_total, x_quad(:,3))
subplot(2,1,2)
plot(t_total, control)
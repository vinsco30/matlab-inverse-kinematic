clear
close all

%% Algorithm set-up
run('../trajectory_practice/square_inv.m');

% Robot params
a1 = 0.5;
a2 = 0.5;
d1 = 1.0;
K = 300*eye(4);

%Joints initial conditions with classic inverse kinematics 
[th1, th2] = scara_ik_xy(p0,a1,a2);

%Initialization desired trajectory
xd = squeeze(T_tot(1:3,4,:));
xd = [xd; eul(1,:)];
d_xd = squeeze(d_T_tot(1:3,4,:));
d_xd = [d_xd; we(3,:)];
n = size(xd,2);

%Home position
xe = [p0; 0];

%Initialization joint vector and errors
d_q = zeros(4,size(xd,2));
q = zeros(4,size(xd,2));
q(:,1) = [th1 th2 p0(3) -th1-th2]';
err_x = zeros(1,n-1);
err_y = zeros(1,n-1);
err_z = zeros(1,n-1);
err_or = zeros(1,n-1);

%% Inverse Kinematics Algorithm (Jacobian Inverse)
for i=2:n-1
    e = xd(:,i) - xe;
    err_x(1,i) = abs(e(1));
    err_y(1,i) = abs(e(2));
    err_z(1,i) = abs(e(3));
    err_or(1,i) = abs(e(4));
    d_q(:,i) = inv(jacobian(q(1,i-1),q(2,i-1),a1,a2))*(d_xd(:,i) + K*e);
    q(:,i) = q(:,i-1) + (d_q(:,i)*0.001);
    Re = [-sin(q(1,i)+q(2,i)+q(4,i)), cos(q(1,i)+q(2,i)+q(4,i)), 0;
        cos(q(1,i)+q(2,i)+q(4,i)), sin(q(1,i)+q(2,i)+q(4,i)), 0;
        0 0 -1];
    zyx_e = rotm2eul(Re);
    T = compute_scara_fk(q(:,1),a1,a2,0);
    xe = [a1*cos(q(1,i))+a2*cos(q(1,i)+q(2,i)); a1*sin(q(1,i))+a2*sin(q(1,i)+q(2,i));
        q(3,i); q(1)+q(2)+q(4)];
end



%% Plot positions
figure('Units', 'inches', 'Position', [0, 0, 7, 5]);
subplot(4,1,1)
plot(q(1,1:n-1),'Color','[0,0,0]',LineWidth=3);
title('Joint Trajectory','fontsize',20, 'interpreter','latex')
ylabel('$q_1$ $[rad]$','fontsize',20, 'interpreter','latex')
grid on
xlim([0 n]);
ax = gca;
ax.XTick = 0:100:n;
ax.XTickLabel = {'0', '1', '2', '3', '4', '5', '6', '7', '8'};
set(gca, 'TickLabelInterpreter', 'latex', 'FontSize', 20);
subplot(4,1,2)
plot(q(2,1:n-1),'Color','[0,0,0]',LineWidth=3);
ylabel('$q_2$ $[rad]$','fontsize',20, 'interpreter','latex')
grid on
xlim([0 n]);
ax = gca;
ax.XTick = 0:100:n;
ax.XTickLabel = {'0', '1', '2', '3', '4', '5', '6', '7', '8'};
set(gca, 'TickLabelInterpreter', 'latex', 'FontSize', 20);
set(gca, 'TickLabelInterpreter', 'latex');
subplot(4,1,3)
plot(q(3,1:n-1),'Color','[0,0,0]',LineWidth=3);
ylabel('$q_3$ $[m]$','fontsize',20, 'interpreter','latex')
grid on
xlim([0 n]);
ax = gca;
ax.XTick = 0:100:n;
ax.XTickLabel = {'0', '1', '2', '3', '4', '5', '6', '7', '8'};
set(gca, 'TickLabelInterpreter', 'latex', 'FontSize', 20);
set(gca, 'TickLabelInterpreter', 'latex');
subplot(4,1,4)
plot(q(4,1:n-1),'Color','[0,0,0]',LineWidth=3);
grid on
ylabel('$q_4$ $[rad]$','fontsize',20, 'interpreter','latex')
xlim([0 n]);
ax = gca;
ax.XTick = 0:100:n;
xlabel('$t$ $[s]$','fontsize',20,'interpreter','latex')
ax.XTickLabel = {'0', '1', '2', '3', '4', '5', '6', '7', '8'};
set(gca, 'TickLabelInterpreter', 'latex', 'FontSize', 20);
set(gca, 'TickLabelInterpreter', 'latex');
print('plot_ieee_inversion', '-dpdf', '-r300');

%% Plot orientations
figure('Units', 'inches', 'Position', [0, 0, 7, 5]);
subplot(4,1,1)
plot(err_x(1,:),'Color','[0,0,0]',LineWidth=3);
title('Trajectory Error','fontsize',20, 'interpreter','latex')
ylabel('$e_x$ $[m]$','fontsize',20, 'interpreter','latex')
grid on
xlim([0 n]);
ax = gca;
ax.XTick = 0:100:n;
ax.XTickLabel = {'0', '1', '2', '3', '4', '5', '6', '7', '8'};
set(gca, 'TickLabelInterpreter', 'latex', 'FontSize', 20);
subplot(4,1,2)
plot(err_y(1,:),'Color','[0,0,0]',LineWidth=3);
ylabel('$e_y$ $[m]$','fontsize',20, 'interpreter','latex')
grid on
xlim([0 n]);
ax = gca;
ax.XTick = 0:100:n;
ax.XTickLabel = {'0', '1', '2', '3', '4', '5', '6', '7', '8'};
set(gca, 'TickLabelInterpreter', 'latex', 'FontSize', 20);
set(gca, 'TickLabelInterpreter', 'latex');
subplot(4,1,3)
plot(err_z(1,:),'Color','[0,0,0]',LineWidth=3);
ylabel('$e_z$ $[m]$','fontsize',20, 'interpreter','latex')
grid on
xlim([0 n]);
ax = gca;
ax.XTick = 0:100:n;
ax.XTickLabel = {'0', '1', '2', '3', '4', '5', '6', '7', '8'};
set(gca, 'TickLabelInterpreter', 'latex', 'FontSize', 20);
set(gca, 'TickLabelInterpreter', 'latex');
subplot(4,1,4)
plot(err_or(1,:),'Color','[0,0,0]',LineWidth=3);
grid on
ylabel('$e_{\phi}$ $[rad]$','fontsize',20, 'interpreter','latex')
xlim([0 n]);
ax = gca;
ax.XTick = 0:100:n;
xlabel('$t$ $[s]$','fontsize',20,'interpreter','latex')
ax.XTickLabel = {'0', '1', '2', '3', '4', '5', '6', '7', '8'};
set(gca, 'TickLabelInterpreter', 'latex', 'FontSize', 20);
set(gca, 'TickLabelInterpreter', 'latex');
function plotMotions(t,x,v)
% plotMotions.m     e.anderlini@ucl.ac.uk     30/04/2019
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This function is used to plot the translations and rotations in the
% inertial reference frame and the translational and rotational velocities
% in the body-fixed reference frame of the AUV.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Translations and rotations in the inertial reference frame:
figure;
subplot(3,1,1);
plot(t,x(:,1));
ylabel('$x$ [m]','Interpreter','Latex');
grid on;
set(gca,'TickLabelInterpreter','Latex');
subplot(3,1,2);
plot(t,x(:,2),'Color',[0.8500,0.3250,0.0980]);
ylabel('$z$ [m]','Interpreter','Latex');
grid on;
set(gca,'TickLabelInterpreter','Latex');
subplot(3,1,3);
plot(t,rad2deg(wrapToPi(x(:,3))),'Color',[0.9290,0.6940,0.1250]);
xlabel('Time [s]','Interpreter','Latex');
ylabel('$\theta$ [$^\circ$]','Interpreter','Latex');
grid on;
set(gca,'TickLabelInterpreter','Latex');
set(gcf,'color','w');

%% Translational and rotational velocity in the body-fixed reference frame:
figure;
subplot(3,1,1);
plot(t,v(:,1));
ylabel('$\dot{x}$ [m/s]','Interpreter','Latex');
grid on;
set(gca,'TickLabelInterpreter','Latex');
subplot(3,1,2);
plot(t,v(:,2),'Color',[0.8500,0.3250,0.0980]);
ylabel('$\dot{z}$ [m/s]','Interpreter','Latex');
grid on;
set(gca,'TickLabelInterpreter','Latex');
subplot(3,1,3);
plot(t,rad2deg(v(:,3)),'Color',[0.9290,0.6940,0.1250]);
xlabel('Time [s]','Interpreter','Latex');
ylabel('$\dot{\theta}$ [$^\circ$/s]','Interpreter','Latex');
grid on;
set(gca,'TickLabelInterpreter','Latex');
set(gcf,'color','w');

end
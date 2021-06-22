function plotControl(t,torque,hydroplane_angle,revs)
% plotControl.m     e.anderlini@ucl.ac.uk     30/04/2019
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This function plots the hydroplane angle and the input torque to the
% motor, as well as the resulting propeller revolutions.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Translations and rotations in the inertial reference frame:
figure;
subplot(2,1,1);
yyaxis left;
plot(t,torque);
ylabel('Motor Torque [Nm]','Interpreter','Latex');
yyaxis right;
plot(t,30/pi*revs,'--');
ylabel('Propeller Revolutions [rpm]','Interpreter','Latex');
grid on;
set(gca,'TickLabelInterpreter','Latex')
subplot(2,1,2);
plot(t,hydroplane_angle,'Color',[0.9290,0.6940,0.1250]);
xlabel('Time [s]','Interpreter','Latex');
ylabel('Fin Angle [$^\circ$]','Interpreter','Latex');
grid on;
set(gca,'TickLabelInterpreter','Latex')
set(gcf,'color','w');

end
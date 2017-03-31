%%
[t,X,u,Fz] = fcn_visualize(x);

%%
subplot(2,2,1)
plot(t,X(:,1),'b')
hold on 
plot(t,X(:,2),'r')
xlabel('Time [s]')
ylabel('z-position [m]')
legend('z [m]','dz [m/s]')

subplot(2,2,2)
plot(t,Fz,'k')
hold on
xlabel('Time [s]')
ylabel('Fz [N]')
grid on


subplot(2,2,3)
plot(t,u(1,:),'b')
hold on
plot(t,u(2,:),'r')
xlabel('Time [s]')
ylabel('Joint torques [Nm]')
legend('tau1','tau2')

subplot(2,2,4)
plot(-u(1,:),dq1,'b')
hold on
plot(-u(2,:),dq2,'r')
xlabel('torque [Nm]')
ylabel('dq [rad/s]')
legend('hip','knee','location','northwest')





%Declaration of robot parameters

alpha =[-pi/2 0 -pi/2 pi/2 -pi/2 0];
a1 = 0.312; d1= 0.324;
a2= 1.075; d2 = 0;
a3 = 0.225 ;d3 = 0;
a4 =0; d4=1.280;
a5 = 0 ; d5 =0;
a6 = 0.215 ; d6 =0;
a=[a1 a2 a3 a4 a5 a6];
d=[d1 d2 d3 d4 d5 d6];

% theta can be changed here to see different forward kinematic resukts of
% the robot
theta=[q1 q2 0 pi 0 0];
q1 = linspace(-pi,pi,5);
q2 = linspace(-pi,pi,5);

for i = 1:length(q1)
    for j = 1: length(q2)
    theta(1) = q1(i);
    theta(2) = q2(i);
% Calling of DH function to calculate transformation matrix for each frame.
DH1 = DH(theta(1), d(1), alpha(1),a(1));
DH2 = DH(theta(2), d(2), alpha(2),a(2));
DH3 = DH(theta(3), d(3), alpha(3),a(3));
DH4 = DH(theta(4), d(4), alpha(4),a(4));
DH5 = DH(theta(5), d(5), alpha(5),a(5));
DH6 = DH(theta(6), d(6), alpha(6),a(6));
% end effector kinematics
fkine= DH1*DH2*DH3*DH4*DH5*DH6;
f_arm= DH1*DH2*DH3;
f_wrist = DH4*DH5*DH6;
disp("The result for forward kinematics for q1 and q2 at :")
disp([theta(1), theta(2)])
disp("is:")
disp(fkine)
    end
end
% arm kinematics
% wrist kinematics

% f_ar_wrist should be equal to f_arm * f_wrist
f_ar_wrist = f_arm * f_wrist;








% using inbuilt methods of robotic toolbox to find the forward kinematics

% reassigning theta(1) and theta(2) for plot test
L(1) = Link([0, d(1), a(1), alpha(1), 0]);
L(2) = Link([-pi/2, d(2), a(2), alpha(2), 0]);
L(3) = Link([theta(3), d(3), a(3), alpha(3), 0]);
L(4) = Link([theta(4), d(4), a(4), alpha(4), 0]);
L(5) = Link([theta(5), d(5), a(5), alpha(5), 0]);
L(6) = Link([theta(6), d(6), a(6), alpha(6), 0]);



Fanuc1 = SerialLink(L, 'name', 'fanuc');
fanuc_fkine= Fanuc1.fkine([0 -pi/2 theta(3) theta(4) theta(5) theta(6)]);
% plot of the pictorial representation of the robot
Fanuc1.plot([0 -pi/2 theta(3) theta(4) theta(5) theta(6)]);


% % % % Inverse Kinematics Solution for q1(0) and q2(0) above in linspace(-pi,pi,5)

q1=atan2(f_arm(1,2), -f_arm(2,2))
q23= atan2(-f_arm(3,1), -f_arm(3,3));

s2 = (-(f_arm(3,4)-d1-a3*f_arm(3,1))/(a2));
c2_1= sqrt(1-s2^2);
c2_2= -sqrt(1-s2^2);

q2 = atan2(s2,c2_1)
q2_1 = atan2(s2,c2_2)

q3= q23-q2;
q3_1= q23-q2_1;



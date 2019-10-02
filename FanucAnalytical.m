syms q1 q2 q3 q4 q4 q5 q6;
syms a1 a2 a3 a4 a5 a6;
syms d1 d2 d3 d4 d5 d6;

a=[a1 a2 a3 0 0 0];
d=[d1 0 0 d4 0 d6];
theta=[q1 q2 q3 q4 q5 q6];
alpha =[-pi/2 0 -pi/2 pi/2 -pi/2 0];


DH1 = DH(theta(1), d(1), alpha(1),a(1));
DH2 = DH(theta(2), d(2), alpha(2),a(2));
DH3 = DH(theta(3), d(3), alpha(3),a(3));
DH4 = DH(theta(4), d(4), alpha(4),a(4));
DH5 = DH(theta(5), d(5), alpha(5),a(5));
DH6 = DH(theta(6), d(6), alpha(6),a(6));


fkine= DH1*DH2*DH3*DH4*DH5*DH6

f_arm =  DH1*DH2*DH3
f_wrist = DH4*DH5*DH6

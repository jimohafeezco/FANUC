% 
% 




function T =DH(theta, d , alpha , a)

Rz = trotz(theta);
Tz = transl([0 , 0, d]);
Tx = transl([a, 0 , 0]);
Rx = trotx(alpha);

T = Rz*Tz*Rx*Tx;
end



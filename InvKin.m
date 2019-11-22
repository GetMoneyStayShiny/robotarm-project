%clc;
clear;
T = @(a,al,d,th)([cos(th) -sin(th)*cos(al) sin(th)*sin(al) a*cos(th);
                  sin(th) cos(th)*cos(al) -cos(th)*sin(al) a*sin(th);
                  0 sin(al) cos(al) d;
                  0 0 0 1]);
R =  @(a,al,d,th)([cos(th) -sin(th)*cos(al) sin(th)*sin(al) ;
                  sin(th) cos(th)*cos(al) -cos(th)*sin(al) ;
                  0 sin(al) cos(al)]);
              

syms th1 th2 th3 th4 th5 th6


 d1 = 0.1273;
 a1 =  pi/2;
 l2 = -0.612;
 l3 = -0.5723;
 d4 = 0.163941;	
 a4 = pi/2;	
 d5 = 0.1157;	
 a5 = -pi/2;
 d6 = 0.0922;
 
 %syms th1 th2 th3 th4 th5 th6 d1 a1 l2 l3 d4 a4 d5 a5 d6

T1 = T(0,a1,d1,th1);
T2 = T(l2,0,0,th2);
T3 = T(l3,0,0,th3);
T4 = T(0,a4,d4,th4);
T5 = T(0,a5,d5,th5);
T6 = T(0,0,d6,th6);

T1 = [cos(th1) 0 sin(th1) 0; sin(th1) 0 -cos(th1) 0; 0 1 0 1273/10000; 0 0 0 1];
T4 = [cos(th4) 0 sin(th4) 0; sin(th4) 0 -cos(th4) 0; 0 1 0 1476649253021493/9007199254740992; 0 0 0 1];
T5 = [cos(th5) 0 sin(th5) 0; -sin(th5) 0 cos(th5) 0; 0 -1 0 1157/10000; 0 0 0 1];
%T3 = [1 0 0 d1; 0 0 1 0; 0 -1 0 0; 0 0 0 1];
%T4 = [1 0 0 d2; 0 -1 0 0; 0 0 -1 0; 0 0 0 1]

%T4 = T(d2,pi,0,th4);

T06 = T1*T2*T3*T4*T5*T6;
T05 = T1*T2*T3*T4*T5;
T04 = T1*T2*T3*T4;
T03 = T1*T2*T3;
T02 = T1*T2;
T01 = T1;

T_bf = simplify(vpa(T06))

Po = [0;0;0;1];
 P0 = Po(1:3);
 P1 = T1(1:3,4);
 P2o=T1*T2;
 P2=P2o(1:3,4);
  P3o=T1*T2*T3;
 P3=P3o(1:3,4);
  P4o=T1*T2*T3*T4;
 P4=P4o(1:3,4);
   P5o=T1*T2*T3*T4*T5;
 P5=P5o(1:3,4);
   P6o=T1*T2*T3*T4*T5*T6;
 P6=P6o(1:3,4);
 
 
  Z0 = [0;0;1];
 Z1 = T1(1:3,3);
 Z2 = P2o(1:3,3);
 Z3 = P3o(1:3,3);
 Z4 = P4o(1:3,3);
 Z5 = P5o(1:3,3);
 Z6 = P6o(1:3,3);
 
 
 J_temp = [cross(Z0,(P6-P0)) cross(Z1,(P6-P1)) cross(Z2,(P6-P2)) cross(Z3,(P6-P3)) cross(Z4,(P6-P4)) cross(Z5,(P6-P5));
           Z0 Z1 Z2 Z3 Z4 Z5];
 
       
       
 %P_e = [1;0;0.2];
 invJ = J_temp(1:3,4);
 %dq = invJ'*P_e;
 %J_v = J_temp*dq;
 
%pseudoInv = J_temp'\(J_temp*J_temp')
 
%newT = [0 -sin(phi) cos(phi)*sin(theta); 0 cos(phi) sin(phi)*sin(theta); 1 0 cos(theta)];
%zeroes3x3 = [0 0 0; 0 0 0; 0 0 0];
%T_phi = [eye(3,3) zeroes3x3; zeroes3x3 newT];
%J_A = inv(T_phi)*J_temp;
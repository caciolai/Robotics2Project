%% PRELIMINARY COMPUTATIONS
disp("Doing some preliminary computations...");
tic;
x1 = q(1:n);
x2 = dq(1:n);
x3 = q(n+1:2*n);
x4 = dq(n+1:2*n);
%Sum of Coriolis and gravity term
c = C + G;
Minv = inv(M);
Jinv = inv(J);
dMinv = diffM(Minv,x1,x2);
dCdx1 = jacobian(c,x1);
dCdx2 = jacobian(c,x2);
toc;
%% COMPUTATION OF SPONG CHANGE OF COORDINATES (3.10-3.13) FOR CONTROLLERS AND CICCARELLA OBSERVER
disp("Computing Spong change of coordinates...");
tic;
y_1 = x1;
y_2 = x2;
y_3 = (-Minv*(c + K*(x1 -x3)));
f4 =  (-dMinv*( c + K*(x1- x3))-Minv*(dCdx1*x2 + dCdx2*y_3 + K*x2));
y_4 = (f4 + M\(K*x4));
if simplify_all
    y_3 = simplify(simplifyFraction(y_3));
    f4 = simplify(simplifyFraction(f4));
    y_4 = simplify(simplifyFraction(y_4));
end
phi_x = [y_1;y_2;y_3;y_4];
toc;
%% COMPUTATION OF CHANGE OF COORDINATES FOR CICCARELLA OBSERVER
disp("Computing Ciccarella change of coordinates...");
tic;
f_x = [x2; -Minv*(c + K*(x1-x3)) ;x4; Jinv*K*(x1-x3)];
%T is the (linear) transformation matrix from Spong to Ciccarella change of
%coordinates
T = zeros(4*n,4*n);
for i = 1 : n
    idx = 4*i - 3;
    Bi = zeros(1,n);
    Bi(i) = 1;
    Bicell = repmat({Bi},1,4);
    T(idx:(idx+3),:) = blkdiag(Bicell{:});
end
eta_x = (T*phi_x);

toc;
%% COMPUTATION OF Q(x) FOR CICCARELLA OBSERVER
disp("Computing the Q(x) for the Ciccarella observers...");
tic;
Q = (jacobian(eta_x,[x1;x2;x3;x4]));
if simplify_all
    Q = simplify(Q);
end
%WARNING : POSSIBLE COMPUTATION OF INVERSE OF Q
% %Qinv = simplify(inv(Q));
toc;

%% COMPUTATION OF Q1(x) FOR CICCARELLA WITH MOTORS' POSITIONS
eta1_x = zeros(4*n,1,'sym');
for i = 1 : n
    idx = 4*i - 3;
    hi = x3(i);
    eta1_x(idx) = hi;
    eta1_x(idx+1) = lieDerivative(f_x,hi,1,[x1;x2;x3;x4]);
    eta1_x(idx+2) = lieDerivative(f_x,hi,2,[x1;x2;x3;x4]);
    eta1_x(idx+3) = lieDerivative(f_x,hi,3,[x1;x2;x3;x4]);
end
Q1 = (jacobian(eta1_x,[x1;x2;x3;x4]));
if simplify_all
    Q1 = simplify(Q1);
end

%% COMPUTATION OF Q2(x) FOR CICCARELLA WITH MOTORS' POSITIONS AND ELASTIC TORQUE
eta2_x = zeros(4*n,1,'sym');
y_c = [x3;K*(x1-x3)];
for i = 1 : 2*n
    idx = 2*i - 1;
    eta2_x(idx) = y_c(i);
    eta2_x(idx+1) = lieDerivative(f_x,y_c(i),1,[x1;x2;x3;x4]);
end
Q2 = (jacobian(eta2_x,[x1;x2;x3;x4]));
if simplify_all
    Q2 = simplify(Q2);
end

%% COMPUTATION OF Q3(x) FOR CICCARELLA WITH MOTORS' POSITIONS AND JOINT ACCELERATION
eta3_x = zeros(4*n,1,'sym');
y_c = [x3;-Minv*(c + K*(x1-x3))];
for i = 1 : 2*n
    idx = 2*i - 1;
    eta3_x(idx) = y_c(i);
    eta3_x(idx+1) = lieDerivative(f_x,y_c(i),1,[x1;x2;x3;x4]);
end
Q3 = (jacobian(eta3_x,[x1;x2;x3;x4]));
if simplify_all
    Q3 = simplify(Q3);
end


%% COMPUTATION OF F(x1,x2,x3,x4) for Tomei controller (3.19)
disp("Computing F(x1,x2,x3,x4) for the Tomei controller...");
tic;
F = (jacobian(f4,x1)*x2 + jacobian(f4,x2)*y_3 + jacobian(f4,x3)*x4 + dMinv*K*x4 + Minv*K*Jinv*K*(x1-x3));
if simplify_all
    F = simplify(simplifyFraction(F));
end
toc;
%% COMPUTATION OF DECOUPLING MATRIX
disp("Computing the decoupling matrix of the robot (for output y = q1)...");
tic;
U = simplify(Minv*K*Jinv);
toc;

%% ALTERNATIVE COMPUTATION OF inv(Q(x)) WITHOUT DOING INVERSE!
%COMPUTATION OF THE INVERSE CHANGE OF COORDINATES OF TOMEI (3.14 - 3.17)
disp("Computing the inverse of phi_x...");
tic;
y1 = sym('y1',[n,1],'real');
y2 = sym('y2',[n,1],'real');
y3 = sym('y3',[n,1],'real');
y4 = sym('y4',[n,1],'real');
z  = sym('z' ,[4*n,1],'real');
My = subs(M,x1,y1);
cy = subs(c, [x1(:);x2(:)],[y1(:);y2(:)]);
tmp = simplify(y1 + inv(K)*(My*y3 + cy));
if simplify_all
    tmp = simplify(simplifyFraction(tmp));
end
f4y = subs(f4, [x1(:);x2(:);x3(:);x4(:)],[y1(:);y2(:);tmp(:);y4(:)]);
phi_inv = [y1;y2;(y1 + inv(K)*(My*y3 + cy));(inv(K)*(My*(y4-f4y)))];
if simplify_all
    phi_inv = simplify(simplifyFraction(phi_inv));
end
toc;
%%Matrix test for checking the inverse is ok , it should return [q1;dq1;q2;dq2]
%test = simplify(subs(phi_inv,[y1;y2;y3;y4],phi_x));
Tinv = inv(T);
%Comutation of the inverse change of coordinates of the Ciccarella observer
disp("Computing the inverse change of eta_x...");
tic;
eta_inv = (subs(phi_inv, [y1;y2;y3;y4],Tinv*z));
if simplify_all
    eta_inv = simplify(simplifyFraction(eta_inv));
end
toc;

%%Matrix test for checking the inverse is ok , it should return [q1;dq1;q2;dq2]
%test1 = simplify(subs(eta_inv,z,eta_x));

%% NOT USED IN THE SCRIPT BUT LEAVED VOLUNTARILY
%% INVERSE OF Q USING RELATION AMONG JACOBIANS OF CHANGE OF COORDINATES
% disp("Computing the Jacobian of eta_inv...");
% tic;
% deta = (jacobian(eta_inv,z));
% if simplify_all
%     deta = simplify(simplifyFraction(deta));
% end
% toc;
% disp("Computing the inverse of Q using the Jacobian of eta_inv...");
% tic;
% Qinv = (subs(deta,z,eta_x));
% if simplify_all
%     Qinv = simplify(simplifyFraction(Qinv));
% end
% toc;

%% COMPUTATION OF THE MATRICES FOR TOMEI OBSERVER
B_t = eye(n);

%% COMPUTATION OF THE GAIN OF CICCARELLA OBSERVERS
disp("Computing the gain of the Ciccarella(1) observer...");
tic;
%Matrices in the Brunowski canonical form for next computations
A_c1 = [0 1 0 0;0 0 1 0; 0 0 0 1; 0 0 0 0]';
C_c1 = [1 0 0 0]';
K_obs_1 = zeros(4*n,n);
for i = 1 : n
    idx = 4*i - 3;
    modei = modes_c(1,idx:idx+3);
    Kobsi = mypole(modei);
    K_obs_1(idx:idx+3,i) = Kobsi;
end
%Check of poles assignment
% A_c1 = A_c1';
% C_c1 = C_c1';
% Aicell1 = repmat({A_c1},1,n);
% Cicell1 = repmat({C_c1},1,n); 
% A_c1 = blkdiag(Aicell1{:});
% C_c1 = blkdiag(Cicell1{:});
% disp("The eigenvalues for the Ciccarella (1) observer are :");
% disp(eig(A_c1 - K_obs_1*C_c1));
toc;
disp("Computing the gain of the Ciccarella(2) observer...");
tic;
%Matrices in the Brunowski canonical form for next computations
K_obs_2 = zeros(4*n,2*n);
for i = 1 : n
    idx = 4*i - 3;
    idx1 = 2*i - 1;
    K_obs_2(idx,idx1) = -modes_c(1,idx); 
    modei = modes_c(1,idx+1:idx+3);
    Kobsi = mypole(modei);
    K_obs_2(idx+1:idx+3,2*i) = Kobsi;
end
%Check of poles assignment
% A_c2 = [0 1 0 0;0 0 1 0; 0 0 0 1; 0 0 0 0];
% C_c2 = [1 0 0 0; 0 1 0 0];
% Aicell2 = repmat({A_c2},1,n);
% Cicell2 = repmat({C_c2},1,n); 
% A_c2 = blkdiag(Aicell2{:});
% C_c2 = blkdiag(Cicell2{:});
% disp("The eigenvalues for the Ciccarella (2) observer are :");
% disp(eig(A_c2 - K_obs_2*C_c2));
toc;

toc;
disp("Computing the gain of the Ciccarella(3)/(4) observer...");
tic;
%Matrices in the Brunowski canonical form for next computations
K_obs_3 = zeros(4*n,2*n);
for i = 1 : 2*n
    idx = 2*i - 1;
    modei = modes_c(1,idx:idx+1);
    Kobsi = mypole(modei);
    K_obs_3(idx:idx+1,i) = Kobsi; 
end
%Check of poles assignment
% A_c3 = [0 1 ; 0 0];
% C_c3 = [1 0];
% Aicell3 = repmat({A_c3},1,2*n);
% Cicell3 = repmat({C_c3},1,2*n); 
% A_c3 = blkdiag(Aicell3{:});
% C_c3 = blkdiag(Cicell3{:});
% disp("The eigenvalues for the Ciccarella (3) observer are :");
% disp(eig(A_c3 - K_obs_3*C_c3));
% toc;


%% COMPUTATION OF THE GAIN FOR THE FEEDBACK LINEARIZING CONTROLLER
disp("Computing the gain for the FL controller...");
tic;
modes_ctr = modes_ctr1(1,1:4*n);%to adapt to case of 2R
A_ctr = zeros(4*n,4*n);
A_ctr(1:3*n,n+1:4*n) = eye(3*n);
B_ctr = zeros(4*n,n);
B_ctr(3*n + 1:4*n,1:n) = eye(n);
K_ctr = zeros(n,4*n);
%Coordinate transformation
T_ctr = zeros(4*n,4*n);
A1_ctr = [0 1 0 0;
      0 0 1 0;
      0 0 0 1;
      0 0 0 0];
B1_ctr = [0 0 0 1]';
for i = 1 : n
   idx = 4*i - 3;
   row_ctr = zeros(1,n);
   row_ctr(i) = 1;
   row_ctr_cell = repmat({row_ctr},1,4);
   T_ctr(idx:idx+3,:) = blkdiag(row_ctr_cell{:});
   K_ctr(i,idx:idx+3) = -mypole1(modes_ctr(idx:idx+3));
end
K_ctr = K_ctr*T_ctr;
A_bar = A_ctr+B_ctr*K_ctr;
P_ctr = lyap(A_bar,Q_ctr);
%Check of poles assignment
disp("The eigenvalues of A+BK are :");
disp(eig(A_bar));
toc;
%% COMPUTATION OF INTEGRAL MANIFOLD FOR CONTROLLER 2 OF SPONG
disp("Computing the (rigid) integral manifold for control purposes...");
tic;
mi = inv(K);
q_d = sym('q_d',[n 1],'real');
d1q_d = sym('d1q_d',[n 1],'real');
d2q_d = sym('d2q_d',[n 1],'real');
d3q_d = sym('d3q_d',[n 1],'real');
d4q_d = sym('d4q_d',[n 1],'real');
d2q = sym('d2q',[n 1],'real');
d3q = sym('d3q',[n 1],'real');
a2 = simplify(-Minv*c);
A1 = -Minv;
A2 = simplify(-(Minv + Jinv));
B2 = -Jinv;
B2inv = -J;
u0 = simplify((M+J)*( d2q_d + Kp*(q_d - x1) + Kd*(d1q_d - x2)) + c);
h0 = simplify(-inv(A2)*(a2 + B2*u0));
toc;
%% COMPUTATION OF THE GAIN FOR CONTROLLER 2 OF TOMEI
disp("Computing the gain for the fast control...");
tic;
At = zeros(2*n,2*n);
At(1:n,n+1:2*n) = eye(n);
Bt = zeros(2*n,n);
Bt(n+1:2*n,1:n) = K;
pls_ctr2 = modes_ctr2(1:2*n);
K_ctr2 = -place(At,Bt,pls_ctr2);
%%Check if eigenvalues are assigned correctly
%eig(At + Bt*K_ctr2)
toc;
%% COMPUTATION OF THE SLOW CONTROL
disp("Computing the slow control for the system...");
tic;
dh0 = (jacobian(h0,[x1;x2;q_d;d1q_d;d2q_d]) * [x2;d2q;d1q_d;d2q_d;d3q_d]);
if simplify_all
   dh0 = simplify(simplifyFraction(dh0)); 
end
ddh0 = (jacobian(dh0,[x1;x2;d2q;q_d;d1q_d;d2q_d;d3q_d]) * [x2;d2q;d3q;d1q_d;d2q_d;d3q_d;d4q_d]);
if simplify_all
   ddh0 = simplify(simplifyFraction(ddh0));
end
u1 = simplify(inv(B2)*ddh0);
us = simplify(u0 + mi*u1);
toc;
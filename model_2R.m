%% 2R ELASTIC PLANAR JOINT ROBOT MODEL
% We will employ the same assumptions of Spong, that lead to two sets of
% differential equations for the model, of the form
% M(q1) ddq1 + c(q1, dq1) + g(q1) + k(q1 - q2) = 0
% J ddq2 - k(q1 - q2) = u

%% ASSUMPTIONS

% ASSUMPTION 1: we include the rotor mass as part of the preceding link, so
% that each of these m_i defined below comprises both the mass of link i
% and of rotor i.
%
% ASSUMPTION 2: the links center of mass positions are on the principal axis
% of the link at a distance d from the origin of RF_{i-1}.

%% START BUILD OF THE MODEL
%% Declaration of symbolic variables
% The state q can be partioned in (q1, q2) where 
% q1 are the positions of the three links
% q2 are the positions of the three motors
tic
q = sym('q', [2*n, 1], 'real');
dq = sym('dq', [2*n 1], 'real');
g0 = 9.81;
g = [0 -g0 0]';
disp("Symbolic environment set.");
toc
%% Direct kinematics
tic
% DH table for planar 2R robot under gravity

% FOR 2R
DH_table = [
    0,      0,      l(1),   q(1);
    0,      0,      l(2),   q(2)
];


% disp("The Denavit-Hartenberg table for the robot is:");
% disp(DH_table);

% Ai homogeneous transformation matrix from RF_{i-1} to RF_i
A = zeros([4 4 n], 'sym');
for i=1:n
    A(:, :, i) = dh_matrix(DH_table(i, 1), DH_table(i, 2), ...
        DH_table(i, 3), DH_table(i, 4));
end

% Rotation matrices
R = zeros([3 3 n], 'sym');
for i=1:n
    R(:, :, i) = A(1:3, 1:3, i);
end

% Positions of the center of masses of link i, expressed in RF_i so that
% they are constant (and negative).

% FOR 2R
rc(:, 1) = [-d(1) 0 0]';
rc(:, 2) = [-d(2) 0 0]';


% Positions of CoM_i expressed in RF_0
rc0 = zeros([3 n], 'sym');
A0i = eye(4);
for i=1:n
    A0i = A0i * A(:, :, i);
    rc0_i_hom = A0i * [rc(:, i); 1];
    rc0(:, i) = rc0_i_hom(1:3);
end

% Distances of RF_i from RF_{i-1}, expressed in RF_i so that they are
% constant

% FOR 2R
r(:, 1) = [l(1) 0 0]'; % along y axis since alpha_1 = pi/2
r(:, 2) = [l(2) 0 0]'; 

disp("Relevant kinematic quantities computed.");
toc
%% Derivation of the model: Kinetic energy of the links
tic
% Moving frame algorithm

% Initialization of previous velocities.
w_prev = [0 0 0]';
v_prev = [0 0 0]';

T = 0; % kinetic energy of the links

% constant z vector
z = [0 0 1]';
disp("Deriving kinetic energy of the links...");
for i = 1:n
    Ri = R(:, :, i);    % Rotation matrix from RF_{i-1} to RF_i
    ri = r(:, i);       % Distance of RF_i from RF_{i-1} expressed in RF_i
    rci = rc(:, i);     % Position of CoM of link i expressed in RF_i
    % recursive formula for omega_i
    wi = Ri' * (w_prev + (dq(i) * z));

    % recursive formula for v_i
    vi = Ri' * v_prev + cross(wi, ri);

    % formula for vc_i
    vci = vi + cross(wi, rci);

    wi = simplify(collect(wi, dq(i)));
    vci = simplify(collect(vci, dq(i)));
    
    Ti = (1/2) * m(i) * (vci'*vci) + (1/2) * (wi' * I(:, :, i) * wi);
    Ti = simplify(collect(Ti, dq), 'steps', 100);
%     disp(sprintf("The kinetic energy of link %i is:", i));
%     disp(Ti);
    T = T + Ti;
    
    w_prev = wi;
    v_prev = vi;
end
disp("Done.");
% disp("The kinetic energy of the links is:");
% disp(T);
toc

%% Derivation of the model: Kinetic energy of the motors
% Simply J ddq in the model
J = diag(rg.^2 .* Im_zz);  % Motors inertia matrix

%% Derivation of the model: Potential (gravitational) energy of the links
tic
U_g = 0;
disp("Deriving gravitational potential energy...");
for i = 1:n
    U_gi = -m(i) * (g' * rc0(:, i));
    U_g = U_g + U_gi;
end
disp("Done.");
% disp("The gravitational potential energy is:");
% disp(U_g);
toc

%% Derivation of the model: Potential (elastic) energy of the links
% Simply K(q1-q2) in the model.
K = diag(k_el);

%% Derivation of the model: Rigid manipulator inertia matrix
% Extract entries of inertia matrix
tic
disp("Deriving rigid manipulator inertia matrix...");
M = zeros([n n], 'sym');
for i = 1:n
    for j = 1:n
        M(i,j) = diff(diff(T,dq(i)),dq(j));
    end
end
disp("Done.");
toc
% disp("The rigid manipulator inertia matrix is:");
% disp(M);

%% Derivation of the model: Rigid manipulator Coriolis term
tic
disp("Deriving Coriolis and centrifugal term...");
C = zeros([n 1], 'sym');
for k=1:n
    C_k = 0;
    for i=1:n
        for j=1:n
            c_kij = (1/2)*(diff(M(k, j), q(i)) + diff(M(k, i), q(j)) - diff(M(i, j), q(k)));
            C_k = C_k + (c_kij * dq(i) * dq(j));
        end
    end
    C(k) = C_k;
end
disp("Done.");
toc

%% Derivation of the model: Rigid manipulator Coriolis factorization matrix
tic
disp("Deriving Coriolis factorization matrix...");
S = zeros([n n], 'sym');
for k=1:n
    for j=1:n
        S_kj = 0;
        for i=1:n
           c_kij = (1/2)*( diff(M(k, j), q(i)) + diff(M(k, i), q(j)) - diff(M(i, j), q(k)));
           S_kj = S_kj + (c_kij * dq(i));
        end
        S(k, j) = S_kj;
    end
end
disp("Checking consistency...");

% Check that C(q, dq) = S(q, dq) dq
check = simplify(C - S*dq(1:n), 'steps', 100);
disp(check);

% Check that M_dot - 2S is skew symmetric
syms x [n 1] real
check = simplify(x'*(time_diff(M, q(1:n), dq(1:n)) - 2*S)*x, 'steps', 100);
disp(check);

disp("Done.")
% disp("The rigid manipulator Coriolis factorization matrix is:");
% disp(S);
toc

%% Derivation of the model: Gravity term
tic
disp("Deriving gravity term...");
G = jacobian(U_g, q(1:n))';

G = simplify(G, 'steps', 100);
disp("Done.");
toc
% disp("The rigid manipulator gravity term is:");
% disp(G);


%% END BUILD OF THE MODEL
disp("Model derivation complete.");
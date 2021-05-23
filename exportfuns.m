%% THIS SCRIPT EXPORTS ALL THE SYMBOLIC EXPRESSION REQUIRED FOR THE SIMULATION IN FUNCTIONS
disp("Exporting all the symbolic expression for simulation purposes, may require some time...");
disp("Exporting dynamical model : inertia (M) , coriolis (C), gravity (G),...");
tic;
matlabFunction(M,'Vars',{x1},'File',[pth,'M_fun'],'Optimize',optimize_export);
matlabFunction(c,'Vars',{x1,x2},'File',[pth,'C_fun'],'Optimize',optimize_export);
matlabFunction(S,'Vars',{x1,x2},'File',[pth,'S_fun'],'Optimize',optimize_export);
matlabFunction(G,'Vars',{x1},'File',[pth,'G_fun'],'Optimize',optimize_export);
toc;
disp("Exporting matrix Q(x) , inv(Q(x)) and associated change of coordinates...");
tic;
matlabFunction(Q,'Vars',{[x1;x2;x3;x4]},'File',[pth,'Q_fun'],'Optimize',optimize_export);
matlabFunction(Q1,'Vars',{[x1;x2;x3;x4]},'File',[pth,'Q1_fun'],'Optimize',optimize_export);
matlabFunction(Q2,'Vars',{[x1;x2;x3;x4]},'File',[pth,'Q2_fun'],'Optimize',optimize_export);
matlabFunction(Q3,'Vars',{[x1;x2;x3;x4]},'File',[pth,'Q3_fun'],'Optimize',optimize_export);

% matlabFunction(Qinv,'Vars',{[x1;x2;x3;x4]},'File',[pth,'Qinv_fun'],'Optimize',optimize_export);
matlabFunction(eta_inv,'Vars',{z},'File',[pth,'eta_inv_fun'],'Optimize',optimize_export);
matlabFunction(eta_x,'Vars',{[x1;x2;x3;x4]},'File',[pth,'eta_fun'],'Optimize',optimize_export);
matlabFunction(U,'Vars',{[x1;x2;x3;x4]},'File',[pth,'U_fun'],'Optimize',optimize_export);
toc;
disp("Exporting F(x1,x2,x3,x4) and Spong change of coordinates...");
tic
matlabFunction(F,'Vars',{[x1;x2;x3;x4]},'File',[pth,'F_fun'],'Optimize',optimize_export);
matlabFunction(phi_x,'Vars',{[x1;x2;x3;x4]},'File',[pth,'phix_fun'],'Optimize',optimize_export);
toc;
disp("Exporting the controls...");
tic;
matlabFunction(h0,'Vars',{[x1;x2;d2q;d3q],[q_d;d1q_d;d2q_d;d3q_d;d4q_d]},'File',[pth,'h0_fun'],'Optimize',optimize_export);
matlabFunction(dh0,'Vars',{[x1;x2;d2q;d3q],[q_d;d1q_d;d2q_d;d3q_d;d4q_d]},'File',[pth,'dh0_fun'],'Optimize',optimize_export);
matlabFunction(us,'Vars',{[x1;x2;d2q;d3q],[q_d;d1q_d;d2q_d;d3q_d;d4q_d]},'File',[pth,'us_fun'],'Optimize',optimize_export);
matlabFunction(A2,'Vars',{[x1;x2;d2q;d3q],[q_d;d1q_d;d2q_d;d3q_d;d4q_d]},'File',[pth,'A2_fun'],'Optimize',optimize_export);
matlabFunction(u0,'Vars',{[x1;x2;d2q;d3q],[q_d;d1q_d;d2q_d;d3q_d;d4q_d]},'File',[pth,'u0_fun'],'Optimize',optimize_export);
toc;
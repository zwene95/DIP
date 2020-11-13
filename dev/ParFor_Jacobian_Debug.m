%%
obj_par = CostObject;
obj_seq = CostObject;
%%
obj_par.Problem = problem;
obj_seq.Problem = problem;
%%
setup_par = setup;
setup_seq = setup;
setup_par.Solver.Parallel = 1;
setup_seq.Solver.Parallel = 0;
%%
obj_par.Setup = setup_par;
obj_seq.Setup = setup_seq;
%%
[j_par, j_jac_par] = obj_par.ObservabilityCostFcn(testData{:}); 
[j_seq, j_jac_seq] = obj_seq.ObservabilityCostFcn(testData{:});
max(abs(j_jac_par-j_jac_seq))

%%
[jac_par,jac_seq] = debug_Jacobians(testData{:});



%%
syms data [18 51];
reshape(data,[],1);
% [j_jac_par',j_jac_seq',abs(j_jac_par'-j_jac_seq'),[data;0;0]]
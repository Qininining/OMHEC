function test_core()
here=fileparts(mfilename('fullpath'));
addpath(fullfile(here,'..','src'),fullfile(here,'..','examples'));
xi=[0.1;-0.05;0.02;0.2;0.3;-0.1]; T=mhec.expSE3(xi);
assert(norm(T*mhec.inverseTransform(T)-eye(4),'fro')<1e-12);
assert(norm(T(1:3,1:3)'*T(1:3,1:3)-eye(3),'fro')<1e-12);
cfg=mhec.default_config(); s=mhec.initialize(T,cfg);
p=[1;2;3]; motion=[0.01;0.02;-0.03;0.1;0.2;0.05];
[~,J,V]=mhec.residual_model(T,motion,p,p,eye(3));
assert(all(eig(V)>0));
% The paper Jacobian linearizes the instantaneous motion, not finite Exp motion.
x=mhec.adjointSE3(T)*motion; base=mhec.skew(x(1:3))*p+x(4:6);
numeric=zeros(3,6); h=1e-7;
for k=1:6
    d=zeros(6,1); d(k)=h;
    x=mhec.adjointSE3(mhec.expSE3(d)*T)*motion;
    numeric(:,k)=(mhec.skew(x(1:3))*p+x(4:6)-base)/h;
end
assert(norm(J-numeric,'fro')<1e-6);
for k=1:cfg.observabilityInitializationSteps-1
    s=mhec.update(s,motion,p,p,'iekf');
    assert(isequal(s.Tcs,T));
end
failed=false;
try, mhec.solve_position_only_ls(repmat(eye(4),1,1,6),zeros(3,6));
catch err, failed=contains(err.message,'rank deficient'); end
assert(failed,'Degenerate PO-LS inputs must fail explicitly');
failed=false;
try, mhec.initialize(zeros(4)); catch, failed=true; end
assert(failed);
gold=load(fullfile(here,'fixtures','before_extraction.mat'));
data=generate_dataset(gold.cfg,gold.seed,10,1);
results=run_methods(data);
for m=1:3
    assert(max(abs(results(m).Tcs-gold.histories(m).estimates),[],'all')<1e-8);
    assert(max(abs(results(m).rotation-gold.histories(m).rotation),[],'all')<1e-7);
    assert(max(abs(results(m).translation-gold.histories(m).translation),[],'all')<1e-8);
    if m>1
        a=results(m).covariance(:,:,2:end); b=gold.histories(m).covariance(:,:,2:end);
        assert(max(abs(a-b),[],'all')<1e-8);
        for k=2:size(a,3)
            assert(norm(a(:,:,k)-a(:,:,k)','fro')<1e-10);
            assert(min(eig(a(:,:,k)))>-1e-10);
        end
    end
end
fprintf('PASS: SE(3), Jacobian, initialization, degeneracy and pre-extraction trajectories/covariances.\n');
end

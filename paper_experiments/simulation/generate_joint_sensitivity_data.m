function generate_joint_sensitivity_data(cfg, studyIndices)
% Joint initial-error/excitation sweep used only by Fig. 3.
if nargin<2,studyIndices=1:4;end
folder=fullfile(cfg.dataDir,'joint_sensitivity');if ~isfolder(folder),mkdir(folder);end
for s=studyIndices
 st.initialRotation=s<=2;st.stepRotation=mod(s,2)==1;
 if st.initialRotation,st.levels=cfg.rotationLevelsDeg;type='rotation';else,st.levels=cfg.translationLevels;type='translation';end
 if st.stepRotation,st.steps=cfg.jointRotationStepsDeg;else,st.steps=cfg.jointTranslationSteps;end
 st.statistic='terminal_step';st.stepIndex=cfg.numSteps;
 st.raw=nan(numel(st.levels),numel(st.steps),cfg.numTrials,3,2);
 for ix=1:numel(st.levels)
  for iy=1:numel(st.steps)
   local=cfg;r0=cfg.initialRotationDeg;t0=cfg.initialTranslation;
   if st.initialRotation,r0=st.levels(ix);else,t0=st.levels(ix);end
   if st.stepRotation,local.rotationStep=deg2rad(st.steps(iy));else,local.translationStep=st.steps(iy);end
   for n=1:cfg.numTrials
    seed=cfg.seed+sum(double(type))*10000+n;
    h=simulate_joint_trial(local,seed,r0,t0);
    for m=1:3
     st.raw(ix,iy,n,m,:)=[h(m).rotation(cfg.numSteps+1),h(m).translation(cfg.numSteps+1)];
    end
   end
  end
  fprintf('Fig. 3: study %d, initial error %g\n',s,st.levels(ix));
 end
 assert(all(isfinite(st.raw),'all'));
 studies=repmat(st,1,s);
 save(fullfile(folder,sprintf('study_%d.mat',s)),'cfg','studies','-v7.3');
end
end

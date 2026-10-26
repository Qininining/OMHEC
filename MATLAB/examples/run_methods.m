function results = run_methods(data)
%RUN_METHODS Compare three estimators on identical observations.
n=size(data.twists,2); names={'PO-LS','PR-FGD','PR-IEKF'};
for m=1:3
    state=mhec.initialize(data.initial,data.cfg,data.pointCovariance);
    history.Tcs=repmat(data.initial,1,1,n+1);
    history.covariance=nan(6,6,n+1);
    history.rotation=zeros(n+1,1); history.translation=zeros(n+1,1);
    [history.rotation(1),history.translation(1)]=transformError(data.truth.Tcs,state.Tcs);
    for k=1:n
        if m==1
            if k>=data.cfg.observabilityInitializationSteps
                try
                    state.Tcs=mhec.inverseTransform(mhec.solve_position_only_ls(data.poses(:,:,1:k+1),data.points(:,1:k+1)));
                catch err
                    if ~contains(err.message,'rank deficient'), rethrow(err); end
                end
            end
        else
            if m==2, method='fgd'; else, method='iekf'; end
            state=mhec.update(state,data.twists(:,k),data.points(:,k),data.points(:,k+1),method);
            history.covariance(:,:,k+1)=state.P;
        end
        history.Tcs(:,:,k+1)=state.Tcs;
        [history.rotation(k+1),history.translation(k+1)]=transformError(data.truth.Tcs,state.Tcs);
    end
    history.name=names{m};
    results(m)=history; %#ok<AGROW>
end
end

function [rotationError, translationError] = transformError(truth, estimate)
Rdelta = estimate(1:3, 1:3)' * truth(1:3, 1:3);
cosAngle = min(1, max(-1, (trace(Rdelta) - 1) / 2));
rotationError = rad2deg(acos(cosAngle));
translationError = norm(estimate(1:3, 4) - truth(1:3, 4));
end


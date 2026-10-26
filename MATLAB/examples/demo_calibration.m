function results = demo_calibration(showPlots)
%DEMO_CALIBRATION Minimal method demonstration; no paper plotting or LaTeX.
if nargin<1, showPlots=true; end
here=fileparts(mfilename('fullpath'));
addpath(fullfile(here,'..','src')); addpath(here);
data=generate_dataset();
results=run_methods(data);
for k=1:numel(results)
    fprintf('%s: rotation %.6g deg, translation %.6g mm\n', ...
        results(k).name,results(k).rotation(end),results(k).translation(end));
    disp(results(k).Tcs(:,:,end));
end
if showPlots
    figure('Name','Hand-eye calibration');
    subplot(2,1,1); hold on;
    for k=1:numel(results), plot(0:data.cfg.numSteps,results(k).rotation); end
    xlabel('Update'); ylabel('Rotation error (deg)'); grid on;
    legend({results.name});
    subplot(2,1,2); hold on;
    for k=1:numel(results), plot(0:data.cfg.numSteps,results(k).translation); end
    xlabel('Update'); ylabel('Translation error (mm)'); grid on;
    legend({results.name});
end
end

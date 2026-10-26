function generate_joint_sensitivity_figure(cfg)
root=fullfile(cfg.dataDir,'joint_sensitivity'); w=6.40; h=10.75;
if ~isfolder(cfg.figureDir), mkdir(cfg.figureDir); end
axesWidth=3.05; axesHeight=2.05;
maxima=zeros(1,2);
for sid=1:4
 d=load(fullfile(root,sprintf('study_%d.mat',sid)));
 assert(strcmp(d.studies(sid).statistic,'terminal_step') && d.studies(sid).stepIndex==cfg.numSteps, 'Regenerate Fig. 3 data with the terminal-step statistic.');
 for q=1:2, maxima(q)=max(maxima(q),max(mean(d.studies(sid).raw(:,:,:,:,q),3),[],'all')); end
end
limits=zeros(1,2);
for q=1:2
 spacing=10^floor(log10(maxima(q)/3));
 steps=[1 2 2.5 3 4 5 6 8 10]*spacing; tickStep=steps(find(steps>=maxima(q)/3,1));
 limits(q)=3*tickStep;
end
f=figure('Visible','off','Color','w','Units','centimeters','Position',[1 1 w h], ...
 'PaperUnits','centimeters','PaperSize',[w h],'PaperPosition',[0 0 w h], ...
 'Renderer','opengl','InvertHardcopy','off');
colors=[0 .447 .698;.94 .66 .08;.72 .12 .08]; methods={'PO-LS','PR-FGD','PR-IEKF'};
for m=1:3
 x=.34+(m-1)*1.93;
 annotation(f,'rectangle',[x/w (h-.20)/h .26/w .07/h],'FaceColor',colors(m,:),'EdgeColor','none');
 label(f,[x+.34 h-.29 1.6 .25],methods{m},7.5,'normal','left');
end
rowOrder=[4 3 2 1];
for row=1:4
 s=rowOrder(row);
 a=load(fullfile(root,sprintf('study_%d.mat',s))); st=a.studies(s);
 for metric=1:2
  k=2*(row-1)+metric; x=.16+(metric-1)*3.08; y=h-.42-axesHeight-(row-1)*2.55;
  % Smaller surfaces and tighter packing, with a fixed intended print size.
  ax=axes(f,'Units','centimeters','Position',[x y axesWidth axesHeight]);
  draw_joint_surface(ax,st,metric,[-45 30],.70,limits(metric)); ax.FontSize=6.1; ax.LineWidth=.35;
  ax.GridAlpha=.12; ax.LabelFontSizeMultiplier=1;
  if metric==1, zlabel(ax,'$e_R$ (deg)','Interpreter','latex');
  else, zlabel(ax,'$e_t$ (mm)','Interpreter','latex'); end
  ax.ZLabel.FontSize=7.0; ax.ZLabel.Units='normalized';
  ax.ZLabel.Position=[-.14 .48 0]; ax.ZLabel.Rotation=90;
  if st.initialRotation
   ax.XTick=[1 20 40]; initialLabel='$\|\delta\mathbf{\phi}_0\|_2$ (deg)';
  else
   ax.XTick=[.05 1.2 2]; initialLabel='$\|\delta\mathbf{\rho}_0\|_2$ (mm)';
  end
  if st.stepRotation, ylabel(ax,'$\|{}^c\mathbf{\phi}_k^d\|_2$ (deg)','Interpreter','latex'); else, ylabel(ax,'$\|{}^c\mathbf{\rho}_k^d\|_2$ (mm)','Interpreter','latex'); end
  xlabel(ax,initialLabel,'Interpreter','latex');
  % Match both horizontal-axis labels: typography and mirrored placement.
  set([ax.XLabel ax.YLabel],'FontSize',6.5,'FontWeight','normal', ...
      'HorizontalAlignment','center','VerticalAlignment','middle');
  ax.XLabel.Units='normalized'; ax.XLabel.Position=[.82 -.10 0]; ax.XLabel.Rotation=28;
  ax.YLabel.Units='normalized'; ax.YLabel.Position=[.02 -.10 0]; ax.YLabel.Rotation=-28;
  ax.ZTick=linspace(0,limits(metric),4);
  labels=compose('%g',ax.ZTick); labels(1)={''}; ax.ZTickLabel=labels;
  label(f,[x-.15 y+1.63 .40 .26],sprintf('(%c)',char('a'+k-1)),8.2,'normal','left');
 end
end
drawnow;
print(f,fullfile(cfg.figureDir,'fig_sim_initial_error.png'),'-dpng','-r600');
savefig(f,fullfile(cfg.figureDir,'fig_sim_initial_error.fig'));
% Embed the rendered surface at its exact physical size; direct OpenGL PDF
% output changes text proportions on some MATLAB releases.
texFile=fullfile(cfg.figureDir,'fig_sim_initial_error.tex');
fid=fopen(texFile,'w');assert(fid>=0);closer=onCleanup(@() fclose(fid));
fprintf(fid,'\\documentclass[border=0pt]{standalone}\n\\usepackage{graphicx}\n\\begin{document}\n\\includegraphics[width=6.40cm]{fig_sim_initial_error.png}\n\\end{document}\n');
clear closer;
previous=pwd;restore=onCleanup(@() cd(previous));cd(cfg.figureDir);
[status,output]=system('pdflatex -interaction=batchmode -halt-on-error fig_sim_initial_error.tex');
assert(status==0,'Figure PDF export requires pdflatex and standalone: %s',output);
fprintf('Canvas %.4f x %.2f cm; axes %.4f x %.4f cm.\n',w,h,axesWidth,axesHeight);
end
function label(f,pos,str,size,weight,align)
fp=f.Position; pos=pos./[fp(3) fp(4) fp(3) fp(4)];
interpreter='tex'; if contains(str,'$'), interpreter='latex'; end
font='Times New Roman'; color=[.15 .17 .20];
if ~isempty(regexp(str,'^\([a-h]\)$','once'))
 font='Times New Roman'; weight='normal'; interpreter='none'; color=[0 0 0];
end
annotation(f,'textbox',pos,'String',str,'Interpreter',interpreter,'FontName',font, ...
 'FontSize',size,'FontWeight',weight,'HorizontalAlignment',align,'VerticalAlignment','middle', ...
 'LineStyle','none','Margin',0,'Color',color);
end

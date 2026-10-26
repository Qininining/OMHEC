function checks=draw_joint_surface(ax,st,metric,angles,opacity,sharedLimit)
% View from the minimum-X/minimum-Y quadrant: both small values share
% the near bottom corner, with both axes increasing away from that corner.
if nargin<4, angles=[-45 30]; end
if nargin<5, opacity=.70; end
colors=[0 .447 .698;.94 .66 .08;.72 .12 .08];
values=squeeze(mean(st.raw(:,:,:,:,metric),3));
[X,Y]=meshgrid(st.levels,log2(st.steps)); triangles=[];
for ix=1:size(X,2)-1
    for iy=1:size(X,1)-1
        v=sub2ind(size(X),[iy iy+1 iy+1 iy],[ix ix ix+1 ix+1]);
        triangles=[triangles;v([1 2 3]);v([1 3 4])]; %#ok<AGROW>
    end
end
hold(ax,'on'); heights=zeros(numel(X),3);
for m=1:3
    z=values(:,:,m)'; heights(:,m)=z(:);
    patch(ax,'Faces',triangles,'Vertices',[X(:),Y(:),z(:)],'FaceColor',colors(m,:), ...
        'FaceAlpha',opacity,'EdgeColor','none','FaceLighting','none','HandleVisibility','off');
end
checks=[];
for pair=[1 2;1 3;2 3]'
    [lines,residual]=equalErrorSegments(X(:),Y(:),heights(:,pair(1)),heights(:,pair(2)),triangles);
    assert(residual<1e-10);
    % Keep the exact intersection calculation for verification only.
    % Do not overpaint the translucent sheets with coarse, angular contours.
    checks=[checks;pair',size(lines,1)/3,residual]; %#ok<AGROW>
end
ax.FontName='Times New Roman'; ax.FontSize=10; ax.LineWidth=.6; ax.TickLength=[.007 .007];
ax.XColor=[.30 .32 .35]; ax.YColor=ax.XColor; ax.ZColor=ax.XColor;
ax.GridColor=[.70 .72 .75]; ax.GridAlpha=.13;
grid(ax,'on'); ax.Projection='orthographic'; ax.Box='off';
ax.XDir='normal'; ax.YDir='normal'; ax.ZDir='normal'; view(ax,angles);
ax.XTickLabelRotation=0; ax.YTickLabelRotation=0; pbaspect(ax,[1.12 1 .85]);
ax.YTick=log2(st.steps([1 3 5])); ax.YTickLabel=compose('%g',st.steps([1 3 5]));
xlim(ax,st.levels([1 end])); ylim(ax,log2(st.steps([1 end])));
if st.initialRotation
    ax.XTick=[1 10 20 40]; xlabel(ax,'$\|\delta\mathbf{\phi}_0\|_2$ (deg)','Interpreter','latex');
else
    ax.XTick=[.05 .5 1.2 2]; xlabel(ax,'$\|\delta\mathbf{\rho}_0\|_2$ (mm)','Interpreter','latex');
end
if st.stepRotation, ylabel(ax,'$\Delta r$ (deg)','Interpreter','latex'); else, ylabel(ax,'$\Delta t$ (mm)','Interpreter','latex'); end
top=max(values,[],'all');
if metric==1
    zlabel(ax,'Mean final e_R (deg)');
    lim=8; ax.ZTick=[0 2 4 6 8];
else
    zlabel(ax,'Mean final e_t (mm)');
    lim=2.6; ax.ZTick=[0 .8 1.6 2.4];
end
if nargin>=6, lim=sharedLimit; end
zlim(ax,[0 lim]); labels=compose('%g',ax.ZTick); labels(1)={''}; ax.ZTickLabel=labels;
assert(top<lim && all(isfinite(values),'all'));
assert(all(diff(st.levels)>0) && all(diff(st.steps)>0));
cameraDirection=ax.CameraPosition-ax.CameraTarget;
assert(cameraDirection(1)<0 && cameraDirection(2)<0, ...
    'Both minimum horizontal coordinates must face the viewer.');
end

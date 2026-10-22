function [lines,maxResidual]=equalErrorSegments(x,y,zA,zB,triangles)
% Exact intersections of piecewise-linear surfaces sharing one triangulation.
lines=zeros(0,3); maxResidual=0; tol=1e-12; edges=[1 2;2 3;3 1];
for t=1:size(triangles,1)
    ids=triangles(t,:); d=zA(ids)-zB(ids);
    if all(d>tol) || all(d < -tol) || all(abs(d)<tol), continue; end
    hits=zeros(0,3);
    for e=1:3
        i=ids(edges(e,1)); j=ids(edges(e,2)); di=zA(i)-zB(i); dj=zA(j)-zB(j);
        if abs(di)<tol, hits(end+1,:)=[x(i) y(i) zA(i)]; end %#ok<AGROW>
        if di*dj<0
            alpha=di/(di-dj);
            za=zA(i)+alpha*(zA(j)-zA(i)); zb=zB(i)+alpha*(zB(j)-zB(i));
            maxResidual=max(maxResidual,abs(za-zb));
            hits(end+1,:)=[x(i)+alpha*(x(j)-x(i)),y(i)+alpha*(y(j)-y(i)),za]; %#ok<AGROW>
        end
    end
    if size(hits,1)>=2
        hits=uniquetol(hits,1e-11,'ByRows',true);
        if size(hits,1)==2, lines=[lines;hits;nan(1,3)]; end %#ok<AGROW>
    end
end
end


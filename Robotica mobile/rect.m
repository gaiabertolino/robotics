
function XY = rect(x,y,dx,dy)
    alpha = 0.02;
    nx = 2*dx/alpha;
    for i=1:nx
        XY(i,:) = [x-dx+(alpha*i), y+dy];
    end
    ny = 2*dy/alpha;
    for i=1:ny
        XY(i+nx,:) = [x+dx, y+dy-(alpha*i)];
    end
    for i=1:nx
        XY(i+nx+ny,:) = [x+dx-(alpha*i), y-dy];
    end
    for i=1:ny
        XY(i+nx+ny+nx,:) = [x-dx, y-dy+(alpha*i)];
    end
end
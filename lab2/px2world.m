function xyz = px2world(x,y,f,au,u0,av,v0,R,t)
% Pixel coordinates to world coordinates

% Direction vectors
xc = R(1,:)';
yc = R(2,:)';
zc = R(3,:)';

% Pixel coordinates in camera coordinate system
cX = f*(x - u0) / au;
cY = f*(y - v0) / av;

% Pixel coordinates
xyz = t + cX*xc + cY*yc + f*zc;

end


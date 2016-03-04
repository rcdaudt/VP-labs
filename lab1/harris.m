function coords = harris(image)
% Implementation of Harris corner detector

% To double
im = double(image);

% Get image size
s = size(im);


% Derivative masks
dx = [-1 0 1; -1 0 1; -1 0 1];
dy = dx';

% Image derivatives
Ix = conv2(double(im), dx, 'same');
Iy = conv2(double(im), dy, 'same');

% Gaussian smoothing and calculating components of M
g = fspecial('gaussian',9,1);
Ix2 = conv2(Ix.^2, g, 'same');
Iy2 = conv2(Iy.^2, g, 'same');
Ixy = conv2(Ix.*Iy, g, 'same');


k = ones(3);
Ix2 = conv2(Ix2, k, 'same');
Iy2 = conv2(Iy2, k, 'same');
Ixy = conv2(Ixy, k, 'same');

% Calculating matrix E - 0.270561 seconds
E = zeros(s);
tic
for i = 1:s(1)
    for j = 1:s(2)
        M = [Ix2(i,j) Ixy(i,j);Ixy(i,j) Iy2(i,j)];
%         evs = sort(eig(M));
%         E(i,j) = (evs(1)+evs(2))*evs(1)/evs(2);
        E(i,j) = min(eig(M));
    end
end
toc

% Calculating matrix R - 0.000388 seconds
k = 0.04;
tic
R = Ix2.*Iy2 - Ixy.^2 - k*((Ix2+Iy2).^2);
toc

% Local non-maximum supression
E_sup = lnms(E,5);
R_sup = lnms(R,5);

% figure;
% imshow(E,[]);
% figure;
% imshow(R,[]);


% Finding largest values
N = 81;
E_vec = E_sup(:); % Vectorize E_sup
maxima = sort(E_vec,'descend'); % Sort in descending order
maxima = maxima(1:N); % Select predefined amount N of elements
coords = zeros(N,2); % Initialize vector to store coordinates
for i = 1:N % Find the selected elements
    found = find(E == maxima(i)); % Two steps to avoid problems with multiple values
    [coords(i,1), coords(i,2)] = ind2sub(s,found(1));
end

% Subpixel precision
for i = 1:N
    row = coords(i,1);
    col = coords(i,2);
    [dx,dy] = fit_paraboloid(E(row-1:row+1,col-1:col+1));
    coords(i,:) = coords(i,:) + [dx,dy];
end
    
end


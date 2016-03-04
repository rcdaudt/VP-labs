function out = lnms(A,n)
% Local non-maximum supression

% Note: kernel size = 2*n+1

% Get size
s = size(A);

% Initialise output
out = A;

% Loop through all pixels
for i = 1:s(1)
    for j = 1:s(2)
        % Find boundaries of local kernel
        row_from = max([1 (i-n)]);
        row_to = min([s(1) (i+n)]);
        col_from = max([1 (j-n)]);
        col_to = min([s(2) (j+n)]);
        
        % Find max in region
        local_max = max(max(A(row_from:row_to,col_from:col_to)));
        
        % Supress if not local maximum
        if out(i,j) ~= local_max
            out(i,j) = 0;
        end
    end
end

end


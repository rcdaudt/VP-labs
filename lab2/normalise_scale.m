function normalised = normalise_scale(raw)
% Normalise scale of projection points

normalised = zeros(size(raw));
for i = 1:size(raw,1)
    normalised(i,:) = raw(i,:)./raw(end,:);
end

end


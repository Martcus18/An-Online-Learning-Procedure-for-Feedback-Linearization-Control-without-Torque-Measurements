function [dataset] = dataset_normalization(X)
    if(min(X,[],1) ~= max(X,[],1))
        dataset = (X - min(X,[],1)) ./ (max(X,[],1) - min(X,[],1));
    else
        dataset = X;
end
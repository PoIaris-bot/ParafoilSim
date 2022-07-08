function [varargout] = lepoly(n, x)
if nargout == 1
    if n == 0
        varargout{1} = ones(size(x));
        return;
    end
    if n == 1
        varargout{1} = x;
        return;
    end

    polylst = ones(size(x));
    poly = x;  % L_0(x) = 1, L_1(x) = x
    for k = 2:n  % Three-term recurrence relation:
        % kL_k(x) = (2k - 1)xL_{k - 1}(x) - (k - 1)L_{k - 2}(x)
        polyn = ((2 * k - 1) * x .* poly - (k - 1) * polylst) / k;
        polylst = poly;
        poly = polyn;
    end
    varargout{1} = polyn;
end

if nargout == 2
    if n == 0
        varargout{2} = ones(size(x));
        varargout{1} = zeros(size(x));
        return;
    end
    if n == 1
        varargout{2} = x;
        varargout{1} = ones(size(x));
        return;
    end

    polylst = ones(size(x));
    pderlst = zeros(size(x));
    poly = x;
    pder = ones(size(x));
    % L_0 = 1, L_0' = 0, L_1 = x, L_1' = 1
    for k = 2:n  % Three-term recurrence relation:
        % kL_k(x) = (2k - 1)xL_{k - 1}(x) - (k - 1)L_{k - 2}(x)
        polyn = ((2 * k - 1) * x .* poly - (k - 1) * polylst) / k;
        % L_k'(x) = L_{k - 2}'(x) + (2k - 1)L_{k - 1}(x)
        pdern = pderlst + (2 * k - 1) * poly;
        polylst = poly;
        poly = polyn;
        pderlst = pder;
        pder = pdern;
    end
    varargout{2} = polyn;
    varargout{1} = pdern;
end
end
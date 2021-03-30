function [sorted, idx] = sortsyms(vars)
    [~,idx] = sort(arrayfun(@(x) length(char(x)), vars), 'descend');
    sorted = vars(idx);
end


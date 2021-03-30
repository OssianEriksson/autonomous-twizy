function t = taylor1(f0, f, x)
    t = f0 + subs(diff(f, x), x, 0)*x;
end


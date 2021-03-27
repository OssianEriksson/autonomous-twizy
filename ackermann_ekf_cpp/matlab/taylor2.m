function t = taylor2(f0, f, x)
    df_dx = diff(f, x);
    t = f0 + subs(df_dx, x, 0)*x + subs(diff(df_dx, x), x, 0)*x^2/2;
end


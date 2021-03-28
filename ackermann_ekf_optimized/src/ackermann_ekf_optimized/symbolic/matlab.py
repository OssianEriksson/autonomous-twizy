from sympy import sympify, Matrix

def from_matlab_syms_string(string):
    rows = string.count(';') + 1
    string = string.replace(';', ',')
    expr = sympify(string)
    if isinstance(expr, list):
        expr = Matrix(expr).reshape(rows, int(len(expr) / rows))
    return expr
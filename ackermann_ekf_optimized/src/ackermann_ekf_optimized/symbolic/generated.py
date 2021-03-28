from pathlib import Path
from ackermann_ekf_optimized.symbolic.matlab import from_matlab_syms_string

generated_path = Path(__file__).parent.parent.parent.parent / 'generated'

def from_generated_file(path):
    return from_matlab_syms_string((generated_path / path).read_text())

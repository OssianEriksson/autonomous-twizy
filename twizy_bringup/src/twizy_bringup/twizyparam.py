import yaml
import subprocess
import re
from pathlib import Path
import copy
import math


class _Eval:
    def __init__(self, expr):
        self.expr = expr

    @staticmethod
    def constructor(loader, node):
        return _Eval(node.value)


class _Include:
    @staticmethod
    def constructor(loader, node):
        return parse(subprocess.getoutput(f'echo {node.value}'))


class _Merge:
    @staticmethod
    def constructor(loader, node):
        return _Merge()

    @staticmethod
    def representer(dumper, data):
        return dumper.represent_scalar('!merge', '')


class _Variable:
    def __init__(self, name):
        self.name = name

    @staticmethod
    def constructor(loader, node):
        return _Variable(node.value)


def _merge(src, dst):
    for key, value in src.items():
        if isinstance(value, dict):
            _merge(value, dst.setdefault(key, {}))
        elif key not in dst:
            dst[key] = value
    return dst


def _traverse_dict(d, callback):
    kv = d.copy().items() if isinstance(d, dict) else enumerate(d.copy())
    for k, v in kv:
        for a in (k, v):
            if isinstance(a, (dict, list)):
                _traverse_dict(a, callback)
        callback(d, k, v)


yaml.add_constructor('!merge', _Merge.constructor, yaml.SafeLoader)
yaml.add_constructor('!eval', _Eval.constructor, yaml.SafeLoader)
yaml.add_constructor('!include', _Include.constructor, yaml.SafeLoader)
yaml.add_constructor('!variable', _Variable.constructor, yaml.SafeLoader)

yaml.add_representer(_Merge, _Merge.representer, yaml.SafeDumper)


def parse(path):
    text = Path(path).read_text()
    text = re.sub(r'^( *)<< *:', r'\1!merge :', text, flags=re.M)
    parsed = yaml.load(text, yaml.SafeLoader)

    def extract_variables(root, k, v):
        if isinstance(k, _Variable):
            del root[k]
            variables[k.name] = v
        elif isinstance(v, _Eval):
            del root[k]
    variables = copy.deepcopy(parsed)
    _traverse_dict(variables, extract_variables)

    def eval_expressions(root, k, v):
        if isinstance(v, _Eval):
            root[k] = eval(v.expr, {
                'math': math,
                **variables
            })
        elif isinstance(k, _Variable):
            del root[k]
    _traverse_dict(parsed, eval_expressions)

    text = yaml.safe_dump(parsed)
    text = re.sub(r'^( *).*?!merge.*?(\n?.*?):', r'\1<<:\2 ', text, flags=re.M)
    parsed = yaml.safe_load(text)

    return parsed


def load(path):
    return yaml.safe_dump(parse(path))

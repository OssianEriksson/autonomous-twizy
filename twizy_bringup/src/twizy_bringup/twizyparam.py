"""Enhanced YAML parsing for roslaunch

This script can be used to parse YAML-files before uploading the contents as
parameters to the ROS parameter server in launch files. The following entry in
a roslaunch file accomplishes this:

.. code-block:: XML

    <rosparam subst_value="true">
        $(eval exec('global load; from twizy_bringup.twizyparam import load') or load(f'{find("my_pkg")}/my_params.yaml'))
    </rosparam>

This is quite the hack, but it's atleast something before the launching system
is overhauled in ROS 2
"""

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
        # Arguments to the !include tag will be expanded by the shell, allowing
        # for things like !include $(rospack find my_pkg)/my_file.yaml
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
    """Merges two dictionaries recursively

    :param src: Source dictionary
    :type src: dict
    :param dst: Destination dictionary, will have src merged into it
    :type dst: dict

    :return: dst, now with src merged into it
    """

    for key, value in src.items():
        if isinstance(value, dict):
            _merge(value, dst.setdefault(key, {}))
        elif key not in dst:
            dst[key] = value
    return dst


def _traverse_dict(d, callback):
    """Calls a callback for every key-value pair in d recursively
    
    :param d: A dictionary to traverse
    :type d: list
    :param callback: A callback accepting a dictionary, key and value as
        arguments
    :type callback: callable
    """

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
    """Parse a YAML file

    The following extra tags are supported:

    * !eval: Evaluates a Python expression
    * !include: Includes another YAML file by path
    * !variable: A variable to be used in !eval expressions

    :param path: Path to a YAML file
    :type path: str
    
    :return: A parsed YAML object where the tags defined above have been
        expanded
    """

    # Load the YAML file
    text = Path(path).read_text()
    # Pyyaml does its merging early on, which we don't want. We instead want
    # to e.g. evalueate !eval and !include tags before mergeing. So replace
    # all merges with a custom tag for now so Pyyaml doesn't recognize them
    text = re.sub(r'^( *)<< *:', r'\1!merge :', text, flags=re.M)
    # Parse the YAML string
    parsed = yaml.load(text, yaml.SafeLoader)

    def extract_variables(root, k, v):
        if isinstance(k, _Variable):
            # Variables have so far been stored as objects in the parsed
            # structure to distinguish them from other keys. However we need
            # variables to be accessable by their name like other keys in the
            # structure, so replace the objects keys by string keys where the
            # the string is the variable name
            del root[k]
            variables[k.name] = v
        elif isinstance(v, _Eval):
            # We delete all eval expressions here since we are in general not
            # able to evalueate expressions before the variables have been
            # expanded. Remember that entries in a YAML file are by
            # specification unordered, so we cannot traverse a YAML file line
            # by line, parsing !variables:s and !eval:ss as we go. Instead all
            # !variables are paresed first, then all !eval:s. This does however
            # have the unfortunate side effect of not beeing able to use !eval
            # inside a !variable value
            del root[k]
    variables = copy.deepcopy(parsed)
    # Extract variables
    _traverse_dict(variables, extract_variables)

    def eval_expressions(root, k, v):
        if isinstance(v, _Eval):
            # Evaluate !eval expression. Make the math module available inside
            # the expression
            root[k] = eval(v.expr, {
                'math': math,
                **variables
            })
        elif isinstance(k, _Variable):
            # Remove all variables from the final output
            del root[k]
    # Evaluate !eval expressions only after !variable:s have been parsed
    _traverse_dict(parsed, eval_expressions)

    # Finally convert back to text...
    text = yaml.safe_dump(parsed)
    # ... so we can replace all the !merge tags with << again so mergeing can
    # be taken care of by Pyyaml...
    text = re.sub(r'^( *).*?!merge.*?(\n?.*?):', r'\1<<:\2 ', text, flags=re.M)
    # ... by parsing the string another time
    parsed = yaml.safe_load(text)

    return parsed


def load(path):
    """Load/convert a YAML file

    See :py:func:`parse` for more information.
    
    :param path: Path to a YAML file
    :type path: str

    :return: A plain YAML string where the tags defined above have been
        expanded
    """

    return yaml.safe_dump(parse(path))

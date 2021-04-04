import yaml
import subprocess
import math

class _TwizyLoader(yaml.SafeLoader):
    def __init__(self, stream):
        self.env = {}

        super().__init__(stream)


class _Hidden(yaml.YAMLObject):
    pass


class _Expand(yaml.YAMLObject):
    pass


def _construct_include(loader, node):
    parts = node.value.rsplit(' as ', 1)
    path = subprocess.getoutput(f'echo {parts[0].strip()}')
    parsed = parse(path)

    if len(parts) > 1 and isinstance(loader, _TwizyLoader):
        loader.env[parts[1].strip()] = parsed

    return parsed


def _construct_hidden(loader, node):
    return _Hidden()


def _construct_expand(loader, node):
    return _Expand()


def _construct_eval(loader, node):
    env = {
        **(loader.env if isinstance(loader, _TwizyLoader) else {}),
        'math': math
    }

    return eval(node.value, env)


yaml.add_constructor('!include', _construct_include, _TwizyLoader)
yaml.add_constructor('!hidden', _construct_hidden, _TwizyLoader)
yaml.add_constructor('!expand', _construct_expand, _TwizyLoader)
yaml.add_constructor('!eval', _construct_eval, _TwizyLoader)


def parse(path):
    with open(path, 'r') as f:
        parsed = yaml.load(f, _TwizyLoader)

    def reconstruct(src, dst=None):
        dst = type(src)() if dst is None else dst

        if isinstance(src, dict):
            kv = list(src.items())
            def add(k, v): dst[k] = v
        elif isinstance(src, list):
            kv = enumerate(src)
            def add(k, v): dst.append(v)
        else:
            return src

        for k, v in kv:
            if isinstance(k, _Hidden):
                pass
            elif isinstance(k, _Expand):
                if not isinstance(v, (dict, list)):
                    raise ValueError('Can only !expand dicts and lists')
                if not isinstance(src, type(v)):
                    raise ValueError('Can only !expand into same type')

                for vk, vv in v.items():
                    add(vk, vv)
            elif isinstance(v, (list, dict)):
                child = type(v)()
                add(k, child)
                reconstruct(v, child)
            else:
                add(k, v)
        return dst

    return reconstruct(parsed)


def load(path):
    return yaml.dump(parse(path))


"""Python 3.12 兼容层：给 ROS Noetic 补回已删除的标准库接口。

本文件名必须是 sitecustomize.py。解释器启动执行 site.py 时会自动 import 它
（前提：本目录在 PYTHONPATH 上，且未使用 python -S）。

补丁内容：
1. collections.Mapping 等：Python 3.10+ 挪到 collections.abc，ROS 仍 from collections import Mapping。
2. inspect.getargspec：3.11+ 删除，rospy 等仍在使用。
3. import setuptools：setuptools 69 自带 distutils，rosclean/roslaunch 需要 import distutils。
"""
import collections
import collections.abc
import inspect

_ABC_NAMES = (
    "Mapping",
    "MutableMapping",
    "Sequence",
    "MutableSequence",
    "Iterable",
    "Iterator",
    "Callable",
    "Set",
    "MutableSet",
    "Container",
    "Hashable",
    "Generator",
)

for _name in _ABC_NAMES:
    if not hasattr(collections, _name) and hasattr(collections.abc, _name):
        setattr(collections, _name, getattr(collections.abc, _name))

if not hasattr(inspect, "getargspec"):
    from collections import namedtuple

    _ArgSpec = namedtuple("ArgSpec", "args varargs keywords defaults")

    def _getargspec(func):
        spec = inspect.getfullargspec(func)
        return _ArgSpec(spec.args, spec.varargs, spec.varkw, spec.defaults)

    inspect.getargspec = _getargspec
    inspect.ArgSpec = _ArgSpec

try:
    import setuptools  # noqa: F401
except ImportError:
    pass

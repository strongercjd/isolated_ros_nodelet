"""Python 3.12 已删除标准库 imp，本模块放在 PYTHONPATH 最前，顶替 `import imp`。

ROS Noetic 的 rosunit 等仍调用 imp.load_source / find_module。
这里用 importlib 做最小兼容，不是完整 CPython 2/3 的 imp。
"""
from __future__ import annotations

import importlib
import importlib.machinery
import importlib.util
import os
import sys
import threading

PY_SOURCE = 1
PY_COMPILED = 2
C_EXTENSION = 3
PY_RESOURCE = 4
PKG_DIRECTORY = 5
C_BUILTIN = 6
PY_FROZEN = 7
PY_CODERESOURCE = 8
IMP_HOOK = 9

_lock = threading.Lock()
_lock_count = 0


def acquire_lock():
    global _lock_count
    _lock.acquire()
    _lock_count += 1


def release_lock():
    global _lock_count
    _lock_count -= 1
    _lock.release()


def lock_held():
    return _lock_count > 0


def reload(module):
    return importlib.reload(module)


def get_suffixes():
    suffixes = []
    for suffix in importlib.machinery.EXTENSION_SUFFIXES:
        suffixes.append((suffix, "rb", C_EXTENSION))
    for suffix in importlib.machinery.SOURCE_SUFFIXES:
        suffixes.append((suffix, "r", PY_SOURCE))
    for suffix in importlib.machinery.BYTECODE_SUFFIXES:
        suffixes.append((suffix, "rb", PY_COMPILED))
    return suffixes


def load_source(name, pathname, file=None):
    """按路径加载 .py，等价于旧 imp.load_source（rosunit 常用）。"""
    spec = importlib.util.spec_from_file_location(name, pathname)
    if spec is None or spec.loader is None:
        raise ImportError("cannot load %s from %s" % (name, pathname))
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


def load_compiled(name, pathname, file=None):
    return load_source(name, pathname, file)


def load_module(name, file, pathname, description):
    _suffix, _mode, type_ = description
    if type_ == PKG_DIRECTORY:
        init = os.path.join(pathname, "__init__.py")
        return load_source(name, init)
    return load_source(name, pathname, file)


class NullImporter(object):
    def __init__(self, path):
        if os.path.isdir(path):
            raise ImportError("Not a directory")

    def find_module(self, fullname, path=None):
        return None


def find_module(name, path=None):
    if path is None:
        spec = importlib.util.find_spec(name)
    else:
        spec = importlib.machinery.PathFinder.find_spec(name, path)
    if spec is None:
        raise ImportError("No module named %s" % name)
    origin = spec.origin
    if spec.submodule_search_locations:
        pkg_path = spec.submodule_search_locations[0]
        return (None, pkg_path, ("", "", PKG_DIRECTORY))
    if origin and origin.endswith(".py"):
        return (open(origin, "r"), origin, (".py", "r", PY_SOURCE))
    if origin and origin != "built-in":
        return (None, origin, ("", "", C_EXTENSION))
    return (None, name, ("", "", C_BUILTIN))


def init_builtin(name):
    return importlib.import_module(name)


def init_frozen(_name):
    return None


def is_builtin(name):
    return name in sys.builtin_module_names


def is_frozen(_name):
    return False


def get_magic():
    return importlib.util.MAGIC_NUMBER


def get_tag():
    return sys.implementation.cache_tag

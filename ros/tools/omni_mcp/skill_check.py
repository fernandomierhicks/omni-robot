"""What a skill's code may contain (standard library only: skillbox.py and the skill's own process both run it).

A skill is a small module: a docstring (what it does), SKILL = {...} (a literal: name, says, params, moves, max_s),
constants, imports of math or random, and functions, one of them run(robot, ...). Inside: ordinary Python (loops,
ifs, try, comprehensions, f-strings), no classes, no async, no with, no global, and no name or attribute that starts
with "_" (that keeps out the ways to the interpreter's internals), nor the frame / code attributes that lead there.
"""

import ast

ALLOWED_IMPORTS = ("math", "random")
BANNED_NAMES = {"eval", "exec", "compile", "open", "input", "globals", "locals", "vars", "getattr", "setattr",
                "delattr", "hasattr", "breakpoint", "help", "memoryview", "type", "object", "super", "classmethod",
                "staticmethod", "property", "exit", "quit", "id", "dir", "bytearray", "bytes", "callable",
                "issubclass"}
BANNED_ATTRS = {"gi_frame", "gi_code", "gi_yieldfrom", "gi_running", "cr_frame", "cr_code", "cr_await", "ag_frame",
                "ag_code", "f_globals", "f_locals", "f_back", "f_builtins", "f_code", "f_trace", "tb_frame",
                "tb_next", "co_code", "co_consts", "mro", "format_map", "with_traceback", "func_globals"}
NODES_BANNED = (ast.ClassDef, ast.AsyncFunctionDef, ast.Await, ast.AsyncFor, ast.AsyncWith, ast.With, ast.Global,
                ast.Nonlocal, ast.Yield, ast.YieldFrom)
MAX_CHARS = 20000


def _literal(v) -> bool:
    """A constant: a literal, or dict(...) of literals (SKILL = dict(name=...))."""
    if isinstance(v, ast.Call) and isinstance(v.func, ast.Name) and v.func.id == "dict" and not v.args:
        return all(k.arg and _literal(k.value) for k in v.keywords)
    try:
        ast.literal_eval(v)
        return True
    except (ValueError, TypeError, SyntaxError, MemoryError, RecursionError):
        return False


def problems(code: str) -> list:
    """[] when the skill may run, else what is wrong with it (each a short sentence with its line)."""
    if len(code) > MAX_CHARS:
        return [f"too long ({len(code)} characters, at most {MAX_CHARS})"]
    try:
        tree = ast.parse(code)
    except SyntaxError as e:
        return [f"syntax error, line {e.lineno}: {e.msg}"]
    bad = []
    for n in ast.walk(tree):
        line = getattr(n, "lineno", "?")
        if isinstance(n, NODES_BANNED):
            bad.append(f"line {line}: {type(n).__name__} is not allowed in a skill")
        elif isinstance(n, ast.Import):
            for a in n.names:
                if a.name not in ALLOWED_IMPORTS:
                    bad.append(f"line {line}: import {a.name} (only {', '.join(ALLOWED_IMPORTS)})")
        elif isinstance(n, ast.ImportFrom):
            if n.module not in ALLOWED_IMPORTS or n.level:
                bad.append(f"line {line}: from {n.module} import (only {', '.join(ALLOWED_IMPORTS)})")
        elif isinstance(n, ast.Name):
            if n.id.startswith("_") or n.id in BANNED_NAMES:
                bad.append(f"line {line}: the name {n.id} is not allowed")
        elif isinstance(n, ast.Attribute):
            if n.attr.startswith("_") or n.attr in BANNED_ATTRS:
                bad.append(f"line {line}: the attribute .{n.attr} is not allowed")
        elif isinstance(n, (ast.FunctionDef, ast.Lambda)):
            args = n.args
            for a in args.posonlyargs + args.args + args.kwonlyargs + [x for x in (args.vararg, args.kwarg) if x]:
                if a.arg.startswith("_"):
                    bad.append(f"line {line}: the argument {a.arg} is not allowed")
            if isinstance(n, ast.FunctionDef) and n.name.startswith("_"):
                bad.append(f"line {line}: the function name {n.name} is not allowed")
        elif isinstance(n, ast.keyword) and n.arg and n.arg.startswith("_"):
            bad.append(f"line {line}: the keyword {n.arg} is not allowed")
    top = [n for n in tree.body if not (isinstance(n, ast.Expr) and isinstance(n.value, ast.Constant))]
    for n in top:
        if not isinstance(n, (ast.FunctionDef, ast.Assign, ast.AnnAssign, ast.Import, ast.ImportFrom)):
            bad.append(f"line {n.lineno}: at the top only constants, imports and functions (found "
                       f"{type(n).__name__})")
        elif isinstance(n, (ast.Assign, ast.AnnAssign)) and n.value is not None and not _literal(n.value):
            bad.append(f"line {n.lineno}: at the top only constants, imports and functions (a value computed "
                       f"there runs before run())")
    if not any(isinstance(n, ast.FunctionDef) and n.name == "run" for n in tree.body):
        bad.append("no run(robot, ...) function")
    try:
        compile(tree, "<skill>", "exec")
    except (SyntaxError, ValueError) as e:
        bad.append(f"doesn't compile: {e}")
    return bad[:12]


def metadata(code: str) -> dict:
    """The skill's docstring and its SKILL literal, read without running it."""
    tree = ast.parse(code)
    meta = {}
    for n in tree.body:
        if isinstance(n, ast.Assign) and any(isinstance(t, ast.Name) and t.id == "SKILL" for t in n.targets):
            v = n.value
            if isinstance(v, ast.Call) and isinstance(v.func, ast.Name) and v.func.id == "dict" and not v.args:
                meta = {k.arg: ast.literal_eval(k.value) for k in v.keywords if k.arg}    # SKILL = dict(name=...)
            else:
                meta = ast.literal_eval(v)
    if not isinstance(meta, dict):
        raise ValueError("SKILL must be a dict literal")
    meta = dict(meta)
    meta["description"] = (ast.get_docstring(tree) or "").strip()
    return meta

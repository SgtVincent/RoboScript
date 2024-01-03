import numpy as np
import json 
import copy
from time import sleep
from pygments import highlight
from pygments.lexers import PythonLexer
from pygments.formatters import TerminalFormatter
import ast
import astunparse
from shapely.geometry import *
from shapely.affinity import *
from src.utils import *
import copy


class LMPBase:
    def __init__(self, name, cfg, lmp_fgen, fixed_vars, variable_vars):
        self._name = name
        self._cfg = cfg

        self._base_messages = self._cfg["messages"]

        self._lmp_fgen = lmp_fgen

        self._fixed_vars = fixed_vars
        self._variable_vars = variable_vars
        self.exec_hist = ""
        self.dump_hist = []

    def clear_exec_hist(self):
        self.exec_hist = ""

    def build_messages(self, query, context="")->Tuple[List[Dict], str]:
        messages = copy.deepcopy(self._base_messages)

        query_message = {"role": "user", "content": ""}

        if self._cfg["maintain_session"]:
            query_message["content"] += f"\n{self.exec_hist}"

        if context != "":
            query_message["content"] += f"\n{context}"

            use_query = f'{self._cfg["query_prefix"]}{query}{self._cfg["query_suffix"]}'
            query_message["content"] += f"\n{use_query}"
        else:
            use_query = f'{self._cfg["query_prefix"]}{query}{self._cfg["query_suffix"]}'
            query_message["content"] += use_query

        messages.append(query_message)

        return messages, use_query
    

    def __call__(self, query, context="", **kwargs):
        raise NotImplementedError
    

class LMPFGenBase:
    def __init__(self, cfg, fixed_vars, variable_vars):
        self._cfg = cfg

        self._fixed_vars = fixed_vars
        self._variable_vars = variable_vars

        self._base_messages = self._cfg["messages"]

    def create_new_fs_from_code(self, code_str, other_vars=None, fix_bugs=False, return_src=False):
        fs, f_assigns = {}, {}
        f_parser = FunctionParser(fs, f_assigns)
        f_parser.visit(ast.parse(code_str))
        for f_name, f_assign in f_assigns.items():
            if f_name in fs:
                fs[f_name] = f_assign

        if other_vars is None:
            other_vars = {}

        new_fs = {}
        srcs = {}
        for f_name, f_sig in fs.items():
            all_vars = merge_dicts([self._fixed_vars, self._variable_vars, new_fs, other_vars])
            if not var_exists(f_name, all_vars):
                f, f_src = self.create_f_from_sig(
                    f_name, f_sig, new_fs, fix_bugs=fix_bugs, return_src=True
                )

                # recursively define child_fs in the function body if needed
                f_def_body = astunparse.unparse(ast.parse(f_src).body[0].body)
                child_fs, child_f_srcs = self.create_new_fs_from_code(
                    f_def_body, other_vars=all_vars, fix_bugs=fix_bugs, return_src=True
                )

                if len(child_fs) > 0:
                    new_fs.update(child_fs)
                    srcs.update(child_f_srcs)

                    # redefine parent f so newly created child_fs are in scope
                    gvars = merge_dicts([self._fixed_vars, self._variable_vars, new_fs, other_vars])
                    lvars = {}

                    exec_safe(f_src, gvars, lvars)

                    f = lvars[f_name]

                new_fs[f_name], srcs[f_name] = f, f_src

        if return_src:
            return new_fs, srcs
        return new_fs


    def create_f_from_sig(self, f_name, f_sig, other_vars=None, fix_bugs=False, return_src=False):
        raise NotImplementedError



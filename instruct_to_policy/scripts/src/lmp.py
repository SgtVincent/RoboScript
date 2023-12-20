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
import openai
from openai.error import APIConnectionError, RateLimitError

from src.utils import *
from src.constants import *
from src.config import *
from src.env.env import Env


class LMP:
    def __init__(self, name, cfg, lmp_fgen, fixed_vars, variable_vars):
        self._name = name
        self._cfg = cfg

        self._base_messages = self._cfg["messages"]

        # self._stop_tokens = list(self._cfg["stop"])

        self._lmp_fgen = lmp_fgen

        self._fixed_vars = fixed_vars
        self._variable_vars = variable_vars
        self.exec_hist = ""
        self.dump_hist = []

    def clear_exec_hist(self):
        self.exec_hist = ""

    def build_messages(self, query, context=""):
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
        messages, use_query = self.build_messages(query, context=context)

        while True:
            try:
                response = openai.ChatCompletion.create(
                    messages=messages,
                    # stop=self._stop_tokens,
                    temperature=self._cfg["temperature"],
                    model=self._cfg["model"],
                    max_tokens=self._cfg["max_tokens"],
                )
                code_str = response["choices"][0]["message"]["content"].strip()
                break

            except (RateLimitError, APIConnectionError) as e:
                print(f"OpenAI API got err {e}")
                print("Retrying after 10s.")
                sleep(10)

        if self._cfg["include_context"] and context != "":
            to_exec = f"{context}\n{code_str}"
            to_log = f"{context}\n{use_query}\n{code_str}"
        else:
            to_exec = code_str
            to_log = f"{use_query}\n{to_exec}"

        to_log_pretty = highlight(to_log, PythonLexer(), TerminalFormatter())
        print(f"LMP {self._name} exec:\n\n{to_log_pretty}\n")
        if not self._cfg["debug_mode"]:
            new_fs = self._lmp_fgen.create_new_fs_from_code(code_str)
        else:
            new_fs, src_fs = self._lmp_fgen.create_new_fs_from_code(code_str, return_src=True)
        self._variable_vars.update(new_fs)

        gvars = merge_dicts([self._fixed_vars, self._variable_vars])
        lvars = kwargs

        if not self._cfg["debug_mode"]:
            exec_safe(to_exec, gvars, lvars)
        else:
            # append a dictionary of context, query, src_fs, code_str, gvars and lvars to dump_hist
            self.dump_hist.append(
                {
                    "context": context,
                    "query": use_query,
                    "src_fs": src_fs,
                    "code_str": code_str,
                    "gvars": list(gvars.keys()),
                    "lvars": list(lvars.keys()),
                }
            )

        self.exec_hist += f"\n{to_exec}"

        if self._cfg["maintain_session"]:
            self._variable_vars.update(lvars)

        if self._cfg["has_return"]:
            return lvars[self._cfg["return_val_name"]]


class LMPFGen:
    def __init__(self, cfg, fixed_vars, variable_vars):
        self._cfg = cfg

        # self._stop_tokens = list(self._cfg["stop"])
        self._fixed_vars = fixed_vars
        self._variable_vars = variable_vars

        self._base_messages = self._cfg["messages"]

    def create_f_from_sig(self, f_name, f_sig, other_vars=None, fix_bugs=False, return_src=False):
        print(f"Creating function: {f_sig}")

        use_query = f'{self._cfg["query_prefix"]}{f_sig}{self._cfg["query_suffix"]}'
        query_message = {"role": "user", "content": use_query}
        # prompt = f'{self._base_prompt}\n{use_query}'
        messages = copy.deepcopy(self._base_messages)
        messages.append(query_message)

        while True:
            try:
                response = openai.ChatCompletion.create(
                    messages=messages,
                    # stop=self._stop_tokens,
                    temperature=self._cfg["temperature"],
                    model=self._cfg["model"],
                    max_tokens=self._cfg["max_tokens"],
                )
                f_src = response["choices"][0]["message"]["content"].strip()
                break

            except (RateLimitError, APIConnectionError) as e:
                print(f"OpenAI API got err {e}")
                print("Retrying after 10s.")
                sleep(10)

        if fix_bugs:
            f_src = openai.Edit.create(
                model=self._cfg["model"],
                input="# " + f_src,
                temperature=0,
                instruction="Fix the bug if there is one. Improve readability. Keep same inputs and outputs. Only small changes. No comments.",
            )["choices"][0]["text"].strip()

        if other_vars is None:
            other_vars = {}
        gvars = merge_dicts([self._fixed_vars, self._variable_vars, other_vars])
        lvars = {}

        exec_safe(f_src, gvars, lvars)

        f = lvars[f_name]

        to_print = highlight(f"{use_query}\n{f_src}", PythonLexer(), TerminalFormatter())
        print(f"LMP FGEN created:\n\n{to_print}\n")

        if return_src:
            return f, f_src
        return f

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


# LMP setup utils
def setup_LMP(env, cfg_tabletop):
    # LMP env wrapper
    cfg_tabletop = copy.deepcopy(cfg_tabletop)
    cfg_tabletop["env"] = dict()
    #   cfg_tabletop['env']['init_objs'] = list(env.obj_name_to_id.keys())
    # TODO: load table top from simulation 
    cfg_tabletop["env"]["coords"] = lmp_tabletop_coords
    LMP_env = env

    # prepare vars including APIs and constants
    fixed_vars, variable_vars = prepare_vars(LMP_env)

    # creating the function-generating LMP
    lmp_fgen = LMPFGen(cfg_tabletop["lmps"]["fgen"], fixed_vars, variable_vars)

    # creating other low-level LMPs
    variable_vars.update(
        {
            k: LMP(k, cfg_tabletop["lmps"][k], lmp_fgen, fixed_vars, variable_vars)
            for k in ["parse_obj_name", "parse_question", "transform_shape_pts"]
        }
    )

    # creating the LMP that deals w/ high-level language commands
    lmp_tabletop_ui = LMP(
        "tabletop_ui", cfg_tabletop["lmps"]["tabletop_ui"], lmp_fgen, fixed_vars, variable_vars
    )

    return lmp_tabletop_ui

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
from .lmp_base import LMPBase, LMPFGenBase

from src.openai_api_key import OPENAI_API_KEY
openai.api_key = OPENAI_API_KEY

class LMPOpenAI(LMPBase):
    def __init__(self, name, cfg, lmp_fgen, fixed_vars, variable_vars):
        super().__init__(name, cfg, lmp_fgen, fixed_vars, variable_vars)
        
        self._stop_tokens = self._cfg.get("stop", [])

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


class LMPFGenOpenAI(LMPFGenBase):
    def __init__(self, cfg, fixed_vars, variable_vars):
        super().__init__(cfg, fixed_vars, variable_vars)

        self._stop_tokens = self._cfg.get("stop", [])

    def create_f_from_sig(self, f_name, f_sig, other_vars=None, fix_bugs=False, return_src=False):
        print(f"Creating function: {f_sig}")

        use_query = f'{self._cfg["query_prefix"]}{f_sig}{self._cfg["query_suffix"]}'
        query_message = {"role": "user", "content": use_query}
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

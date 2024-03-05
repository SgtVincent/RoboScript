import numpy as np
import json 
import copy
from time import sleep
from pygments import highlight
from pygments.lexers import PythonLexer
from pygments.formatters import TerminalFormatter
import ast
import astunparse

from src.utils import *
from .lmp_base import LMPBase, LMPFGenBase
from .perplexity import (
    Client, 
    LabsClient, 
    labs_cookies, 
    labs_headers, 
    available_web_gui_models, 
    available_labs_models
)

MISTRAL_EXTRA_PROMPT = \
'''
\nPlease pay attention to the model output format and imitate it. DO NOT generate anything invalid in python scripts. 
'''

class LMPPPLX(LMPBase):
    def __init__(self, name, cfg, lmp_fgen, fixed_vars, variable_vars):
        super().__init__(name, cfg, lmp_fgen, fixed_vars, variable_vars)
        self._stop_tokens = self._cfg.get("stop", [])
        self.query_interval = self._cfg.get("query_interval", 3)
        self.message_priority = self._cfg.get("message_priority", 100)
        self.initialize_client()

    def __del__(self):
        super().__del__()
        # manually delete client to avoid memory leak
        del self.client

    def reset_client(self):
        del self.client 
        self.initialize_client()

    def initialize_client(self):
        '''
        If using GPT-4, use Client with cookies and headers;
        If using Llama, mistral, pplx, use LabsClient. 
        '''
        if self._cfg["model"] in available_labs_models:
            self.client = LabsClient(headers=labs_headers, cookies=labs_cookies)
            # wait for 5s to avoid rate limit error or anti-bot error
            sleep(self.query_interval)
        elif self._cfg["model"] in available_web_gui_models:
            raise ValueError(f"Model {self._cfg['model']} not supported in class LMPPPLX.")
            # self.client = Client(cookies=cookies, headers=headers)
        else:
            raise ValueError(f"Model {self._cfg['model']} not supported in class LMPPPLX.")
        
    def build_messages(self, query, context=""):
        messages, use_query = super().build_messages(query, context)
        # extra processing if neccessary
        # if using perplexity labs, add priority key to each message
        if self._cfg["model"] in available_labs_models:
            for message in messages:
                message["priority"] = self.message_priority

        if self._cfg["model"] in ['mistral-medium']:
            messages[0]['content'] += MISTRAL_EXTRA_PROMPT

        return messages, use_query

    def __call__(self, query, context="", **kwargs):
        messages, use_query = self.build_messages(query, context=context)

        trial_count = 5
        while trial_count > 0:
            try:
                if self._cfg["model"] in available_labs_models:
                    response = self.client.send_messages(
                        messages=messages,
                        model=self._cfg["model"]
                    )
                    code_str = response["output"].strip()
                    sleep(self.query_interval)
                break
            
            except Exception as e:
                print(f"LMPPPLX: Perplexity API got err {e}")
                print(f"Retrying after {self.query_interval}s.")
                sleep(self.query_interval)
                trial_count -= 1
        
        if trial_count == 0:
            print(f"LMPPPLX: Perplexity API failed to generate code for query {use_query}.")
            return None
                
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


class LMPFGenPPLX(LMPFGenBase):
    def __init__(self, cfg, fixed_vars, variable_vars):
        super().__init__(cfg, fixed_vars, variable_vars)
        self._stop_tokens = self._cfg.get("stop", [])
        self.query_interval = self._cfg.get("query_interval", 3)
        self.message_priority = self._cfg.get("message_priority", 100)
        self.initialize_client()
        
    def __del__(self):
        # manually delete client to avoid memory leak
        del self.client
        
    def initialize_client(self):
        '''
        If using GPT-4, use Client with cookies and headers;
        If using Llama, mistral, pplx, use LabsClient. 
        '''
        if self._cfg["model"] in available_labs_models:
            self.client = LabsClient(headers=labs_headers, cookies=labs_cookies)
            # wait for 3s to avoid rate limit error or anti-bot error
            sleep(self.query_interval)
        elif self._cfg["model"] in available_web_gui_models:
            raise NotImplementedError
        else:
            raise ValueError(f"Model {self._cfg['model']} not supported in class LMPPPLX.")

    def build_messages(self, f_sig):
        messages = copy.deepcopy(self._base_messages)

        use_query = f'{self._cfg["query_prefix"]}{f_sig}{self._cfg["query_suffix"]}'
        query_message = {"role": "user", "content": use_query}
        messages.append(query_message)

        # extra processing if neccessary
        # if using perplexity labs, add priority key to each message
        if self._cfg["model"] in available_labs_models:
            for message in messages:
                message["priority"] = self.message_priority

        return messages, use_query

    def create_f_from_sig(self, f_name, f_sig, other_vars=None, fix_bugs=False, return_src=False):
        print(f"Creating function: {f_sig}")

        messages, use_query = self.build_messages(f_sig)

        trial_count = 5
        while trial_count > 0:
            try:
                if self._cfg["model"] in available_labs_models:
                    response = self.client.send_messages(
                        messages=messages,
                        model=self._cfg["model"]
                    )
                    f_src = response['output'].strip()
                    sleep(self.query_interval)
                break 
            except Exception as e:
                print(f"LMPFGenPPLX: Perplexity API got err {e} in")
                print(f"Retrying after {self.query_interval}.")
                sleep(self.query_interval)
                trial_count -= 1
                
        if trial_count == 0:
            print(f"LMPFGenPPLX: Perplexity API failed to generate code for query {use_query}.")
            return None

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

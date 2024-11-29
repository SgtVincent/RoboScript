import numpy as np
import copy
from time import sleep
from pygments import highlight
from pygments.lexers import PythonLexer
from pygments.formatters import TerminalFormatter
import ast
import astunparse
import google.generativeai as genai

from src.utils import *
from .lmp_base import LMPBase, LMPFGenBase

GOOGLE_API_KEY = "Please enter your Google API key here"
genai.configure(api_key=GOOGLE_API_KEY)


GEMINI_FORMAT_PROMPT= \
'''
\nPlease pay attention to the model output format and imitate it. DO NOT import tools yourself. Assume they are already imported in python running context.
'''

def process_response(response)->str:
    '''
    Up to Jan. 2024, Gemini-pro prefers to return a markdown-format document string.
    Need to convert it to a python code string.
    '''
    markdown_str = response.candidates[0].content.parts[0].text.strip()
    code_str = markdown_str.replace("```python", "").replace("```", "")
    return code_str

class LMPGemini(LMPBase):
    def __init__(self, name, cfg, lmp_fgen, fixed_vars, variable_vars):
        super().__init__(name, cfg, lmp_fgen, fixed_vars, variable_vars)
        self.stop_tokens = self._cfg.get("stop", [])
        self.model_name = self._cfg.get("model", "gemini-pro")
        self.temperature = self._cfg.get("temperature", 0.0)
        self.generation_config = genai.types.GenerationConfigDict(
            candidate_count=1,
            temperature=self.temperature,
        )
        if len(self.stop_tokens) > 0:
            self.generation_config["stop_sequences"] = self.stop_tokens

        self.model = genai.GenerativeModel(self.model_name)

    def build_messages(self, query, context=""):
        """
        Build messages for using gemini API.

        The Gemini multi-turn conversations api has input format:
        messages = [
            {
                "role":"user",
                "parts":[
                    {"text": "Write the first line of a story about a magic backpack."}
                ]
            },
            {
                "role": "model",
                "parts":[
                    {"text": "In the bustling city of Meadow brook, lived a young girl named Sophie. She was a bright and curious soul with an imaginative mind."}
                ]
            },
            ...
        ]
        """
        messages, use_query = super().build_messages(query, context)
        # extra processing:
        # convert openai format messages to gemini format messages
        for message in messages:
            message["parts"] = [{"text": message["content"]}]
            del message["content"]
        # role: 'system' key-value pair is not valid in gemini api, replace it with 'role': 'user' and prepend 'System prompt:' to the first message
        # then insert a dummy reply from model right after the first message to make it interleave with the user message, otherwise gemini api will throw error
        # "400: Please ensure that multiturn requests ends with a user role or a function response."
        messages[0]["role"] = "user"
        messages[0]["parts"][0]["text"] = "System prompt: " + messages[0]["parts"][0]["text"] + GEMINI_FORMAT_PROMPT
        messages.insert(1, {"role": "model", "parts": [{"text": "# Understood. This is a dummy reply."}]})
        
        # role: 'assistant' key-value pair is not valid in gemini api, replace it with 'role': 'model'
        for message in messages:
            if message["role"] == "assistant":
                message["role"] = "model"
        
        return messages, use_query        

    def __call__(self, query, context="", **kwargs):
        messages, use_query = self.build_messages(query, context=context)

        while True:
            try:
                response = self.model.generate_content(messages)
                code_str = process_response(response)
                break

            except Exception as e:
                print(f"Gemini API got err {e}")
                print("Retrying after 5s.")
                sleep(5)

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
            new_fs, src_fs = self._lmp_fgen.create_new_fs_from_code(
                code_str, return_src=True
            )
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


class LMPFGenGemini(LMPFGenBase):
    def __init__(self, cfg, fixed_vars, variable_vars):
        super().__init__(cfg, fixed_vars, variable_vars)
        self.stop_tokens = self._cfg.get("stop", [])
        self.model = genai.GenerativeModel("gemini-pro")
        self.temperature = self._cfg.get("temperature", 0.0)
        self.generation_config = genai.types.GenerationConfigDict(
            candidate_count=1,
            temperature=self.temperature,
        )
        if len(self.stop_tokens) > 0:
            self.generation_config["stop_sequences"] = self.stop_tokens


    def build_messages(self, f_sig, base_messages, context=""):
        """
        Build messages for using gemini API.

        The Gemini multi-turn conversations api has input format:
        messages = [
            {
                "role":"user",
                "parts":[
                    {"text": "Write the first line of a story about a magic backpack."}
                ]
            },
            {
                "role": "model",
                "parts":[
                    {"text": "In the bustling city of Meadow brook, lived a young girl named Sophie. She was a bright and curious soul with an imaginative mind."}
                ]
            },
            ...
        ]
        """
        use_query = f'{self._cfg["query_prefix"]}{f_sig}{self._cfg["query_suffix"]}'
        query_message = {"role": "user", "content": use_query}
        messages = copy.deepcopy(base_messages)
        messages.append(query_message)
        # extra processing:
        # convert openai format messages to gemini format messages
        for message in messages:
            message["parts"] = [{"text": message["content"]}]
            del message["content"]
        # role: 'system' key-value pair is not valid in gemini api, replace it with 'role': 'user' and prepend 'System prompt:' to the first message
        # then insert a dummy reply from model right after the first message to make it interleave with the user message, otherwise gemini api will throw error
        # "400: Please ensure that multiturn requests ends with a user role or a function response."
        messages[0]["role"] = "user"
        messages[0]["parts"][0]["text"] = "System prompt: " + messages[0]["parts"][0]["text"] + GEMINI_FORMAT_PROMPT
        messages.insert(1, {"role": "model", "parts": [{"text": "# Understood. This is a dummy reply."}]})
        
        # role: 'assistant' key-value pair is not valid in gemini api, replace it with 'role': 'model'
        for message in messages:
            if message["role"] == "assistant":
                message["role"] = "model"
        
        return messages, use_query   

    def create_f_from_sig(
        self, f_name, f_sig, other_vars=None, fix_bugs=False, return_src=False
    ):
        
        print(f"Creating function: {f_sig}")
        # TODO: add option toggle to add code body and previous functions as context 
        messages, use_query = self.build_messages(f_sig, self._base_messages, context="")
        
        while True:
            try:
                response = self.model.generate_content(messages)
                f_src = process_response(response)
                break

            except Exception as e:
                print(f"LMPFGenGemini: Gemini API got err {e}")
                print("Retrying after 5s.")
                sleep(5)

        if other_vars is None:
            other_vars = {}
        gvars = merge_dicts([self._fixed_vars, self._variable_vars, other_vars])
        lvars = {}

        exec_safe(f_src, gvars, lvars)

        f = lvars[f_name]

        to_print = highlight(
            f"{use_query}\n{f_src}", PythonLexer(), TerminalFormatter()
        )
        print(f"LMP FGEN created:\n\n{to_print}\n")

        if return_src:
            return f, f_src
        return f

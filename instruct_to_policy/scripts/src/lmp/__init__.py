import copy 
from src.utils import prepare_vars

from .lmp_base import LMPBase, LMPFGenBase
from .lmp_openai import LMPOpenAI, LMPFGenOpenAI
# from .lmp_poe import LMPPoe, LMPFGenPoe
# from .lmp_perplexity import LMPPPLX, LMPFGenPPLX
# from .lmp_gemini import LMPGemini, LMPFGenGemini


def setup_LMP(env, cfg_tabletop, debug_mode=False, detached_mode=False)->LMPBase:
    '''
    Setup LMP for tabletop manipulation tasks.
    Args: 
        env: the environment wrapper
        cfg_tabletop: the config file for tabletop manipulation tasks
        debug_mode: whether to run in debug mode. Under debug mode, the LMP will only generate and save the code snippet, but not execute it.
    Returns:
        lmp_tabletop_ui: the LMP instance for tabletop manipulation tasks
    '''
    # TODO: implement debug mode and detached mode
    # change all lmp configs to debug mode if set, used in offline code generation
    if debug_mode:
        for lmp_name, lmp_config in cfg_tabletop['lmps'].items():
            cfg_tabletop['lmps'][lmp_name]['debug_mode'] = True
    
    # get LMP class and get its handle     
    lmp_class = cfg_tabletop["lmps"]["tabletop_ui"]["class"]
    lmp_class = globals()[lmp_class]
    lmp_fgen_class = cfg_tabletop["lmps"]["fgen"]["class"]
    lmp_fgen_class = globals()[lmp_fgen_class]
    
    # LMP env wrapper
    cfg_tabletop = copy.deepcopy(cfg_tabletop)

    # prepare vars including APIs and constants
    fixed_vars, variable_vars = prepare_vars(env, detached_mode=detached_mode)

    # creating the function-generating LMP
    lmp_fgen = lmp_fgen_class(cfg_tabletop["lmps"]["fgen"], fixed_vars, variable_vars)


    # TODO: Should we use LLM here or multimodal LM instead?
    # creating other low-level LMPs
    # variable_vars.update(
    #     {
    #         k: lmp_class(k, cfg_tabletop["lmps"][k], lmp_fgen, fixed_vars, variable_vars)
    #         for k in ["parse_obj_name", "parse_question", "transform_shape_pts"]
    #     }
    # )

    # creating the LMP that deals w/ high-level language commands
    lmp_tabletop_ui = lmp_class(
        "tabletop_ui", cfg_tabletop["lmps"]["tabletop_ui"], lmp_fgen, fixed_vars, variable_vars
    )

    return lmp_tabletop_ui 
from .grounding_base import GroundingBase
from .grounding_gt import GroundingGT
try:
    from .grounding_embodiedgpt import GroundingEmbodiedGPT
except:
    print("Grounding Model: EmbodiedGPT not available.")
try:
    from .grounding_glip import GroundingGLIP
except:
    print("Grounding Model: GLIP not available.")

def create_grounding_model(model_name: str, **kwargs)-> GroundingBase:
    """
    Create grounding model.
    Args:
        model_name: String, name of the grounding model.
        kwargs: Dict, arguments for the grounding model.
    """
    if model_name == "embodiedgpt":
        return GroundingEmbodiedGPT(**kwargs)
    elif model_name == "glip":
        return GroundingGLIP(**kwargs)
    elif model_name == "ground_truth":
        return GroundingGT(**kwargs)
    else:
        raise NotImplementedError(f"Grounding model {model_name}  not supported.")
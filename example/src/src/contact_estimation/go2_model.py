"""
Build a Pinocchio Model/Data for the Unitree Go2 from the MJCF model vendored
at example/model/go2.xml (google-deepmind/mujoco_menagerie/unitree_go2), and
patch in the Pinocchio frames contact_detection.py expects ("body", "fl_foot",
"fr_foot", "rl_foot", "rr_foot") since raw MJCF parsing does not produce them.
"""

import os
from pathlib import Path

import numpy as np
import pinocchio as pino

from contact_detection import ContactDetector

LEG_NAMES = ["FL", "FR", "RL", "RR"]  # matches go2.xml body/actuator declaration order
JOINT_SUFFIXES = ["hip", "thigh", "calf"]  # matches go2.xml joint naming, e.g. "FL_hip_joint"
FOOT_FRAME_NAMES = {"FL": "fl_foot", "FR": "fr_foot", "RL": "rl_foot", "RR": "rr_foot"}
# go2.xml <default class="foot"> geom pos, identical offset for all 4 legs,
# expressed in the parent "{LEG}_calf" body's own local frame.
FOOT_LOCAL_OFFSET = np.array([-0.002, 0.0, -0.213])
BASE_ALIAS_FRAME_NAME = "body"      # contact_detection.py hardcodes "body" for the trunk frame
MJCF_BASE_BODY_FRAME_NAME = "base"  # actual body name in go2.xml

# example/src/src/contact_estimation/go2_model.py -> example/model/go2.xml
DEFAULT_MJCF_PATH = Path(__file__).resolve().parents[3] / "model" / "go2.xml"


def _build_pino_model(mjcf_path: str) -> pino.Model:
    """Build a floating-base Pinocchio Model from MJCF, defensively across
    Pinocchio versions (the MJCF-loading entry point has drifted -- newer
    releases expose pinocchio.shortcuts.buildModelsFromMJCF, older ones
    expose pin.buildModelFromMJCF directly)."""
    root_joint = pino.JointModelFreeFlyer()

    try:
        result = pino.buildModelsFromMJCF(mjcf_path, root_joint=root_joint, geometry_types=[])
        return result[0] if isinstance(result, (tuple, list)) else result
    except (AttributeError, TypeError):
        pass

    try:
        return pino.buildModelFromMJCF(mjcf_path, root_joint)
    except (AttributeError, TypeError):
        pass

    raise RuntimeError(
        "No known Pinocchio MJCF-loading API worked on this version; "
        "inspect `import pinocchio; help(pinocchio)` and update _build_pino_model()."
    )


def _add_go2_frames(model: pino.Model) -> None:
    """Mutates `model` in place, adding the 5 operational frames
    contact_detection.py requires but raw MJCF parsing does not produce."""
    base_frame_id = model.getFrameId(MJCF_BASE_BODY_FRAME_NAME, pino.FrameType.BODY)
    base_frame = model.frames[base_frame_id]
    model.addFrame(pino.Frame(
        BASE_ALIAS_FRAME_NAME, base_frame.parentJoint, base_frame_id,
        base_frame.placement, pino.FrameType.OP_FRAME,
    ))

    for leg, foot_name in FOOT_FRAME_NAMES.items():
        calf_frame_id = model.getFrameId(f"{leg}_calf", pino.FrameType.BODY)
        calf_frame = model.frames[calf_frame_id]
        offset_in_calf = pino.SE3(np.eye(3), FOOT_LOCAL_OFFSET)
        # Frame.placement is relative to the parent JOINT, not the parent
        # frame -- so the foot frame's placement (also relative to that same
        # parent joint) must be composed through the calf frame's own
        # placement, not just the raw offset.
        foot_placement = calf_frame.placement * offset_in_calf
        model.addFrame(pino.Frame(
            foot_name, calf_frame.parentJoint, calf_frame_id,
            foot_placement, pino.FrameType.OP_FRAME,
        ))


def load_go2_model(mjcf_path=None):
    """Returns (model, data) for the Go2, with the 5 extra frames
    contact_detection.py needs already patched in."""
    mjcf_path = str(mjcf_path) if mjcf_path is not None else str(DEFAULT_MJCF_PATH)
    if not os.path.exists(mjcf_path):
        raise FileNotFoundError(f"Go2 MJCF model not found at expected path: {mjcf_path}")

    model = _build_pino_model(mjcf_path)

    if model.nq != 19 or model.nv != 18:
        raise AssertionError(
            f"Expected a floating-base Go2 model with nq=19, nv=18; got nq={model.nq}, nv={model.nv}. "
            "This usually means the MJCF was parsed without a JointModelFreeFlyer root joint, "
            "or the model isn't the expected 12-actuated-joint Go2."
        )

    _add_go2_frames(model)
    data = pino.Data(model)
    return model, data


def build_joint_index_maps(model: pino.Model) -> dict:
    """Looks up each of the 12 actuated joints by name so nothing downstream
    hardcodes an assumption about Pinocchio's internal joint ordering.

    Returns:
        {
          "idx_q": {joint_name: idx_q, ...},   # 12 entries
          "idx_v": {joint_name: idx_v, ...},   # 12 entries
          "order": [joint_name, ...],          # length-12, ascending idx_v --
                                                # this is the order v[6:18]
                                                # and tau must use.
        }
    """
    idx_q, idx_v = {}, {}
    for leg in LEG_NAMES:
        for suffix in JOINT_SUFFIXES:
            joint_name = f"{leg}_{suffix}_joint"
            joint = model.joints[model.getJointId(joint_name)]
            idx_q[joint_name] = joint.idx_q
            idx_v[joint_name] = joint.idx_v
    order = sorted(idx_v.keys(), key=lambda name: idx_v[name])
    return {"idx_q": idx_q, "idx_v": idx_v, "order": order}


def make_go2_contact_detector(model, data, bandwidth=30, freq=200, alg="mixing", contact_param_path=None):
    """Thin factory around ContactDetector that gets `nv` right for Go2
    (18, not the library's default of 19) instead of leaving it to the caller."""
    return ContactDetector(
        bandwidth=bandwidth,
        nv=model.nv,
        freq=freq,
        alg=alg,
        pino_model=model,
        pino_data=data,
        contact_param_path=contact_param_path,
    )

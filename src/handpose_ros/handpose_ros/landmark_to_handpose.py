import numpy as np
import matplotlib.pyplot as plt
from dataclasses import dataclass
from typing import Dict, Tuple, Optional, List
from scipy.spatial.transform import Rotation as R

import sys, os
sys.path.insert(0, os.path.dirname(__file__))
from hand_config_loader import get_config, build_finger_joint_map  # ← 외부 로더 사용

# from algebra_utils import *

# ====================== Config를 모듈 전역에서 미리 준비 ======================
_CFG = get_config()
print(f'=====================================================================================')
print(f'{_CFG}')
print(f'=====================================================================================')

# ---------------------- 유틸 ----------------------
def normalize(v: np.ndarray, eps: float = 1e-12) -> np.ndarray:
    n = np.linalg.norm(v)
    return v if n < eps else (v / n)

def angle_between_vectors(v1: np.ndarray, v2: np.ndarray, normal: np.ndarray, degrees: bool = False) -> float:
    v1u = normalize(v1); v2u = normalize(v2)
    dot_prod = float(np.clip(np.dot(v1u, v2u), -1.0, 1.0))
    cross_prod = np.cross(v1u, v2u)
    theta_abs = np.arctan2(np.linalg.norm(cross_prod), dot_prod)
    sign_dir = np.sign(np.dot(normalize(normal), cross_prod))
    return float(sign_dir * (np.rad2deg(theta_abs) if degrees else theta_abs))

def make_homogeneous(rotm: np.ndarray, trans: np.ndarray) -> np.ndarray:
    T = np.eye(4); T[:3, :3] = rotm; T[:3,  3] = trans.reshape(3); return T

def make_inv_homogeneous(T: np.ndarray) -> np.ndarray:
    Rm = T[:3, :3]; t = T[:3, 3]; Ti = np.eye(4); Ti[:3,:3] = Rm.T; Ti[:3,3] = -Rm.T @ t; return Ti

def draw_frame(ax, T: np.ndarray, label: str = "", scale: float = 0.001, linewidth: float = 2.0):
    o = T[:3, 3]; x, y, z = T[:3, 0], T[:3, 1], T[:3, 2]
    ax.quiver(o[0], o[1], o[2], x[0], x[1], x[2], length=scale, color='r', linewidth=linewidth)
    ax.quiver(o[0], o[1], o[2], y[0], y[1], y[2], length=scale, color='g', linewidth=linewidth)
    ax.quiver(o[0], o[1], o[2], z[0], z[1], z[2], length=scale, color='b', linewidth=linewidth)
    if label: ax.text(o[0], o[1], o[2], f" {label}", fontsize=8)

# ---------------------- 데이터 컨테이너 ----------------------
@dataclass
class HandFrames:
    """Transform matrices constructor
    """
    T_input2wrist: np.ndarray   # camera_frame (e.g. color_frame with realsense) to wrist joint
    T_wrist2input: np.ndarray
    landmarks_wrist: np.ndarray # landmarks (x,y,z) based on wrist frame
    T_wrist2joint: Dict[Tuple[str, str], np.ndarray] # transform matrices from wrist coordinate to desired joint coordinate.

# ---------------------- 메인 클래스 ----------------------
class LandmarkToHandPose:
    """
    Configuration wrist frame (1) and each joint frame (20) from using 21 landmarks (mediapipe)
    """
    # ---- config 기반 상수 ----
    # WRIST_INDEX: int = _WRIST_INDEX
    # JOINT_NAMES: List[str] = _JOINT_NAMES
    # FINGER_JOINT_MAP: Dict[str, Dict[str, int]] = _FINGER_JOINT_MAP
    # FINGER_COLOR: Dict[str, tuple] = _FINGER_COLOR

    WRIST_INDEX: int = _CFG["wrist_index"]
    JOINT_NAMES: List[str] = _CFG["joint_names"]
    FINGER_JOINT_MAP: Dict[str, Dict[str, int]] = {
        finger: dict(zip(_CFG["joint_names"], idxs))
        for finger, idxs in _CFG["fingers"].items()
    }
    FINGER_COLOR: Dict[str, tuple] = {
        finger: tuple(rgba)
        for finger, rgba in _CFG["finger_color_rgba"].items()
    }
    
    def __init__(self, landmarks: Optional[np.ndarray] = None,
                 flip_x: bool = True,
                 flip_y: bool = False,
                 flip_z: bool = False,
                 hand_label: str = "left"):
        self.flip_x = bool(flip_x)
        self.flip_y = bool(flip_y)
        self.flip_z = bool(flip_z)
        self.hand_label = hand_label.lower()
        self.landmarks: Optional[np.ndarray] = None
        self.frames: Optional[HandFrames] = None
        if landmarks is not None:
            self.update_landmarks(landmarks)

    @staticmethod
    def from_excel(path: str, sheet_name: Optional[object] = 0, header=None, flip_x: bool = True) -> "LandmarkToHandPose":
        import pandas as pd
        df = pd.read_excel(path, sheet_name=sheet_name, header=header, engine="openpyxl")
        arr = df.to_numpy(dtype=float)
        assert arr.shape == (21, 3), f"Excel must have shape (21,3), got {arr.shape}"
        return LandmarkToHandPose(arr, flip_x=flip_x)

    def _get_landmark_idx(self, finger: str, joint: Optional[str] = None) -> int:
        if finger == "wrist": return self.WRIST_INDEX
        assert joint is not None, "joint name required."
        return self.FINGER_JOINT_MAP[finger][joint]
    
    def update_landmarks(self, landmarks: np.ndarray) -> None:
        assert landmarks.shape == (21, 3), f"Expected (21,3), got {landmarks.shape}"
        lm = landmarks.astype(float, copy=True)
        if self.flip_x: lm[:, 0] = -lm[:, 0]
        if self.flip_y: lm[:, 1] = -lm[:, 1]
        if self.flip_z: lm[:, 2] = -lm[:, 2]
        self.landmarks = lm
        self.frames = None

    def _build_wrist_frame(self, label: str):
        """Make frame with respect to wrist point. Frame Z vector is opposite A and B.

        Args:
            label (str): Hand type - "left" or "right"

        Raises:
            ValueError: MUst be left or right string

        Returns:
            multi array: Transformation matrices
            T_input2wrist: Homo Trans Matrix from input frame to wrist frame. (in this case, input frame is normalized camera frame)
            T_wrist2input: inverse of T_input2wrist
            lm_wrist: landmark represented by wrist frame
        """
        assert self.landmarks is not None, "call update_landmarks() first."
        lm = self.landmarks
        p0  = lm[self.WRIST_INDEX, :].reshape(3,1)
        mcp = self.JOINT_NAMES[0]  # 첫 관절명 (기본 mcp)
        p5  = lm[self.FINGER_JOINT_MAP["index"][mcp], :].reshape(3,1)
        p17 = lm[self.FINGER_JOINT_MAP["pinky"][mcp], :].reshape(3,1)

        if label == 'right':
            y_hat = normalize(np.cross((p5 - p0).ravel(), (p17 - p0).ravel()))
        elif label == 'left':
            y_hat = normalize(np.cross((p17 - p0).ravel(), (p5 - p0).ravel()))
        else:
            raise ValueError("label must be 'left' or 'right'")

        x_hat = normalize(normalize((p5 - p0).ravel()) + normalize((p17 - p0).ravel()))
        z_hat = normalize(np.cross(x_hat, y_hat))

        ## DY
        # cf) Each column of the rotation matrix represents a unit vector x,y,z repectively.
        R_input2wrist = np.stack([x_hat, y_hat, z_hat], axis=1)
        t_input2wrist = p0.ravel()
        T_input2wrist = make_homogeneous(R_input2wrist, t_input2wrist)
        T_wrist2input = make_inv_homogeneous(T_input2wrist)

        lm_for_homo = np.hstack([lm, np.ones((lm.shape[0], 1))])
        lm_wrist = (T_wrist2input @ lm_for_homo.T).T[:, :3]
        return T_input2wrist, T_wrist2input, lm_wrist

    def _compute_wrist_to_joint_transforms(
        self,
        # label: str,
        lm_wrist: np.ndarray,
        joint_idx: int,
        next_joint_idx: int,
        is_thumb: bool = False,
        thumb_deg_offset: float = 75.0
    ) -> np.ndarray:
        """Compute all coordinate systems of each joint.
        It calculated only rotation z and y if the joint is on index~pinky finger.
        Else (thumb) coordinate system is calculated by rot z-y-x (human thumb is twisted compared with another finger)

        Args:
            lm_wrist (np.ndarray): landmark represented by wrist frame
            joint_idx (int): landmark index (mediapipe definition)
            next_joint_idx (int): next joint number (index)
            is_thumb (bool, optional): select thumb or not(other finger type). Defaults to False.
            thumb_deg_offset (float, optional): twisted angle of thumb. Defaults to 75.0.(degeree)

        Returns:
            np.ndarray: _description_
        """
        p_joint = lm_wrist[joint_idx, :].reshape(3,1)
        p_next  = lm_wrist[next_joint_idx, :].reshape(3,1)

        x_unit = np.array([1,0,0]); y_unit = np.array([0,1,0]); z_unit = np.array([0,0,1])
        
        # skeleton vector and projection it to XY plane
        vec = (p_next - p_joint).ravel()
        vec_proj_xy = vec * np.array([1,1,0])

        psi = angle_between_vectors(x_unit, vec_proj_xy, normal=z_unit, degrees=False)
        Rz  = R.from_euler('z', psi, degrees=False)

        # y_prime: new y-axis by Rz
        y_prime_unit = Rz.apply(y_unit)
        phi = angle_between_vectors(vec_proj_xy, vec, y_prime_unit, degrees=False)
        Ry  = R.from_euler('y', phi, degrees=False)

        R_wrist2joint = (Rz * Ry)
        if is_thumb:
            # thumb is twisted.
            Rx = R.from_euler('x', thumb_deg_offset, degrees=True)
            R_wrist2joint = (R_wrist2joint * Rx)

        return make_homogeneous(R_wrist2joint.as_matrix(), p_joint.ravel())

    def get_frame(self, reference: Tuple[str, str], target: Tuple[str, str]) -> np.ndarray:
        assert self.frames is not None, "call compute() first."
        
        def wrist_to(node: Tuple[str,str]) -> np.ndarray:
            return np.eye(4) if node[0]=='wrist' else self.frames.T_wrist2joint[(node[0], node[1])]
        
        T_wrist2ref = wrist_to(reference)
        T_wrist2tgt = wrist_to(target)
        return make_inv_homogeneous(T_wrist2ref) @ T_wrist2tgt

    def compute(self, label: str) -> HandFrames:
        T_input2wrist, T_wrist2input, lm_wrist = self._build_wrist_frame(label=label)
        T_wrist2joint: Dict[Tuple[str,str], np.ndarray] = {}
        for finger, joints in self.FINGER_JOINT_MAP.items():
            joint_names = list(joints.keys())
            for jpos, jname in enumerate(joint_names):
                jidx = self._get_landmark_idx(finger, jname)
                if jname == "tip":
                    prev_name = joint_names[jpos - 1]
                    prev_T = T_wrist2joint[(finger, prev_name)]
                    R_prev = prev_T[:3,:3]; p_tip = lm_wrist[jidx,:]
                    T_wrist2joint[(finger, "tip")] = make_homogeneous(R_prev, p_tip)
                else:
                    nxt = joint_names[min(jpos+1, len(joint_names)-1)]
                    nxt_idx = self._get_landmark_idx(finger, nxt)
                    T_wrist2joint[(finger, jname)] = self._compute_wrist_to_joint_transforms(
                        lm_wrist=lm_wrist,
                        joint_idx=jidx,
                        next_joint_idx=nxt_idx,
                        is_thumb=(finger=="thumb"),
                        thumb_deg_offset=75 if label=='left' else (75+180) if label=='right' else 75
                    )
        self.frames = HandFrames(
            T_input2wrist=T_input2wrist, T_wrist2input=T_wrist2input,
            landmarks_wrist=lm_wrist, T_wrist2joint=T_wrist2joint
        )
        return self.frames

    def plot(self, show_labels: bool = True, frame_scale: float = 0.02, joint_size: int = 28):
        assert self.frames is not None, "call compute() first."
        lm_wrist = self.frames.landmarks_wrist
        finger_joint_indices: Dict[str, list[int]] = {
            finger: [self.FINGER_JOINT_MAP[finger][jn] for jn in self.FINGER_JOINT_MAP[finger].keys()]
            for finger in self.FINGER_JOINT_MAP.keys()
        }

        fig = plt.figure(); ax = fig.add_subplot(111, projection='3d')
        ax.set_title("All Hand Joint Frames (wrist world)")
        ax.set_xlabel("X"); ax.set_ylabel("Y"); ax.set_zlabel("Z")
        ax.grid(True); ax.set_box_aspect([1,1,1])

        wi = self.WRIST_INDEX
        ax.scatter(lm_wrist[wi,0], lm_wrist[wi,1], lm_wrist[wi,2], s=55, c='k')

        for finger, idx_list in finger_joint_indices.items():
            pts = lm_wrist[idx_list, :]
            color = self.FINGER_COLOR.get(finger, (0,0,0,1))
            ax.scatter(pts[:,0], pts[:,1], pts[:,2], s=joint_size, c=[color], edgecolors='k', linewidths=0.3)
            ax.plot(pts[:,0], pts[:,1], pts[:,2], '-', color=color, linewidth=2.0)
            mcp_idx = idx_list[0]
            ax.plot([lm_wrist[wi,0], lm_wrist[mcp_idx,0]],
                    [lm_wrist[wi,1], lm_wrist[mcp_idx,1]],
                    [lm_wrist[wi,2], lm_wrist[mcp_idx,2]], '--', color=color, linewidth=1.6)

        if show_labels:
            for i in range(lm_wrist.shape[0]):
                ax.text(lm_wrist[i,0], lm_wrist[i,1], lm_wrist[i,2], f" {i}", fontsize=8, color='k')

        draw_frame(ax, np.eye(4), label="wrist", scale=frame_scale)
        for (finger, jname), T in self.frames.T_wrist2joint.items():
            draw_frame(ax, T, label=f"{finger[:2]}-{jname}", scale=frame_scale)

        mins = lm_wrist.min(axis=0); maxs = lm_wrist.max(axis=0)
        span = (maxs - mins).max(); center = (maxs + mins)/2.0
        ax.set_xlim(center[0]-span/2, center[0]+span/2)
        ax.set_ylim(center[1]-span/2, center[1]+span/2)
        ax.set_zlim(center[2]-span/2, center[2]+span/2)
        plt.show()

if __name__ == "__main__":
    # Demo: excel data
    solver = LandmarkToHandPose.from_excel("handpose_lefthand_data2.xlsx", sheet_name=0, header=None, flip_x=False)
    frames = solver.compute(label="right")
    solver.plot(show_labels=True)
    pass

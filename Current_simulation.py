import math
import random

import omni.usd
from pxr import PhysxSchema, Gf
from omni.physx import get_physx_interface
from omni.isaac.dynamic_control import _dynamic_control


def _clamp(x, lo, hi):
    return max(lo, min(hi, x))


def setup(db):
    st = db.internal_state

    st.stage = omni.usd.get_context().get_stage()
    st.physx = get_physx_interface()
    st.dc = _dynamic_control.acquire_dynamic_control_interface()

    # RNG for direction selection (Pattern A)
    st.rng = random.Random(int(db.inputs.dir_seed) if hasattr(db.inputs, "dir_seed") else 0)

    # Cached rigid body handle for velocity
    st.body = st.dc.get_rigid_body(str(db.inputs.prim_path))

    # Install/enable PhysxForceAPI on the prim (applies force each step)
    prim = st.stage.GetPrimAtPath(str(db.inputs.prim_path))
    st.force_api = PhysxSchema.PhysxForceAPI.Apply(prim)
    st.force_api.CreateForceEnabledAttr().Set(True)
    st.force_api.CreateWorldFrameEnabledAttr().Set(True)  # interpret force in world frame

    try:
        st.force_api.CreateModeAttr().Set(PhysxSchema.Tokens.force)
    except Exception:
        st.force_api.CreateModeAttr().Set("force")

    st.force_attr = st.force_api.GetForceAttr()

    # Episode state
    st.prev_time = None
    st.prev_z = None
    st.speed = 0.0

    # Fixed direction for current in XY, sampled per episode
    st.dir_x = 1.0
    st.dir_y = 0.0
    _start_new_episode(st)


def _start_new_episode(st):
    # Sample a new fixed direction in XY each episode
    theta = st.rng.uniform(0.0, 2.0 * math.pi)
    st.dir_x = math.cos(theta)
    st.dir_y = math.sin(theta)

    # Reset speed state (optional: start at mean instead)
    st.speed = 0.0
    st.prev_z = None


def compute(db):
    st = db.internal_state

    dt = float(db.inputs.dt)
    if dt <= 1e-9:
        return

    sim_time = float(db.inputs.sim_time) if hasattr(db.inputs, "sim_time") else None

    # Detect restart/reset of simulation time -> start a new "episode"
    if sim_time is not None:
        if st.prev_time is not None and sim_time + 1e-6 < st.prev_time:
            _start_new_episode(st)
        st.prev_time = sim_time

    path = str(db.inputs.prim_path)

    # Read rigid body pose from PhysX (real-time)
    tr = st.physx.get_rigidbody_transformation(path)
    if not tr.get("ret_val", False):
        db.log_warning(f"No rigidbody transform for {path}. Is this the RigidBody prim?")
        return

    pos = tr["position"]
    z = float(pos[2])  # assumes Z-up

    # Read rigid body velocity (world) from Dynamic Control
    # If handle was invalid at setup time, retry once
    if st.body is None:
        st.body = st.dc.get_rigid_body(path)
    v_body = st.dc.get_rigid_body_linear_velocity(st.body)
    vx_body, vy_body, vz_body = float(v_body[0]), float(v_body[1]), float(v_body[2])

    # Inputs
    z_water = float(db.inputs.water_surface_z)
    sx = float(db.inputs.size_x)
    sy = float(db.inputs.size_y)
    sz = float(db.inputs.size_z)

    rho = float(db.inputs.rho)
    g = float(db.inputs.g)
    Cd = float(db.inputs.Cd)

    mean = float(db.inputs.current_mean)
    sigma = float(db.inputs.current_sigma)
    tau = float(db.inputs.current_tau)

    # ---------- Submergence & buoyancy (axis-aligned box) ----------
    bottom = z - 0.5 * sz
    sub_h = _clamp(z_water - bottom, 0.0, sz)
    submerged_frac = (sub_h / sz) if sz > 1e-9 else 0.0

    A_xy = sx * sy
    V_disp = A_xy * sub_h
    F_buoy_z = rho * g * V_disp

    # Vertical water damping (only in water)
    c_lin_z = 2.0 * rho * A_xy
    F_drag_z = -c_lin_z * vz_body * submerged_frac

    # ---------- Current model: speed varies, direction fixed in this episode ----------
    # Ornstein-Uhlenbeck speed process:
    # dU = -(U-mean)/tau dt + sigma dW
    # discrete: U += (-(U-mean)/tau)*dt + sigma*sqrt(dt)*N(0,1)
    tau_safe = max(tau, 1e-6)
    st.speed += (-(st.speed - mean) / tau_safe) * dt + sigma * math.sqrt(dt) * st.rng.gauss(0.0, 1.0)
    st.speed = max(0.0, st.speed)

    # Water velocity (horizontal only)
    vx_w = st.speed * st.dir_x
    vy_w = st.speed * st.dir_y

    # ---------- Horizontal drag from relative velocity ----------
    vrel_x = vx_body - vx_w
    vrel_y = vy_body - vy_w
    vrel_mag = math.sqrt(vrel_x * vrel_x + vrel_y * vrel_y)

    Fx = 0.0
    Fy = 0.0
    if vrel_mag > 1e-6 and submerged_frac > 0.0:
        nx = vrel_x / vrel_mag
        ny = vrel_y / vrel_mag

        # Projected area of a box against flow direction (flow in XY plane)
        # A_perp ≈ |nx|*(sy*sz) + |ny|*(sx*sz)
        A_perp = abs(nx) * (sy * sz) + abs(ny) * (sx * sz)

        # Quadratic drag:
        # F = -0.5*rho*Cd*A_perp*|vrel|*vrel
        drag_scale = 0.5 * rho * Cd * A_perp * submerged_frac
        Fx = -drag_scale * vrel_mag * vrel_x
        Fy = -drag_scale * vrel_mag * vrel_y

    # Total force (world frame)
    Fz = F_buoy_z + F_drag_z
    st.force_attr.Set(Gf.Vec3f(float(Fx), float(Fy), float(Fz)))

    # Optional debug outputs
    if hasattr(db.outputs, "force_x"):
        db.outputs.force_x = float(Fx)
    if hasattr(db.outputs, "force_y"):
        db.outputs.force_y = float(Fy)
    if hasattr(db.outputs, "force_z"):
        db.outputs.force_z = float(Fz)
    if hasattr(db.outputs, "current_speed"):
        db.outputs.current_speed = float(st.speed)
    if hasattr(db.outputs, "current_dir_x"):
        db.outputs.current_dir_x = float(st.dir_x)
    if hasattr(db.outputs, "current_dir_y"):
        db.outputs.current_dir_y = float(st.dir_y)
    if hasattr(db.outputs, "submerged_height"):
        db.outputs.submerged_height = float(sub_h)
    if hasattr(db.outputs, "submerged_fraction"):
        db.outputs.submerged_fraction = float(submerged_frac)

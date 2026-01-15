import math
import random

import omni.usd
from pxr import PhysxSchema, Gf
from omni.physx import get_physx_interface

# Dynamic Control for real-time rigid body velocity
try:
    from omni.isaac.dynamic_control import _dynamic_control
    _DC_OK = True
except Exception:
    _DC_OK = False


def _clamp(x, lo, hi):
    return max(lo, min(hi, x))


def _sample_unit_vector_3d(rng: random.Random):
    """Uniform random direction on a sphere."""
    u = rng.uniform(-1.0, 1.0)          # cos(theta)
    phi = rng.uniform(0.0, 2.0 * math.pi)
    s = math.sqrt(max(0.0, 1.0 - u*u))
    dx = s * math.cos(phi)
    dy = s * math.sin(phi)
    dz = u
    return dx, dy, dz


def _start_new_episode(st):
    # Random 3D direction for current (fixed during this episode)
    st.dir_x, st.dir_y, st.dir_z = _sample_unit_vector_3d(st.rng)

    # Reset OU speed state (optional: set to mean, but 0 is ok)
    st.speed = 0.0


def setup(db):
    st = db.internal_state

    st.stage = omni.usd.get_context().get_stage()
    st.physx = get_physx_interface()

    # RNG (Pattern A)
    seed = int(db.inputs.dir_seed) if hasattr(db.inputs, "dir_seed") else 0
    st.rng = random.Random(seed)

    # Dynamic control init (velocity)
    st.dc = None
    st.body = None
    if _DC_OK:
        st.dc = _dynamic_control.acquire_dynamic_control_interface()
        st.body = st.dc.get_rigid_body(str(db.inputs.prim_path))

    # Install/enable ForceAPI on the target prim
    prim_path = str(db.inputs.prim_path)
    prim = st.stage.GetPrimAtPath(prim_path)
    if not prim or not prim.IsValid():
        db.log_error(f"Invalid prim_path: {prim_path}")
        st.force_attr = None
        return

    st.force_api = PhysxSchema.PhysxForceAPI.Apply(prim)
    st.force_api.CreateForceEnabledAttr().Set(True)
    st.force_api.CreateWorldFrameEnabledAttr().Set(True)
    try:
        st.force_api.CreateModeAttr().Set(PhysxSchema.Tokens.force)
    except Exception:
        st.force_api.CreateModeAttr().Set("force")
    st.force_attr = st.force_api.GetForceAttr()

    # Episode tracking
    st.prev_time = None
    st.speed = 0.0
    st.dir_x, st.dir_y, st.dir_z = 1.0, 0.0, 0.0

    _start_new_episode(st)


def compute(db):
    st = db.internal_state
    if st.force_attr is None:
        return

    dt = float(db.inputs.dt) if hasattr(db.inputs, "dt") else 0.0
    if dt <= 1e-9:
        return

    # Detect restart: if sim_time goes backward, new episode (new direction)
    if hasattr(db.inputs, "sim_time"):
        sim_time = float(db.inputs.sim_time)
        if st.prev_time is not None and sim_time + 1e-6 < st.prev_time:
            _start_new_episode(st)
        st.prev_time = sim_time

    prim_path = str(db.inputs.prim_path)

    # --- Position from PhysX (real-time) ---
    tr = st.physx.get_rigidbody_transformation(prim_path)
    if not tr.get("ret_val", False):
        db.log_warning(f"No rigidbody transform for {prim_path} (is this the RigidBody prim?)")
        return

    pos = tr["position"]
    x = float(pos[0])
    y = float(pos[1])
    z = float(pos[2])  # assumes Z-up

    # --- Velocity (world) ---
    vx_body = vy_body = vz_body = 0.0
    if _DC_OK and st.dc is not None:
        if st.body is None:
            st.body = st.dc.get_rigid_body(prim_path)
        if st.body is not None:
            v = st.dc.get_rigid_body_linear_velocity(st.body)
            vx_body, vy_body, vz_body = float(v[0]), float(v[1]), float(v[2])

    # --- Inputs ---
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

    # ---------- Buoyancy ----------
    bottom = z - 0.5 * sz
    sub_h = _clamp(z_water - bottom, 0.0, sz)
    submerged_frac = (sub_h / sz) if sz > 1e-9 else 0.0

    A_xy = sx * sy
    V_disp = A_xy * sub_h
    F_buoy_z = rho * g * V_disp

    # ---------- Current: speed varies, direction fixed (3D) ----------
    tau_safe = max(tau, 1e-6)
    # OU speed process: U += (-(U-mean)/tau)*dt + sigma*sqrt(dt)*N(0,1)
    st.speed += (-(st.speed - mean) / tau_safe) * dt + sigma * math.sqrt(dt) * st.rng.gauss(0.0, 1.0)
    st.speed = max(0.0, st.speed)

    vx_w = st.speed * st.dir_x
    vy_w = st.speed * st.dir_y
    vz_w = st.speed * st.dir_z

    # ---------- Drag based on relative velocity (3D) ----------
    vrel_x = vx_body - vx_w
    vrel_y = vy_body - vy_w
    vrel_z = vz_body - vz_w
    vrel_mag = math.sqrt(vrel_x*vrel_x + vrel_y*vrel_y + vrel_z*vrel_z)

    Fx = Fy = F_drag_z = 0.0
    if vrel_mag > 1e-6 and submerged_frac > 0.0:
        nx = vrel_x / vrel_mag
        ny = vrel_y / vrel_mag
        nz = vrel_z / vrel_mag

        # Projected area for an axis-aligned box against flow direction:
        # faces: +/-X area = sy*sz, +/-Y area = sx*sz, +/-Z area = sx*sy
        A_perp = abs(nx) * (sy * sz) + abs(ny) * (sx * sz) + abs(nz) * (sx * sy)

        # Quadratic drag: F = -0.5*rho*Cd*A_perp*|vrel|*vrel
        drag_scale = 0.5 * rho * Cd * A_perp * submerged_frac
        Fx = -drag_scale * vrel_mag * vrel_x
        Fy = -drag_scale * vrel_mag * vrel_y
        F_drag_z = -drag_scale * vrel_mag * vrel_z

    # Extra linear damping in Z (helps reduce bobbing); acts only when submerged
    c_lin_z = 2.0 * rho * A_xy
    F_lin_z = -c_lin_z * vrel_z * submerged_frac

    # Total force
    Fz = F_buoy_z + F_drag_z + F_lin_z
    st.force_attr.Set(Gf.Vec3f(float(Fx), float(Fy), float(Fz)))

    # ---------- Outputs ----------
    # HUD text (2 decimals)
    hud = (
        f"z= {z:.2f}\n"
        f"current speed= {st.speed:.2f} m/s\n"
        f"current dir= ({st.dir_x:.2f}, {st.dir_y:.2f}, {st.dir_z:.2f})"
    )
    if hasattr(db.outputs, "hud_text"):
        db.outputs.hud_text = hud

    if hasattr(db.outputs, "current_speed"):
        db.outputs.current_speed = float(st.speed)
    if hasattr(db.outputs, "current_dir_x"):
        db.outputs.current_dir_x = float(st.dir_x)
    if hasattr(db.outputs, "current_dir_y"):
        db.outputs.current_dir_y = float(st.dir_y)
    if hasattr(db.outputs, "current_dir_z"):
        db.outputs.current_dir_z = float(st.dir_z)
    if hasattr(db.outputs, "z_center"):
        db.outputs.z_center = float(z)

    if hasattr(db.outputs, "force_x"):
        db.outputs.force_x = float(Fx)
    if hasattr(db.outputs, "force_y"):
        db.outputs.force_y = float(Fy)
    if hasattr(db.outputs, "force_z"):
        db.outputs.force_z = float(Fz)

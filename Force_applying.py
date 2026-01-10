import omni.usd
from pxr import PhysxSchema, Gf

def setup(db):
    state = db.internal_state
    state.stage = omni.usd.get_context().get_stage()

    prim = state.stage.GetPrimAtPath(db.inputs.prim_path)
    if not prim or not prim.IsValid():
        db.log_error(f"Invalid prim_path: {db.inputs.prim_path}")
        state.force_api = None
        state.force_attr = None
        return

    # Ensure ForceAPI exists
    state.force_api = PhysxSchema.PhysxForceAPI(prim)
    if not state.force_api:
        state.force_api = PhysxSchema.PhysxForceAPI.Apply(prim)  # :contentReference[oaicite:12]{index=12}

    # Enable + world frame
    state.force_api.CreateForceEnabledAttr().Set(True)           # :contentReference[oaicite:13]{index=13}
    state.force_api.CreateWorldFrameEnabledAttr().Set(True)      # :contentReference[oaicite:14]{index=14}

    # Mode = "force"
    try:
        state.force_api.CreateModeAttr().Set(PhysxSchema.Tokens.force)  # :contentReference[oaicite:15]{index=15}
    except Exception:
        state.force_api.CreateModeAttr().Set("force")

    # Cache the force attribute handle for fast updates
    state.force_attr = state.force_api.GetForceAttr()            # :contentReference[oaicite:16]{index=16}

def compute(db):
    state = db.internal_state
    if state.force_attr is None:
        return

    Fz = float(db.inputs.force_z)

    # Isaac is usually Z-up with gravity (0,0,-9.81), so buoyancy is +Z.
    # Write physxForce:force = (0,0,Fz) :contentReference[oaicite:17]{index=17}
    state.force_attr.Set(Gf.Vec3f(0.0, 0.0, Fz))

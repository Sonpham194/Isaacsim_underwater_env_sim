# This script is executed the first time the script node computes, or the next time
# it computes after this script is modified or the 'Reset' button is pressed.
#
# The following callback functions may be defined in this script:
#     setup(db): Called immediately after this script is executed
#     compute(db): Called every time the node computes (should always be defined)
#     cleanup(db): Called when the node is deleted or the reset button is pressed
# Available variables:
#    db: og.Database The node interface - attributes are exposed in a namespace like db.inputs.foo and db.outputs.bar.
#                    Use db.log_error, db.log_warning to report problems in the compute function.
#    og: The omni.graph.core module


def _clamp(x, lo, hi):
    return max(lo, min(hi, x))

def compute(db):
    z = db.inputs.z_center
    z_water = db.inputs.water_surface_z

    sx = db.inputs.size_x
    sy = db.inputs.size_y
    sz = db.inputs.size_z

    rho = db.inputs.rho
    g = db.inputs.g

    # Axis-aligned cube assumption (no rotation)
    bottom = z - 0.5 * sz

    # submerged height along Z
    sub_h = _clamp(z_water - bottom, 0.0, sz)

    # displaced volume (rectangular prism)
    displaced_vol = sx * sy * sub_h

    # buoyancy magnitude (upward)
    Fz = rho * g * displaced_vol

    db.outputs.force_z = Fz
    if hasattr(db.outputs, "submerged_height"):
        db.outputs.submerged_height = sub_h
    if hasattr(db.outputs, "submerged_fraction"):
        db.outputs.submerged_fraction = (sub_h / sz) if sz > 1e-9 else 0.0

"""Robot classes defined by this repo rather than by OmniGibson.

Importing a class here registers it with OmniGibson's robot registry, which is what lets
config.yaml refer to it by name. Keeping them on this side of the boundary is deliberate: the
BEHAVIOR-1K checkout that the Fetch baseline of chapters 1-20 runs against is never modified.
"""

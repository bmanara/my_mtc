# `my_mtc` package (used alongside `my_ur` package)

This package is supposed to simulate a pick and place for the `my_ur` robot, using Moveit Task Constructor.

Follow instructions on `my_ur` README on how to use this package alongside.

### Known bugs
- [x] Loads of warnings for the "place" planning/stage. Clear all warnings in order to pass... [Should be relatively easy, just tweak some numbers]
- [x] Error regarding no JointController found for arm and gripper joints [fixed, changed move to place to only deal with one group]
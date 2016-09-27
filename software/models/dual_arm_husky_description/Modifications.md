# Modifications to URDF

1. Replace robotiq_s_model_visualization with robotiq_hand_description
2. Currently deactivated bumblee2 (requires pointgrey_camera_description)
3. Currently deactivated PTU
4. Replaced ``l_palm`` and ``r_palm`` with ``left_palm`` and ``right_palm``
5. Fix colours in URDF (because we are using an older Drake that doesn't yet support materials - cf. [#2586](https://github.com/RobotLocomotion/drake/pull/2586))
  1. Replace ``<material name="black"/>`` with ``<material name="black"><color rgba="0 0 0 1"/></material>``
  2. Replace ``<material name="green"/>`` with ``<material name="Gazebo/LightGrey"><color rgba=".7 .7 .7 1.0" /></material>``
  3. Replace ``<material name="yellow"/>`` with ``<material name="yellow"><color rgba="1 1 0 1" /></material>``
  4. base_link: <material name="black"><color rgba="0 0 0 0" /></material>
  5. wheels: <material name="Gazebo/DarkGrey"><color rgba=".175 .175 .175 1.0" /></material>
  6. top_chassis: <material name="Gazebo/DarkYellow"><color rgba=".7 .7 0 1" /></material>
  7. bumper, rear_bumper: <material name="black"><color rgba="0 0 0 0" /></material>
  8. dual_arm_bulkhead: <material name="Gazebo/DarkGrey"><color rgba=".175 .175 .175 1.0" /></material>
  9. shoulder, wrists: LightGrey replaced with DarkGrey <material name="Gazebo/DarkGrey"><color rgba=".175 .175 .175 1.0" /></material>
  10. <material name="Gazebo/LightGrey"><color rgba=".7 .7 .7 1.0" /></material>

#### References

* [Gazebo Material Reference](https://bitbucket.org/osrf/gazebo/src/73fae73d688a67241403ed58636f5da0fd72e314/media/materials/scripts/gazebo.material?at=default&fileviewer=file-view-default)


## Packages left to add:

1. ``flir_ptu_description``
2. ``pointgrey_camera_description``

Both elements are manually commented out in the URDF at the moment.

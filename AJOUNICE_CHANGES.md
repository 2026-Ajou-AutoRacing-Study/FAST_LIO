# AjouNice integration changes

This submodule contains a small project-specific patch on top of the upstream
FAST-LIO repository. Keep these changes minimal when updating the submodule.

## Odometry contract

AjouNice remaps upstream `/Odometry` to `/localization/odom` and uses this
frame contract:

- `header.frame_id`: `camera_init`
- `child_frame_id`: `body` (the IMU origin)
- pose: `camera_init -> body`
- twist: velocity of the `body` origin, expressed in `body`

## Source changes

### `src/IMU_Processing.hpp`

- Added a read-only getter for `angvel_last`.
- `angvel_last` is the latest IMU angular velocity after subtracting the gyro
  bias estimated by the FAST-LIO ESKF.

### `src/laserMapping.cpp`

- Publishes the internal ESKF linear velocity in `Odometry.twist.linear` after
  rotating it from `camera_init` axes into `body` axes.
- Publishes the bias-corrected IMU angular velocity in
  `Odometry.twist.angular`.
- Fills pose covariance before publishing the message, avoiding the upstream
  one-message delay.
- Maps the first six ESKF covariance states to ROS order
  `[x, y, z, rotation-x, rotation-y, rotation-z]` without swapping the
  position and rotation blocks.
- Broadcasts `camera_init -> body` before publishing the odometry message at
  the same timestamp.

## Known limitation

`Odometry.twist.covariance` remains zero. FAST-LIO exposes linear-velocity
state covariance, but a complete 6x6 twist covariance also needs a defined
angular-rate uncertainty model. Consumers must not interpret the current zero
matrix as proven zero uncertainty.

## Update checklist

When changing the submodule revision:

1. Reapply or port the two source changes above.
2. Confirm the ESKF state ordering in `include/use-ikfom.hpp`.
3. Verify `/localization/odom` frame IDs, non-zero twist while moving, and
   matching TF/odometry timestamps.
4. Re-run the FAST-LIO versus Ego evaluation bag.

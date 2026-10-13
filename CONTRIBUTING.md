# Contributing

1. Fork the repository, or create a branch if you have write access.
2. Branch from `jazzy_devel` and open the pull request against `jazzy_devel`, not `main`.
3. Build and test in the container before opening it:

   ```bash
   colcon build --symlink-install --merge-install --cmake-args -DFranka_DIR:PATH=/home/fer_ros2/deps/libfranka/build && colcon test && colcon test-result --test-result-base build
   ```

4. CI must pass, and every pull request is reviewed by the maintainer before it is merged.

Conventions:

- One topic per pull request. Describe what changed and how it was tested.
- C++ is formatted with clang-format 18 using `ros2_ws/src/.clang-format` (`{` on its own
  line everywhere). `colcon test` checks it.
- Python passes `ament_flake8` and `ament_pep257`, both run by `colcon test`.

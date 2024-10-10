1. Setup whoami for `Anymal`:

   ```bash
   source ./setup_shell.sh
   poetry run create_rai_ws --name quadruped --destination-directory src/examples
   ```

2. Add it's documentation:

   ```bash
   cd src/examples/quadruped_whoami/description
   cd documentation
   wget https://www.anybotics.com/anymal-technical-specifications.pdf
   cd -
   ```

   ```bash
   mkdir -p urdf
   cd urdf
   wget https://raw.githubusercontent.com/ANYbotics/anymal_c_simple_description/refs/heads/master/urdf/anymal.urdf
   cd -
   ```

   ```bash
   cd rai
   poetry run parse_whoami_package src/examples/quadruped_whoami/description
   colcon build --symlink-install
   ```

3.

```bash
ros2 run rai_whoami rai_whoami_node --ros-args -p robot_description_package:="quadruped_whoami"
python examples/quadruped-inspection-demo.py

./QuadrupedInspectionDemo.GameLauncher -bg_ConnectToAssetProcessor=0
ros2 topic echo /anomalies
ros2 run rai_nomad nomad --ros-args -p image_topic:=/base/camera_image_color

```

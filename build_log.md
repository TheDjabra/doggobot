# Build log

Dated engineering record: what was done, what broke, and why decisions went the way they
did. Written as the work happens, not reconstructed afterward. Source material for the
final report and the project writeup.

---

## 2026-08-25 - Container and repository set up

**Container.** The final project runs in its own container, `Doggobot`, cloned from the
graded-assignment container `robocar_team_4` rather than built fresh from the upstream
image. The reason is that three fixes made during the assignment live only in that
container's filesystem and would be lost by pulling the stock image again:

1. A settle delay added to `pyvesc/VESC/VESC.py`, fixing a partial-read race where the
   library decoded a 43-byte VESC reply after only 4 bytes had arrived and returned the
   string `None`.
2. `setuptools` pinned to 69.5.1, because the shipped 82.0.1 is above `colcon-core`'s own
   declared ceiling and breaks `--symlink-install`.
3. A corrected `source_ros2` alias that also sources `sensor2_ws`, without which the OAK-D
   packages are invisible to ROS2.

Cloned with `docker commit robocar_team_4 team4_final:base`, then run as `Doggobot`.
Verified after creation that all three fixes survived.

**Domain ID.** Set to 66 to keep this node graph separate from the assignment container's.
This had to be changed in two places, not one: the image's own `bashrc_docker.sh` exports
`ROS_DOMAIN_ID=96` on every interactive shell and overrides whatever `docker run -e` set,
so the authoritative value is the later export in `/root/.bashrc`. Verified through an
actual interactive shell rather than trusting the flag.

**Flags changed from the class `robocar_docker()` function.** Dropped `--device /dev/video0`,
which does not exist on this Pi (only `/dev/video19` through `23`, the ISP and codec nodes)
and would have made `docker run` fail. `--privileged` shares the host `/dev` regardless,
which is how the VESC and OAK-D are reachable.

**Added a source bind mount**, `/home/pi/doggobot` on the host to
`/home/projects/ros2_ws/src/doggobot` in the container. The class container mounts no
source volume, so any code written inside it exists only in a container layer, is not in
version control, and dies with the container or at kit return. A bind mount can only be
added at container creation, so it was free now and would have cost a full recreate later.

**Inherited calibration.** The clone carries the assignment's tuned values, so this
container can drive the ROS2 laps as-is: `Kp 0.2 / Ki 0.0 / Kd 0.1`, steering limits
+/-0.8, `max_throttle 0.382`, `min_throttle 0.363`.

**Note:** the two containers must not run at the same time. Separate domain IDs keep their
node graphs apart but do not stop them contending for the same VESC serial port and the
same OAK-D on the USB bus.

**Repository.** Scaffolded as an `ament_python` package so `colcon build` finds it, and
pushed to GitHub. The Pi holds a working clone at the bind-mount path.

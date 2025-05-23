set -x          # shows each command and its arguments
colcon build --build-base build/release --cmake-args --preset release_gcc
ln -sf build/compile_commands.json .

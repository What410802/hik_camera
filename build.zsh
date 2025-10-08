cd $(dirname $0)
if [ ! -d "config" ]; then
	mkdir config
fi
cd ../..
colcon build --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

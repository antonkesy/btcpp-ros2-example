all: dependencies build bringup

COLCON_BUILD_FLAGS= --symlink-install \
										--continue-on-error --cmake-args \
										-Wno-dev \
										-DCMAKE_EXPORT_COMPILE_COMMANDS=1 \
										-DCMAKE_BUILD_TYPE=Release \
										--no-warn-unused-cli

bringup: build
	@echo "Make sure you the workspace is sourced: 'source install/setup.bash'"
	ros2 launch bringup bringup.launch.py

clean:
	rm build log install -rf
	rm src/tools/Groot/build -rf

dependencies:
	rosdep install --from-paths . --ignore-src -y -r

build: FORCE
	colcon build ${COLCON_BUILD_FLAGS}

webots: FORCE
	webots ./src/webots/worlds/test_world.wbt &

groot: dependencies
	cd ./src/tools && $(MAKE) groot_bringup

FORCE: ;

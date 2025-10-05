all: build start

build: Dockerfile
	docker build --target base -t btcpp_ros2_example .
	docker build --target base -t nvidia_btcpp_ros2_example .

# CPU
start:
	@echo Running container without nvidia
	xhost +local:docker || true
	docker compose up -d btcpp_ros2_example --remove-orphans
	-docker exec -it btcpp_ros2_example bash
	@echo "Remember to type 'make stop' (if desired) since the container continues to run in the background"

connect:
	-docker exec -it btcpp_ros2_example bash
	@echo "Remember to type 'make stop' (if desired) since the container continues to run in the background"

stop:
	docker stop taco_workspace_cpu

# NVIDIA
start-nvidia:
	@echo Running container without nvidia
	xhost +local:docker || true
	docker compose up -d nvidia_btcpp_ros2_example --remove-orphans
	-docker exec -it nvidia_btcpp_ros2_example bash
	@echo "Remember to type 'make stop' (if desired) since the container continues to run in the background"

connect-nvidia:
	-docker exec -it nvidia_btcpp_ros2_example bash
	@echo "Remember to type 'make stop-nvidia' (if desired) since the container continues to run in the background"

stop-nvidia:
	docker stop taco_workspace_nvidia

# CI
ci: Dockerfile
	docker build --target ci -t ci .

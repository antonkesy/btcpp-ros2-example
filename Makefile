all: build start

build: Dockerfile
	docker build --target base -t btcpp_ros2 .

start:
	xhost +local:docker || true
	docker compose up -d btcpp_ros2_example --remove-orphans
	-docker exec -it btcpp_ros2_example bash
	@echo "Remember to type 'make stop' (if desired) since the container continues to run in the background"

connect:
	-docker exec -it btcpp_ros2_example bash
	@echo "Remember to type 'make stop' (if desired) since the container continues to run in the background"

stop:
	docker stop btcpp_ros2_example

ci: Dockerfile
	docker build --target ci -t ci .

clean:
	docker compose down --remove-orphans
	docker rmi btcpp_ros2 ci || true

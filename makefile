all: build start

build: Dockerfile
	docker build --target base -t btcpp_ros2_example .
	docker build --target base -t nvidia_btcpp_ros2_example .

start:
	@echo Running container without nvidia
	xhost +local:docker || true
	docker compose run --rm btcpp_ros2_example

start-nvidia:
	@echo Running container with nvidia
	xhost +local:docker || true
	docker compose run --rm nvidia_btcpp_ros2_example

apt-nvidia-toolkit:
	curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey \
		| gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg \
  && curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
    sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
    tee /etc/apt/sources.list.d/nvidia-container-toolkit.list \
  && \
    apt-get update && \
	apt-get install -y nvidia-container-toolkit
	nvidia-ctk runtime configure --runtime=docker

connect:
	@echo "Root pwd='toor'"
	ssh -p 22 root@localhost

mount:
	fusermount -u /ros2_ws || true
	mkdir -p /ros2_ws
	sshfs -o allow_other root@localhost:/ros2_ws /ros2_ws

ci: Dockerfile
	docker build --target ci -t ci .

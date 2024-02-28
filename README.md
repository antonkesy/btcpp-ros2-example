# example

Simple BTCPP integration example using ROS2 and Webots.

# Preq

- Docker
- Webots (optional)

  - WINDOWS: add WEBOTS_GATEWAY to docker host (WSL)
  ```
  export WEBOTS_GATEWAY=`ip route show | grep -i default | awk '{ print $3}'`
  ```

# how to start

VSCode dev container or `make` to run all

## Remote Webots

### Locally (recommended)

check connection
`nc -zv localhost 1234`
make sure firewall is off for webots port
`ip route show | grep -i default | awk '{ print $3}'` -> ip for controller to connect remotly to host windows

### Inside Docker

# How to edit

## VSCode

[dev container](https://code.visualstudio.com/docs/devcontainers/containers)

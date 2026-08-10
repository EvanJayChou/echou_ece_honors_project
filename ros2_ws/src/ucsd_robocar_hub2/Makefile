.ONESHELL:
SHELL := /bin/bash
.DEFAULT_GOAL := build-docker

.PHONY: build-docker
build-docker:
	@IMG_NAME=${IMG_NAME}
	@SSH_FILE_PATH=${SSH_FILE_PATH}
	if [ "${IMG_NAME}" == "" ]; then
		IMG_NAME="ucsd_robocar:test"
		echo ${IMG_NAME}
	fi
	eval $(ssh-agent)
	if [ -z "$${SSH_FILE_PATH}" ] ; then
		ssh-add ~/.ssh/id_ed25519
	else
		ssh-add ${SSH_FILE_PATH}
	fi
	DOCKER_BUILDKIT=1 docker build \
		--network=host \
		-f tools/image/Dockerfile \
		--target ucsd_robocar2 \
		--ssh default=${SSH_AUTH_SOCK} \
        --build-arg BASE_IMAGE=ros:jazzy-ros-base-noble \
		--build-arg ROS_DISTRO=jazzy \
		--build-arg ROS_SOURCE=jazzy \
		-t $${IMG_NAME} .

.PHONY: docker-cache-clean
docker-cache-clean:
	docker builder prune --all --force

.PHONY: session
session:
	@IMG_NAME="${IMG_NAME}"
	@CONT_NAME="${CONT_NAME}"
	@RUNTIME="${RUNTIME}"
	@TAG="${TAG}"
	@ENTRYPOINT="${ENTRYPOINT}"
	if [ "${CONT_NAME}" == "" ]; then
		CONT_NAME="image_tester"
	fi
	if [ "${TAG}" == "" ]; then
		TAG="stable"
	fi
	if [ "${IMG_NAME}" == "" ]; then
		IMG_NAME="ghcr.io/ucsd-ecemae-148/ucsd_robocar:$${TAG}"
	fi
	if [ "${RUNTIME}" = "nvidia" ]; then
		echo "RUNTIME is set to nvidia"
		xhost +
		docker run \
			--name $${CONT_NAME} \
			--runtime nvidia \
			-it \
			--rm \
			--privileged \
			--net=host \
			--gpus all \
			-e NVIDIA_DRIVER_CAPABILITIES=all \
			-e DISPLAY=${DISPLAY} \
			-v /dev/bus/usb:/dev/bus/usb \
			--device-cgroup-rule='c 189:* rmw' \
			--device /dev/video0 \
			--volume=/dev/input:/dev/input \
			--volume=${HOME}/.Xauthority:/root/.Xauthority:rw \
			--volume=/tmp/.X11-unix/:/tmp/.X11-unix \
			--volume=${PWD}:/home/projects/ros2_ws/src/ucsd_robocar_hub2 \
			$${IMG_NAME} ${ENTRYPOINT}
	else
		xhost +
		docker run \
			--name $${CONT_NAME} \
			-it \
			--rm \
			--privileged \
			--net=host \
			-e DISPLAY=${DISPLAY} \
			-v /dev/bus/usb:/dev/bus/usb \
			--device-cgroup-rule='c 189:* rmw' \
			--device /dev/video0 \
			--volume=/dev/input:/dev/input \
		    --volume=${HOME}/.Xauthority:/root/.Xauthority:rw \
			--volume=/tmp/.X11-unix/:/tmp/.X11-unix \
			--volume=${PWD}:/home/projects/ros2_ws/src/ucsd_robocar_hub2 \
			$${IMG_NAME} ${ENTRYPOINT}
	fi


.PHONY: join-session
join-session:
	@CONT_NAME="${CONT_NAME}"
	docker exec -it ${CONT_NAME} /bin/bash
FROM	ubuntu:20.04

ENV	DEBIAN_FRONTEND=noninteractive

RUN	apt-get -y update && \
	apt-get install -y \
		g++ \
		make \
		cmake \
		build-essential \
		xorg-dev \
		libglu1-mesa-dev \
		mesa-common-dev \
		libglew-dev \
		libglfw3-dev \
		libglm-dev \
	&& rm -rf /var/lib/apt/lists/*

WORKDIR	/usr/src/workspace

COPY	. .

# Set up the environment variable and create the directory
ENV	XDG_RUNTIME_DIR=/tmp/runtime-root
RUN	mkdir -p $XDG_RUNTIME_DIR && chmod 700 $XDG_RUNTIME_DIR

RUN	make re

CMD	["./ft_kalman", "--graph"]


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
		bc \
	&& rm -rf /var/lib/apt/lists/*

WORKDIR	/usr/src/workspace

COPY	. .

# Set up the environment variable and create the directory
ENV	XDG_RUNTIME_DIR=/tmp/runtime-root
RUN	mkdir -p $XDG_RUNTIME_DIR && chmod 700 $XDG_RUNTIME_DIR

ARG	NOISE=1
RUN	ACCELEROMETER_NOISE=$(echo "scale=6; ${NOISE} * 0.001" | bc) \
	&& GPS_NOISE=$(echo "scale=6; ${NOISE} * 0.1" | bc) \
	&& sed -i "s|@ACCELEROMETER_NOISE@|${ACCELEROMETER_NOISE}|" /usr/src/workspace/incs/Define.hpp \
	&& sed -i "s|@GPS_NOISE@|${GPS_NOISE}|" /usr/src/workspace/incs/Define.hpp

RUN	make

RUN	chmod +x entrypoint.sh

ENTRYPOINT ["/usr/src/workspace/entrypoint.sh"]

FROM	ubuntu:20.04

ENV	DEBIAN_FRONTEND=noninteractive

RUN	apt-get -y update \
	&& apt-get install -y --no-install-recommends \
	&& rm -rf /var/lib/apt/lists/*

WORKDIR	/usr/src/workspace

COPY	. .

RUN	chmod +x imu-sensor-stream-linux
RUN	chmod +x entrypoint.sh

ENTRYPOINT ["/usr/src/workspace/entrypoint.sh"]

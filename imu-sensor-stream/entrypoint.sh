#!/bin/bash

if [ "$ENTROPY" -eq 0 ]; then
	exec ./imu-sensor-stream-linux -s 42 -d "${DURATION}" -p 4242
else
	exec ./imu-sensor-stream-linux -e -d "${DURATION}" -p 4242
fi

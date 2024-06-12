#!/bin/bash

if [ "$ENTROPY" -eq 0 ]; then
	if [ "$FILTERSPEED" -eq 0 ]; then
		exec ./imu-sensor-stream-linux -s 42 -d "${DURATION}" -p 4242 -n "${NOISE}"
	else
		exec ./imu-sensor-stream-linux -s 42 -d "${DURATION}" -p 4242 --filterspeed -n "${NOISE}"
	fi
else
	if [ "$FILTERSPEED" -eq 0 ]; then
		exec ./imu-sensor-stream-linux -e -d "${DURATION}" -p 4242 -n "${NOISE}"
	else
		exec ./imu-sensor-stream-linux -e -d "${DURATION}" -p 4242 --filterspeed -n "${NOISE}"
	fi
fi

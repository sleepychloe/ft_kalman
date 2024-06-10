#!/bin/bash

if [ "$GRAPH" -eq 0 ]; then
	exec ./ft_kalman $DURATION
else
	exec ./ft_kalman $DURATION --graph
fi

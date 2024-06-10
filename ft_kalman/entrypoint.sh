#!/bin/bash

if [ "$GRAPH" -eq 0 ]; then
	exec ./ft_kalman
else
	exec ./ft_kalman --graph
fi

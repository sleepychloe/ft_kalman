#!/bin/bash

if [ "$GRAPH" -eq 0 ]; then
	if [ "$ADAPTIVE" -eq 0 ]; then
		exec ./ft_kalman $DURATION
	else
		exec ./ft_kalman $DURATION --adaptive
	fi
else
	if [ "$ADAPTIVE" -eq 0 ]; then
		exec ./ft_kalman $DURATION --graph
	else
		exec ./ft_kalman $DURATION --graph --adaptive
	fi
fi

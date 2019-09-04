#!/bin/bash

echo "OFF\r" > /dev/ttyUSB0

sleep 1

echo "ON\r" > /dev/ttyUSB0

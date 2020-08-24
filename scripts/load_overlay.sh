#!/bin/bash

# Copyright (C) 2020 Swaraj Hota

name=$(basename $1)
name=${name%.*}

mkdir -p /sys/kernel/config/device-tree/overlays/$name
dtc -O dtb -@ $1 -o - > /sys/kernel/config/device-tree/overlays/$name/dtbo

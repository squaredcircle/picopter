#!/bin/bash

mkfs -q /dev/ram1 2048
mkdir -p /piksi_dev
mount /dev/ram1 /piksi_dev
df -H | grep piksi_dev

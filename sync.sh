#!/bin/bash
rsync -arzvh --exclude='.git/' --exclude='ros/build' --exclude='ros/devel' ./ $1

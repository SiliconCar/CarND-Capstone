#!/bin/bash
rsync -arzvh --exclude='*.pyc' --exclude='.git/' --exclude='ros/build' --exclude='ros/devel' ./ $1

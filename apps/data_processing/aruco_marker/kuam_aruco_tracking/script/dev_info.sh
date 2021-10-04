#!/bin/bash

if [ "$#" -ne 1 ]; then
    echo "Please input device number."
    exit 0
fi

v4l2-ctl --list-formats-ext -d $1
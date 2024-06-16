#!/bin/bash

#"sudo apt-get install valgrind"

# --tool=memcheck --leak-check=full 
path=$(date +'%H-%M-%S-%N')
G_SLICE=always-malloc G_DEBUG=gc-friendly  valgrind -v  --tool=memcheck --leak-check=full --track-origins=yes --num-callers=100 --log-file=valgrind-$path.log $@
echo "write log to valgrind-$path.log"


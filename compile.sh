#!/bin/bash

for program in maxflow preflow ; do 
scons program=$program variant=optimized -j 4 
scons program=$program variant=debug -j 4 
if [ "$?" -ne "0" ]; then 
        echo "compile error in $program. exiting."
        exit
fi
done

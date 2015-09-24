#!/bin/bash

sed -i -e "s@\(\s\+\)Inline\s\+{\s\+url\s\+\"\(.*\)\".*@\1Surface {\n\1  visual [ Inline { url \"\2\" } ] # visual\n\1  collision [ Inline { url \"\2\" } ] # collision\n\1} # surface@" $1

# sed -i -e 's@collision \[ Inline { url \"\(.*\)\.wrl@collision [ Inline { url \"collision_directory/\1\.wrl@'

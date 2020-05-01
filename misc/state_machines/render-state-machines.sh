#!/usr/bin/env bash

find . -type f -iname "*.gv" -print0 | xargs -0 -n1 -I{} \
  basename {} .gv | xargs -n1 -I{} \
  dot -Tpng "{}.gv" -o "{}.png"

#!/bin/sh
find . -name "*.v" -exec  sed -i 's/\t/   /g' '{}' \;


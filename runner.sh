#!/bin/sh
set -e

bin=$1
shift

#user="pi"
#host="raspberrypi.local"
#path=""

user="debian" # password: tmppwd
host="c3.oresat.org"
path="ax5043"

target="$user@$host"

scp "$bin" "$target:$path"
ssh -t $target RUST_BACKTRACE=1 "/home/$user/$path/$(basename "$bin")" "$@"

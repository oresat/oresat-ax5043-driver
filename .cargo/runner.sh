#!/bin/sh
set -e

user=$1
host=$2
path=$3
bin=$4
config=$5
shift 5

target="$user@$host"

scp "$bin" "$config" "$target:$path"
ssh -t $target "cd /home/$user/$path/ ; RUST_BACKTRACE=1 ./$(basename "$bin")" "$@"

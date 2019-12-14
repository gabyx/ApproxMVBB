#!/bin/bash

function updateCIConfig() {
    VAR="$1"
    VALUE="$2"
    eval "$VAR='$VALUE'"
    sed -i -E "s@export $VAR=\".*\"@$VAR=\"$VALUE\"@g" "$ROOT_PATH/ci-config.sh"
}

function loadCIConfig() {
    source "$ROOT_PATH/ci-config.sh"
}
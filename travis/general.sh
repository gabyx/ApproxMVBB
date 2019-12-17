#!/bin/bash

if [ -z "$ROOT_PATH" ]; then
    echo "Root path not set!"
    exit 1
fi
APPROXMVBB_CI_CONFIG="$ROOT_PATH/ci-config.sh"

function updateCIConfig() {
    initCIConfig "$APPROXMVBB_CI_CONFIG" || return 1
    VAR="$1"
    VALUE="$2"

    eval "$VAR='$VALUE'"
    if grep -q "$VAR" "$APPROXMVBB_CI_CONFIG"; then
        sed -i "s|export $VAR=.*|export $VAR='$VALUE'|"  "$APPROXMVBB_CI_CONFIG"
    else
        echo "export $VAR='$VALUE'" >> "$APPROXMVBB_CI_CONFIG"
    fi

    return 0
}

function sourceCIConfig() {
    source "$APPROXMVBB_CI_CONFIG"
}

function initCIConfig() {

    [ -f "$1" ] || echo "#!/bin/bash" > "$1"
    return 0
}
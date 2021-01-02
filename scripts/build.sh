#!/bin/bash

#
# A32NX
# Copyright (C) 2020 FlyByWire Simulations and its contributors
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.
#

set -e

# get directory of this script relative to root
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

### HEY YOU!!!!
# WONDERING HOW THIS WORKS?
# Each "step" gets its own function, named "build_xyz".
# When adding a new function, make sure you invoke the
# function at the bottom with the rest.

build_instruments() {
    npm run build:instruments
}

build_fmgc() {
    npm run build:fmgc
}

build_behavior() {
    node "${DIR}/../src/behavior/build.js"
}

build_model() {
    node "${DIR}/../src/model/build.js"
}

build_fbw() {
    "${DIR}/../src/fbw/build.sh"
}

build_fadec() {
    #"${DIR}/../src/fadec/build.sh"
    cp src/fadec/fadec.wasm A32NX/SimObjects/AirPlanes/Asobo_A320_NEO/panel/
}

build_manifests() {
    node "${DIR}/build.js"
}

build_systems() {
    cargo build --target wasm32-wasi --release
    cp target/wasm32-wasi/release/systems.wasm A32NX/SimObjects/AirPlanes/Asobo_A320_NEO/panel/
}

build_metadata() {
    if [ -z "${GITHUB_ACTOR}" ]; then
        GITHUB_ACTOR="$(git log -1 --pretty=format:'%an <%ae>')"
    fi
    if [ -z "${GITHUB_EVENT_NAME}" ]; then
        GITHUB_EVENT_NAME="manual"
    fi
    if [ -z "${GITHUB_REF}" ]; then
        GITHUB_REF="$(git show-ref HEAD)"
    fi
    if [ -z "${GITHUB_SHA}" ]; then
        GITHUB_SHA="$(git show-ref -s HEAD)"
    fi
    GITHUB_BUILT="$(date -u -Iseconds)"
    jq -n \
        --arg built "${GITHUB_BUILT}" \
        --arg ref "${GITHUB_REF##*/}" \
        --arg sha "${GITHUB_SHA}" \
        --arg actor "${GITHUB_ACTOR}" \
        --arg event_name "${GITHUB_EVENT_NAME}" \
        '{ built: $built, ref: $ref, sha: $sha, actor: $actor, event_name: $event_name }' \
        > "${DIR}/../A32NX/build_info.json"
}

if [ -z "$1" ]; then
    set -x
    build_instruments
    build_fmgc
    build_behavior
    build_model
    build_systems
    build_fbw
    build_fadec
else
    name="build_${1}"
    set -x
    $name
fi

# always invoke manifest+metadata because fast and useful
build_manifests
build_metadata

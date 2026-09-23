#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."

pros_toolchain="${PROS_TOOLCHAIN:-$HOME/Library/Application Support/Code/User/globalStorage/sigbots.pros/install/pros-toolchain-macos}"
if [[ -x "$pros_toolchain/bin/arm-none-eabi-g++" ]]; then
    export PATH="$pros_toolchain/bin:$PATH"
fi

exec make "$@"

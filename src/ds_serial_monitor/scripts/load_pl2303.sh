#!/bin/bash
# Install + load pl2303 (calls full build if needed)
exec sudo bash "$(dirname "$0")/build_and_install_pl2303.sh" "$@"

#!/usr/bin/env sh

# Suppress ShellCheck warning about not following external file
# shellcheck disable=SC1091
. "$(poetry env info --path)"/bin/activate

# Suppress ShellCheck warning about not following external file
# shellcheck disable=SC1091

case "$SHELL" in
    *bash)
        . install/setup.bash
        echo "Sourced bash install"
        ;;
    *zsh)
        . install/setup.zsh
        echo "Sourced zsh install"
        ;;
    *fish)
        echo "fish is not supported"
        ;;
    *sh)
        . install/setup.sh
        echo "Sourced sh install."
        ;;
    *)
        echo "Unknown shell: $0"
        ;;
esac

export PYTHONPATH
PYTHONPATH="$(dirname "$(dirname "$(poetry run which python)")")/lib/python$(poetry run python --version | awk '{print $2}' | cut -d. -f1,2)/site-packages:$PYTHONPATH"

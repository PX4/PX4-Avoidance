#!/bin/bash

# If run from the tools directory, fail
if [ ! -d .git ] ; then
  echo "Commit hooks not installed, run from main directory! (./tools/set_up_commit_hooks)"
  exit 1
fi

#If there are already commit hooks installed
if [ -f .git/hooks/pre-commit ] ; then
  echo "You already have commit hooks, manually delete them and run again!"
  exit 1
fi

cat >> .git/hooks/pre-commit <<- EOM
#!/bin/bash
#
# An example hook script to verify what is about to be committed.
# Called by "git commit" with no arguments.  The hook should
# exit with non-zero status after issuing an appropriate message if
# it wants to stop the commit.
#!/bin/bash

STYLE="google"

format_file() {
  file="${1}"
  clang-format-3.8 -i -style=${STYLE} ${1}
  git add ${1}
}

case "${1}" in
  --about )
    echo "Runs clang-format on source files"
    ;;
  * )
    for file in `git diff-index --cached --name-only HEAD` ; do
      if [[ ${file} == *".cpp" ]] || [[ ${file} == *".h" ]] || [[ ${file} == *".hpp" ]]; then
        echo
        format_file "${file}"
      fi
    done
    ;;
esac
EOM
echo "Commit hooks successfully installed!"
exit 0

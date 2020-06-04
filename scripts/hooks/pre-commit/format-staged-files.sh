#!/bin/bash
for FILE in $(git diff-index --cached --name-only HEAD); do
    if [ -f ${FILE} ]; then
        case ${FILE} in
            *.cpp | *.h | *.hpp)
                clang-format -i ${FILE} -style=file
                git add ${FILE}
                ;;
            *.py)
                autopep8 --in-place --aggressive --aggressive ${FILE}
                git add ${FILE}
                ;;
            *)
                ;;
        esac
    fi
done

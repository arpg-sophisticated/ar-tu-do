#!/bin/bash
for FILE in $(git diff-index --cached --name-only HEAD); do
    if [ -f ${FILE} ]; then
        case ${FILE} in
            *.cpp | *.h | *.hpp)
                clang-format -i ${FILE} -style=file
                git add ${FILE}
                ;;
            *)
                ;;
        esac
    fi
done

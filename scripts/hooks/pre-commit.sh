#!/bin/bash
cd "$(git rev-parse --show-toplevel)"

# Clang and Python Format Pre-Commit Hook
scripts/hooks/pre-commit/format-staged-files.sh


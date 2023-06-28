#!/bin/bash

set -e

PYTHON=$1
BUILD_DIR=$2
LIB_FILE=$3

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

STUBS_GEN_DIR="${BUILD_DIR}/franky-stubs-gen"
LIB_DIR="$(dirname "${LIB_FILE}")"

cd "${BUILD_DIR}"
mkdir -p "${STUBS_GEN_DIR}"

${PYTHON} -m venv "${STUBS_GEN_DIR}/venv"
source "${STUBS_GEN_DIR}/venv/bin/activate"
(cd ${SCRIPT_DIR} && python "setup.py" egg_info -e "${STUBS_GEN_DIR}" > /dev/null)
pip install -r "${STUBS_GEN_DIR}/franky_panda.egg-info/requires.txt" > /dev/null
pip install pybind11-stubgen > /dev/null

PYTHONPATH="${LIB_DIR}" pybind11-stubgen --no-setup-py -o "${STUBS_GEN_DIR}" _franky
sed "s/import _franky/import franky._franky as _franky/" "${STUBS_GEN_DIR}/_franky-stubs/__init__.pyi" > "${LIB_DIR}/_franky.pyi"


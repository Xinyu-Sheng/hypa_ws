#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VENV_DIR="${SCRIPT_DIR}/.venv"
PYTHON_BIN="${PYTHON_BIN:-python3}"

if ! command -v "${PYTHON_BIN}" >/dev/null 2>&1; then
  echo "未找到 Python 可执行文件: ${PYTHON_BIN}" >&2
  exit 1
fi

if [[ ! -d "${VENV_DIR}" ]]; then
  "${PYTHON_BIN}" -m venv "${VENV_DIR}"
fi

# shellcheck disable=SC1091
source "${VENV_DIR}/bin/activate"
python -m pip install --upgrade pip
python -m pip install numpy

echo "venv 已准备完成: ${VENV_DIR}"
echo "激活命令: source \"${VENV_DIR}/bin/activate\""

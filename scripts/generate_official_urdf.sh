#!/usr/bin/env bash
set -euo pipefail

usage() {
  printf '%s\n' \
    "Usage:" \
    "  $0 /absolute/path/to/model_input.xacro output.urdf" \
    "" \
    "Example:" \
    "  $0 /home/xense/franka_description/robots/fr3/fr3.urdf.xacro models/fr3_franka_hand.urdf"
}

if [[ ${1:-} == "-h" || ${1:-} == "--help" ]]; then
  usage
  exit 0
fi
if (( $# != 2 )); then
  usage >&2
  exit 2
fi

source_xacro_input=$1
output_path=$2

if [[ "${source_xacro_input}" != /* ]]; then
  echo "The input xacro path must be absolute: ${source_xacro_input}" >&2
  exit 1
fi
if [[ ! -f "${source_xacro_input}" ]]; then
  echo "The input xacro file was not found: ${source_xacro_input}" >&2
  exit 1
fi
if [[ "${source_xacro_input}" != *.xacro ]]; then
  echo "The input file must end with .xacro: ${source_xacro_input}" >&2
  exit 1
fi
if [[ "${output_path}" != *.urdf ]]; then
  echo "The output file must end with .urdf: ${output_path}" >&2
  exit 1
fi

source_xacro_dir=$(cd "$(dirname "${source_xacro_input}")" && pwd -P)
source_xacro="${source_xacro_dir}/$(basename "${source_xacro_input}")"

python_bin=$(command -v python 2>/dev/null || true)
if [[ -z "${python_bin}" ]]; then
  echo "Python was not found. Activate the fr3sim mamba environment first." >&2
  exit 1
fi
if ! PYTHONNOUSERSITE=1 PYTHONPATH= AMENT_PREFIX_PATH= \
  "${python_bin}" -c 'import xacro, yaml' >/dev/null 2>&1; then
  echo "The active Python cannot import xacro and PyYAML." >&2
  echo "Install this project from a ROS-isolated Python environment or run:" >&2
  echo "  PYTHONNOUSERSITE=1 PYTHONPATH= AMENT_PREFIX_PATH= python -m pip install 'xacro==2.1.1' 'PyYAML>=6.0'" >&2
  exit 1
fi

mkdir -p "$(dirname "${output_path}")"
output_dir=$(cd "$(dirname "${output_path}")" && pwd -P)
output_path="${output_dir}/$(basename "${output_path}")"
if [[ -e "${output_path}" && "${source_xacro}" -ef "${output_path}" ]]; then
  echo "The input xacro and output URDF must be different files." >&2
  exit 1
fi

temporary_output=$(mktemp "${output_dir}/.$(basename "${output_path}").XXXXXX")
cleanup() {
  rm -f -- "${temporary_output}"
}
trap cleanup EXIT

# PyPI xacro normally asks ament-index-python to resolve $(find package_name).
# Inject a small local resolver instead: it discovers the package containing
# the input xacro and can also search roots listed in XACRO_PACKAGE_PATH. This
# keeps conversion independent of ROS and leaves the source package untouched.
PYTHONNOUSERSITE=1 PYTHONPATH= AMENT_PREFIX_PATH= \
XACRO_SOURCE_PATH="${source_xacro}" \
XACRO_PACKAGE_PATH="${XACRO_PACKAGE_PATH:-}" \
  "${python_bin}" -c '
import os
import sys
import types
from pathlib import Path
from xml.etree import ElementTree

source_path = Path(os.environ["XACRO_SOURCE_PATH"]).resolve()


def package_name(directory):
    package_xml = directory / "package.xml"
    if not package_xml.is_file():
        return ""
    try:
        return (ElementTree.parse(package_xml).findtext("name") or "").strip()
    except (ElementTree.ParseError, OSError):
        return ""


def containing_package(path):
    directory = path.parent
    while True:
        if package_name(directory):
            return directory
        parent = directory.parent
        if parent == directory:
            return None
        directory = parent


local_package = containing_package(source_path)
search_roots = []
if local_package is not None:
    search_roots.extend((local_package, local_package.parent))
search_roots.extend(
    Path(entry).expanduser().resolve()
    for entry in os.environ.get("XACRO_PACKAGE_PATH", "").split(os.pathsep)
    if entry
)


def get_package_share_directory(requested_name):
    checked = set()
    for root in search_roots:
        for candidate in (root, root / requested_name):
            candidate = candidate.resolve()
            if candidate in checked:
                continue
            checked.add(candidate)
            if package_name(candidate) == requested_name:
                return str(candidate)
    raise RuntimeError(
        "Cannot resolve xacro package {!r}. Put its package root or its "
        "parent directory in XACRO_PACKAGE_PATH.".format(requested_name)
    )


ament_index = types.ModuleType("ament_index_python")
ament_packages = types.ModuleType("ament_index_python.packages")
ament_packages.get_package_share_directory = get_package_share_directory
ament_index.packages = ament_packages
sys.modules["ament_index_python"] = ament_index
sys.modules["ament_index_python.packages"] = ament_packages

import xacro

xacro.main()
' -o "${temporary_output}" "${source_xacro}"

"${python_bin}" -c '
import sys
from xml.etree import ElementTree

document = ElementTree.parse(sys.argv[1])
root_name = document.getroot().tag.rsplit("}", 1)[-1]
if root_name != "robot":
    raise SystemExit("Generated XML root must be <robot>, got <{}>".format(root_name))
' "${temporary_output}"

if command -v check_urdf >/dev/null 2>&1; then
  check_urdf "${temporary_output}" >/dev/null
else
  echo "check_urdf was not found; only XML and <robot> validation were run." >&2
fi

mv -f -- "${temporary_output}" "${output_path}"
trap - EXIT

echo "Generated URDF: ${output_path}"
echo "Source xacro: ${source_xacro}"

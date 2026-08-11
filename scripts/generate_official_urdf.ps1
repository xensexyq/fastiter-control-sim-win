[CmdletBinding()]
param(
    [Parameter(Mandatory = $true, Position = 0)]
    [string] $SourceXacro,

    [Parameter(Mandatory = $true, Position = 1)]
    [string] $OutputUrdf
)

$ErrorActionPreference = "Stop"

function Fail([string] $Message) {
    throw $Message
}

if (-not [IO.Path]::IsPathRooted($SourceXacro)) {
    Fail "The input xacro path must be absolute: $SourceXacro"
}
if (-not [IO.Path]::IsPathRooted($OutputUrdf)) {
    Fail "The output URDF path must be absolute: $OutputUrdf"
}

$source = [IO.Path]::GetFullPath($SourceXacro)
$output = [IO.Path]::GetFullPath($OutputUrdf)
if (-not (Test-Path -LiteralPath $source -PathType Leaf)) {
    Fail "The input xacro file was not found: $source"
}
if ([IO.Path]::GetExtension($source) -ne ".xacro") {
    Fail "The input file must end with .xacro: $source"
}
if ([IO.Path]::GetExtension($output) -ne ".urdf") {
    Fail "The output file must end with .urdf: $output"
}
if ([StringComparer]::OrdinalIgnoreCase.Equals($source, $output)) {
    Fail "The input xacro and output URDF must be different files."
}

$pythonCommand = Get-Command python -ErrorAction SilentlyContinue
if ($null -eq $pythonCommand) {
    Fail "Python was not found. Activate the fr3sim-win Conda environment first."
}
$python = $pythonCommand.Source
& $python -c "import xacro, yaml"
if ($LASTEXITCODE -ne 0) {
    Fail "The active Python cannot import xacro and PyYAML. Install xacro==2.1.1 and PyYAML first."
}

$outputDirectory = Split-Path -Parent $output
New-Item -ItemType Directory -Force -Path $outputDirectory | Out-Null
$temporary = Join-Path $outputDirectory ("." + [IO.Path]::GetFileName($output) + "." + [Guid]::NewGuid().ToString("N") + ".tmp")
$resolverFile = Join-Path ([IO.Path]::GetTempPath()) ("fr3_xacro_resolver_" + [Guid]::NewGuid().ToString("N") + ".py")

$oldPythonNoUserSite = $env:PYTHONNOUSERSITE
$oldPythonPath = $env:PYTHONPATH
$oldAmentPrefixPath = $env:AMENT_PREFIX_PATH
$oldXacroSourcePath = $env:XACRO_SOURCE_PATH
$oldXacroPackagePath = $env:XACRO_PACKAGE_PATH

$resolver = @'
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
                # xacro evaluates $(find ...) inside a Python expression. Use
                # forward slashes so Windows backslashes are not interpreted
                # as escape sequences (for example, ``\f``).
                return candidate.as_posix()
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
'@

try {
    $env:PYTHONNOUSERSITE = "1"
    $env:PYTHONPATH = ""
    $env:AMENT_PREFIX_PATH = ""
    $env:XACRO_SOURCE_PATH = $source
    if ($null -eq $oldXacroPackagePath) {
        $env:XACRO_PACKAGE_PATH = ""
    }
    [IO.File]::WriteAllText($resolverFile, $resolver, [Text.UTF8Encoding]::new($false))
    & $python $resolverFile -o $temporary $source
    if ($LASTEXITCODE -ne 0) {
        Fail "xacro conversion failed."
    }

    & $python -c "from pathlib import Path; import sys; from xml.etree import ElementTree; root=ElementTree.parse(sys.argv[1]).getroot(); assert root.tag.rsplit('}', 1)[-1] == 'robot', 'Generated XML root must be <robot>'; print('Generated XML validated as <robot>.')" $temporary
    if ($LASTEXITCODE -ne 0) {
        Fail "Generated XML did not contain a <robot> root."
    }

    $checkUrdf = Get-Command check_urdf -ErrorAction SilentlyContinue
    if ($null -ne $checkUrdf) {
        & $checkUrdf.Source $temporary
        if ($LASTEXITCODE -ne 0) {
            Fail "check_urdf validation failed."
        }
    } else {
        Write-Warning "check_urdf was not found; XML and <robot> validation were run."
    }

    Move-Item -LiteralPath $temporary -Destination $output -Force
    Write-Output "Generated URDF: $output"
    Write-Output "Source xacro: $source"
}
finally {
    if (Test-Path -LiteralPath $temporary) {
        Remove-Item -LiteralPath $temporary -Force
    }
    if (Test-Path -LiteralPath $resolverFile) {
        Remove-Item -LiteralPath $resolverFile -Force
    }
    $env:PYTHONNOUSERSITE = $oldPythonNoUserSite
    $env:PYTHONPATH = $oldPythonPath
    $env:AMENT_PREFIX_PATH = $oldAmentPrefixPath
    $env:XACRO_SOURCE_PATH = $oldXacroSourcePath
    $env:XACRO_PACKAGE_PATH = $oldXacroPackagePath
}

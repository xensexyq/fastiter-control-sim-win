# FR3 Pinocchio C++ 仿真

本项目使用 Franka 官方 `franka_description` 模型，实现 FR3 机械臂的：

- C++ 正运动学、雅可比、逆运动学和最小加加速度关节轨迹
- Python/pybind11 调用接口
- MeshCat 三维可视化
- Qt FK/IK 滑条控制面板
- xacro 转换、Python smoke test 和可选 C++ 测试

项目对 Linux 和原生 Windows 都提供完整功能。Windows 使用 Visual Studio/MSVC
和 conda-forge 的 Pinocchio，不需要 ROS、WSL 或 MSYS2。

默认 URDF 为 `models/fr3_franka_hand.urdf`，末端坐标系为 `fr3_hand_tcp`。

## 快速验证

激活对应环境后执行：

```bash
python tests/smoke_test.py
python examples/fr3_sim.py --headless --mode demo
```

`--headless` 不需要三维网格资源。需要 MeshCat 或 Qt 时，先按下文准备
`franka_description`。

## 工程结构

```text
environment.yml                          Linux/macOS Conda 环境
environment-windows.yml                  原生 Windows Conda 环境
CMakeLists.txt                           C++/pybind11 构建配置
cpp/include/fr3_control_sim/             C++ 公共头文件
cpp/src/robot_model.cpp                  Pinocchio FK/IK/轨迹实现
cpp/src/bindings.cpp                     pybind11 绑定
python/fr3_control_sim/                  Python 包和 MeshCat 显示
examples/fr3_sim.py                      命令行 FK/IK/demo 入口
examples/fr3_sim_qt.py                   Qt FK/IK 滑条控制界面
models/fr3_franka_hand.urdf              已生成的 FR3 + Franka Hand URDF
scripts/generate_official_urdf.sh        Linux/macOS xacro 转换脚本
scripts/generate_official_urdf.ps1       Windows PowerShell xacro 转换脚本
tests/smoke_test.py                      Python/pybind 回归测试
cpp/tests/test_kinematics.cpp            C++ 回归测试
```

## 官方模型和网格资源

仓库跟踪生成后的 URDF，但不跟踪官方仓库中的大型 DAE 网格文件。这样可以保持
代码仓库轻量，同时仍然兼容官方模型。准备可视化资源：

```bash
git clone --depth 1 https://github.com/frankarobotics/franka_description.git \
  third_party/franka_description
```

Windows PowerShell 写法：

```powershell
git clone --depth 1 https://github.com/frankarobotics/franka_description.git `
  third_party\franka_description
```

两个示例会自动搜索以下目录：

1. `FRANKA_DESCRIPTION_ROOT` 环境变量指定的目录
2. 项目内 `third_party/franka_description`
3. 项目根目录下的 `franka_description`
4. Linux 兼容路径 `/home/xense/fastiter/franka_description`

已有其他副本时，可以设置：

```powershell
$env:FRANKA_DESCRIPTION_ROOT = "D:\path\to\franka_description"
```

## Linux/macOS 安装

需要 Conda 或 Mamba。推荐使用项目环境文件：

```bash
git clone https://github.com/xensedyl/fastiter-control-sim.git
cd fastiter-control-sim
mamba env create -f environment.yml
mamba activate fr3sim
python -m pip install -e .
```

`environment.yml` 会安装 Pinocchio、Eigen、urdfdom、tinyxml2、编译器和 Python
依赖，并清空继承自 ROS 的 `PYTHONPATH` 和 `AMENT_PREFIX_PATH`。已有环境可更新：

```bash
mamba env update -n fr3sim -f environment.yml
mamba deactivate
mamba activate fr3sim
python -m pip install -e .
```

如果需要完整构建日志：

```bash
python -m pip install -v -e .
```

## 原生 Windows 安装

下面的流程已在 Windows 11、Python 3.11、Visual Studio Build Tools 2022、
Pinocchio 4.x 环境中验证。

### 1. 安装 MSVC 和 Windows SDK

安装 Visual Studio Build Tools 2022 的 **Desktop development with C++** 工作负载，
必须包含 MSVC 编译器、Windows SDK 和 CMake 集成。管理员 PowerShell 可执行：

```powershell
winget install --id Microsoft.VisualStudio.2022.BuildTools --exact `
  --accept-package-agreements --accept-source-agreements --silent `
  --override "--wait --passive --add Microsoft.VisualStudio.Workload.VCTools --includeRecommended"
```

安装完成后重新打开 PowerShell。普通 PowerShell 可以构建；也可以在
“x64 Native Tools Command Prompt for VS 2022” 中执行后续命令。

### 2. 创建 Conda 环境

只使用 conda-forge，避免本机 `defaults` 源带来的 ABI 冲突：

```powershell
conda env create --override-channels -f environment-windows.yml
conda activate fr3sim-win
```

安装 Windows 端的 Python 工具和 UI 依赖：

```powershell
python -m pip install cmake pybind11 meshcat "xacro==2.1.1" "PyYAML>=6.0" "PySide6>=6.6,<7"
```

### 3. 获取可视化网格

无头模式可以跳过此步；MeshCat 和 Qt 必须执行：

```powershell
git clone --depth 1 https://github.com/frankarobotics/franka_description.git `
  third_party\franka_description
```

### 4. 编译并安装

```powershell
python -m pip install -e . --no-deps --no-build-isolation
```

Windows 推荐这两个 pip 选项：依赖已经由 Conda/pip 明确安装，并确保 setuptools
使用当前环境中的 MSVC、CMake、Pinocchio 和 pybind11。修改 C++ 后重复此命令；
只修改 Python 文件无需重装。

### 5. 验证

```powershell
python --version
python -c "import pinocchio; print('Pinocchio:', pinocchio.__version__)"
python -c "import fr3_control_sim; print(fr3_control_sim.__file__)"
python tests\smoke_test.py
python examples\fr3_sim.py --headless --mode demo
python examples\fr3_sim_qt.py --help
```

### 6. 启动可视化

```powershell
python examples\fr3_sim.py --mode demo
python examples\fr3_sim_qt.py
```

不自动打开浏览器时：

```powershell
python examples\fr3_sim.py --mode demo --no-open-browser
python examples\fr3_sim_qt.py --no-open-browser
```

### Windows 与 Linux 的差异

- 编译器：Windows 使用 Visual Studio/MSVC；Linux 使用 conda-forge `cxx-compiler`。
- 环境文件：Windows 使用 `environment-windows.yml`；Linux 使用 `environment.yml`。
- 构建：Windows 推荐 `--no-deps --no-build-isolation`；Linux 可直接 `pip install -e .`。
- xacro：Windows 使用 PowerShell 脚本；Linux/macOS 使用 Bash 脚本。
- 网格：两端的 MeshCat/Qt 都需要 `franka_description`；无头模式不需要。

## 运行仿真

### Headless demo

```bash
python examples/fr3_sim.py --headless --mode demo
```

### MeshCat demo

```bash
python examples/fr3_sim.py --mode demo
python examples/fr3_sim.py --mode demo --no-open-browser
```

### FK

交互模式会持续等待关节角输入，单位为度；输入 `q`、`quit`、`exit` 或空行退出：

```bash
python examples/fr3_sim.py --mode fk
```

单次 FK：

```bash
python examples/fr3_sim.py --mode fk --q 0 -45 0 -135 0 90 45
```

### IK

交互模式：

```bash
python examples/fr3_sim.py --mode ik
```

单次 IK 的目标可以是 `x y z`，也可以是 `x y z roll pitch yaw`；位置单位为米，
RPY 单位为弧度：

```bash
python examples/fr3_sim.py --mode ik --target 0.35 0.10 0.45
python examples/fr3_sim.py --mode ik --target 0.35 0.10 0.45 3.1415926 0.0 0.2
```

IK 成功后，C++ 生成 50 Hz 的最小加加速度关节轨迹并由 MeshCat 播放。

### Qt 控制面板

```bash
python examples/fr3_sim_qt.py
python examples/fr3_sim_qt.py --mode ik
python examples/fr3_sim_qt.py --no-meshcat
```

FK 页签控制 7 个关节角；IK 页签控制 XYZ/RPY。不可达目标会显示错误，并保留上次
成功姿态。

## Python 接口

```python
import numpy as np
from fr3_control_sim import IKOptions, RobotModel, pose_from_xyz_rpy

model = RobotModel("models/fr3_franka_hand.urdf")
q0 = model.home_configuration()
target = pose_from_xyz_rpy(
    np.array([0.35, 0.10, 0.45]),
    np.array([3.1415926, 0.0, 0.2]),
)
result = model.inverse_kinematics(target, q0, IKOptions())
trajectory = model.minimum_jerk_trajectory(q0, result.q, 2.0, 0.02)
```

## xacro 转换为 URDF

项目已经包含可用的 `models/fr3_franka_hand.urdf`，正常安装和运行不需要重新生成。
若需要从官方 xacro 重新生成：

Linux/macOS：

```bash
./scripts/generate_official_urdf.sh \
  /absolute/path/to/franka_description/robots/fr3/fr3.urdf.xacro \
  models/fr3_franka_hand.urdf
```

Windows PowerShell（两个路径都必须是绝对路径）：

```powershell
& .\scripts\generate_official_urdf.ps1 `
  "D:\path\to\franka_description\robots\fr3\fr3.urdf.xacro" `
  "D:\Projects\0-FastIter-Arm\simulation\models\fr3_franka_hand.urdf"
```

如果 xacro 依赖其他软件包，将其根目录或共同父目录加入 `XACRO_PACKAGE_PATH`。
脚本会在替换目标 URDF 前完成 XML 和 `<robot>` 根节点校验；可用时还会运行
`check_urdf`。

## 测试与重新构建

Python smoke test：

```bash
python tests/smoke_test.py
```

修改 C++ 后重新构建：

```bash
python -m pip install -e .
```

Windows：

```powershell
python -m pip install -e . --no-deps --no-build-isolation
```

## 常见问题

### `cannot resolve package://franka_description/...`

这是可视化网格目录未准备好。执行上面的 `git clone`，或设置
`FRANKA_DESCRIPTION_ROOT`。`--headless` 模式不需要网格。

### Windows 找不到 `cl.exe` 或 CMake 编译失败

确认已安装 Visual Studio Build Tools 的 C++ 工作负载和 Windows SDK，重新打开
PowerShell 后再执行安装命令。也可以从 “x64 Native Tools Command Prompt for VS 2022”
运行 pip 构建。

### `pip` 找到错误的 Python 或 ROS 包

先确认：

```powershell
python -c "import sys; print(sys.executable)"
python -c "import fr3_control_sim; print(fr3_control_sim.__file__)"
```

Linux 环境若曾执行过 ROS `setup.bash`，重新激活 `fr3sim`，并确保
`PYTHONPATH`、`AMENT_PREFIX_PATH` 没有指向其他 Python 环境。

### pip 为什么不能替代 Conda

pip 负责 Python 包和当前项目的 C++ 扩展构建；Pinocchio、Eigen、urdfdom、tinyxml2
等 ABI 敏感的 C++ 依赖必须由对应平台的 conda-forge 或 Visual Studio 工具链提供。

## 许可证

本项目源代码采用 [MIT License](LICENSE)。由 Franka 官方 `franka_description` 生成的
模型和网格仍遵循其原始 Apache-2.0 许可证。

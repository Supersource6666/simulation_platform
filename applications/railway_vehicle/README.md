# 25T（浦镇）普速客车垂向动力学模型

## 按讨论新增：10DOF 车轨双向耦合基线

新入口 **railway_coupled** 实现 10DOF 垂向/点头车辆、Euler–Winkler 模态梁、四轴 Hertz 单边接触、非线性静平衡、预测校正及频率约束内部子步。
支持单轨/双轨、规定变速度和实测/正弦不平顺，提供 FEM 对照与收敛扫描。

编译、运行、理论符号、六阶段完成状态及验证边界见 **[车轨耦合说明](docs/coupled_vertical.md)**，
默认参数见 [coupled_vertical.json](config/coupled_vertical.json)。
该入口采用独立应用内的矩阵求解器，参数仍包含未标定的轨道、Hertz 和质心点头惯量假设。

以下介绍原有 **railway_vehicle** 的 7DOF / 27DOF 路径。原有空间模型尚未接入新的柔性轨道反馈。

本次验证与已知问题见 [验证结果](docs/coupled_validation_results.md)：旧 `vehicle_3d_checks` 的小运动范围超限已在改动前轨道函数的独立构建中复现。


当前默认车型为 **25T YZ 硬座车满载**，采用用户提供的《25T(浦镇)车辆动力学参数.docx》校正车辆参数。运行速度暂设 **120 km/h**，属于工况假设，文档没有给出速度。

这是**文档参数校正后的 7 自由度垂向简化模型**，不是经过实测验证的完整空间车辆模型。保留 1 个车体、2 个构架、4 个等效簧下轮对的垂向平动；点头、侧滚、摇头、纵向和横向运动仍受约束。

作为 Chrono 使用者，本项目独立链接已编译的 Core。本次校正只修改 applications/railway_vehicle，不修改引擎代码。

## 最新默认输入：文件轨道不平顺

已接入用户指定的 origin_excitation_data.txt，当前默认配置改为
[25t_yz_loaded_measured.json](config/25t_yz_loaded_measured.json)。
数据单位、四列使用范围、插值、人工引入段、运行与复算见
[实测轨道输入说明](docs/measured_track_input.md)。

**实际垂向激励为 (Lv+Rv)/2；Ld/Rd 保留并输出，当前不参与动力学求解。**
最新结果为 output/25t_measured_120kmh，已运行前 12 s，不是全线路时程。
下文的正弦线路配置和结果作为此前文档参数校正的基准保留。

## 27 自由度空间扩展 (点头 / 蛇行 / 横向动力学)

新增 `model_3d` 配置块即可开启空间动力学；现有 7 自由度配置不受影响。
空间模型扩展内容（详见 `src/VehicleModel3D.h/.cpp` 与 `config/25t_yz_loaded_3d.json`）：

| 模块 | 自由度 / 物理 | 主要实现 |
|---|---|---|
| 点头 (Pitch) | 车体、构架的点头角 θ (绕 Y 轴) | `ChLinkRSDA` 在二级和一级弹簧位置引入防点头、抗扭刚度/阻尼 |
| 蛇行 (Hunting) | 轮对横向 y 与摇头 ψ (绕 Z 轴) | Kalker 线性蠕变 `f22`、`f33`，等效锥度 λ 重力刚度，自旋阻尼 |
| 横向动力学 | 车体、构架 y (横向平动)，车体/构架摇头 ψ (绕 Z 轴) 与侧滚 φ (绕 X 轴) | 一系/二系横向弹簧阻尼 (`ChLinkTSDA`)、抗蛇行减振器、抗侧滚扭杆 |
| 轮轨接触 | 双侧独立垂向支承 (Lv, Rv)、横向接触刚度、蠕变力、锥度效应、轨道超高与曲线向心 | 自定义 `ChLinkTSDA::ForceFunctor` 与 `ChLinkRSDA::TorqueFunctor` |

27 自由度合计：车体 + 2 构架各 5 个 (z, pitch, y, roll, yaw) = 15；4 轮对各 3 个 (z, y, yaw) = 12。
轮对保留轴向、侧滚自由度约束（与 25T 文档惯量近似一致）。

数值积分器由 `EULER_IMPLICIT_LINEARIZED` 改为 `HHT`，α = -0.05（轻度数值阻尼），用于承受蠕变非线性
而不破坏线性化假设。

空间模型自检见 `tests/vehicle_3d_checks.cpp`，CMake 注册为 `vehicle_3d_checks`。
`railway_vehicle --config 25t_yz_loaded_3d.json --self-test` 同时运行内置的 `SelfTest3D`。

3D 参数集中在 `model_3d.*`：`primary_lat_k_N_m` / `secondary_lat_k_N_m`、`yaw_secondary_k_N_m_rad` /
`yaw_secondary_c_N_m_rad`（抗蛇行减振器）、`roll_secondary_k_N_m_rad` / `roll_secondary_c_N_m_rad`
（抗侧滚扭杆）、`wheel_conicity`、`rolling_gauge_m`、`creep_lat_N` / `creep_long_N` / `creep_spin_N_m`
（Kalker 线性系数）、`lateral_contact_k_N_m`、`track_curvature_1_m`、`track_cant_rad`。

## 先看这些文件

- [25T YZ 满载默认参数](config/25t_yz_loaded.json)：实际运行参数与来源/假设说明。
- [校正对照表](docs/25t_parameter_mapping.md)：单位、按轴箱/空气簧换算、未使用的参数。
- [Word 原文提取](sources/25t_document_text.txt)：完整表格文本。
- [机器可读源表与 SHA256](sources/25t_document_tables.json)：用于追溯。
- [模型装配](src/VehicleModel.cpp)：刚体、悬挂、静载预压和支承力。
- [减振器分段曲线](src/DamperCurve.h)：插值、反向力和外推。
- [参数读取](src/Parameters.cpp)：读取并校验 JSON。
- [运行入口](src/main.cpp)：步进、结果输出与自检。

## 主要校正

| 项目 | 原示例 | 当前默认 25T YZ 满载 |
|---|---:|---:|
| 车体质量 | 38000 kg | 52100 kg |
| 每架构架质量 | 3000 kg | 2160 kg |
| 每轴等效簧下质量 | 1800 kg | 1600 kg，包含轮对和轴箱 |
| 转向架中心距 | 17.5 m | 18 m |
| 轴距 | 2.5 m | 2.6 m |
| 车体/构架质心高度 | 2.0 / 1.0 m | 2.096 / 0.5365 m |
| 轮对中心高度 | 0.46 m | 0.4575 m，新轮滚动圆半径 |
| 每轴合计一系垂向刚度 | 2.4 MN/m | 1.982 MN/m |
| 每架合计二系垂向刚度 | 0.6 MN/m | 0.74 MN/m |
| 一系阻尼 | 常数 20000 N·s/m | 每轴 2 个减振器的文档力—速度曲线 |
| 二系阻尼 | 常数 30000 N·s/m | 每架 2 个减振器的文档力—速度曲线 |
| 运行速度 | 300 km/h | 暂定 120 km/h，非文档参数 |

总质量 62820 kg，每轴静态支承力 154066.05 N。位移均相对于静载平衡位置，重力保持启用，弹簧预压根据各级承载自动计算。

一系弹簧下/上作用点高度使用 0.620 / 0.926 m，二系空气簧下/上表面使用 0.600 / 0.959 m。作用点通过局部偏置连接刚体，不能把构架质心高度直接当作一系或二系弹簧安装点。

## 车型和载荷切换

| 配置名中的车型 | 空车车体质量 | 满载车体质量 |
|---|---:|---:|
| yz | 38.6 t | 52.1 t |
| yw | 39.8 t | 47.3 t |
| rw | 40.6 t | 45.3 t |
| ca | 37.0 t | 45.0 t |
| control | 46.5 t | 52.1 t |

文件命名为 config/25t_<车型>_<empty 或 loaded>.json，各配置同时带入对应质心高度。表中为“不含转向架”的车体质量。文档对满载的说明包含整备质量和乘客载荷；硬座满载按超员 30% 计，其他车型按定员计。

原来的 config/example.json 保留为历史示例和回归测试，不再是默认配置。历史 output/example_run 也保留，不应当作 25T 结果。

## 编译、运行与绘图

在 simulation_platform 根目录运行 PowerShell：

```powershell
cmake -S applications/railway_vehicle -B build/railway_vehicle -G "Visual Studio 17 2022" -A x64 "-DChrono_DIR=$((Get-Location).Path)/build/core/cmake"
cmake --build build/railway_vehicle --config Release --parallel 6 -- /p:CL_MPCount=6
ctest --test-dir build/railway_vehicle -C Release --output-on-failure

./build/railway_vehicle/Release/railway_vehicle.exe --config applications/railway_vehicle/config/25t_yz_loaded.json --duration 12 --output applications/railway_vehicle/output/25t_run_01
python applications/railway_vehicle/scripts/plot_results.py applications/railway_vehicle/output/25t_run_01/response.csv
```

修改 JSON 后使用显式 --config 即可运行，无需重新编译。省略 --config 时读取 EXE 旁 config/25t_yz_loaded_measured.json（构建时复制的实测配置）。

默认 dt=0.001 s，默认持续 8 s；这里显式运行 12 s，以观察完整不平顺区段后的衰减。--duration 必须是 --dt 的整数倍。使用 --flat 将幅值覆盖为零，--dt 0.0005 可检查更小步长，--help 查看选项。输出目录必须不存在，以免覆盖历史运行。

输出文件：
- response.csv：时间、7 个刚体垂向位移/速度/加速度、4 个轨道输入、轮对支承力、一二系悬挂力。
- input_parameters.json：本次输入参数的快照，包含源文档哈希与假设。
- summary.txt：实际模型名称、参数状态、总质量、静态轴支承、时间步长、速度和 flat 覆盖等。
- response.png / response.svg：绘图脚本生成。图标题从结果快照读取车型，不再固定标为高速动车。

CSV 里 dz 是静平衡位置的增量，Z 向上为正。支承力按每轮对合计，不是单轮轮重。加速度为积分器状态，没有做舒适度加权。绘图依赖 Python/matplotlib，C++ 仿真不依赖 Python。

## 模型适用范围与待补充项

- 7 个垂向平动自由度；不含车体和构架点头，因此不能给出完整垂向—点头耦合响应。
- 一系、正常充气二系采用线性刚度；未实现空簧失气工况。
- 文档给出的减振器速度点采用奇对称分段线性力；未提供零点与反向曲线，(0,0)、奇对称及末段线性外推是建模假设。
- 减振器接头刚度虽然在文档中给出，但当前采用刚性连接简化，未建串联弹性和减振器内部自由度。横向偏置和不同安装纵向位置对本版垂向平动等效无影响；升级空间模型时必须恢复。
- 轮轨支承仍为原示例的线性双向等效支承，参数来自假设，不是 Hertz/蠕滑力模型。支承变为零或拉力时程序报错退出。
- 轨道是有限长度平滑窗正弦，四轴沿同一轮迹按轴距和速度延迟。无左右差异、曲线、超高和真实不平顺数据。
- 文档中的惯量已保存在配置 reference_only_not_used 中，当前旋转自由度被约束，未用于求解。源文档注明车体参数原点位于车体中心轨面位置；未确认惯量参考轴前，不直接当作质心惯量赋值。

## 文档参数校正阶段的正弦基准结果

三项 CTest 均通过：旧示例回归、25T YZ 空车、25T YZ 满载。覆盖静平衡总载荷、1 s 平直轨道位移漂移、减振器表格节点/插值/奇对称/外推、不平顺时延及导数、正支承力和步长减半位移检查。

YZ 空车/满载的静平衡漂移均为 0 m。步长从 1 ms 减到 0.5 ms 时，全时程最大位移差分别约 4.18491e-6 / 4.18573e-6 m。阈值为输入幅值的 3%，属于数值自检，不是车辆性能验收；未验证加速度的步长收敛。

正弦基准配置 YZ 满载、120 km/h、12 s、dt=1 ms 的结果：
- 车体相对静平衡位移峰值约 0.360235 mm；
- 最小轮对支承力约 152263 N；
- 输出目录 output/25t_yz_loaded_120kmh。

这次响应与旧高速动车示例同时改变了质量、悬挂、速度等因素，不能把差异归因于某一个参数。

## 重新导入原始文档

```powershell
python applications/railway_vehicle/scripts/import_25t_parameters.py "E:/CIE/25T(浦镇)车辆动力学参数.docx"
```

导入会重新生成 10 个 25T 配置（覆盖对这些生成文件的手动修改）及源表提取。自定义工况请另存 JSON。导入器检查表格结构和关键特性点；文档结构变化时应复核映射。

实现沿用原项目已验证的 Chrono Core 独立链接与弹簧回调模式，参考[官方 Core 手册](https://api.projectchrono.org/development/manual_core.html)与[弹簧示例](https://github.com/projectchrono/chrono/blob/main/src/demos/mbs/demo_MBS_spring.cpp)。车辆参数来自用户文档，不来自官方示例。

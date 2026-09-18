# 文件轨道不平顺输入

当前默认配置为 config/25t_yz_loaded_measured.json：25T（浦镇）YZ 满载、暂定 120 km/h，接入指定文件：

E:/railway_data_processing_scripts/convert2dynamics_input/output/origin_excitation_data.txt

源文件 SHA256：6805cdbf4741b9377259a7ae2177be2fbe10f9e0008964f76cf3241ab8c9b8b3

项目保存原样副本 data/origin_excitation_data.txt；[数据清单](../sources/track_input_manifest.json)记录来源、范围、列含义和统计。不更改 E 盘原文件，不做重复单位转换，不对原始测点滤波、去均值或幅值缩放。

## 数据含义

经同目录 measurement2simpack.py 的单位转换和列映射确认：

| 列 | 含义 | 单位 |
|---|---|---|
| s | 里程坐标 | m |
| Lv | 左轨高低，来源为“70米左高低” | m |
| Rv | 右轨高低，来源为“70米右高低” | m |
| Ld | 左轨轨向，来源为“70米左轨向” | m |
| Rd | 右轨轨向，来源为“70米右轨向” | m |

该转换脚本已将 mm 除以 1000。程序保持输入正负号，按 Z 向上为正施加高低；没有另行反号。该文件是转换后的不平顺输入，不等同于未经测量系统处理的线路绝对几何。

共 53348 个点，s=0～13336.75 m，间距全部为 0.25 m。四列极值（m）：

| 列 | 最小值 | 最大值 |
|---|---:|---:|
| Lv | -0.010610 | 0.005780 |
| Rv | -0.008430 | 0.006330 |
| Ld | -0.005240 | 0.006020 |
| Rd | -0.004920 | 0.006170 |

## 当前实际参与动力学的部分

**当前仍是 7 自由度垂向模型。**

每轴左右支承刚度/阻尼取相等值，轮对侧滚被约束，故两侧合力的等效垂向输入为：

r = (Lv + Rv) / 2

r_dot = speed * (dLv/ds + dRv/ds) / 2

这保留了当前约束模型的垂向合力，但不保留左右高低差产生的侧滚力矩和单轮轮重差。Ld、Rd 四轴各自插值并输出，但**不参与当前动力学求解**；横向/转动自由度和实际轮轨力模型尚未建立，不能把轨向列当作垂向激励，不能声称四列均已用于空间车辆计算。

轮轨支承仍采用前一版假设的每轴合计刚度 1e8 N/m、阻尼 20000 N·s/m。这些不因导入实测高低而变成已标定的轮轨参数。

## 里程、插值和边界

以首个实测 s 为基准，四轴在时刻 t 的采样位置为：

s_i(t) = s_first + speed*t - track_start_m - axle_delay_i

四轴距离延迟为 0、2.6、18、20.6 m。在 120 km/h 下相应时间延迟为 0、0.078、0.54、0.618 s。车辆坐标中轮对不实际前进，速度用于沿线路采样。

对 Lv、Rv、Ld、Rd 分别使用保形分段三次 Hermite（PCHIP）插值，经过所有原始节点；阻尼使用插值函数的解析导数乘运行速度。PCHIP 为 C1 连续，二阶导数不保证连续，因此加速度仍需单独检查步长收敛。

起始设置：
- track_start_m=40：前轴在 t=1.2 s 到达实测 s=0。
- track_lead_in_m=5：在实测起点前 5 m，用三次 Hermite 连接零位移/零斜率与首个测点位移/斜率。
- 前轴 t≤1.05 s 输入为零，1.05～1.2 s 为人工引入段；后续各轴按距离延迟。
- 引入段属于建模假设，不是测量数据；CSV 的 measured 标志为 0。实测区间中标志为 1。
- 不在测点范围外向后延长、循环或补零；前轴超过最后测点会报错。运行前检查 duration 不得超过 (track_start+13336.75)/speed，即当前约 401.3025 s。
- 若用 1 ms 步长，duration 仍需是步长的整数倍。末轴尚有 20.6 m 延迟，不应声称前轴到达末端时四轴均已走完全部数据。

## 编译与使用

在 simulation_platform 根目录：

```powershell
cmake --build build/railway_vehicle --config Release --parallel 6 -- /p:CL_MPCount=6
ctest --test-dir build/railway_vehicle -C Release --output-on-failure

./build/railway_vehicle/Release/railway_vehicle.exe --config applications/railway_vehicle/config/25t_yz_loaded_measured.json --duration 12 --output applications/railway_vehicle/output/measured_run_01
python applications/railway_vehicle/scripts/plot_results.py applications/railway_vehicle/output/measured_run_01/response.csv
```

配置的 track_file 路径相对于 JSON 所在目录，而非启动目录。程序启动时加载一次，共享只读数据供四轴力回调使用。省略 --config 时默认读取 EXE 旁复制的实测配置。

旧 config/25t_yz_loaded.json 及其他不含 track_file 的配置仍使用原正弦线路，可用于回归对照。--flat 对两种输入都有效，将实际轨道激励关闭。

如源文件更新，重新执行导入：

```powershell
python applications/railway_vehicle/scripts/import_track_input.py "E:/railway_data_processing_scripts/convert2dynamics_input/output/origin_excitation_data.txt"
```

这会更新项目数据副本、清单和生成的实测配置；自定义运行工况请另存 JSON。改变源数据后应重新构建以更新 EXE 旁副本，或直接传源目录的 --config。

## 输出和复算

在原响应字段之外，每轴增加：

- track_i_s_m：该轴采样里程，起始引入段可为负。
- track_i_Lv_m、Rv_m、Ld_m、Rd_m：四通道插值值（引入段为人工连接值）。
- track_i_measured：是否位于实测区间。
- 原 track_i_z_m：实际使用的左右高低平均值。

每次运行将轨道文件复制为结果目录中的 track_input.txt，并把 input_parameters.json 的 track_file 重写成该相对文件名，保存有效 track_enabled 状态。因此可只用结果目录复算，不需要原始 E 盘路径。源文档和源轨道的来源元数据也随配置保留。

```powershell
./build/railway_vehicle/Release/railway_vehicle.exe --config applications/railway_vehicle/output/measured_run_01/input_parameters.json --duration 12 --output applications/railway_vehicle/output/replay_01
```

模型版本、平台、运行时间和步长也需要保持一致。位移或力的末尾浮点位可能不同，不承诺 CSV 字节一致。默认 summary.txt 的 track_vertical_scale_m 在文件模式中表示全文件左右平均高低的最大绝对值，不是额外施加的正弦幅值。

## 已完成的验证

5 项 CTest 通过：旧示例、25T YZ 空车、满载、实测输入动力学检查和文件插值检查。
- 校验全部 53348 个测点的四列数值被插值准确保持。
- 校验中点不超出相邻数据范围、解析导数、四轴时延、引入段的值与斜率连续性。
- 拒绝错误列名、缺列、重复/逆序里程、非有限值及越界采样。
- 12 s 时程下 1 ms/0.5 ms 的最大刚体位移差约 0.06405 mm；静平衡漂移为 0。
- 用输出快照复算，最大位移差约 4.53e-12 m、最大加速度差约 3.87e-7 m/s²、最大支承力差约 0.000241 N。轨道输入差异处于浮点末尾精度范围。

本次运行 output/25t_measured_120kmh：
- 25T YZ 满载，120 km/h，12 s，1 ms。
- 前轴实测里程覆盖 0～360 m，末轴在终点采样 339.4 m；**未运行完整 13.34 km 时程**。
- 车体相对静平衡位移峰值约 2.73470 mm。
- 最小轮对支承力约 130211 N，保持为正。
- 加速度未经舒适度加权；数值自检不是实车性能验证。

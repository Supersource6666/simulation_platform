# 10DOF 车轨双向耦合基线

本应用按照讨论中的“理论模型建立 → 单模块实现 → 双向耦合 → 数值稳定性 → 验证 → 三维扩展”推进。
新入口为 `railway_coupled`，默认配置为 `config/coupled_vertical.json`（320 阶模态）。
本次实测运行记录见 [验证结果](coupled_validation_results.md)。

这里采用 **Chrono 使用者的独立应用模式**：CMake 通过 `find_package(Chrono CONFIG REQUIRED)` 获取链接目标和 Eigen 等依赖。
新增的线性小运动车辆和模态递推由应用自身的矩阵求解器计算，没有调用 `ChSystem::DoStepDynamics`。
旧入口 `railway_vehicle` 继续提供原有 7DOF / 27DOF Chrono 刚体模型。
因此两个入口的积分器、接触物理及输出坐标含义不同，结果需要按明确工况比较。

## 六阶段及验收

| 阶段 | 本次实现 | 验收与边界 |
|---|---|---|
| 理论模型建立 | 坐标、单位、10DOF 悬挂装配、梁方程、Hertz 符号、接口 | 下文给出方程；惯量、轨道、Hertz 系数仍为显式工况假设 |
| 单模块实现 | 车辆、解析正弦模态梁、空间不平顺、速度曲线、接触 | 车辆谐响应及规定正弦轮对位移、静挠度解析解、恒速解析移动荷载、变速 Duhamel 积分检查 |
| 双向耦合 | 四轴单轨；可选八接触双轨；非线性静平衡；预测校正 | 总支承力与重力平衡、左右对称、作用反作用、脱离和再接触事件 |
| 数值稳定性 | 力/位移双判据、Newton 校正、线搜索、内部子步、失败回滚 | 内部频率分辨率独立于输出步长；未收敛不会提交状态 |
| 验证 | CTest、时间步/模态扫描、独立 Hermite FEM 空间离散、JSON/Markdown 报告 | 数值交叉验证；未做实测车辆性能验收 |
| 三维扩展 | 响应接口含三维平动和转动向量，轨道通过接口替换 | 新耦合模型尚无横向、扭转、轮对侧滚、蠕滑；既有 27DOF 路径尚未接入柔性轨道 |

## 编译与运行

在仓库根目录运行 PowerShell：

```powershell
cmake -S applications/railway_vehicle -B build/railway_vehicle -G "Visual Studio 17 2022" -A x64 -DChrono_DIR=D:/ZhuangDayuanFiles/DongLiXueFangZhen/simulation_platform/build/core/cmake
cmake --build build/railway_vehicle --config Release --target railway_coupled coupled_checks --parallel 4
ctest --test-dir build/railway_vehicle -C Release -R coupled_physics_checks --output-on-failure

./build/railway_vehicle/Release/railway_coupled.exe --config applications/railway_vehicle/config/coupled_vertical.json --output build/coupled_run
./build/railway_vehicle/Release/railway_coupled.exe --config applications/railway_vehicle/config/coupled_vertical.json --speed 0 --flat --duration 0.1 --output build/coupled_static
./build/railway_vehicle/Release/railway_coupled.exe --config applications/railway_vehicle/config/coupled_vertical.json --rails 2 --output build/coupled_twin
./build/railway_vehicle/Release/railway_coupled.exe --config applications/railway_vehicle/config/coupled_vertical.json --backend fem --elements 240 --output build/coupled_fem
```

`Chrono_DIR` 可指向 Chrono 的构建树或安装树，独立使用时通常推荐安装树。仓库当前已有构建树可复用。
所有输出目录必须不存在；已有运行不会被覆盖。`--dt`、`--duration`、`--modes`、`--elements`、`--rails`、`--speed` 和 `--flat` 会保存到可重跑的输入快照中。
`--duration / --dt` 必须是整数。

## 对象与文件

```text
src/coupled/
  Vehicle10DOF       车辆质量/悬挂矩阵、Newmark 预测
  TrackDynamics     ITrackDynamics、模态状态、解析梁/FEM 后端
  SimulationConfig  参数校验、速度积分、Hertz 单边定律
  CoupledSimulation 静平衡、接触映射、耦合校正、子步、状态提交
  main              CLI、参数快照、CSV、事件和计时
```

`Vehicle10DOF` 不依赖具体梁类。`CoupledSimulation` 通过 `ITrackDynamics` 投影载荷、预测轨道状态和查询响应。
FEM 后端保留全部自由单元自由度，再对离散矩阵对角化以共用振子时间递推；它与解析正弦模态的**空间离散独立**，时间传播器共用。

## 理论和单位

### 车辆

$$q_v=[z_c,\theta_c,z_{b1},\theta_{b1},z_{b2},\theta_{b2},z_{w1},z_{w2},z_{w3},z_{w4}]^T.$$

全部平动 `z` 向上为正。车辆点头角定义为车头抬起的纵向斜率，刚体上纵向偏置为 `l` 的点有 `z+l theta` 位移。
它等于右手系绕 Y 转角的负值。角度单位 rad。

令一根等效悬挂的伸长量为 `b q`，则装配 `K += k b^T b`、`C += c b^T b`。
一系参数按每轴合计，二系按每架合计，沿用现有 25T 参数映射。车辆方程为：

$$M_v\ddot q_v+C_v\dot q_v+K_vq_v=F_g+B_v^TF_N.$$

平动质量、悬挂刚度来自已导入配置。阻尼取原减振器表首段斜率乘数量，作为**线性单模块验证基线**；原有分段曲线的非线性形状不用于本模型。
点头惯量显式使用 `mc*18²/12` 与 `mb*2.6²/12` 的估算值，未把参考轴不确定的文档惯量直接当作质心惯量。

### 单根钢轨

$$\rho A w_{tt}+c_fw_t+EIw_{xxxx}+k_fw=p(x,t),\quad 0\le x\le L.$$

边界为 `w(0)=w(L)=0`、`w''(0)=w''(L)=0`，使用 `phi_n=sin(n pi x/L)`。

$$M_n=\rho A L/2,\quad C_n=c_fL/2,\quad K_n=[EI(n\pi/L)^4+k_f]L/2.$$

`rhoA`: kg/m；`EI`: N·m²；`kf`: N/m²；`cf`: N·s/m²。
Winkler 参数是沿线路分布的系数，不能直接填入单个轨下垫板的 N/m 刚度。
轨道自重计入无车辆参考形状；当前只计算轮轨载荷引起的变形。

向下轮轨载荷的模态力为：

$$Q_n(t)=-\sum_i F_{N,i}(t)\phi_n(x_i(t)).$$

每阶模态在内部步长 `h` 下使用矩阵指数更新。将状态增广为 `[q, qdot, Q, Qdot]`，在一步内线性插值广义力。
该一阶保持（FOH）形式对固定位置、恒定广义力也成立；离散矩阵按实际 `h` 缓存，速度变化本身不会改变矩阵。
这是模态截断后的时间递推，不能宣称已经求得无限阶精确格林函数。

### 轨道响应与移动位置

查询 `Evaluate(x,state)` 返回固定空间位置的 `w`、`w_t`、`w_tt` 和右手 Y 转角 `-w_x`、角速度 `-w_xt`。
移动接触点沿轨面速度应为 `w_t+v*w_x`，不应将输出的 `w_t` 误认为随车导数。
当前 Hertz 接触只依赖压缩量，没有人为增加接触阻尼，因此没有把两种速度混用到接触力中。

速度曲线写成 `[[time_s,speed_m_s], ...]`，首点时间为 0，时间严格递增，速度非负。
段内速度线性变化、位置按 `v0*t+0.5*a*t²` 精确积分，最后一个节点后保持末速度。
四轴位置 `xi=xc+li`。可以停车；暂不支持反向行驶或牵引/制动动力学。
整个工况必须让全部轮对处于有限梁内部，入口预检并在步进时再次检查。不能把出界轮对静默钳制到端点。

不平顺复用已有窗口正弦或实测 PCHIP，新增空间位置入口。单轨用 `(Lv+Rv)/2`，双轨分别用 `Lv`、`Rv`。
输入导数直接取空间插值导数，时间导数为 `r'(xi)*vi`。

### Hertz 与静平衡

$$\delta_i=w(x_i)+r(x_i)-z_{w,i},\qquad F_{N,i}=K_H\max(\delta_i,0)^{3/2}.$$

`gap=-delta`；`penetration=max(delta,0)`。`delta<=0` 时无拉力，接触状态为 Separation。
车辆受力为正向上，钢轨受力等量向下。
单轨模式每个接触代表一整个轮对；双轨模式每个接触代表单轮。切换模式时 `KH` 和轨道参数的等效关系应重新校准，不能默认物理等价。
双轨目前共享参数，但拥有独立状态。10DOF 轮对无侧滚，左右不平顺差异只改变两侧轮重及轨道响应。

初始化将车辆及各轨状态联合求解：

$$R(q)=Kq-F_g-B^TF_N(r-Bq)=0,$$
$$J(q)=K+B^T\operatorname{diag}(\tfrac32K_H\sqrt{\max(\delta,0)})B.$$

使用 Newton 和回溯线搜索，失败时拒绝运行。初始速度和加速度取零。
位移是相对未变形悬挂/未承载轨道的坐标，**包含重力静挠度**；与旧 CSV 的“相对静平衡 dz”不同。动态增量可减去首行。

## 耦合稳定性

每个内部步内，车辆和轨道都从上一步已接受状态计算候选终点，终点位移是终点接触力的仿射函数。
先用上一步接触力预测，再用 Hertz 残差进行 Newton 校正和非负力线搜索。
停止判据同时要求：

$$\frac{\|F-F_{Hertz}\|_2}{\max(\|F\|_2,F_{ref})}<\varepsilon_F,\quad
\|\Delta\delta\|_\infty<\varepsilon_w.$$

最大迭代次数默认 10。未收敛、非有限数或越界会拒绝整步，恢复车辆、全部钢轨、接触力、事件及时间。
触点事件仅在成功的内部步终点记录；时间精度受内部步长限制，尚未做事件根定位。

仅靠迭代收敛不能保证混合积分器稳定。程序对质量归一化的车辆、钢轨及 Hertz 切线刚度估计最高频率上界，要求内部步满足：

$$h\,\omega_{bound}\le \mathtt{max\_phase\_increment}.$$

默认相位增量为 0.2，可进一步减小用于独立收敛测试。接触切线改变后重新检查，必要时细分。
这是保守的频率分辨率控制，仍需通过工况收敛检查判断误差，不能视为任意非线性工况的数学稳定性证明。
`dt_s` 是外部推进和 CSV 采样间隔；实际积分可能有多个内部步。`internal_steps` 和 `minimum_internal_dt_s` 明确记录实际工作量。
大量模态、细 FEM 网格将增加内部步数，尚未达到硬实时保证。

## 输出与复算

- `response.csv`：10 个自由度的位移/速度/加速度；每个接触的绝对线路位置、力、间隙、压缩量、状态、梁响应及不平顺；累计内部步数。
- `events.csv`：轮对编号、轨号、脱离/再接触状态和内部步事件时间，编号从 0 开始。
- `coupled_config.json`、`vehicle_config.json`：本次有效参数快照，包含命令行覆盖；实测输入会复制为 `track_input.txt`。
- `summary.txt`：完成/失败状态、初始化残差、最大力残差、最大迭代数、内部最小步长、实际墙钟与进程时钟计时。

若步进失败，CSV 仅含之前成功的输出步，summary 明确标为 failed，进程返回非零。
输出文件快照可作为下次 `--config` 输入；它在文件移动后仍能找到同目录的车辆与实测输入。

## 收敛与 FEM 对照

```powershell
python applications/railway_vehicle/scripts/validate_coupled.py --exe build/railway_vehicle/Release/railway_coupled.exe --config applications/railway_vehicle/config/coupled_vertical.json --output build/coupled_validation_study --duration 2
```

脚本使用标准库，无需 numpy/matplotlib。扫描外部步长 `0.01,0.001,0.0005,0.0001 s`，模态数 `5,10,20,40,80`，FEM 单元数 `120,240`。
默认 320 模态参考解，另计算 640 模态以检查参考解自身，并将相位增量减半独立检查内部积分误差；报告内部步数以区分输出步长和积分精度。
比较全部轮轨力、车辆各自由度加速度、触点钢轨位移的 RMSE、按参考 RMS 归一化的 NRMSE、最大绝对误差以及事件数量/时间。
零参考响应不计算相对误差。时程在同一物理时刻对齐。结果为 `validation.json` 和 `validation.md`。

“PASS”只表示该次求解正常退出；“Force <1%”单独报告。
少模态可能在短时平顺工况中碰巧满足轮轨力 RMS 阈值，仍需检查加速度、模态趋势、参考解细化和强激励/接触事件。
FEM 与解析模态共用时间递推，因此这项对照主要检查空间离散、载荷投影和反馈；恒速解析解/变速数值积分另行检验传播器。
计时包含 CSV 输出，进程时钟的语义与平台有关，不能据一次扫描宣称模态法必然更快。

## 后续门槛

1. 用已确认参考轴的质心惯量、实测轨道参数与 Hertz 几何材料参数重新标定。
2. 对时变强激励分别细化相位增量、模态数、FEM 网格，并检查离轨事件时间。
3. 如需保留减振器完整曲线，在车辆校正器内加入悬挂非线性及一致切线，再做独立验证。
4. 垂向验证后增加轮对侧滚、横向和扭转轨道响应及三维接触映射，随后对接原有 27DOF 路径。
5. 正确性确定后才做振型查表、矩阵/内存复用和纯求解计时；实时固定迭代应与收敛模式分开验收。

## 参考

本地裁剪树未包含 `doxygen/documentation/manuals`、`src/demos` 和 `template_project`，因此检查了官方对应材料，并复用本项目已有独立应用与悬挂装配模式：

- [Chrono Core 手册](https://api.projectchrono.org/development/manual_core.html)
- [官方独立项目 CMake 模板](https://github.com/projectchrono/chrono/blob/main/template_project/CMakeLists.txt)
- [Chrono 弹簧示例](https://github.com/projectchrono/chrono/blob/main/src/demos/mbs/demo_MBS_spring.cpp)
- [TU Delft Euler–Bernoulli / Hermite 梁单元说明](https://teachbooks.tudelft.nl/computational-modelling/structural_linear/euler_bernouilli.html)

车辆数据来源及单位换算仍见 [25T 参数映射](25t_parameter_mapping.md)。上述参考不提供本工况的标定参数。

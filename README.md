# 25T YZ客车31DOF动力学仿真平台

本仓库基于 Project Chrono。当前铁路车辆应用已经删减为一个可复现模型：25T YZ满载客车31DOF空间动力学模型。

应用目录：applications/railway_vehicle

## 当前模型

31个自由度组成如下：

- 车体：垂向、点头、横移、侧滚、摇头，共5DOF
- 两个构架：每个5DOF，共10DOF
- 四个轮对：每个包含垂向、横移、摇头、侧滚，共16DOF

主要功能：

- 四个轮对分别读取轨道激励，并区分左右轮
- 接入实测Lv、Rv、Ld、Rd轨道不平顺
- 接入平面曲率、超高和空间速度数据
- 曲率、超高和速度按绝对里程同步
- 采用单边Hertz垂向轮轨接触
- 输出车体、构架、轮对及左右轮加速度
- 输出左右轮、内外轨轮轨法向力和减载率
- 所有31DOF结果图使用绝对里程作为横坐标

## 标准可复现工况

标准配置文件：

    applications/railway_vehicle/config/25t_yz_loaded_3d_measured_plan_180s.json

工况设置：

| 项目 | 数值 |
|---|---:|
| 仿真时间 | 180 s |
| 积分步长 | 0.001 s |
| 起始绝对里程 | 641000 m |
| 运行方向 | 里程增大方向 |
| 路线数据长度 | 前50 km |
| 空间采样间隔 | 0.25 m |
| 超高源数据单位 | mm |
| 速度源数据单位 | km/h |
| 超高计算基准轨距 | 1.435 m |
| 新轮直径 | 0.915 m |
| Hertz常数 | 9.37e10 N/m^(3/2) |

曲率和超高采用同号约定：

- 右曲线：曲率为正，右轨为外轨，正常超高为正
- 左曲线：曲率为负，左轨为外轨，正常超高为负

所有必要输入已经保存在项目内：

    applications/railway_vehicle/data/origin_excitation_data.txt
    applications/railway_vehicle/data/route/plan_50km.csv
    applications/railway_vehicle/data/route/cant_50km.csv
    applications/railway_vehicle/data/route/speed_50km.csv

## 编译与测试

在simulation_platform根目录打开PowerShell：

    cmake -S applications/railway_vehicle -B build/railway_vehicle -G "Visual Studio 17 2022" -A x64 "-DChrono_DIR=$((Get-Location).Path)/build/core/cmake"
    cmake --build build/railway_vehicle --config Release --parallel 6
    ctest --test-dir build/railway_vehicle -C Release --output-on-failure

删减后的测试套件只包含：

- track_input_checks：轨道、曲率、超高和速度输入检查
- vehicle_31dof_checks：静载平衡、Hertz接触、左右轮和31DOF动态响应检查

## 一键复现180 s结果

推荐使用带输入哈希检查的一键脚本：

    powershell -ExecutionPolicy Bypass -File applications/railway_vehicle/scripts/run_31dof_repro.ps1

指定输出目录：

    powershell -ExecutionPolicy Bypass -File applications/railway_vehicle/scripts/run_31dof_repro.ps1 -Output applications/railway_vehicle/output/31dof_reproduced_180s

脚本依次完成：

1. 检查配置和全部输入文件的SHA256
2. 运行180 s、dt=0.001 s的31DOF仿真
3. 生成响应图、左右轮轮轨力图和曲线偏载图

## 直接运行

    .\build\railway_vehicle\Release\railway_vehicle.exe --config applications/railway_vehicle/config/25t_yz_loaded_3d_measured_plan_180s.json --duration 180 --dt 0.001 --output applications/railway_vehicle/output/31dof_run_180s

输出目录必须尚不存在，程序不会覆盖已有结果。

## 查看仿真进度

另开一个PowerShell窗口。以下命令每次都会重新读取CSV，避免重复显示旧变量：

    $p = "applications/railway_vehicle/output/31dof_run_180s/response.csv"
    $t = [double](Get-Content -LiteralPath $p -Tail 1).Split(",")[0]
    "当前进度：$t / 180 s（$([math]::Round($t / 180 * 100, 1))%）"

持续刷新：

    $p = "applications/railway_vehicle/output/31dof_run_180s/response.csv"
    while (Get-Process -Name railway_vehicle -ErrorAction SilentlyContinue) { $t = [double](Get-Content -LiteralPath $p -Tail 1).Split(",")[0]; Write-Host "`r当前进度：$t / 180 s（$([math]::Round($t / 180 * 100, 1))%）" -NoNewline; Start-Sleep 10 }

## 单独绘图

    python applications/railway_vehicle/scripts/plot_results.py applications/railway_vehicle/output/31dof_run_180s/response.csv --stride 3

生成：

- response.png：车辆总体响应
- response_wheel_rail_normal_force.png：四个轮对左右轮法向力
- response_curve_load_transfer.png：速度、曲率、超高、未平衡横向加速度和内外轨偏载

轮轨力图的横坐标直接读取track_1_mileage_m，单位为绝对里程km。

## response.csv中的轮轨力列

单位均为N。

| 轮对 | 左轮法向力 | 右轮法向力 | 内轨力 | 外轨力 | 减载率 |
|---|---:|---:|---:|---:|---:|
| 1号 | ES/149 | EV/152 | EW/153 | EX/154 | EY/155 |
| 2号 | FB/158 | FE/161 | FF/162 | FG/163 | FH/164 |
| 3号 | FK/167 | FN/170 | FO/171 | FP/172 | FQ/173 |
| 4号 | FT/176 | FW/179 | FX/180 | FY/181 | FZ/182 |

左右轮列是模型直接计算值；内外轨列根据当前曲率符号重新排列。

## 基线结果与完整性检查

完整180 s本地基线：

    applications/railway_vehicle/output/31dof_hertz_filtered_180s

紧凑参考结果：

- applications/railway_vehicle/reference/summary_180s.txt
- applications/railway_vehicle/reference/response_180s.png
- applications/railway_vehicle/reference/wheel_rail_normal_force_180s.png
- applications/railway_vehicle/reference/curve_load_transfer_180s.png

复现清单：

    applications/railway_vehicle/repro_manifest.json

清单记录5个输入文件、4个参考文件和完整response.csv的SHA256。基线CSV包含180001行数据、182列，末时刻为180 s。

## 模型适用范围

当前模型是31DOF降阶车辆模型，不是完整轮轨型面接触求解器。需要注意：

- Hertz接触使用给定的标量K_H
- 尚未导入LM/CN60完整型面坐标
- 尚未进行接触点搜索、接触椭圆变化和轮缘接触计算
- 横向蠕滑采用线性Kalker系数，没有非线性饱和
- 二系横向参数和Hertz常数尚未通过实车试验标定
- 出现单轮法向力为0时表示模型发生轮轨接触截断，不能直接作为脱轨安全结论

详细说明见applications/railway_vehicle/README.md和applications/railway_vehicle/docs/plan_profile_input.md。

## Project Chrono

Project Chrono是采用BSD许可证的开源多物理场仿真框架。本项目使用其Chrono Core多体动力学能力。

- 官网：https://projectchrono.org/
- 文档：https://api.projectchrono.org/
- 许可证：https://projectchrono.org/license-chrono.txt

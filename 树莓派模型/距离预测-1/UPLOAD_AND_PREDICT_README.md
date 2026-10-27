# 上传测试点数据并预测距离

这个程序使用 `10.20亚克力` 原始对照标准，处理新的测试点数据。程序不会修改输入的 `.pridb` 或 `.tradb` 文件。

## 使用前准备

每次测试必须保留一对同名文件，并放在同一目录：

```text
测试数据.pridb
测试数据.tradb
```

例如，`测试01.pridb` 必须对应 `测试01.tradb`。当前程序读取 Vallen 数据库中的 `SetType=2` AE 事件。

## 最简单的使用方法

1. 把新的 `.pridb` 和 `.tradb` 放到一个测试文件夹中。
2. 在这台 Windows 电脑上，将该 `.pridb` 文件，或整个测试文件夹，拖到 `上传测试数据并预测距离.cmd` 上。无需先复制到树莓派。
3. 黑色窗口会直接显示“疑似局放／未检出明显信号／无法判定”和距离提示；关闭窗口前可阅读结果。
4. 程序会在 `distance_predictor_10.20\output` 中生成两个结果文件：

   - `distance_predictions_时间戳.json`：完整结果和质量检查；
   - `distance_predictions_时间戳.csv`：适合用 Excel 打开查看。

也可以在 PowerShell 中运行：

```powershell
cd "C:\Users\pc\Desktop\gis局放数据库\distance_predictor_10.20"
node .\predict_distance_10.20.js "D:\你的测试数据文件夹"
```

一次处理多个文件：

```powershell
node .\predict_distance_10.20.js "D:\测试数据\测点1.pridb" "D:\测试数据\测点2.pridb"
```

指定结果输出目录：

```powershell
node .\predict_distance_10.20.js "D:\测试数据" --out "D:\测试结果"
```

## 输出如何理解

- `has_partial_discharge`：`true` 表示与对照信号相近、疑似局放；空值表示未能确认，不等于确定没有局放；
- `distance_cm`：程序给出的距离估计；
- `distance_text`：中文距离判断；
- `nearest_reference_distance_cm`：最接近的 10.20 标准距离组；
- `rms_median`：主距离参数；
- `energy_median`：辅助交叉参数；
- `rms_energy_difference_cm`：两个估计结果的差异，差异较大时要检查探头接触和采集条件；
- `warnings`：事件数不足、波形不完整或两个参数不一致等提示。

## 当前标准的适用范围

当前标准只适用于 10.20 亚克力实验中的同一探头、同一增益、同一采样设置和 0°轴线。标准距离组为 0、10、20、30、40、50 cm。结果是当前实验条件下的粗略估计，不能直接当作跨设备、跨角度的通用距离测量。

本地程序会核对 `.tradb` 波形是否为 2 MHz、每条 200,000 点；条件不符、无可用波形或有效事件少于 30 条时，不输出数值距离。低于阈值的结果也不能排除 50 cm 之外的弱信号。当前仅有一组纯噪声对照，尚不能把“疑似局放”当作正式报警结论。

程序默认读取：

```text
..\reference_database\reference_standard_10.20
```

如果需要指定其他标准目录，可运行：

```powershell
node .\predict_distance_10.20.js "D:\测试数据" --standard "D:\标准目录"
```

输出结果只保存在 `output` 目录中。若 `.tradb` 尾部有损坏波形，程序仍会尽量使用可读取的 AE 事件，并在 `warnings` 中说明。

# `scripts/policy` 工具使用说明

- [`scripts/policy` 工具使用说明](#scriptspolicy-工具使用说明)
  - [export\_pt\_to\_onnx.py — 简要使用说明（`scripts/policy/export_pt_to_onnx.py`）](#export_pt_to_onnxpy--简要使用说明scriptspolicyexport_pt_to_onnxpy)
    - [环境](#环境)
    - [快速开始](#快速开始)
      - [1) 自动推断维度（推荐）](#1-自动推断维度推荐)
      - [2) 手动指定维度/隐藏层](#2-手动指定维度隐藏层)
    - [参数](#参数)
    - [导出规格](#导出规格)
    - [备注](#备注)
  - [export\_pt\_to\_trt.py — 简要使用说明（`scripts/policy/export_pt_to_trt.py`）](#export_pt_to_trtpy--简要使用说明scriptspolicyexport_pt_to_trtpy)
    - [环境](#环境-1)
    - [快速开始](#快速开始-1)
      - [固定 batch（包括 batch=1）](#固定-batch包括-batch1)
    - [参数（精简）](#参数精简)
    - [导出与构建规格](#导出与构建规格)
    - [命名后缀（新增）](#命名后缀新增)
    - [备注](#备注-1)
  - [inspect\_pt.py — 简要使用说（`scripts/policy/inspect_pt.py`）](#inspect_ptpy--简要使用说scriptspolicyinspect_ptpy)
    - [环境](#环境-2)
    - [快速开始](#快速开始-2)
    - [参数](#参数-1)
    - [示例输出（节选）](#示例输出节选)
    - [注意](#注意)

---

## export\_pt\_to\_onnx.py — 简要使用说明（`scripts/policy/export_pt_to_onnx.py`）

将 **PyTorch ckpt (`.pt/.pth`) 的 policy MLP** 导出为 **ONNX**：自动识别多种权重结构；支持从权重**自动推断**输入/输出/隐藏层维度（`--infer-dims`）；仅导出 `net_container.*` 与 `policy_layer.*`（自动剥离 `module.` 前缀）。

---

### 环境

* Python 3.8+
* PyTorch（`--weights-only` 需 **torch ≥ 2.0**）

---

### 快速开始

#### 1) 自动推断维度（推荐）

```bash
python scripts/policy/export_pt_to_onnx.py \
  src/drone_control/model/xxx.pt \
  src/drone_control/model/xxx.onnx \
  --drone-key drone_0 \
  --infer-dims \
  --activation elu \
  --weights-only \
  --verbose
```

#### 2) 手动指定维度/隐藏层

```bash
python scripts/policy/export_pt_to_onnx.py \
  src/drone_control/model/xxx.pt \
  src/drone_control/model/xxx.onnx \
  --drone-key drone_0 \
  --in-dim 220 --out-dim 2 \
  --hidden "1024,1024,1024,512,512,512,256,256,256" \
  --activation elu \
  --weights-only
```

---

### 参数

* `input`：必填；ckpt 路径（`.pt/.pth`）。
* `output`：必填；导出 ONNX 路径（`.onnx`）。
* `--drone-key`：选择子模型（默认 `drone_0`；若顶层无该键会自动回退）。
* `--policy-key`：policy 分支键名（默认 `policy`）。
* `--activation`：隐藏层激活函数（`relu/elu/gelu/tanh/sigmoid`）。
* `--hidden`：隐藏层维度列表，可写成 `"1024,1024,512"` 或 `"[1024,1024,512]"`。
* `--in-dim` / `--out-dim`：输入/输出维度（未开 `--infer-dims` 时**必须**提供）。
* `--infer-dims`：从权重自动推断 `in/out/hidden`（优先于手动设置）。
* `--weights-only`：仅加载权重（**torch ≥ 2.0**；老版本会自动回退普通加载）。
* `--device cpu`：加载到 CPU（固定）。
* `--verbose`：打印结构与维度信息。

---

### 导出规格

* 输入张量名：`STATES`（动态批维）
* 输出张量名：`ACTIONS`（动态批维）
* `dynamic_axes={"STATES": {0: "batch"}, "ACTIONS": {0: "batch"}}`
* `opset_version=17`，`do_constant_folding=True`

---

### 备注

* 仅导出 `net_container.*` 与 `policy_layer.*`；会自动剥离 `module.` 前缀。
* 支持多种 ckpt 结构：
  `{"drone_0":{"policy":{...}}}`, `{"policy":{...}}`, 直接 `state_dict`，或 `{"state_dict":{...}}` 等。
* 若提示 `state_dict 为空/键不匹配`，请先用你的 `inspect_pt.py` 检查权重结构或调整 `--drone-key/--policy-key`。

---

## export\_pt\_to\_trt.py — 简要使用说明（`scripts/policy/export_pt_to_trt.py`）

一键把 **PyTorch ckpt (`.pt/.pth`)** 的 policy MLP 导出为 **ONNX**，再用 **TensorRT `trtexec`** 生成 **Engine (`.engine`)**。内部先调用同目录的 `export_pt_to_onnx.py`，再调用 `trtexec`。

### 环境

* Python 3.8+
* PyTorch（仅用于读取/推断权重；`--weights-only` 需 **torch ≥ 2.0**）
* NVIDIA 驱动 + CUDA + **TensorRT**（需有 `trtexec`；可放在 `PATH` 或用 `--trtexec` 指定）

### 快速开始

**1) 自动推断维度（推荐）**

```bash
python scripts/policy/export_pt_to_trt.py \
  src/drone_control/model/xxx.pt \
  --infer-dims --weights-only \
  --drone-key drone_0 --activation elu \
  --fp16 --workspace 4096 \
  --min-batch 1 --opt-batch 8 --max-batch 32 \
  --force
```

**2) 手动指定维度/隐藏层**

```bash
python scripts/policy/export_pt_to_trt.py \
  src/drone_control/model/xxx.pt \
  --in-dim 220 --out-dim 2 \
  --hidden "1024,1024,1024,512,512,512,256,256,256" \
  --drone-key drone_0 --activation elu \
  --fp16 --workspace 4096 \
  --min-batch 1 --opt-batch 8 --max-batch 32
```

**3) 指定输出路径与 trtexec**

```bash
python scripts/policy/export_pt_to_trt.py \
  src/drone_control/model/xxx.pt \
  --onnx out/model.onnx --engine out/model.engine \
  --trtexec /usr/src/tensorrt/bin/trtexec \
  --infer-dims
```

#### 固定 batch（包括 batch=1）

将 `--min-batch / --opt-batch / --max-batch` 设为**同一个数**即可固定批大小：

```bash
# 固定 batch=1
python scripts/policy/export_pt_to_trt.py xxx.pt \
  --infer-dims --weights-only \
  --min-batch 1 --opt-batch 1 --max-batch 1 \
  --fp16
```

---

### 参数（精简）

**通用/路径**

* `ckpt`：必填；输入 ckpt（`.pt/.pth`）
* `--onnx` / `--engine`：导出 ONNX / Engine 的目标路径（默认与 ckpt 同名）
* `--force`：若目标已存在则覆盖
* `--python`：调用 ONNX 导出脚本使用的 Python（默认当前解释器）

**传给 `export_pt_to_onnx.py`**

* `--drone-key` / `--policy-key`：子模型键 / 分支键（默认 `drone_0` / `policy`）
* `--infer-dims`：从权重自动推断 `in/out/hidden`（推荐开启）
* `--in-dim` / `--out-dim` / `--hidden`：手动维度与隐藏层（未开 `--infer-dims` 时需提供 `in/out`）
* `--activation`：`relu/elu/gelu/tanh/sigmoid`
* `--weights-only`：仅加载权重（**torch ≥ 2.0**）
* `--verbose-onnx`：打印导出细节

**TensorRT 构建（`trtexec`）**

* `--trtexec`：`trtexec` 可执行路径（未给则从 `PATH` 或 `/usr/src/tensorrt/bin/trtexec` 寻找）
* `--workspace`：构建显存上限（MiB），默认 `4096`
* `--fp16` / `--int8`：启用 FP16 / INT8（INT8 仅设置标志，本脚本不含校准器）
* `--min-batch` / `--opt-batch` / `--max-batch`：动态 batch 形状（仅 batch 维动态）

### 导出与构建规格

* ONNX 由 `export_pt_to_onnx.py` 生成：

  * 输入张量：`STATES`（动态批维）
  * 输出张量：`ACTIONS`（动态批维）
  * `dynamic_axes={"STATES": {0: "batch"}, "ACTIONS": {0: "batch"}}`
  * `opset_version=17`，`do_constant_folding=True`
* TensorRT 形状绑定：`STATES:{min|opt|max_batch}x{in_dim}`（仅 batch 动态；`in_dim` 自动/手动获取）

### 命名后缀（新增）

脚本支持为导出文件自动添加后缀，区分**精度**与**批大小**（仅当你**未显式指定**输出路径时默认生效）：

* 开关：`--suffix-mode {auto|always|off}`（默认 `auto`）

  * `auto`：仅在使用**默认文件名**时加后缀
  * `always`：总是加后缀（即使你手动指定了路径）
  * `off`：不加后缀
* 规则：`_{precision}_{batch}`，示例：

  * 动态：`_fp16_b1-8-32`
  * 固定：`_fp16_b1`
* 示例（固定 batch=1）：生成 `xxx_fp16_b1.onnx` 与 `xxx_fp16_b1.engine`

---

### 备注

* 未开 `--infer-dims` 且未提供 `--in-dim/--out-dim` 会报错提示补齐参数。
* 仅导出 `net_container.*` 与 `policy_layer.*`（自动剥离 `module.` 前缀）；兼容权重结构：
  `{"drone_0":{"policy":{...}}}`、`{"policy":{...}}`、直接 `state_dict`、或 `{"state_dict":{...}}`。
* 看不清 ckpt 结构时，先用 `scripts/policy/inspect_pt.py`。
* INT8 实际部署需标定/量化流程，本脚本只传递 `--int8` 开关，不提供校准器。

---

## inspect\_pt.py — 简要使用说（`scripts/policy/inspect_pt.py`）

递归查看 **PyTorch `.pt/.pth`** 检查点结构：自动识别 `dict/list/tuple/Tensor`，打印张量 **shape/dtype**，支持关键词过滤、递归深度与每层元素数限制，默认 **CPU** 加载（免 CUDA 依赖）。

---

### 环境

* Python 3.8+
* PyTorch（`--weights-only` 需 **torch>=2.0**）

---

### 快速开始

```bash
# 单文件
python scripts/policy/inspect_pt.py checkpoints/model.pth

# 目录递归遍历所有 .pt/.pth
python scripts/policy/inspect_pt.py checkpoints/

# 关键词过滤（键名包含任一子串）
python scripts/policy/inspect_pt.py model.pth --pattern encoder layer1

# 限制递归深度与每层最大条目
python scripts/policy/inspect_pt.py model.pth --max-depth 2 --max-items 50

# 打印非 Tensor 标量的值
python scripts/policy/inspect_pt.py model.pth --show-scalars

# 仅加载权重（torch>=2.0，更省内存/更快）
python scripts/policy/inspect_pt.py model.pth --weights-only
```

---

### 参数

* `input`：必填；单个 `.pt/.pth` 文件或包含这些文件的目录。
* `--device cpu`：加载到 CPU（固定值）。
* `--weights-only`：仅加载权重（**torch>=2.0**，老版本会自动回退普通加载）。
* `--pattern <k1> <k2> ...`：按 **dict 键名** 过滤（多个关键词为 OR）。
* `--max-depth <int>`：限制递归深度（`0` 仅顶层；更深传更大值）。
* `--max-items <int>`：每层最多打印的元素数量。
* `--show-scalars`：打印非 Tensor 标量实际值（长字符串自动截断）。

> 说明：即便父键未命中 `--pattern`，仍会继续递归以发现子层匹配；未命中会标注 `(filtered by --pattern)`。

---

### 示例输出（节选）

```
================================================================================
加载文件: /abs/path/model.pth
顶层类型: dict
state_dict: dict
    model.layer1.weight: Tensor (64, 3, 7, 7) torch.float32
    model.layer1.bias:   Tensor (64,)        torch.float32
meta: dict
    epoch: int
    best_acc: float
```

---

### 注意

* **安全**：`torch.load()` 使用 pickle 反序列化，请只加载可信来源的 ckpt。
* 若报 `weights-only` 相关语法错误，请将代码中的 `args.weights-only` 改为 `args.weights_only`（下划线）。

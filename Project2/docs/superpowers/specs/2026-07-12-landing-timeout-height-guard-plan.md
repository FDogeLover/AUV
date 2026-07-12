# 一键降落纯超时兜底加高度判断 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 给一键降落的"纯10秒超时兜底"锁桨路径加高度判断——高度已知且≤0.5m才允许强制锁桨，高度未知或明显偏高时改为放弃自动锁定、永久等待人工接管，避免坠机风险。同时让Python侧`land()`能感知这个新状态并联动调整自己的超时行为。

**Architecture:** STM32固件(`User_Task.c`)新增全局标志`land_timeout_gaveup_f`，在纯超时兜底分支加高度门控；复用`my_protocol.c`帧2里`motor_pwm_mask`字节的空闲bit4承载这个状态，不扩展帧长度；Python侧`Lprotocol.py`解析出这一位，`Mission_GPT.py`的`land()`检测到该状态后跳过自己的25秒超时、持续维持T265速度参考直到近地强制锁定②生效或人工中止脚本。

**Tech Stack:** STM32F407 (Keil/C，`edit_firmware.py`安全编辑GBK/UTF-8混合编码文件)、Python 3 (`pytest`，项目既有TDD惯例)。

设计文档：`docs/superpowers/specs/2026-07-12-landing-timeout-height-guard-design.md`

---

## 重要提醒：固件文件编辑规则

**严禁对`.c`/`.h`文件使用Read/Edit工具**——可能静默转换编码，永久损坏中文注释。本计划里所有固件编辑都通过调用`edit_firmware.py`的`safe_replace()`函数完成（用Python脚本导入调用，而不是命令行传参，因为本次改动涉及多行代码块，命令行参数手工输入CRLF换行符容易匹配失败——这是本项目过去踩过的坑，见`.claude/CLAUDE.md`已知问题7记录）。

每个固件编辑脚本都通过`text.index(锚点)`+按行切片的方式，从文件**已有内容**里程序化提取要替换的原文，而不是手工重新输入，从而保证换行符/空白字符精确匹配。

每次固件编辑后必须跑：
```bash
python edit_firmware.py verify <file>
```
以及大括号配对检查（本计划Task 4提供）。

---

### Task 1: 固件 — User_Task.c 新增全局标志 `land_timeout_gaveup_f`

**Files:**
- Modify: `ANO_LX_FC_倾角保护版/FcSrc/User_Task.c`（GBK编码，13-14行附近）
- Modify: `ANO_LX_FC_倾角保护版/FcSrc/User_Task.h`（ASCII编码，跨文件访问需要extern声明）

**背景**：`land_timeout_cnt`等状态目前是`UserTask_OneKeyCmd()`函数内的`static`局部变量，`my_protocol.c`打包遥测帧时访问不到。项目里已有先例——`mission_stage`/`mission_done_flag`是**文件作用域全局变量**（非static，非局部），在`User_Task.h`里用`extern`声明供其他文件访问（`my_protocol.c`的`buf[3] = mission_stage;`就是这么用的）。本次新增的`land_timeout_gaveup_f`要用同样的模式。

- [ ] **Step 1: 编写并运行固件编辑脚本，在 User_Task.c 新增全局变量声明**

创建临时脚本 `scratch_task1.py`（项目根目录）：

```python
import sys
sys.path.insert(0, ".")
from edit_firmware import safe_replace

FILEPATH = "ANO_LX_FC_倾角保护版/FcSrc/User_Task.c"

old = "u8 mission_done_flag=0;// 当前任务完成后通知上位机状态"
new = (
    "u8 mission_done_flag=0;// 当前任务完成后通知上位机状态\r\n"
    "u8 land_timeout_gaveup_f=0;"
    "// 2026-07-12新增:纯超时兜底判定高度仍偏高时置1,永久放弃自动锁定,供my_protocol.c打包遥测"
)

if not safe_replace(FILEPATH, old, new):
    sys.exit(1)
print("Task 1 (User_Task.c) done")
```

运行：
```bash
python scratch_task1.py
```
预期输出以 `[OK] 编码=gbk` 结尾，退出码0。

- [ ] **Step 2: 编写并运行固件编辑脚本，在 User_Task.h 新增 extern 声明**

创建临时脚本 `scratch_task1b.py`：

```python
import sys
sys.path.insert(0, ".")
from edit_firmware import safe_replace

FILEPATH = "ANO_LX_FC_倾角保护版/FcSrc/User_Task.h"

old = "extern u8 mission_done_flag;"
new = "extern u8 mission_done_flag;\r\nextern u8 land_timeout_gaveup_f;"

if not safe_replace(FILEPATH, old, new):
    sys.exit(1)
print("Task 1b (User_Task.h) done")
```

运行：
```bash
python scratch_task1b.py
```
预期输出以 `[OK] 编码=ascii` 结尾，退出码0。

- [ ] **Step 3: 验证两个文件编码完整性**

```bash
python edit_firmware.py verify "ANO_LX_FC_倾角保护版/FcSrc/User_Task.c"
python edit_firmware.py verify "ANO_LX_FC_倾角保护版/FcSrc/User_Task.h"
```
预期两条都输出 `[OK] ...中文可读`（User_Task.h这次改动不含中文，但脚本仍会正常通过）。

- [ ] **Step 4: 删除临时脚本**

```bash
rm scratch_task1.py scratch_task1b.py
```

---

### Task 2: 固件 — 移除函数内重复的局部 static 声明

**Files:**
- Modify: `ANO_LX_FC_倾角保护版/FcSrc/User_Task.c`（`UserTask_OneKeyCmd()`函数体内，第22行附近）

**背景**：Task 1把`land_timeout_gaveup_f`提升为全局变量后，`UserTask_OneKeyCmd()`函数体内绝不能再声明同名的局部`static`变量（否则函数内部引用到的是遮蔽全局变量的局部变量，`my_protocol.c`那边读到的全局变量永远是初始值0，逻辑失效且难以察觉）。本次新增的这个变量从未在函数体内被声明为局部static过（Task 1只加了全局声明，函数体内目前还没有任何对`land_timeout_gaveup_f`的引用），所以这一步实际上不需要删除任何东西——只是确认这一点，避免后续Task 3误加了局部声明。

- [ ] **Step 1: 确认函数体内没有局部同名声明（只读检查，不修改）**

```bash
python -c "
raw = open('ANO_LX_FC_倾角保护版/FcSrc/User_Task.c','rb').read()
text = raw.decode('gbk')
n = text.count('land_timeout_gaveup_f')
assert n == 1, f'期望Task 1完成后User_Task.c里只有1处(全局声明定义那一行)，实际{n}处'
print('OK: 目前只有Task 1新增的1处全局声明定义，函数体内尚无局部声明')
"
```
（这一步是纯检查，`assert`失败说明前面Task 1的替换出了问题——比如替换了多次、或者函数体内已经意外存在同名局部声明——应停下排查而不是继续。）

---

### Task 3: 固件 — 修改纯超时兜底分支，加高度判断

**Files:**
- Modify: `ANO_LX_FC_倾角保护版/FcSrc/User_Task.c`（`UserTask_OneKeyCmd()`函数体内，`land_timeout_cnt >= 500`分支）

- [ ] **Step 1: 编写并运行固件编辑脚本**

创建临时脚本 `scratch_task3.py`：

```python
import sys
sys.path.insert(0, ".")
from edit_firmware import safe_replace

FILEPATH = "ANO_LX_FC_倾角保护版/FcSrc/User_Task.c"

# 用程序化提取而非手工输入，保证与文件内实际的CRLF/空白字符精确匹配
raw = open(FILEPATH, "rb").read()
text = raw.decode("gbk")
start = text.index("if (land_timeout_cnt >= 500)")
lines = text[start:].split("\r\n")
old = "\r\n".join(lines[:9])  # if(...) { FC_Lock(); pwm_m1..m4=0; landing_f=1; }  共9行

expected_old = (
    "if (land_timeout_cnt >= 500)  //约10秒(50Hz)\r\n"
    "\t      {\r\n"
    "\t          FC_Lock();\r\n"
    "\t          pwm_to_esc.pwm_m1 = 0;\r\n"
    "\t          pwm_to_esc.pwm_m2 = 0;\r\n"
    "\t          pwm_to_esc.pwm_m3 = 0;\r\n"
    "\t          pwm_to_esc.pwm_m4 = 0;\r\n"
    "\t          landing_f = 1;\r\n"
    "\t      }"
)
assert old == expected_old, "提取到的原文跟预期不一致，先核实文件是否已被其他改动修改"

new = (
    "if (land_timeout_cnt >= 500 && land_timeout_gaveup_f == 0)  //约10秒(50Hz)，只判定一次\r\n"
    "\t      {\r\n"
    "\t          if (ano_of.work_sta && ano_of.of_alt_cm <= 50)  "
    "//2026-07-12新增:高度数据有效且<=0.5m才允许强制锁定，宁可错杀不错放\r\n"
    "\t          {\r\n"
    "\t              FC_Lock();\r\n"
    "\t              pwm_to_esc.pwm_m1 = 0;\r\n"
    "\t              pwm_to_esc.pwm_m2 = 0;\r\n"
    "\t              pwm_to_esc.pwm_m3 = 0;\r\n"
    "\t              pwm_to_esc.pwm_m4 = 0;\r\n"
    "\t              landing_f = 1;\r\n"
    "\t          }\r\n"
    "\t          else  //高度未知或明显偏高，永久放弃自动锁定，等人工介入\r\n"
    "\t          {\r\n"
    "\t              land_timeout_gaveup_f = 1;\r\n"
    "\t          }\r\n"
    "\t      }"
)

if not safe_replace(FILEPATH, old, new):
    sys.exit(1)
print("Task 3 done")
```

运行：
```bash
python scratch_task3.py
```
预期输出以 `[OK] 编码=gbk` 结尾。如果`assert old == expected_old`失败，**停下**——说明文件内容跟本计划编写时不一致，需要重新读取当前内容再调整脚本，不要强行继续。

- [ ] **Step 2: 验证编码完整性**

```bash
python edit_firmware.py verify "ANO_LX_FC_倾角保护版/FcSrc/User_Task.c"
```

- [ ] **Step 3: 用只读方式确认改动内容正确**

```bash
python -c "
raw = open('ANO_LX_FC_倾角保护版/FcSrc/User_Task.c','rb').read()
text = raw.decode('gbk')
assert 'land_timeout_gaveup_f == 0' in text
assert 'ano_of.of_alt_cm <= 50' in text
assert 'land_timeout_gaveup_f = 1;' in text
print('OK: 高度判断逻辑已写入')
"
```

- [ ] **Step 4: 删除临时脚本**

```bash
rm scratch_task3.py
```

---

### Task 4: 固件 — 重置标志 + 大括号配对检查

**Files:**
- Modify: `ANO_LX_FC_倾角保护版/FcSrc/User_Task.c`（新任务开始时重置降落状态机的代码块，第133-136行附近）

- [ ] **Step 1: 编写并运行固件编辑脚本，在任务开始时重置新标志**

创建临时脚本 `scratch_task4.py`：

```python
import sys
sys.path.insert(0, ".")
from edit_firmware import safe_replace

FILEPATH = "ANO_LX_FC_倾角保护版/FcSrc/User_Task.c"

old = (
    "land_cmd_sent_f = 0;\r\n"
    "\t\t\t\tlanding_f = 0;\r\n"
    "\t\t\t\tlanding_cnt = 0;\r\n"
    "\t\t\t\tland_timeout_cnt = 0;"
)
new = (
    "land_cmd_sent_f = 0;\r\n"
    "\t\t\t\tlanding_f = 0;\r\n"
    "\t\t\t\tlanding_cnt = 0;\r\n"
    "\t\t\t\tland_timeout_cnt = 0;\r\n"
    "\t\t\t\tland_timeout_gaveup_f = 0;"
)

if not safe_replace(FILEPATH, old, new):
    sys.exit(1)
print("Task 4 done")
```

运行：
```bash
python scratch_task4.py
```

- [ ] **Step 2: 验证编码完整性**

```bash
python edit_firmware.py verify "ANO_LX_FC_倾角保护版/FcSrc/User_Task.c"
```

- [ ] **Step 3: 大括号配对检查（项目惯例，每次固件改动后必做）**

```bash
python -c "
data = open('ANO_LX_FC_倾角保护版/FcSrc/User_Task.c','rb').read()
oc, cc = data.count(b'{'), data.count(b'}')
print('open', oc, 'close', cc, 'match', oc==cc)
assert oc == cc
"
```

- [ ] **Step 4: 删除临时脚本**

```bash
rm scratch_task4.py
```

- [ ] **Step 5: Commit（固件文件不属于git仓库正常提交流程的一部分吗？先确认）**

本项目`ANO_LX_FC_倾角保护版/`目录跟`drone_control/`同属一个git仓库（本机仓库），按现有惯例正常`git add`+`git commit`：

```bash
git add "ANO_LX_FC_倾角保护版/FcSrc/User_Task.c" "ANO_LX_FC_倾角保护版/FcSrc/User_Task.h"
git commit -m "$(cat <<'EOF'
fix: 一键降落纯超时兜底加高度判断——高度>0.5m或未知时放弃自动锁桨，等人工接管

新增全局标志land_timeout_gaveup_f，纯超时兜底(10秒)分支加ano_of.of_alt_cm<=50判断，
只有高度确认已经足够低才允许强制锁桨；否则永久放弃这条路径的自动锁定权(近地强制
锁定②不受影响，继续独立运行)。避免飞机仍在空中时被无条件切断电机。

设计文档: docs/superpowers/specs/2026-07-12-landing-timeout-height-guard-design.md

Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>
EOF
)"
```

---

### Task 5: 固件 — my_protocol.c 打包新状态位到帧2

**Files:**
- Modify: `ANO_LX_FC_倾角保护版/Mycode/my_protocol.c`（UTF-8编码，`pi_send()`函数内`buf2[21]`打包处）

**背景**：frame2里`motor_pwm_mask`字节只用了bit0~3，bit4~7空闲，复用bit4承载`land_timeout_gaveup_f`，不需要扩展帧长度。

- [ ] **Step 1: 编写并运行固件编辑脚本**

创建临时脚本 `scratch_task5.py`：

```python
import sys
sys.path.insert(0, ".")
from edit_firmware import safe_replace

FILEPATH = "ANO_LX_FC_倾角保护版/Mycode/my_protocol.c"

raw = open(FILEPATH, "rb").read()
text = raw.decode("utf-8")
start = text.index("buf2[21] = (pwm_to_esc.pwm_m1")
lines = text[start:].split("\r\n")
old = "\r\n".join(lines[:4])

expected_old = (
    "buf2[21] = (pwm_to_esc.pwm_m1 != 0 ? 0x01 : 0) |\r\n"
    "                   (pwm_to_esc.pwm_m2 != 0 ? 0x02 : 0) |\r\n"
    "                   (pwm_to_esc.pwm_m3 != 0 ? 0x04 : 0) |\r\n"
    "                   (pwm_to_esc.pwm_m4 != 0 ? 0x08 : 0);"
)
assert old == expected_old, "提取到的原文跟预期不一致，先核实文件是否已被其他改动修改"

new = (
    "buf2[21] = (pwm_to_esc.pwm_m1 != 0 ? 0x01 : 0) |\r\n"
    "                   (pwm_to_esc.pwm_m2 != 0 ? 0x02 : 0) |\r\n"
    "                   (pwm_to_esc.pwm_m3 != 0 ? 0x04 : 0) |\r\n"
    "                   (pwm_to_esc.pwm_m4 != 0 ? 0x08 : 0) |\r\n"
    "                   (land_timeout_gaveup_f != 0 ? 0x10 : 0);"
    "  // 2026-07-12新增:bit4=纯超时兘底是否已放弃自动锁定(高度仍偏高)"
)

if not safe_replace(FILEPATH, old, new):
    sys.exit(1)
print("Task 5 done")
```

运行：
```bash
python scratch_task5.py
```

- [ ] **Step 2: 验证编码完整性**

```bash
python edit_firmware.py verify "ANO_LX_FC_倾角保护版/Mycode/my_protocol.c"
```

- [ ] **Step 3: 大括号配对检查**

```bash
python -c "
data = open('ANO_LX_FC_倾角保护版/Mycode/my_protocol.c','rb').read()
oc, cc = data.count(b'{'), data.count(b'}')
print('open', oc, 'close', cc, 'match', oc==cc)
assert oc == cc
"
```

- [ ] **Step 4: 确认 my_protocol.c 能看到 land_timeout_gaveup_f**

已核实 `my_protocol.c` 顶部已有 `#include "User_Task.h"`（跟`#include "my_protocol.h"`相邻，在`pi_send()`能直接用`mission_stage`这个先例上也印证了这一点），Task 1新增的`extern u8 land_timeout_gaveup_f;`声明会自动对`my_protocol.c`可见，不需要额外加include。这一步只需确认一遍：

```bash
grep -n "#include \"User_Task.h\"" "ANO_LX_FC_倾角保护版/Mycode/my_protocol.c"
```
预期有输出（已确认存在）。

- [ ] **Step 5: 删除临时脚本**

```bash
rm scratch_task5.py
```

- [ ] **Step 6: Commit**

```bash
git add "ANO_LX_FC_倾角保护版/Mycode/my_protocol.c"
git commit -m "$(cat <<'EOF'
feat: 帧2(0x02)复用motor_pwm_mask字节的空闲bit4承载land_timeout_gaveup_f状态

不扩展帧长度，复用已有的诊断字节。配合上一次commit的User_Task.c高度判断改动。

Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>
EOF
)"
```

---

### Task 6: Python — Lprotocol.py 解析新状态位

**Files:**
- Modify: `drone_control/basic_radar/Lcode/Lprotocol.py:113-136`
- Test: `drone_control/basic_radar/test_lprotocol_frame2_timestamp.py`（在已有文件里追加测试类）

**已核实现有辅助函数的真实签名**（`test_lprotocol_frame2_timestamp.py`）：
- `_build_frame2(motor_pwm_mask)` — 构造一个合法帧2字节串，`motor_pwm_mask`是最后一个数据字节(19字节数据的第19个，即`data[18]`)
- `_make_serial_fc(frame_bytes)` — 返回一个`Serial_fc`实例，`.ser`是伪造串口，会依次吐出`frame_bytes`
- 调用方式是 `fc.listen_fc(rxbuffer=[0] * 14)`（同步跑一次，`FakeSerial.read()`吐完缓冲区后自动把`fclisten_running`置False让循环退出）

- [ ] **Step 1: 写失败测试**

在 `drone_control/basic_radar/test_lprotocol_frame2_timestamp.py` 文件末尾追加：

```python
class TestLandTimeoutGaveupBit:
    def test_bit4_set_parses_as_land_timeout_gaveup_true(self):
        """motor_pwm_mask字节的bit4=1时，debug_data里land_timeout_gaveup应为True，
        且不影响原有bit0~3(motor_pwm_mask本身，诊断电机PWM用)的解析。"""
        # 0x1F = 0b00011111: bit0-3全1(m1~m4非零) + bit4=1(land_timeout_gaveup)
        fc = _make_serial_fc(_build_frame2(motor_pwm_mask=0x1F))

        fc.listen_fc(rxbuffer=[0] * 14)

        assert fc.debug_data["motor_pwm_mask"] == 0x1F
        assert fc.debug_data["land_timeout_gaveup"] is True

    def test_bit4_clear_parses_as_land_timeout_gaveup_false(self):
        """bit4=0时应为False，不是None——跟'字段不存在'(老固件/未收到帧2)要能区分开。"""
        fc = _make_serial_fc(_build_frame2(motor_pwm_mask=0x0F))  # bit0-3全1，bit4=0

        fc.listen_fc(rxbuffer=[0] * 14)

        assert fc.debug_data["land_timeout_gaveup"] is False
```

- [ ] **Step 2: 运行测试确认失败**

```bash
cd drone_control/basic_radar
python -m pytest test_lprotocol_frame2_timestamp.py -v
```
预期：新增的2个测试FAIL（`KeyError: 'land_timeout_gaveup'`），其余测试仍PASS。

- [ ] **Step 3: 实现解析逻辑**

编辑 `drone_control/basic_radar/Lcode/Lprotocol.py`，在第125行`motor_pwm_mask = data[18]`之后、`self.debug_data = {`字典构造前，新增一行：

```python
                    land_timeout_gaveup = bool(motor_pwm_mask & 0x10)
```

并在 `self.debug_data = {...}` 字典里新增一个键值对：

```python
                            "land_timeout_gaveup": land_timeout_gaveup,
```

（用 Edit 工具正常编辑，`Lprotocol.py` 是纯 UTF-8 Python 文件，不受.c/.h文件的编码限制规则约束。）

- [ ] **Step 4: 运行测试确认通过**

```bash
python -m pytest test_lprotocol_frame2_timestamp.py -v
```
预期：全部PASS。

- [ ] **Step 5: 运行完整测试套件确认无回归**

```bash
python -m pytest -q
```
预期：全部PASS，数量应为之前基线(51个)+2个新增=53个。

- [ ] **Step 6: Commit**

```bash
git add drone_control/basic_radar/Lcode/Lprotocol.py drone_control/basic_radar/test_lprotocol_frame2_timestamp.py
git commit -m "$(cat <<'EOF'
feat: Lprotocol.py解析帧2新增的land_timeout_gaveup状态位(bit4)

Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>
EOF
)"
```

---

### Task 7: Python — Mission_GPT.py land() 检测新状态、联动超时行为

**Files:**
- Modify: `drone_control/basic_radar/Mission_GPT.py:642-714`（`land()`函数）
- Test: `drone_control/basic_radar/test_land_logging.py`（追加测试类）

**已核实现有辅助函数的真实签名**（`test_land_logging.py`）：
- `_make_mission_for_land()` — 返回一个`mission`实例，`m.realsense`已设为`FakeRealsense(pos=(0.12,-0.05,0.03), yaw=0.01)`，`m.state="LAND"`；**`m.serial_fc_ref`默认是`None`**，需要测试里自己赋值
- `FakeSerialFcRef(laser_height_m, motor_pwm_mask=None)` — `self.debug_data = {"motor_pwm_mask": motor_pwm_mask}`（`motor_pwm_mask`为`None`时`debug_data`是空字典`{}`）；这个类目前不支持传入`land_timeout_gaveup`，测试里构造完实例后要手动在`debug_data`字典里补上这个键
- `TransientUnlockList(base, seq)` — 模拟`re_fc[5]`(unlock_sta)按`seq`列表逐次变化，序列用完后保持最后一个值不变；用来让`unlock_sta`一直读到1(不确认)，逼近超时路径
- 每个测试方法内部用 `import Mission_GPT as mg` 拿到模块引用做`monkeypatch`，不是模块级导入
- 日志断言走 `monkeypatch.setattr(mg.logger, "warning"/"info", lambda msg, *a, **k: logged.append(("warning"/"info", msg)))` 再检查 `("info", "降落确认：已上锁") in logged` 这种二元组形式

- [ ] **Step 1: 写失败测试**

在 `drone_control/basic_radar/test_land_logging.py` 文件末尾追加：

```python
class TestLandTimeoutGaveupHandling:
    """2026-07-12：固件新增land_timeout_gaveup状态(纯超时兜底判定高度仍偏高、
    已放弃自动锁桨)，land()要能感知并联动调整自己的超时行为——不然Python侧
    25秒后自己先关串口退出，会切断固件那边"等人工介入"期间悬停所需的T265
    速度参考，跟固件的设计意图冲突。"""

    def test_logs_warning_once_when_gaveup_detected(self, monkeypatch):
        """检测到land_timeout_gaveup=True时打印一次warning日志，不重复刷屏。"""
        import Mission_GPT as mg
        m = _make_mission_for_land()
        m._log_file = io.StringIO()
        m.re_fc = TransientUnlockList([0] * 14, seq=[1] * 30)  # 一直不确认(始终为1)
        m.serial_fc_ref = FakeSerialFcRef(laser_height_m=0.8, motor_pwm_mask=15)
        m.serial_fc_ref.debug_data["land_timeout_gaveup"] = True

        logged = []
        monkeypatch.setattr(mg.logger, "warning", lambda msg, *a, **k: logged.append(("warning", msg)))
        monkeypatch.setattr(mg.logger, "info", lambda msg, *a, **k: logged.append(("info", msg)))

        # land()检测到gaveup后会跳过25秒超时、无限期循环——用一个计数器在set_speed
        # 里数到N次后抛出特定异常主动打断循环，验证循环确实没有自己退出。
        call_count = {"n": 0}

        class _StopLoop(Exception):
            pass

        def _counting_set_speed(*a, **k):
            call_count["n"] += 1
            if call_count["n"] >= 20:
                raise _StopLoop()
        m.set_speed = _counting_set_speed

        try:
            m.land()
        except _StopLoop:
            pass

        gaveup_warnings = [msg for (_lvl, msg) in logged if "已放弃自动锁桨" in msg]
        assert len(gaveup_warnings) == 1  # 只打一次，不重复

    def test_skips_25s_timeout_when_gaveup_true(self, monkeypatch):
        """检测到gaveup=True后，即使超过LAND_CONFIRM_TIMEOUT_S也不应该走
        "确认超时，强制退出"分支。"""
        import Mission_GPT as mg
        monkeypatch.setattr(mg, "LAND_CONFIRM_TIMEOUT_S", 0.1)  # 很短的超时，方便测试
        m = _make_mission_for_land()
        m._log_file = io.StringIO()
        m.re_fc = TransientUnlockList([0] * 14, seq=[1] * 30)
        m.serial_fc_ref = FakeSerialFcRef(laser_height_m=0.8, motor_pwm_mask=15)
        m.serial_fc_ref.debug_data["land_timeout_gaveup"] = True

        logged = []
        monkeypatch.setattr(mg.logger, "warning", lambda msg, *a, **k: logged.append(("warning", msg)))
        monkeypatch.setattr(mg.logger, "info", lambda msg, *a, **k: logged.append(("info", msg)))

        call_count = {"n": 0}

        class _StopLoop(Exception):
            pass

        def _counting_set_speed(*a, **k):
            call_count["n"] += 1
            if call_count["n"] >= 20:  # 20轮*sleep(0.03) ≈ 0.6秒，远超0.1秒的超时阈值
                raise _StopLoop()
        m.set_speed = _counting_set_speed

        try:
            m.land()
        except _StopLoop:
            pass

        timeout_msgs = [msg for (_lvl, msg) in logged if "确认超时，强制退出" in msg]
        assert len(timeout_msgs) == 0  # 不应该触发旧的超时退出分支

    def test_normal_timeout_still_works_when_gaveup_none(self, monkeypatch):
        """字段为None(老固件/未收到帧2，或者serial_fc_ref本身没有这个debug_data键)时，
        行为不变，仍按LAND_CONFIRM_TIMEOUT_S正常超时退出——回归守卫，确保这次改动
        不破坏旧行为。"""
        import Mission_GPT as mg
        monkeypatch.setattr(mg, "LAND_CONFIRM_TIMEOUT_S", 0.1)
        m = _make_mission_for_land()
        m._log_file = io.StringIO()
        m.re_fc = TransientUnlockList([0] * 14, seq=[1] * 30)  # 始终不确认，逼近超时
        m.serial_fc_ref = FakeSerialFcRef(laser_height_m=0.8, motor_pwm_mask=15)
        # 不设置 land_timeout_gaveup 键，debug_data.get("land_timeout_gaveup") 返回 None

        logged = []
        monkeypatch.setattr(mg.logger, "warning", lambda msg, *a, **k: logged.append(("warning", msg)))
        monkeypatch.setattr(mg.logger, "info", lambda msg, *a, **k: logged.append(("info", msg)))

        m.land()  # 应该正常在0.1秒超时后自己退出，不需要外部打断

        timeout_msgs = [msg for (_lvl, msg) in logged if "确认超时，强制退出" in msg]
        assert len(timeout_msgs) == 1
```

- [ ] **Step 2: 运行测试确认部分失败**

```bash
cd drone_control/basic_radar
python -m pytest test_land_logging.py::TestLandTimeoutGaveupHandling -v
```
预期：`test_logs_warning_once_when_gaveup_detected` 和 `test_skips_25s_timeout_when_gaveup_true` 应该FAIL（当前`land()`还没有这个逻辑，会在0.1秒超时后正常走旧的"确认超时"分支并退出，不会跑够20轮`set_speed`调用触发`_StopLoop`）；`test_normal_timeout_still_works_when_gaveup_none` 应该已经PASS（这就是当前的既有行为，作为回归基线）。

- [ ] **Step 3: 实现 land() 的新逻辑**

编辑 `drone_control/basic_radar/Mission_GPT.py`：

在 `land()` 函数里 `t_start = time.time()` 和 `unlock_confirm_count = 0` 之后（第642-643行），新增：

```python
        gaveup_logged = False
```

在第672-677行（`motor_pwm_mask`/`motor_pwm_mask_t`提取那段）之后新增：

```python
            land_timeout_gaveup = None
            if self.serial_fc_ref is not None:
                with lock:
                    land_timeout_gaveup = self.serial_fc_ref.debug_data.get("land_timeout_gaveup")
            if land_timeout_gaveup and not gaveup_logged:
                logger.warning("降落纯超时兜底判定高度仍偏高，已放弃自动锁桨，需要人工介入")
                gaveup_logged = True
```

修改第711-713行的超时判断，加上`gaveup_logged`门控：

```python
            if not gaveup_logged and time.time() - t_start >= LAND_CONFIRM_TIMEOUT_S:
                logger.warning("降落确认超时，强制退出")
                break
```

- [ ] **Step 4: 运行测试确认全部通过**

```bash
python -m pytest test_land_logging.py -v
```
预期：全部PASS，包含新增的3个测试。

- [ ] **Step 5: 运行完整测试套件确认无回归**

```bash
python -m pytest -q
```
预期：全部PASS，数量应为Task 6完成后的53个 + 本次新增3个 = 56个。

- [ ] **Step 6: Commit**

```bash
git add drone_control/basic_radar/Mission_GPT.py drone_control/basic_radar/test_land_logging.py
git commit -m "$(cat <<'EOF'
feat: land()检测到固件放弃自动锁桨(高度偏高)时，跳过25秒超时并持续维持T265速度参考

避免Python侧自己的超时先切断悬停所需的速度参考，与固件"等人工介入"的设计意图保持一致。

Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>
EOF
)"
```

---

### Task 8: 同步到板子，跑测试，推送

**Files:** 无新文件，纯同步+验证

- [ ] **Step 1: 同步Python改动到 ubuntu-pi**

```bash
scp drone_control/basic_radar/Mission_GPT.py drone_control/basic_radar/Lcode/Lprotocol.py \
    drone_control/basic_radar/test_land_logging.py drone_control/basic_radar/test_lprotocol_frame2_timestamp.py \
    ubuntu-pi:/home/sunrise/Desktop/FJJ/basic_radar/
ssh ubuntu-pi "chown sunrise:sunrise /home/sunrise/Desktop/FJJ/basic_radar/Mission_GPT.py /home/sunrise/Desktop/FJJ/basic_radar/Lcode/Lprotocol.py /home/sunrise/Desktop/FJJ/basic_radar/test_land_logging.py /home/sunrise/Desktop/FJJ/basic_radar/test_lprotocol_frame2_timestamp.py"
```

- [ ] **Step 2: 板子上跑测试**

```bash
ssh ubuntu-pi "cd /home/sunrise/Desktop/FJJ && python3 -m pytest basic_radar/ -p no:anyio -q"
```
预期：全部PASS，56个。

- [ ] **Step 3: 板子上提交（换行符核对）**

按项目惯例，先核对这几个文件在板子仓库里原有的换行符约定（`git show HEAD:<file> | file -` vs 当前`file <file>`），不一致则用`sed`转换后再commit——具体步骤参照`.claude/CLAUDE.md`"Pi sync line endings"这条已有约定，执行时现场核对，不要假设。

```bash
ssh ubuntu-pi "cd /home/sunrise/Desktop/FJJ && git add basic_radar/Mission_GPT.py basic_radar/Lcode/Lprotocol.py basic_radar/test_land_logging.py basic_radar/test_lprotocol_frame2_timestamp.py && git commit -m 'feat: land()感知固件新增的land_timeout_gaveup状态(问题7/9安全隐患修复配套)'"
```

- [ ] **Step 4: 本机推送**

```bash
git push
```

---

### Task 9: 手动 — Keil编译烧录（用户操作，无法脚本化）

**这一步需要用户在Keil里手动完成，不是Claude能直接执行的操作。**

- [ ] 用户打开Keil工程，重新编译 `ANO_LX_FC_倾角保护版`
- [ ] 确认编译无新增警告/错误（尤其注意 `my_protocol.c` 新增的 `land_timeout_gaveup_f` 引用是否能正确解析——如果Task 5 Step 4发现需要额外加`#include "User_Task.h"`但没有加，这里会报未声明标识符的编译错误，届时需要回到Task 5补上include）
- [ ] 用烧录器刷入飞控
- [ ] 告知Claude"烧好了"，进入下一步验证

---

### Task 10: 验证 — 台架/真机测试（需要与用户讨论具体方式）

**这一步涉及真实硬件行为验证，不是纯代码任务。**

- [ ] 与用户讨论：拆桨台架测试 vs 直接低风险真机验证（参照项目历史上问题7/9类似改动的先例，两种方式都用过）
- [ ] 设计验证场景：**这次改动的核心行为（"高度>0.5m时超时不锁桨"）在正常低风险测试航线里很难自然触发**（正常测试都是从低高度触发降落），需要专门设计能验证这条新路径的测试方法。可选方向（与用户讨论后确定，不在本计划里预先决定）：
  - 台架测试：拆桨情况下，人为让`land_cmd_sent_f`置1但保持机体明显偏离地面(不适用，拆桨台架通常放在桌面，本来就是"贴地"状态，测不出"高度偏高"这个分支)
  - 真机测试：设计一个会让`OneKey_Land()`合理延迟超过10秒仍未完成、但飞机本身处于安全测试高度(比如1米)的场景——需要小心设计，这类场景本身就是要制造"降落异常慢"的条件，带有一定不确定性，务必人工全程遥控器待命
  - 也可以只验证"高度已经很低"这条路径完全没有回归（复用已有的低高度降落测试场景，确认修改后正常降落流程未受影响），而把"高度偏高不锁桨"这条新路径的验证标记为"理论已验证(代码逻辑+单元测试)，真机专项验证留待有相关场景自然出现或后续专门设计测试时机再做"
- [ ] 验证完成后，更新 `.claude/CLAUDE.md` 已知问题7/9记录本次修复的落地状态和验证结果（含样本量、局限性说明，遵循本项目一贯的记录风格）

---

## 范围外提醒（呼应设计文档）

- 不实现"超时时自动改为慢速下降"
- 不改动近地强制锁定②本身的逻辑
- 不改动凌霄IMU自己的`OneKey_Land()`CMD序列
- 不在本计划里同步`basic/`、`original/`两个版本（`Mission_GPT.py`/`Lprotocol.py`的改动只落在`basic_radar/`，后续视需要再考虑同步）

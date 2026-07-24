# 日志加入 roll/pitch — 设计

## 背景

CLAUDE.md 已知问题第6条（高度控制异常）的三个候选原因都需要验证"机体倾角"和"高度异常"是否同步发生，但目前 `flight_data.jsonl` 和终端输出都没有记录 roll/pitch，飞控其实已经在下行帧里回传了这两个值（`re_fc[1]`=roll_x100, `re_fc[2]`=pitch_x100），只是 `Mission_GPT.py` 没有读取使用。

## 目标

把 roll/pitch 加进日志和终端输出，为下次实测验证高度问题做准备。

## 改动内容

只改 `drone_control/basic/Mission_GPT.py` 的 `navigate()` 方法（318-358行附近）：

1. 在读取 `of1_dx/of1_dy` 的同一个 `with lock` 块里，一并读取 `roll_x100 = self.re_fc[1]`、`pitch_x100 = self.re_fc[2]`，转换成角度：`roll_deg = roll_x100/100.0`、`pitch_deg = pitch_x100/100.0`
2. `flight_data.jsonl` 日志记录新增字段：`"roll_pitch": [round(roll_deg, 2), round(pitch_deg, 2)]`
3. 终端输出字符串追加：`| att=(roll_deg:+.1f,pitch_deg:+.1f)`（格式风格跟现有 `t265v=(...)` 一致）

## 非目标

- 不改协议、不改飞控固件（roll/pitch 早就在下行帧里，纯粹是上位机没读）
- 不新增分析脚本（等下次实测数据出来再看要不要扩展 `analyze_of_t265_correlation.py` 或写新脚本关联倾角和高度异常）
- 只改 basic 版，不改全功能版 `drone_control/original/Mission_GPT.py`

## 验证

- 改完本地跑 `python -m py_compile Mission_GPT.py` 确认语法正确
- 同步到 ubuntu-pi 后，下次实测飞行时终端能看到 `att=(...)` 字段、`flight_data.jsonl` 里有 `roll_pitch` 字段即为验证通过（本次不做实际飞行验证，等后续实测）

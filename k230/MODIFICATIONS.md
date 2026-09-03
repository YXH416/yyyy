# Codex 修改记录

本目录从 2026-07-27 起采用连续修改编号。每一轮实际改动使用一个
`MOD-NNN` 编号，相关源码注释、备份、清单、验证结果和回退脚本统一使用
同一编号。后续编号从 `MOD-002` 继续。

## MOD-001 — K230 脱机 LAB 阈值调节 App

- 日期：2026-07-27
- 状态：已实现，已完成主机端静态验证；需要在 K230 实机上完成触摸与帧率验收
- 固件基线：Yahboom K230 GUI SDCard V1.4.3（2026-01-20）
- 目标：在现有 Yahboom LVGL 桌面中增加可脱机运行的 LAB 阈值编辑器

### 功能

- 桌面自动发现“阈值调节 / Threshold Tuner”App。
- 编辑器同时显示实时源图和 LAB 二值图。
- 通过触摸滑块调节 `L min/max`、`A min/max`、`B min/max`。
- 支持二值结果反转、恢复默认值、保存并立即供颜色检测使用。
- 阈值保存到 `/sdcard/configs/threshold_config.json`。
- 每次覆盖保存时保留上一版
  `/sdcard/configs/threshold_config.bak.json`。
- 原“颜色识别 > 颜色检测”在进入 Demo 时读取已保存阈值；配置缺失或无效时
  自动回退到原红色阈值 `(32, 55, 26, 92, -3, 41)`。

### 使用

1. 启动 K230 GUI，解锁桌面。
2. 滑动到“阈值调节”图标并打开。
3. 点击“打开阈值编辑器”。
4. 对照左侧源图和右侧二值图拖动六个滑块。
5. 需要时点击 `INVERT`，然后点击 `SAVE & USE`。
6. 点击 `BACK` 返回；打开“颜色识别 > 颜色检测”即可使用保存值。

### 验证

- 4 个修改后的 Python 文件均通过 CPython AST 语法解析。
- 4 个 JSON 文件均通过 JSON 解析。
- 阈值范围、min/max 顺序、读写往返、覆盖保存及 `.bak` 备份均通过测试。
- App 排序和中英文资源键通过测试。
- 未在本目录环境中执行 K230 专用 `lvgl`、`media`、`machine` 模块；实机验收项见
  `.codex_changes/MOD-001/MANIFEST.md`。

### 回退

在本目录打开 PowerShell，运行：

```powershell
powershell -ExecutionPolicy Bypass -File .\.codex_changes\MOD-001\rollback.ps1
```

回退脚本会恢复 MOD-001 前的 4 个原文件，并移除 MOD-001 新增的运行文件。
备份和本修改记录会保留，便于审计。



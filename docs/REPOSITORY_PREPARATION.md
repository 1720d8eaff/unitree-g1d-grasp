# 公开仓库整理记录

整理日期：2026-10-25（Asia/Shanghai）。来源版本：2026-09-11 开发交接快照 v0.3.14。

## 目录整理

- 根目录 README 重写为项目介绍、系统架构、源码目录、离线检查与文档导航。
- 原 README 移至 `docs/DEVELOPER_GUIDE.md`，保留原始交接操作说明。
- 保留 `laptop/`、`onboard/`、`hand/` 等现有模块路径，维持启动链、ROS 包和来源哈希之间的对应关系。
- 移除旧 `.git` 元数据与 Python 缓存；为本次发布创建独立 Git 历史。
- 不收录原 `runs/` 的运行产物及 `dist/` 的 7 组重复构建包与校验文件；保留 `runs/README.md` 和打包工具。
- 5 个混在当前 MoveIt 配置目录下的备份文件移至 `archive/legacy_configuration/`，保留其原相对目录。
- 保留几何变化前的隔离轨迹及哈希绑定的板端导出包，以维持现有完整性检查与来源记录。
- 增加 Git 忽略规则、字节保真属性、文档索引、独立校验工具与 Linux 离线 CI。

## 验证范围

- 筛查普通文件、保留的嵌套来源包和 DOCX 内部 XML 中的常见凭据模式；不把旧 Git 历史带入新仓库。
- 本地在 WSL Ubuntu 22.04 / Python 3.10 下执行原项目完整性测试，25 / 25 通过；`g1dctl doctor` 通过。
- 按整理后的文件重新生成 `MANIFEST.sha256`，并用 `tools/verify_manifest.py` 校验。
- GitHub Actions 配置 Python 3.10 / 3.12 的离线检查；远端结果以实际工作流状态为准。
- 本次没有连接机器人、执行运动、重新训练模型或重测真机性能。

维护源码后重新生成与检查清单：

```bash
python3 tools/update_manifest.py
python3 tools/verify_manifest.py
python3 -m unittest discover -s tests -v
./g1dctl doctor
```

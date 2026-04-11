# CLAUDE.md

## 项目环境

### UV 环境
本项目使用 [UV](https://docs.astral.sh/uv/) 作为 Python 包管理工具，**不要直接使用 `python` 命令**。

正确的运行方式：
```bash
# 使用 uv run 运行脚本
uv run python test_ankle_pd.py

# 或者先激活环境
uv shell
python test_ankle_pd.py
```

## 开发注意事项

1. **始终使用 `uv run`** 来执行 Python 脚本，确保依赖正确加载

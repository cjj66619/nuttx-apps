# httpnode — 已知 Bug 记录 & 修复笔记

> 本文件记录在 ESP32-S3 / openvela (NuttX) 上调试 httpnode 时踩到的坑。
> 如果你要修改 httpnode_main.c，请先读完这个文件。

---

## BUG-001 · EXCCAUSE=3/0x1c — handle_request 中调用 send() 崩溃

**日期**：2026-05-20  
**严重程度**：致命（板子重启）  
**现象**：`httpnode &` 启动正常，手机浏览器访问 `:8080` 立即触发 panic：

```
xtensa_user_panic: User Exception: EXCCAUSE=0003   ← LoadStoreAlignmentCause
xtensa_user_panic: User Exception: EXCCAUSE=001c   ← LoadProhibited (第二阶段)
LBEG: 40056fc5  LEND: 40056fe7   ← IRAM 硬件 memcpy 循环
VADDR: 00000000                   ← 访问地址 0（NULL）
A7: 0x1800 / 0x1000              ← = sizeof(body)，说明崩在 body 的 send 里
```

### 根因（3 个叠加 bug）

1. **recv() 被错误移除** — 浏览器的 GET 请求留在 lwIP IOB 链中未被消费。
   随后的 `send()` 触发 lwIP 处理这些 IOB，而 ESP32-S3 PSRAM-backed IOB
   的对齐不满足硬件 memcpy 要求 → **EXCCAUSE=3**。

2. **MSG_NOSIGNAL + 分块 send** — NuttX ESP32-S3 TCP 栈的 bug：
   每个 `send(MSG_NOSIGNAL)` 都申请新 pbuf/IOB；多次调用耗尽 IOB 池，
   `pbuf_alloc()` 返回 NULL，内部 `memcpy(NULL->payload, ...)` →
   **EXCCAUSE=0x1c, VADDR=0**。

3. **get_ip() / socket() 在 handle_request 里调用** — `socket()` 操作
   触发 `up_saveusercontext` 崩溃（已在更早版本修复，此处作备忘）。

### 错误诊断路径（走弯路的经过）

| 步骤 | 错误操作 | 结果 |
|------|---------|------|
| 1 | 认为 recv() 是 EXCCAUSE=3 的原因，移除它 | 崩溃更严重，变成 0x1c |
| 2 | 给 body/hdr 加 `aligned(4)` | 无效，根因不是 buffer 本身的对齐 |
| 3 | 改成 512B 分块 send | 反而触发 IOB 耗尽，崩溃变 0x1c |
| 4 | 加 `aligned(16)` | 部分缓解，但未解决根因 |
| ✅ 5 | 恢复 recv() + 去掉 MSG_NOSIGNAL + 单次 send | **彻底修复** |

### 正确修法

```c
/* 1. 先 recv() 消费请求，清空 IOB */
static char req[256] __attribute__((aligned(16)));
recv(conn, req, sizeof(req) - 1, 0);

/* 2. 单次 send，不用 MSG_NOSIGNAL，不分块 */
send(conn, hdr, strlen(hdr), 0);
send(conn, body, n, 0);

/* 3. body 用 static + aligned(16)，大小 ≤ 4096 */
static char body[4096] __attribute__((aligned(16)));
```

### 永远不能做的事（NuttX ESP32-S3 约束）

- ❌ 在 `handle_request` 里调用 `socket()` / `get_ip()` → crash `up_saveusercontext`
- ❌ `send(conn, ..., MSG_NOSIGNAL)` + 分块循环 → IOB 池耗尽 → EXCCAUSE=0x1c
- ❌ 省略 `recv()` drain → 未消费 IOB + send() → EXCCAUSE=3
- ❌ `mallinfo()` 在 handler 里 → PSRAM alignment fault
- ⚠️  HTML body 尽量 < 1400B（单 TCP MSS），避免 pbuf 链分配

### 验证命令

```
nsh> httpnode &
nsh> inferd demo 2000 &
```

手机浏览器访问 `http://<ip>:8080/`，dashboard 正常显示，3s 自动刷新，
推理数据实时更新，无崩溃。

---

## 参考

- 工作提交：`7d5e594ef` (HEAD, feat/wificonn-example)
- 最早工作版本参考：`1ba8f6545` (feat: httpnode dashboard + inferd)
- NuttX IOB 相关源码：`nuttx/net/utils/net_iob_concat.c`

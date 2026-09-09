#!/usr/bin/env python3
"""Provision Feetech HL-2909-C001 (FT-SCS 总线) 舵机的 ID / 波特率，可选位置校准。

给批量换装舵机用：把每个舵机单独接到 USB-TTL 调试板，依次跑本脚本，把 15 个
(或你给的) 目标 ID 写进每个舵机的 EPROM。**每次只接一台，不要广播写 ID**——
FT-SCS 的 WRITE 走单个 ID；若强行用 0xFE 广播写 ID，总线上所有舵机会被改成同一个
ID（比如 id=5），此后彼此冲突，只能全部返厂或逐个 PING 排查。这条纪律是硬性的。

使用：
  # 只读校验：确认总线上(逐个)每个 id 已配好 ID=目标值、BAUD=0、LOCK=1
  python3 provision-hls.py --port /dev/ttyUSB0 --check

  # 真正配置：对每个目标 ID 逐台解锁→写 ID/BAUD→锁定→回读校验
  python3 provision-hls.py --port /dev/ttyUSB0

  # 连"放到机械中位后再校到中位"(位置校准 0x0B, 无参数=中位)
  python3 provision-hls.py --port /dev/ttyUSB0 --home

接线/供电/波特率与 read_hls_registers.py 相同：舵机接 12V(9-14V)；GND/供电/信号
按调试板丝印；出厂默认 1 Mbps。若目标舵机波特率已被改过，加 --baud 指定当前值。

协议依据: docs/ftscs_protocol.md (FT-SCS 自定义协议: 帧 FF FF|ID|LEN|INST|PARA|CHK,
CHK = ~(ID+LEN+INST+PARA)&0xFF, 多字节小端)；寄存器依据: docs/feetech_hls_memtable.md
(5=主ID, 6=波特率档位(0=1M), 55=锁标志(0=解锁/1=锁定), 0x0B=位置校准)。
"""

from __future__ import annotations

import argparse
import sys

# ---------------- FT-SCS 协议原语 (纯字节构造, 不依赖串口) ----------------
#
# These builders and the constant table are imported from the same style as
# microduck_rl/scripts/read_hls_registers.py so the wire format stays
# provably identical. Keeping them serial-free is deliberate: it lets
# `--help` (argparse) and any unit test run without pyserial installed.

HEAD = b"\xFF\xFF"
INST_PING = 0x01
INST_READ = 0x02
INST_WRITE = 0x03
INST_CALIBRATE = 0x0B
BROADCAST = 0xFE  # 广播 ID: 禁止用于写 ID

# HLS 寄存器 (DEC, 来自 feetech_hls_memtable.md)
REG_ID = 0x05          # 主 ID
REG_BAUD = 0x06        # 波特率档位, 0 = 1 Mbps
REG_LOCK = 0x37        # 锁标志 (55), 0=解锁 1=锁定
BAUD_1MBPS = 0x00      # 0..7 = 1M..38.4k


def frame_checksum(pid: int, length: int, instr: int, params: bytes) -> int:
    return (~(pid + length + instr + sum(params))) & 0xFF


def build_ping(pid: int) -> bytes:
    return HEAD + bytes([pid, 2, INST_PING]) + bytes([frame_checksum(pid, 2, INST_PING, b"")])


def build_read(pid: int, addr: int, nbytes: int) -> bytes:
    params = bytes([addr, nbytes])
    ln = len(params) + 2
    return HEAD + bytes([pid, ln, INST_READ]) + params + bytes([frame_checksum(pid, ln, INST_READ, params)])


def build_write(pid: int, addr: int, data: bytes) -> bytes:
    params = bytes([addr]) + bytes(data)
    ln = len(params) + 2
    return HEAD + bytes([pid, ln, INST_WRITE]) + params + bytes([frame_checksum(pid, ln, INST_WRITE, params)])


def build_calibrate(pid: int) -> bytes:
    """0x0B 位置校准, 无参数 = 把当前位置校准为中位。"""
    return HEAD + bytes([pid, 2, INST_CALIBRATE]) + bytes([frame_checksum(pid, 2, INST_CALIBRATE, b"")])


# ---------------- 串口层 (惰性导入 pyserial) ----------------

def _require_pyserial():
    try:
        import serial  # noqa: F401
    except ImportError:
        print("需要 pyserial:  pip install pyserial   或   uv run --with pyserial python scripts/provision-hls.py")
        sys.exit(2)
    return serial


class HlsBus:
    """单台舵机的一条半双工 FT-SCS 总线连接。

    - `ping`: 探测一个 ID 是否存在。
    - `read`: 读 nbytes 个寄存器字节 (应答帧校验和校验通过才算成功)。
    - `write`: 写寄存器字节 (FT-SCS 的 WRITE 返回状态包)。
    - 只在 open/unlock/write/lock 间被一次配置流程使用；进程结束 close。
    """

    def __init__(self, port: str, baud: int):
        serial = _require_pyserial()
        try:
            self._ser = serial.Serial(port, baud, timeout=0.3)
        except Exception as e:  # 串口打不开: 给操作者人话, 不裸抛
            raise RuntimeError(f"无法打开串口 {port}: {e}") from e
        self._ser.reset_input_buffer()

    def close(self):
        try:
            self._ser.close()
        except Exception:
            pass

    def _read_frame(self):
        ser = self._ser
        head = ser.read(2)
        if len(head) != 2 or head != HEAD:
            return None
        pid = ser.read(1)
        ln = ser.read(1)
        if len(pid) != 1 or len(ln) != 1:
            return None
        body = ser.read(ln[0])  # ERR(1) + DATA + CHECK(1) == LEN
        if len(body) != ln[0]:
            return None
        err = body[0]
        data = body[1:-1]
        chk = body[-1]
        expect = (~(pid[0] + ln[0] + err + sum(data))) & 0xFF
        if chk != expect:
            return None  # 校验和错: 当作噪声/超时, 不升级成异常
        return pid[0], err, data

    def ping(self, pid: int, timeout: float | None = None) -> bool:
        # A bounded probe: used by the 0..253 scan so a bus with *no* servo
        # does not stall 253 × the 0.3 s read timeout (~76 s).
        old = self._ser.timeout
        if timeout is not None:
            self._ser.timeout = timeout
        try:
            self._ser.reset_input_buffer()
            self._ser.write(build_ping(pid))
            return self._read_frame() is not None
        finally:
            self._ser.timeout = old

    def read(self, pid: int, addr: int, nbytes: int):
        self._ser.reset_input_buffer()
        self._ser.write(build_read(pid, addr, nbytes))
        f = self._read_frame()
        if f is None or f[0] != pid:
            return None
        return bytes(f[2][:nbytes])

    def write(self, pid: int, addr: int, data: bytes) -> bool:
        self._ser.reset_input_buffer()
        self._ser.write(build_write(pid, addr, data))
        # 非广播 WRITE 会有应答帧 (状态包); 读到且校验和通过即可, 不细究状态字。
        return self._read_frame() is not None


def list_ports():
    serial = _require_pyserial()
    from serial.tools import list_ports
    return sorted(list_ports.comports(), key=lambda p: p.device)


# ---------------- 配置流程 ----------------

def find_single_servo(bus: HlsBus) -> int:
    """找总线上"当前"的那一台舵机 (单台连接)。

    FT-SCS 只有一条总线; 单台连接时应当恰好有一台响应。扫描 1..253 (跳过 0/广播):
      - 恰好一台 → 返回它的 id;
      - 多台 → 说明没做到"单台连接", 直接报错中止 (写 ID 会互相覆盖);
      - 零台 → 报错 (检查供电/信号/波特率)。
    出厂默认 id=1; 若出厂 id 已被改过, 扫描也能找到。
    """
    found = [cand for cand in range(1, 254) if bus.ping(cand, timeout=0.05)]
    if not found:
        raise RuntimeError(
            "没有 PING 到任何舵机! 检查: 供电 9-14V / 信号线接对 (调试板SIG↔舵机SIG) / "
            "波特率是否 1M (用 --baud 调整) / 是否真的只接了一台。")
    if len(found) > 1:
        raise RuntimeError(
            f"检测到多台舵机 (ids={found})。写 ID 必须逐台单机连接——请只接一台再跑, "
            f"或先拔掉多余的。")
    return found[0]


def provision_one(bus: HlsBus, current_id: int, target_id: int) -> dict:
    """把 current_id 那台改成 target_id, 并设 BAUD=0(1M), 最后锁定回读。

    顺序 (EEPROM 写入须先解锁, 写完回锁):
      1. 解锁 (写 55=0)                    —— 走 current_id
      2. 写 ID(5)=target、BAUD(6)=0       —— 走 current_id (ID 未生效前仍用老 id)
      3. 锁定 (写 55=1)                    —— ID 已变, 走 target_id
      4. 回读校验 ID(5)/BAUD(6)/LOCK(55)   —— 走 target_id
    """
    status = {
        "current_id": current_id, "target_id": target_id,
        "unlock": False, "write_id": False, "write_baud": False,
        "lock": False, "readback": None,
    }

    # 1&2: 解锁 + 写 ID/BAUD (用老 id)
    if not bus.write(current_id, REG_LOCK, bytes([0])):
        status["error"] = "解锁失败 (check power/wiring)"
        return status
    status["unlock"] = True

    if not bus.write(current_id, REG_ID, bytes([target_id])):
        status["error"] = "写 ID 失败"
        return status
    status["write_id"] = True

    if not bus.write(current_id, REG_BAUD, bytes([BAUD_1MBPS])):
        status["error"] = "写 BAUD 失败"
        return status
    status["write_baud"] = True

    # 3: ID 已生效, 用新 id 锁定
    if not bus.write(target_id, REG_LOCK, bytes([1])):
        status["error"] = "加锁失败 (ID 可能已生效; 重试或重启舵机)"
        return status
    status["lock"] = True

    # 4: 回读校验 (读 EEPROM 不需解锁)
    id_raw = bus.read(target_id, REG_ID, 1)
    baud_raw = bus.read(target_id, REG_BAUD, 1)
    lock_raw = bus.read(target_id, REG_LOCK, 1)
    status["readback"] = {
        "id": id_raw[0] if id_raw else None,
        "baud": baud_raw[0] if baud_raw else None,
        "lock": lock_raw[0] if lock_raw else None,
    }
    return status


def verify_one(bus: HlsBus, target_id: int) -> dict:
    """只读校验某台 id (无写操作)。读 ID/BAUD/LOCK 并与期望比对。"""
    status = {"target_id": target_id, "present": False, "readback": None}
    if not bus.ping(target_id):
        status["error"] = "PING 无应答"
        return status
    status["present"] = True
    id_raw = bus.read(target_id, REG_ID, 1)
    baud_raw = bus.read(target_id, REG_BAUD, 1)
    lock_raw = bus.read(target_id, REG_LOCK, 1)
    status["readback"] = {
        "id": id_raw[0] if id_raw else None,
        "baud": baud_raw[0] if baud_raw else None,
        "lock": lock_raw[0] if lock_raw else None,
    }
    return status


def calibrate_one(bus: HlsBus, target_id: int) -> bool:
    """位置校准指令 0x0B (无参数 = 校准到中位)。

    注意: 这会把 当前位置 硬设成中位——只在舵机已装到机械中位后再做,
    否则所有舵机的中位都会错。HLS ≥3.43 支持; 不支持扭矩开关写128。
    """
    bus._ser.reset_input_buffer()
    bus._ser.write(build_calibrate(target_id))
    return bus._read_frame() is not None


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", default=None, help="串口设备, 如 /dev/ttyUSB0; 缺省自动探测")
    ap.add_argument("--baud", type=int, default=1000000, help="当前舵机波特率 (默认出厂 1 Mbps)")
    ap.add_argument("--ids", type=int, nargs="+", default=[10, 11, 12, 13, 14, 20, 21, 22, 23, 24, 30, 31, 32, 33, 34],
                    help="目标 ID 列表 (默认 = 15 个: 右腿 10-14, 左腿 20-24, 颈/头/嘴 30-34, 与 model::JOINT_IDS 同集合)")
    ap.add_argument("--check", action="store_true", help="只读校验: PING 每个 id 并读 ID/BAUD/LOCK, 不写任何值")
    ap.add_argument("--home", action="store_true", help="对每个 id 发送 0x0B 位置校准 (无参数=校到中位)")
    ap.add_argument("--list-ports", action="store_true", help="只列出串口设备")
    args = ap.parse_args()

    if args.list_ports:
        for p in list_ports():
            print(f"{p.device:<16} {p.description}")
        return 0

    if args.port is None:
        ports = list_ports()
        if not ports:
            print("[!] 没有检测到任何串口设备。请插上 USB-TTL 转换器后重试 (或 --list-ports 查看)。")
            return 1
        args.port = ports[0].device
        print(f"[i] 自动选择串口: {args.port} ({ports[0].description})")

    # 去重且保留输入顺序; 禁止 0/广播, 禁止重复 ID (会把两台改成同一 id).
    seen = set()
    ids = []
    for i in args.ids:
        if i in (0, 254):
            print(f"[!] ID {i} 是 0/广播, 不能作为目标 ID")
            return 1
        if i in seen:
            print(f"[!] ID {i} 重复, 已去重")
            continue
        seen.add(i)
        ids.append(i)
    if not ids:
        print("[!] 没有可用的目标 ID")
        return 1

    print(f"[i] 串口 {args.port} @ {args.baud} baud。目标 {len(ids)} 台: {ids}")
    print("[i] 确保每次只接一台舵机 (单机连接), 否则中止。")

    bus = HlsBus(args.port, args.baud)
    try:
        if args.check:
            confirm(bus, ids, verify_one, "只读校验")
        else:
            # 逐台单机连接。每台之间提示换舵机 (交互式时), 避免把同一台反复改号。
            interactive = sys.stdin.isatty()
            for n, target in enumerate(ids):
                if interactive and n > 0:
                    try:
                        input(f"    ({n}/{len(ids)}) 请换下一台舵机, 接好后按 Enter 继续 (Ctrl-C 退出): ")
                    except (EOFError, KeyboardInterrupt):
                        print("\n[i] 已中断。")
                        return 0
                current = find_single_servo(bus)
                if current == target:
                    print(f"\n[i] id={current} 已是目标值; 校验后请换下一台 (若还是同一台则不会重复改号)")
                else:
                    print(f"\n[i] 当前舵机 id={current} → 配置成 {target}")
                st = provision_one(bus, current, target)
                ok = st.get("readback") and st["readback"]["id"] == target \
                    and st["readback"]["baud"] == BAUD_1MBPS and st["readback"]["lock"] == 1
                if not ok:
                    print(f"    [FAIL] {st}")
                    bus.close()
                    return 1
                print(f"    [ok] target={st['readback']['id']} baud={st['readback']['baud']} lock={st['readback']['lock']}")
            print("\n[i] 全部配置完成。")
            confirm(bus, ids, verify_one, "最终校验")

        if args.home:
            print("\n[i] 发送位置校准 (0x0B, 无参数=中位)…")
            for target in ids:
                if not calibrate_one(bus, target):
                    print(f"    [FAIL] id={target} 位置校准无应答")
                    bus.close()
                    return 1
                print(f"    [ok] id={target} 已校准到中位")
    finally:
        bus.close()
    return 0


def confirm(bus: HlsBus, ids, fn, label: str):
    print(f"\n== {label} ==")
    ok = True
    for target in ids:
        st = fn(bus, target)
        rb = st.get("readback")
        good = rb and rb["id"] == target and rb["baud"] == BAUD_1MBPS and rb["lock"] == 1
        if not good:
            ok = False
            print(f"  {target:>3}: [FAIL] {st}")
        else:
            print(f"  {target:>3}: [ok] id={rb['id']} baud={rb['baud']} lock={rb['lock']}")
    print("  结果:", "全部通过 ✓" if ok else "存在问题 ✗")
    return ok


if __name__ == "__main__":
    sys.exit(main())

# -*- coding: utf-8 -*-
"""圆周率 pi 压力测试 —— 单文件 Windows 桌面程序（仅用 Python 标准库）

1. 用「一核一进程」跑 Chudnovsky 二进制分割算法计算圆周率，实时显示算到了第几位；
2. 通过 Windows PDH 读取 Intel RAPL 计数器，显示 CPU 封装 / 核心的真实功耗（瓦特）；
3. 显示每个逻辑处理器的占用率，并标注 P 核 / E 核；
4. 统计运行时长、累计计算量、平均 / 峰值功耗与累计耗电。

运行：python PiStressTest.py                  GUI
     python PiStressTest.py --selftest 15   无界面自检 15 秒
"""

from __future__ import annotations

import ctypes
import ctypes.wintypes as wt
import multiprocessing as mp
import os
import queue
import sys
import threading
import time
from collections import deque
from math import isqrt

APP_NAME = "圆周率 π 压力测试"
APP_VERSION = "1.0"

if hasattr(sys, "set_int_max_str_digits"):
    sys.set_int_max_str_digits(200_000_000)

IS_WINDOWS = sys.platform.startswith("win")

# --------------------------------------------------------------------------
# 圆周率计算（Chudnovsky 二进制分割，纯整数运算）
# --------------------------------------------------------------------------

PI_LITERAL_30 = "314159265358979323846264338327"      # 公开常量，用于自检
C3_OVER_24 = 640320 ** 3 // 24
DIGITS_PER_TERM = 14.181647462725477
VERIFY_DIGITS = 400          # 每轮结束后用独立公式校验的前导位数
MAX_TARGET = 20_000_000
ESTIMATED_TDP = 55.0         # 仅在读不到 RAPL 时用于估算功耗（瓦）
HEARTBEAT_TIMEOUT = 180.0    # 主进程失联超过该秒数后，工作进程自行退出


class Aborted(Exception):
    """计算被中止。"""


class SplitCtx:
    """二进制分割上下文：中止检查 + 进度上报。"""

    __slots__ = ("stop", "hb", "hb_timeout", "nodes", "total", "next_check", "on_progress", "last_emit")

    def __init__(self, stop=None, on_progress=None, hb=None, hb_timeout=HEARTBEAT_TIMEOUT):
        self.stop = stop
        self.hb = hb
        self.hb_timeout = hb_timeout
        self.nodes = 0
        self.total = 1
        self.next_check = 128
        self.on_progress = on_progress
        self.last_emit = 0.0


def _bs(a, b, ctx):
    ctx.nodes += 1
    if ctx.nodes >= ctx.next_check:
        ctx.next_check = ctx.nodes + 128
        if ctx.stop is not None and ctx.stop.value:
            raise Aborted
        if ctx.hb is not None and time.time() - ctx.hb.value > ctx.hb_timeout:
            raise Aborted          # 主进程已退出，避免变成孤儿进程继续吃 CPU
        if ctx.on_progress is not None:
            now = time.perf_counter()
            if now - ctx.last_emit >= 0.35:
                ctx.last_emit = now
                ctx.on_progress(min(0.97, ctx.nodes / ctx.total))

    if b - a == 1:
        if a == 0:
            Pab = Qab = 1
        else:
            Pab = (6 * a - 5) * (2 * a - 1) * (6 * a - 1)
            Qab = a * a * a * C3_OVER_24
        Tab = Pab * (13591409 + 545140134 * a)
        if a & 1:
            Tab = -Tab
        return Pab, Qab, Tab

    m = (a + b) // 2
    Pam, Qam, Tam = _bs(a, m, ctx)
    Pmb, Qmb, Tmb = _bs(m, b, ctx)
    return Pam * Pmb, Qam * Qmb, Qmb * Tam + Pam * Tmb


def pi_chudnovsky(n, ctx=None):
    """返回 floor(pi * 10^n)：整数位 + 小数点后 n 位。"""
    if ctx is None:
        ctx = SplitCtx()
    terms = int(n / DIGITS_PER_TERM) + 2
    ctx.total = 2 * terms - 1
    _, Q, T = _bs(0, terms, ctx)
    one = 10 ** n
    return Q * 426880 * isqrt(10005 * one * one) // T


def pi_machin(n):
    """Machin 公式（独立于 Chudnovsky），用于交叉校验。"""
    one = 10 ** n

    def arctan_inv(x):
        total = term = one // x
        x2 = x * x
        i = 1
        sign = -1
        while term:
            term //= x2
            i += 2
            total += sign * (term // i)
            sign = -sign
        return total

    return 4 * (4 * arctan_inv(5) - arctan_inv(239))


def independent_reference(n_digits=VERIFY_DIGITS):
    """独立公式参考值：'3' + (n_digits-1) 位小数。"""
    # 多算 64 位保护位，避免 Machin 级数截断误差污染末位
    return str(pi_machin(n_digits + 64))[:n_digits]


def next_target(cur, dt):
    """按上一轮耗时决定下一轮规模：先快速爬升，够大后维持规模持续施压。"""
    if dt < 0.15:
        mult = 2.6
    elif dt < 0.5:
        mult = 2.0
    elif dt < 1.5:
        mult = 1.55
    elif dt < 4.0:
        mult = 1.30
    elif dt < 12.0:
        mult = 1.18
    elif dt < 30.0:
        mult = 1.10
    else:
        mult = 1.0
    if mult <= 1.0:
        return cur
    return min(MAX_TARGET, max(cur + 250, int(cur * mult)))


# --------------------------------------------------------------------------
# Windows 硬件接口
# --------------------------------------------------------------------------

if IS_WINDOWS:
    _kernel32 = ctypes.WinDLL("kernel32", use_last_error=True)
    _ntdll = ctypes.WinDLL("ntdll")
    _pdh = ctypes.WinDLL("pdh")


class GROUP_AFFINITY(ctypes.Structure):
    _fields_ = [("Mask", ctypes.c_ulonglong), ("Group", wt.WORD), ("Reserved", wt.WORD * 3)]


class PROCESSOR_RELATIONSHIP(ctypes.Structure):
    _fields_ = [
        ("Flags", ctypes.c_ubyte),
        ("EfficiencyClass", ctypes.c_ubyte),
        ("Reserved", ctypes.c_ubyte * 20),
        ("GroupCount", wt.WORD),
        ("GroupMask", GROUP_AFFINITY * 1),
    ]


def query_topology():
    """每个逻辑处理器的信息：{'cpu': i, 'kind': 'P'/'E', 'label': 'P1·T1'}。"""
    n_cpu = os.cpu_count() or 1
    fallback = [{"cpu": i, "kind": "?", "label": "#%d" % i} for i in range(n_cpu)]
    if not IS_WINDOWS:
        return fallback
    try:
        size = wt.DWORD(0)
        _kernel32.GetLogicalProcessorInformationEx(0, None, ctypes.byref(size))
        if size.value == 0:
            return fallback
        buf = ctypes.create_string_buffer(size.value)
        if not _kernel32.GetLogicalProcessorInformationEx(0, buf, ctypes.byref(size)):
            return fallback
        cores = []
        off = 0
        while off < size.value:
            rel_type = ctypes.c_uint.from_buffer(buf, off).value
            rel_size = ctypes.c_uint.from_buffer(buf, off + 4).value
            if rel_size <= 0:
                break
            if rel_type == 0:                       # RelationProcessorCore
                pr = PROCESSOR_RELATIONSHIP.from_buffer(buf, off + 8)
                mask = pr.GroupMask[0].Mask
                cpus = [i for i in range(64) if (mask >> i) & 1]
                cores.append((cpus, pr.EfficiencyClass))
            off += rel_size
        if not cores:
            return fallback
        effs = sorted({c[1] for c in cores})
        low = effs[0]
        info = {}
        idx_p = idx_e = 0
        for cpus, eff in sorted(cores, key=lambda c: min(c[0])):
            kind = "E" if len(effs) > 1 and eff == low else "P"
            if kind == "E":
                idx_e += 1
                for c in cpus:
                    info[c] = {"cpu": c, "kind": "E", "label": "E%d" % idx_e}
            else:
                idx_p += 1
                for j, c in enumerate(cpus, start=1):
                    label = "P%d·T%d" % (idx_p, j) if len(cpus) > 1 else "P%d" % idx_p
                    info[c] = {"cpu": c, "kind": "P", "label": label}
        return [info.get(i, fallback[i]) for i in range(n_cpu)]
    except Exception:
        return fallback


class PROCESSOR_PERF(ctypes.Structure):
    """SYSTEM_PROCESSOR_PERFORMANCE_INFORMATION：内核按 48 字节/条返回。"""

    _fields_ = [
        ("IdleTime", ctypes.c_int64),
        ("KernelTime", ctypes.c_int64),
        ("UserTime", ctypes.c_int64),
        ("DpcTime", ctypes.c_int64),
        ("InterruptTime", ctypes.c_int64),
        ("InterruptCount", wt.ULONG),
    ]


class CpuSampler:
    """NtQuerySystemInformation 采样逐逻辑处理器占用率，无需第三方库。"""

    _CLASS = 8      # SystemProcessorPerformanceInformation

    def __init__(self, n_cpu):
        self.n_cpu = n_cpu
        self._buf = (PROCESSOR_PERF * n_cpu)()
        self._prev = None
        self.usage = [0.0] * n_cpu
        self.total = 0.0
        self.sample()

    def _snapshot(self):
        ret = wt.ULONG()
        rc = _ntdll.NtQuerySystemInformation(
            self._CLASS, ctypes.byref(self._buf), ctypes.sizeof(self._buf), ctypes.byref(ret)
        )
        if rc != 0:
            return None
        return [
            (self._buf[i].IdleTime, self._buf[i].KernelTime + self._buf[i].UserTime)
            for i in range(self.n_cpu)
        ]

    def sample(self):
        cur = self._snapshot()
        if cur is None:
            return self.usage
        if self._prev is not None:
            tot_idle = tot_busy = 0
            for i, (idle, total) in enumerate(cur):
                p_idle, p_total = self._prev[i]
                d_total = total - p_total
                d_idle = idle - p_idle
                if d_total > 0:
                    v = 100.0 * (1.0 - d_idle / d_total)
                    v = 0.0 if v < 0 else (100.0 if v > 100 else v)
                    self.usage[i] = self.usage[i] * 0.4 + v * 0.6      # 指数平滑
                    tot_idle += d_idle
                    tot_busy += d_total
            if tot_busy > 0:
                self.total = 100.0 * (1.0 - tot_idle / tot_busy)
        self._prev = cur
        return self.usage


class MEMORYSTATUSEX(ctypes.Structure):
    _fields_ = [
        ("dwLength", wt.DWORD),
        ("dwMemoryLoad", wt.DWORD),
        ("ullTotalPhys", ctypes.c_ulonglong),
        ("ullAvailPhys", ctypes.c_ulonglong),
        ("ullTotalPageFile", ctypes.c_ulonglong),
        ("ullAvailPageFile", ctypes.c_ulonglong),
        ("ullTotalVirtual", ctypes.c_ulonglong),
        ("ullAvailVirtual", ctypes.c_ulonglong),
        ("ullAvailExtendedVirtual", ctypes.c_ulonglong),
    ]


def available_memory_bytes():
    if not IS_WINDOWS:
        return 4 << 30
    try:
        st = MEMORYSTATUSEX()
        st.dwLength = ctypes.sizeof(st)
        _kernel32.GlobalMemoryStatusEx(ctypes.byref(st))
        return int(st.ullAvailPhys)
    except Exception:
        return 4 << 30


def pick_max_target(n_workers):
    """按可用物理内存给单轮规模定上限（每 worker 约 8 字节/位）。"""
    budget = available_memory_bytes() * 0.35
    per_digit = 8.0 * max(1, n_workers)
    return max(500_000, min(MAX_TARGET, int(budget / per_digit)))


class PDH_FMT_COUNTERVALUE(ctypes.Structure):
    _fields_ = [("CStatus", wt.DWORD), ("pad", wt.DWORD), ("doubleValue", ctypes.c_double)]


PDH_FMT_DOUBLE = 0x00000200
PDH_INVALID_DATA = 0xC0000BC6
PDH_CSTATUS_INVALID_DATA = 0xC0000BBA


def _pdh_enum_objects():
    """PdhEnumObjectsW(...) -> 本机所有计数器集名称。"""
    n = wt.DWORD(0)
    _pdh.PdhEnumObjectsW(None, None, None, ctypes.byref(n), 400, True)     # 400 = PERF_DETAIL_WIZARD
    if n.value == 0:
        return []
    buf = ctypes.create_unicode_buffer(n.value)
    if _pdh.PdhEnumObjectsW(None, None, buf, ctypes.byref(n), 400, True) != 0:
        return []
    return [x for x in buf[: n.value].split("\0") if x]


def _pdh_enum_object_items(obj):
    """返回 (计数器名列表, 实例名列表)。"""
    nc = wt.DWORD(0)
    ni = wt.DWORD(0)
    _pdh.PdhEnumObjectItemsW(None, None, obj, None, ctypes.byref(nc), None, ctypes.byref(ni), 400, 0)
    if nc.value == 0:
        return [], []
    cb = ctypes.create_unicode_buffer(nc.value)
    ib = ctypes.create_unicode_buffer(max(ni.value, 1))
    if _pdh.PdhEnumObjectItemsW(None, None, obj, cb, ctypes.byref(nc), ib, ctypes.byref(ni), 400, 0) != 0:
        return [], []
    counters = [x for x in cb[: nc.value].split("\0") if x]
    instances = [x for x in ib[: ni.value].split("\0") if x]
    return counters, instances


class PowerMeter(threading.Thread):
    """后台线程读 Intel RAPL 功耗计数器（毫瓦 -> 瓦）。"""

    def __init__(self, interval=1.0):
        super().__init__(daemon=True)
        self.interval = interval
        self.lock = threading.Lock()
        self.latest = {"pkg": None, "core": None, "dram": None, "ts": 0.0}
        self.error = None
        self.source = ""
        self._stop = threading.Event()
        self._query = None
        self._handles = {}

    def _discover(self):
        """找到 RAPL 功耗计数器路径（兼容中英文系统）。"""
        ranked = []
        for obj in _pdh_enum_objects():
            low = obj.lower()
            if "energy" in low or "能源" in obj or "电量" in obj:
                ranked.append((0, obj))
            elif "power meter" in low or "电源计量" in obj:
                ranked.append((1, obj))
        for _pri, obj in sorted(ranked):
            counters, instances = _pdh_enum_object_items(obj)
            if not instances:
                continue
            name = None
            for c in counters:
                if c.strip().lower() in ("power", "功率"):
                    name = c.strip()
                    break
            if name is None:
                name = "Power" if any(c.strip().lower() == "energy" for c in counters) else None
            if name is None:
                continue
            return [r"\%s(%s)\%s" % (obj, inst, name) for inst in instances]
        # 兜底：直接按已知英文名枚举实例
        fallback = []
        for inst in ("rapl_package0_pkg", "rapl_package0_pp0", "rapl_package0_pp1", "rapl_package0_dram"):
            fallback.append(r"\Energy Meter(%s)\Power" % inst)
        return fallback

    @staticmethod
    def _classify(path):
        # 只按实例名判断：实例形如 rapl_package0_pkg / _pp0 / _dram
        i = path.rfind("(")
        j = path.rfind(")")
        inst = path[i + 1:j].lower() if 0 <= i < j else path.lower()
        if inst.endswith("_pkg") or inst in ("pkg", "package"):
            return "pkg"
        if "pp0" in inst or "core" in inst:
            return "core"
        if "pp1" in inst or "uncore" in inst:
            return "uncore"
        if "dram" in inst:
            return "dram"
        return None

    def run(self):
        get = _pdh.PdhGetFormattedCounterValue
        get.argtypes = [wt.HANDLE, wt.DWORD, ctypes.POINTER(wt.DWORD), ctypes.POINTER(PDH_FMT_COUNTERVALUE)]
        get.restype = wt.DWORD
        try:
            paths = self._discover()
            if not paths:
                self.error = "未找到 RAPL 功耗计数器"
                return
            self._query = wt.HANDLE()
            if _pdh.PdhOpenQueryW(None, 0, ctypes.byref(self._query)) != 0:
                self.error = "无法创建性能计数器查询"
                return
            for path in paths:
                key = self._classify(path)
                if key is None or key in self._handles:
                    continue
                h = wt.HANDLE()
                ok = _pdh.PdhAddEnglishCounterW(self._query, path, 0, ctypes.byref(h)) == 0
                if not ok:
                    ok = _pdh.PdhAddCounterW(self._query, path, 0, ctypes.byref(h)) == 0
                if ok:
                    self._handles[key] = (h, path)
            if not self._handles:
                self.error = "功耗计数器不可用"
                return
            self.source = next(iter(self._handles.values()))[1]

            # 预热：速率型计数器需要两个采样点，先垫一次，界面能马上看到瓦数
            _pdh.PdhCollectQueryData(self._query)
            self._stop.wait(0.3)

            while not self._stop.is_set():
                t0 = time.time()
                if _pdh.PdhCollectQueryData(self._query) == 0:
                    vals = {}
                    for key, (h, _p) in self._handles.items():
                        typ = wt.DWORD()
                        fv = PDH_FMT_COUNTERVALUE()
                        rc = get(h, PDH_FMT_DOUBLE, ctypes.byref(typ), ctypes.byref(fv))
                        if rc == 0 and fv.CStatus in (0, 1):
                            vals[key] = fv.doubleValue / 1000.0        # 毫瓦 -> 瓦
                        elif rc in (PDH_INVALID_DATA, PDH_CSTATUS_INVALID_DATA):
                            pass                                     # 首个采样点无效，跳过
                    if vals:
                        with self.lock:
                            self.latest.update(vals)
                            self.latest["ts"] = time.time()
                dt = time.time() - t0
                self._stop.wait(max(0.05, self.interval - dt))
        except Exception as exc:
            self.error = "%s: %s" % (type(exc).__name__, exc)

    def stop(self):
        self._stop.set()

    def snapshot(self):
        with self.lock:
            return dict(self.latest)


# --------------------------------------------------------------------------
# 压力测试工作进程
# --------------------------------------------------------------------------


def _put(q, msg):
    try:
        q.put_nowait(msg)
    except Exception:
        pass


def _pin_to_cpu(cpu):
    if not IS_WINDOWS or cpu is None or cpu >= 64:
        return
    h = _kernel32.GetCurrentProcess()
    _kernel32.SetProcessAffinityMask(h, ctypes.c_size_t(1 << cpu))
    _kernel32.SetPriorityClass(h, 0x00008000)        # ABOVE_NORMAL_PRIORITY_CLASS


def pi_worker(widx, cpu, out_q, stop_flag, max_target, heartbeat=None, hb_timeout=HEARTBEAT_TIMEOUT):
    """一个进程占一个逻辑处理器，反复计算圆周率并上报进度。"""
    if heartbeat is None:
        heartbeat = mp.RawValue("d", time.time())
    try:
        _pin_to_cpu(cpu)
    except Exception:
        pass
    try:
        ref = independent_reference(VERIFY_DIGITS)
    except Exception:
        ref = ""

    target = 2000
    work = 0
    rounds = 0

    def on_progress(frac):
        _put(out_q, {"w": widx, "k": "prog", "target": target, "frac": frac})

    try:
        while not stop_flag.value:
            ctx = SplitCtx(stop=stop_flag, on_progress=on_progress, hb=heartbeat, hb_timeout=hb_timeout)
            t0 = time.perf_counter()
            try:
                value = pi_chudnovsky(target, ctx)
            except Aborted:
                break
            dt = time.perf_counter() - t0
            ok = True
            if ref:
                head = value // 10 ** (target - VERIFY_DIGITS + 1)
                ok = str(head) == ref
            work += target
            rounds += 1
            _put(
                out_q,
                {"w": widx, "k": "round", "digits": target, "dt": dt, "ok": ok, "work": work, "rounds": rounds},
            )
            if stop_flag.value:
                break
            target = min(max_target, next_target(target, dt))
    except Exception as exc:
        _put(out_q, {"w": widx, "k": "error", "msg": "%s: %s" % (type(exc).__name__, exc)})
    finally:
        _put(out_q, {"w": widx, "k": "done"})


# --------------------------------------------------------------------------
# 格式化小工具
# --------------------------------------------------------------------------


def fmt_int(n):
    return "{:,}".format(int(n))


def fmt_work(n):
    n = float(n)
    if n >= 1e8:
        return "%.2f 亿位" % (n / 1e8)
    if n >= 1e4:
        return "%.2f 万位" % (n / 1e4)
    return "%d 位" % int(n)


def fmt_duration(sec):
    sec = max(0.0, float(sec))
    h = int(sec // 3600)
    m = int(sec % 3600 // 60)
    s = int(sec % 60)
    if h:
        return "%d:%02d:%02d" % (h, m, s)
    return "%02d:%02d" % (m, s)


def fmt_compact(n):
    n = float(n)
    if n >= 1e6:
        return "%.2fM" % (n / 1e6)
    if n >= 1e3:
        return "%.0fk" % (n / 1e3)
    return "%d" % int(n)


# --------------------------------------------------------------------------
# 图形界面
# --------------------------------------------------------------------------

import tkinter as tk                                    # noqa: E402
from tkinter import font as tkfont                      # noqa: E402

BG = "#0d1117"
CARD = "#161b22"
CARD2 = "#1c2430"
LINE = "#283040"
TEXT = "#e6edf3"
MUTED = "#8b949e"
ACCENT = "#39c5f3"
POWER = "#ff9246"
GOOD = "#3fb950"
WARN = "#d29922"
BAD = "#f85149"

SCALE = 1.0


def sp(x):
    return int(round(x * SCALE))


def enable_dpi_awareness():
    if not IS_WINDOWS:
        return 1.0
    try:
        ctypes.windll.shcore.SetProcessDpiAwareness(1)
    except Exception:
        try:
            ctypes.windll.user32.SetProcessDPIAware()
        except Exception:
            pass
    try:
        return max(1.0, ctypes.windll.user32.GetDpiForSystem() / 96.0)
    except Exception:
        return 1.0


class Card(tk.Frame):
    def __init__(self, master, **kw):
        kw.setdefault("bg", CARD)
        kw.setdefault("highlightbackground", LINE)
        kw.setdefault("highlightthickness", 1)
        kw.setdefault("bd", 0)
        super().__init__(master, **kw)


class PiStressApp:
    def __init__(self, root):
        self.root = root
        self.topo = query_topology()
        self.n_cpu = len(self.topo)
        self.sampler = CpuSampler(self.n_cpu)

        self.power = PowerMeter(interval=1.0)
        self.power.start()

        self.q = None
        self.stop_flag = None
        self.procs = []
        self.workers = []
        self.running = False
        self.t_start = 0.0
        self.t_last = time.time()
        self.max_digits = 0
        self.min_digits = 0
        self.total_work = 0
        self.total_rounds = 0
        self.verified = None
        self.errors = []
        self.power_hist = deque(maxlen=600)
        self.power_energy_wh = 0.0
        self.power_peak = 0.0
        self.power_sum = 0.0
        self.power_n = 0
        self.guard_hits = 0

        self._build_ui()
        self.root.after(120, self._tick)
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

    # ---------------------------------------------------------------- 界面
    def _build_ui(self):
        r = self.root
        r.title("%s v%s" % (APP_NAME, APP_VERSION))
        r.configure(bg=BG)
        w = min(1400, r.winfo_screenwidth() - 40)
        h = min(900, r.winfo_screenheight() - 80)
        r.geometry("%dx%d+%d+%d" % (w, h, 12, 12))
        r.minsize(sp(1080), sp(680))

        fam = set(tkfont.families())
        ui = "Microsoft YaHei UI" if "Microsoft YaHei UI" in fam else "Segoe UI"
        mono = "Cascadia Mono" if "Cascadia Mono" in fam else ("Consolas" if "Consolas" in fam else "Courier New")
        self.f_mono = mono
        self.fonts = {
            "h1": (ui, -sp(20), "bold"),
            "h2": (ui, -sp(15), "bold"),
            "body": (ui, -sp(13)),
            "small": (ui, -sp(12)),
            "tiny": (ui, -sp(11)),
            "big": (mono, -sp(46), "bold"),
            "power": (mono, -sp(34), "bold"),
            "stat": (mono, -sp(14), "bold"),
            "cell": (mono, -sp(11)),
        }

        r.grid_rowconfigure(1, weight=1)
        r.grid_columnconfigure(0, weight=1)

        head = tk.Frame(r, bg=BG)
        head.grid(row=0, column=0, sticky="ew", padx=sp(16), pady=(sp(12), sp(6)))
        tk.Label(head, text="π", bg=BG, fg=ACCENT, font=(mono, -sp(26), "bold")).pack(side="left", padx=(0, sp(10)))
        tbox = tk.Frame(head, bg=BG)
        tbox.pack(side="left")
        tk.Label(tbox, text=APP_NAME, bg=BG, fg=TEXT, font=self.fonts["h1"]).pack(anchor="w")
        tk.Label(
            tbox,
            text="Chudnovsky 多进程计算 · Intel RAPL 真实功耗 · 逐逻辑核占用率",
            bg=BG, fg=MUTED, font=self.fonts["tiny"],
        ).pack(anchor="w")
        self.badge = tk.Label(head, text="等待首轮校验", bg=CARD2, fg=MUTED, font=self.fonts["small"], padx=sp(10), pady=sp(4))
        self.badge.pack(side="right")

        body = tk.Frame(r, bg=BG)
        body.grid(row=1, column=0, sticky="nsew", padx=sp(16), pady=(0, sp(6)))
        body.grid_rowconfigure(0, weight=1)
        body.grid_columnconfigure(0, weight=0)
        body.grid_columnconfigure(1, weight=1)
        self._build_left(body)
        self._build_right(body)
        self._build_footer(r)

    def _build_left(self, parent):
        left = Card(parent)
        left.grid(row=0, column=0, sticky="nsew", padx=(0, sp(12)))
        left.grid_columnconfigure(0, weight=1)
        pad = sp(16)

        tk.Label(left, text="圆周率已算到小数点后", bg=CARD, fg=MUTED, font=self.fonts["small"]).grid(
            row=0, column=0, sticky="w", padx=pad, pady=(pad, 0)
        )
        num = tk.Frame(left, bg=CARD)
        num.grid(row=1, column=0, sticky="w", padx=pad)
        self.lbl_digits = tk.Label(num, text="0", bg=CARD, fg=ACCENT, font=self.fonts["big"])
        self.lbl_digits.pack(side="left")
        tk.Label(num, text="位", bg=CARD, fg=MUTED, font=self.fonts["h2"]).pack(
            side="left", anchor="s", pady=(0, sp(8)), padx=(sp(4), 0)
        )
        self.lbl_digits_sub = tk.Label(left, text="等待开始", bg=CARD, fg=MUTED, font=self.fonts["tiny"])
        self.lbl_digits_sub.grid(row=2, column=0, sticky="w", padx=pad, pady=(0, sp(10)))
        tk.Frame(left, bg=LINE, height=1).grid(row=3, column=0, sticky="ew", padx=pad)

        prog = tk.Frame(left, bg=CARD)
        prog.grid(row=4, column=0, sticky="ew", padx=pad, pady=(sp(12), 0))
        prog.grid_columnconfigure(0, weight=1)
        self.lbl_round = tk.Label(prog, text="本轮：—", bg=CARD, fg=TEXT, font=self.fonts["small"])
        self.lbl_round.grid(row=0, column=0, sticky="w")
        self.lbl_round_pct = tk.Label(prog, text="", bg=CARD, fg=MUTED, font=self.fonts["tiny"])
        self.lbl_round_pct.grid(row=0, column=1, sticky="e")
        self.bar_round = tk.Canvas(prog, height=sp(8), bg=CARD2, highlightthickness=0)
        self.bar_round.grid(row=1, column=0, columnspan=2, sticky="ew", pady=(sp(6), 0))
        self._bar_fill = self.bar_round.create_rectangle(0, 0, 0, sp(8), fill=ACCENT, width=0)

        tk.Label(
            left, text="π = 3.141592653589793238462643383279502884197...",
            bg=CARD, fg=MUTED, font=(self.f_mono, -sp(11)),
        ).grid(row=5, column=0, sticky="w", padx=pad, pady=(sp(12), sp(6)))
        tk.Frame(left, bg=LINE, height=1).grid(row=6, column=0, sticky="ew", padx=pad, pady=(sp(6), sp(8)))

        stats = tk.Frame(left, bg=CARD)
        stats.grid(row=7, column=0, sticky="nsew", padx=pad, pady=(0, pad))
        stats.grid_columnconfigure(1, weight=1)
        self.stat_labels = {}
        for i, (key, name) in enumerate(
            [
                ("elapsed", "运行时长"),
                ("work", "累计计算量"),
                ("rate", "平均吞吐"),
                ("rounds", "完成轮次"),
                ("corehi", "最快核心"),
                ("corelo", "最慢核心"),
                ("pavg", "平均功耗"),
                ("ppeak", "峰值功耗"),
                ("energy", "累计耗电"),
            ]
        ):
            tk.Label(stats, text=name, bg=CARD, fg=MUTED, font=self.fonts["tiny"]).grid(
                row=i, column=0, sticky="w", pady=sp(2)
            )
            lbl = tk.Label(stats, text="—", bg=CARD, fg=TEXT, font=self.fonts["stat"])
            lbl.grid(row=i, column=1, sticky="e", pady=sp(2))
            self.stat_labels[key] = lbl

    def _build_right(self, parent):
        right = tk.Frame(parent, bg=BG)
        right.grid(row=0, column=1, sticky="nsew")
        right.grid_columnconfigure(0, weight=1)
        right.grid_rowconfigure(1, weight=1)
        pad = sp(16)

        top = Card(right)
        top.grid(row=0, column=0, sticky="ew")
        top.grid_columnconfigure(0, weight=1)

        wrap = tk.Frame(top, bg=CARD)
        wrap.grid(row=0, column=0, sticky="ew", padx=pad, pady=(pad, sp(4)))
        wrap.grid_columnconfigure(0, weight=1)
        wrap.grid_columnconfigure(1, weight=1)

        left = tk.Frame(wrap, bg=CARD)
        left.grid(row=0, column=0, sticky="w")
        tk.Label(left, text="CPU 封装功耗 (Intel RAPL)", bg=CARD, fg=MUTED, font=self.fonts["small"]).pack(anchor="w")
        row = tk.Frame(left, bg=CARD)
        row.pack(anchor="w")
        self.lbl_power = tk.Label(row, text="—", bg=CARD, fg=POWER, font=self.fonts["power"])
        self.lbl_power.pack(side="left")
        tk.Label(row, text="W", bg=CARD, fg=POWER, font=self.fonts["h2"]).pack(
            side="left", anchor="s", pady=(0, sp(6)), padx=(sp(4), 0)
        )
        self.lbl_power_src = tk.Label(left, text="正在初始化计数器…", bg=CARD, fg=MUTED, font=self.fonts["tiny"])
        self.lbl_power_src.pack(anchor="w")

        rightbox = tk.Frame(wrap, bg=CARD)
        rightbox.grid(row=0, column=1, sticky="e")
        tk.Label(rightbox, text="CPU 总占用率", bg=CARD, fg=MUTED, font=self.fonts["small"]).pack(anchor="e")
        row2 = tk.Frame(rightbox, bg=CARD)
        row2.pack(anchor="e")
        self.lbl_load = tk.Label(row2, text="—", bg=CARD, fg=ACCENT, font=self.fonts["power"])
        self.lbl_load.pack(side="left")
        tk.Label(row2, text="%", bg=CARD, fg=ACCENT, font=self.fonts["h2"]).pack(
            side="left", anchor="s", pady=(0, sp(6)), padx=(sp(4), 0)
        )
        self.lbl_load_sub = tk.Label(rightbox, text="空闲", bg=CARD, fg=MUTED, font=self.fonts["tiny"])
        self.lbl_load_sub.pack(anchor="e")

        mini = tk.Frame(top, bg=CARD)
        mini.grid(row=1, column=0, sticky="ew", padx=pad, pady=(sp(8), sp(2)))
        self.mini_labels = {}
        for i, (key, name) in enumerate(
            [("core", "核心功耗 pp0"), ("dram", "内存功耗"), ("pavg", "平均功耗"), ("ppeak", "峰值功耗")]
        ):
            mini.grid_columnconfigure(i, weight=1)
            box = tk.Frame(mini, bg=CARD2)
            box.grid(row=0, column=i, sticky="ew", padx=(0 if i == 0 else sp(8), 0))
            tk.Label(box, text=name, bg=CARD2, fg=MUTED, font=self.fonts["tiny"]).pack(
                anchor="w", padx=sp(10), pady=(sp(6), 0)
            )
            lbl = tk.Label(box, text="—", bg=CARD2, fg=TEXT, font=self.fonts["stat"])
            lbl.pack(anchor="w", padx=sp(10), pady=(0, sp(6)))
            self.mini_labels[key] = lbl

        self.spark = tk.Canvas(top, height=sp(78), bg=CARD, highlightthickness=0)
        self.spark.grid(row=2, column=0, sticky="ew", padx=pad, pady=(sp(8), pad))

        cores = Card(right)
        cores.grid(row=1, column=0, sticky="nsew", pady=(sp(12), 0))
        cores.grid_columnconfigure(0, weight=1)
        cores.grid_rowconfigure(1, weight=1)
        head = tk.Frame(cores, bg=CARD)
        head.grid(row=0, column=0, sticky="ew", padx=sp(16), pady=(sp(12), sp(4)))
        tk.Label(head, text="每核心占用率", bg=CARD, fg=TEXT, font=self.fonts["h2"]).pack(side="left")
        self.lbl_cores_sub = tk.Label(
            head, text="%d 个逻辑处理器" % self.n_cpu, bg=CARD, fg=MUTED, font=self.fonts["tiny"]
        )
        self.lbl_cores_sub.pack(side="left", padx=sp(10))
        self.grid_canvas = tk.Canvas(cores, bg=CARD, highlightthickness=0, width=sp(700), height=sp(320))
        self.grid_canvas.grid(row=1, column=0, sticky="nsew", padx=sp(10), pady=(0, sp(10)))
        self.cells = []
        self._grid_size = (0, 0)
        self._layout_cells()
        self.grid_canvas.bind("<Configure>", lambda e: self._layout_cells())
        self.root.after(120, self._relayout)
        self.root.after(600, self._relayout)

    def _relayout(self):
        self._grid_size = (0, 0)
        self._layout_cells()

    def _build_footer(self, r):
        foot = tk.Frame(r, bg=BG)
        foot.grid(row=2, column=0, sticky="ew", padx=sp(16), pady=(sp(4), sp(4)))
        foot.grid_columnconfigure(6, weight=1)

        def make_btn(text, cmd, fg=ACCENT, bg=CARD2):
            return tk.Button(
                foot, text=text, command=cmd, bg=bg, fg=fg, activebackground=LINE, activeforeground=fg,
                font=self.fonts["body"], bd=0, relief="flat", padx=sp(18), pady=sp(7), cursor="hand2",
            )

        self.btn_start = make_btn("开始测试", self.start, fg="#0d1117", bg=GOOD)
        self.btn_start.grid(row=0, column=0, padx=(0, sp(8)))
        self.btn_stop = make_btn("停止", self.stop, fg=TEXT)
        self.btn_stop.grid(row=0, column=1, padx=(0, sp(8)))
        self.btn_reset = make_btn("重置统计", self.reset, fg=MUTED)
        self.btn_reset.grid(row=0, column=2, padx=(0, sp(16)))

        tk.Label(foot, text="并行进程", bg=BG, fg=MUTED, font=self.fonts["small"]).grid(row=0, column=3, padx=(0, sp(6)))
        self.var_threads = tk.StringVar(value=str(self.n_cpu))
        self.spin_threads = tk.Spinbox(
            foot, from_=1, to=self.n_cpu, textvariable=self.var_threads, width=4, font=self.fonts["body"],
            bg=CARD2, fg=TEXT, buttonbackground=CARD, insertbackground=TEXT, relief="flat", justify="center",
        )
        self.spin_threads.grid(row=0, column=4, padx=(0, sp(14)))

        tk.Label(foot, text="选择核心", bg=BG, fg=MUTED, font=self.fonts["small"]).grid(row=0, column=5, padx=(0, sp(6)))
        self.var_mode = tk.StringVar(value="all")
        modes = tk.Frame(foot, bg=BG)
        modes.grid(row=0, column=6, sticky="w")
        has_e = any(t["kind"] == "E" for t in self.topo)
        for val, name in (("all", "全部"), ("p", "仅 P 核"), ("e", "仅 E 核")):
            if val == "e" and not has_e:
                continue
            tk.Radiobutton(
                modes, text=name, value=val, variable=self.var_mode, bg=BG, fg=TEXT, selectcolor=CARD2,
                activebackground=BG, activeforeground=TEXT, font=self.fonts["small"], bd=0,
                highlightthickness=0, command=self._sync_threads,
            ).pack(side="left", padx=(0, sp(8)))

        tk.Label(foot, text="功耗上限", bg=BG, fg=MUTED, font=self.fonts["small"]).grid(
            row=0, column=7, padx=(sp(10), sp(6))
        )
        self.var_limit = tk.StringVar(value="0")
        tk.Spinbox(
            foot, from_=0, to=400, increment=5, textvariable=self.var_limit, width=5, font=self.fonts["body"],
            bg=CARD2, fg=TEXT, buttonbackground=CARD, insertbackground=TEXT, relief="flat", justify="center",
        ).grid(row=0, column=8)
        tk.Label(foot, text="W (0=不限)", bg=BG, fg=MUTED, font=self.fonts["tiny"]).grid(row=0, column=9, padx=(sp(4), 0))

        self.lbl_status = tk.Label(r, text="就绪", bg=BG, fg=MUTED, font=self.fonts["small"], anchor="w")
        self.lbl_status.grid(row=3, column=0, sticky="ew", padx=sp(18), pady=(0, sp(10)))

    def _sync_threads(self):
        mode = self.var_mode.get()
        if mode == "p":
            n = sum(1 for t in self.topo if t["kind"] == "P")
        elif mode == "e":
            n = sum(1 for t in self.topo if t["kind"] == "E")
        else:
            n = self.n_cpu
        if n:
            self.var_threads.set(str(n))

    # ------------------------------------------------------------ 核心网格
    def _layout_cells(self):
        cv = self.grid_canvas
        W = max(cv.winfo_width(), 320)
        H = max(cv.winfo_height(), 150)
        if (W, H) == self._grid_size:
            return
        self._grid_size = (W, H)
        n = self.n_cpu
        gap = sp(6)
        rows_max = max(1, int((H + gap) // (sp(42) + gap)))     # 每格至少约 42px
        cols = min(4, max(1, -(-n // rows_max)))
        rows = (n + cols - 1) // cols
        cw = (W - gap * (cols - 1)) / cols
        ch = (H - gap * (rows - 1)) / rows
        cv.delete("all")
        self.cells = []
        for i in range(n):
            c, rr = i % cols, i // cols
            x0, y0 = c * (cw + gap), rr * (ch + gap)
            x1, y1 = x0 + cw, y0 + ch
            two_line = ch >= sp(40) and cw >= sp(150)
            bar_h = sp(7)
            if two_line:
                name_y, digits_y = y0 + sp(8), y0 + sp(8)
                bar_y0 = y1 - sp(20)
                bar_x1 = x1 - sp(50)
                pct_font = self.fonts["cell"]
                txt_font = self.fonts["cell"]
            else:
                name_y = (y0 + y1) / 2
                digits_y = name_y
                bar_y0 = (y0 + y1) / 2 - bar_h / 2
                bar_x1 = x1 - sp(44)
                pct_font = self.fonts["tiny"]
                txt_font = self.fonts["tiny"]
            bar_y1 = bar_y0 + bar_h
            bg_item = cv.create_rectangle(x0, y0, x1, y1, fill=CARD2, width=0)
            name_item = cv.create_text(
                x0 + sp(9), name_y, text="#%d %s" % (i, self.topo[i]["label"]),
                anchor="nw" if two_line else "w", fill=MUTED, font=txt_font,
            )
            if two_line:
                bar_x0 = x0 + sp(9)
            else:
                bb = cv.bbox(name_item)
                bar_x0 = min(x1 - sp(96), (bb[2] + sp(13)) if bb else x0 + sp(78))
            self.cells.append(
                {
                    "bg": bg_item,
                    "name": name_item,
                    "digits": (
                        cv.create_text(x1 - sp(9), digits_y, text="", anchor="ne", fill=MUTED, font=self.fonts["cell"])
                        if two_line else None
                    ),
                    "bar_bg": cv.create_rectangle(bar_x0, bar_y0, bar_x1, bar_y1, fill=LINE, width=0),
                    "bar": cv.create_rectangle(bar_x0, bar_y0, bar_x0, bar_y1, fill=ACCENT, width=0),
                    "pct": cv.create_text(
                        x1 - sp(8), (bar_y0 + bar_y1) / 2, text="", anchor="e", fill=TEXT, font=pct_font,
                    ),
                    "bar_x0": bar_x0, "bar_x1": bar_x1, "bar_y0": bar_y0, "bar_y1": bar_y1,
                    "x0": x0, "y0": y0, "x1": x1, "y1": y1,
                }
            )
        self._draw_cells()

    def _draw_cells(self):
        if not self.cells:
            return
        cv = self.grid_canvas
        for i, (t, cell) in enumerate(zip(self.topo, self.cells)):
            u = self.sampler.usage[i] if i < len(self.sampler.usage) else 0.0
            w = self.workers[i] if (self.running and i < len(self.workers)) else None
            active = bool(w and w.get("alive"))
            cv.itemconfigure(cell["name"], text="#%d %s" % (i, t["label"]), fill=TEXT if active else MUTED)
            if cell["digits"] is not None:
                cv.itemconfigure(
                    cell["digits"],
                    text=("%s 位" % fmt_int(w["digits"])) if (active and w.get("digits")) else ("计算中" if active else "未启用"),
                )
            bx0, bx1 = cell["bar_x0"], cell["bar_x1"]
            frac = max(0.0, min(1.0, u / 100.0))
            cv.coords(cell["bar"], bx0, cell["bar_y0"], bx0 + (bx1 - bx0) * frac, cell["bar_y1"])
            col = GOOD if u >= 85 else (ACCENT if u >= 40 else (WARN if u >= 5 else MUTED))
            cv.itemconfigure(cell["bar"], fill=col)
            cv.itemconfigure(cell["pct"], text="%3.0f%%" % u, fill=TEXT if u >= 5 else MUTED)

    # ------------------------------------------------------------ 功耗曲线
    def _draw_spark(self):
        cv = self.spark
        W, H = max(cv.winfo_width(), 100), max(cv.winfo_height(), 40)
        cv.delete("all")
        hist = list(self.power_hist)
        if len(hist) < 2:
            cv.create_text(W / 2, H / 2, text="功耗曲线（运行后显示）", fill=MUTED, font=self.fonts["tiny"])
            return
        vmax = max(60.0, max(v for _t, v in hist))
        vmax = float(int(vmax / 20 + 1) * 20)
        x0, y0, x1, y1 = sp(34), sp(6), W - sp(6), H - sp(14)
        for i in range(3):
            y = y0 + (y1 - y0) * i / 2
            cv.create_line(x0, y, x1, y, fill=LINE, dash=(2, 3))
            cv.create_text(x0 - sp(6), y, text="%dW" % int(vmax * (1 - i / 2)), fill=MUTED, anchor="e", font=self.fonts["tiny"])
        pts = []
        for i, (_t, v) in enumerate(hist):
            pts += [x0 + (x1 - x0) * i / max(1, len(hist) - 1), y1 - (y1 - y0) * min(1.0, v / vmax)]
        cv.create_polygon([x0, y1] + pts + [x1, y1], fill="#3a2417", outline="")
        cv.create_line(pts, fill=POWER, width=2)
        last = hist[-1][1]
        ly = y1 - (y1 - y0) * min(1.0, last / vmax)
        cv.create_oval(x1 - sp(4), ly - sp(4), x1 + sp(4), ly + sp(4), fill=POWER, outline="")

    # ------------------------------------------------------------ 运行控制
    def _selected_cpus(self):
        mode = self.var_mode.get()
        if mode == "p":
            cpus = [t["cpu"] for t in self.topo if t["kind"] == "P"]
        elif mode == "e":
            cpus = [t["cpu"] for t in self.topo if t["kind"] == "E"]
        else:
            cpus = [t["cpu"] for t in self.topo]
        try:
            n = int(float(self.var_threads.get()))
        except Exception:
            n = len(cpus)
        return cpus[: max(1, min(len(cpus), n))]

    def start(self):
        if self.running:
            return
        cpus = self._selected_cpus()
        max_target = pick_max_target(len(cpus))
        self.q = mp.Queue()
        self.stop_flag = mp.RawValue("i", 0)
        self.heartbeat = mp.RawValue("d", time.time())
        self.workers = [{"digits": 0, "target": 0, "frac": 0.0, "alive": True, "rounds": 0} for _ in cpus]
        self.procs = []
        for i, cpu in enumerate(cpus):
            p = mp.Process(
                target=pi_worker,
                args=(i, cpu, self.q, self.stop_flag, max_target, self.heartbeat),
                daemon=True,
            )
            p.start()
            self.procs.append(p)
        self.running = True
        self.t_start = time.time()
        self.t_last = self.t_start
        self.errors = []
        self.btn_start.configure(state="disabled", bg=CARD2, fg=MUTED)
        self.spin_threads.configure(state="disabled")
        self.lbl_status.configure(
            text="运行中：%d 个进程，单轮规模上限 %s 位（按可用内存自动设定）" % (len(cpus), fmt_compact(max_target)),
            fg=GOOD,
        )

    def stop(self):
        if not self.running:
            return
        self.running = False
        if self.stop_flag is not None:
            self.stop_flag.value = 1
        for p in self.procs:
            p.join(timeout=1.0)
        for p in self.procs:
            if p.is_alive():
                p.terminate()
        self.procs = []
        for w in self.workers:
            w["alive"] = False
        self.btn_start.configure(state="normal", bg=GOOD, fg="#0d1117")
        self.spin_threads.configure(state="normal")
        self.lbl_status.configure(
            text="已停止 · 最快核心算到 %s 位 · 峰值 %.1f W · 累计耗电 %.2f Wh"
            % (fmt_int(self.max_digits), self.power_peak, self.power_energy_wh),
            fg=WARN,
        )
        self._draw_cells()

    def reset(self):
        if self.running:
            self.stop()
        self.max_digits = 0
        self.min_digits = 0
        self.total_work = 0
        self.total_rounds = 0
        self.verified = None
        self.power_hist.clear()
        self.power_energy_wh = 0.0
        self.power_peak = 0.0
        self.power_sum = 0.0
        self.power_n = 0
        self.workers = []
        self.lbl_digits.configure(text="0")
        self.lbl_digits_sub.configure(text="等待开始")
        self.lbl_round.configure(text="本轮：—")
        self.lbl_round_pct.configure(text="")
        for lbl in self.stat_labels.values():
            lbl.configure(text="—")
        self.lbl_status.configure(text="已重置", fg=MUTED)
        self._draw_cells()
        self._draw_spark()

    def _on_close(self):
        try:
            self.running = False
            if self.stop_flag is not None:
                self.stop_flag.value = 1
            for p in self.procs:
                p.join(timeout=0.5)
            for p in self.procs:
                if p.is_alive():
                    p.terminate()
            self.power.stop()
        except Exception:
            pass
        self.root.destroy()

    # ------------------------------------------------------------ 主循环
    def _drain(self):
        if self.q is None:
            return
        while True:
            try:
                m = self.q.get_nowait()
            except queue.Empty:
                break
            except Exception:
                break
            idx = m.get("w", 0)
            if idx >= len(self.workers):
                continue
            state = self.workers[idx]
            kind = m.get("k")
            if kind == "round":
                state.update(
                    digits=m["digits"], target=m["digits"], frac=1.0, rounds=m.get("rounds", 0),
                    ok=m.get("ok"), dt=m.get("dt", 0.0),
                )
                self.total_work += m["digits"]
                self.total_rounds += 1
                self.max_digits = max(self.max_digits, m["digits"])
                lives = [s["digits"] for s in self.workers if s.get("alive") and s.get("digits")]
                self.min_digits = min(lives) if lives else 0
                if m.get("ok"):
                    self.verified = True
                elif m.get("ok") is False:
                    self.verified = False
            elif kind == "prog":
                state["target"] = m["target"]
                state["frac"] = m["frac"]
            elif kind == "error":
                if m.get("msg") not in self.errors:
                    self.errors.append(m.get("msg"))
            elif kind == "done":
                state["alive"] = False

    def _tick(self):
        try:
            self._update()
        except Exception as exc:
            print("tick error:", exc, file=sys.stderr)
        finally:
            self.root.after(120, self._tick)

    def _update(self):
        now = time.time()
        if self.running and self.heartbeat is not None:
            self.heartbeat.value = now          # 告诉工作进程主进程还活着
        self._drain()
        self.sampler.sample()
        snap = self.power.snapshot()

        if self.running:
            for i, p in enumerate(self.procs):
                if not p.is_alive() and self.workers[i].get("alive"):
                    self.workers[i]["alive"] = False

        pkg = snap.get("pkg")
        fresh = now - snap.get("ts", 0) < 5
        if pkg is not None and fresh:
            dt = max(0.05, now - self.t_last)
            self.power_energy_wh += pkg * dt / 3600.0
            self.power_peak = max(self.power_peak, pkg)
            self.power_sum += pkg
            self.power_n += 1
            if not self.power_hist or now - self.power_hist[-1][0] >= 0.9:
                self.power_hist.append((now, pkg))
        self.t_last = now

        # 功耗区
        if pkg is not None and fresh:
            self.lbl_power.configure(text="%.1f" % pkg, fg=POWER)
            self.lbl_power_src.configure(text="Intel RAPL · 实时采样", fg=MUTED)
        elif self.power.error and self.running:
            est = ESTIMATED_TDP * self.sampler.total / 100.0
            self.lbl_power.configure(text="%.1f" % est, fg=MUTED)
            self.lbl_power_src.configure(text="估算值：本机读不到 RAPL 功耗计数器", fg=WARN)
        else:
            self.lbl_power.configure(text="—", fg=MUTED)
            self.lbl_power_src.configure(text=self.power.error or "正在初始化…", fg=WARN)
        core_w = snap.get("core")
        dram_w = snap.get("dram")
        self.mini_labels["core"].configure(text=("%.1f W" % core_w) if core_w is not None else "—")
        self.mini_labels["dram"].configure(text=("%.1f W" % dram_w) if dram_w else "不支持")
        self.mini_labels["pavg"].configure(text=("%.1f W" % (self.power_sum / self.power_n)) if self.power_n else "—")
        self.mini_labels["ppeak"].configure(text=("%.1f W" % self.power_peak) if self.power_n else "—")

        # 占用率
        self.lbl_load.configure(text="%.1f" % self.sampler.total, fg=GOOD if self.sampler.total >= 90 else ACCENT)
        self.lbl_load_sub.configure(
            text=("正在施压：%d 个进程" % sum(1 for w in self.workers if w.get("alive"))) if self.running else "空闲",
            fg=MUTED,
        )

        # 位数
        elapsed = (now - self.t_start) if self.running else 0.0
        if self.running or self.max_digits:
            self.lbl_digits.configure(text=fmt_int(self.max_digits))
            if self.running:
                self.lbl_digits_sub.configure(
                    text="最快核心 · %d 个核心并行 · 最慢 %s 位" % (len(self.workers), fmt_int(self.min_digits)),
                    fg=MUTED,
                )
        leader = None
        for s in self.workers:
            if s.get("alive") and s.get("target") and (leader is None or s["target"] > leader["target"]):
                leader = s
        if leader:
            self.lbl_round.configure(text="领先核心本轮目标：%s 位" % fmt_int(leader["target"]))
            self.lbl_round_pct.configure(text="%.0f%%" % (leader["frac"] * 100))
            self.bar_round.coords(self._bar_fill, 0, 0, max(1, self.bar_round.winfo_width()) * leader["frac"], sp(8))

        # 统计
        if self.running:
            self.stat_labels["elapsed"].configure(text=fmt_duration(elapsed))
            self.stat_labels["work"].configure(text=fmt_work(self.total_work))
            self.stat_labels["rate"].configure(text="%s 位/秒" % fmt_compact(self.total_work / max(0.5, elapsed)))
            self.stat_labels["rounds"].configure(text=fmt_int(self.total_rounds))
            self.stat_labels["corehi"].configure(text=fmt_int(self.max_digits))
            self.stat_labels["corelo"].configure(text=fmt_int(self.min_digits))
            self.stat_labels["pavg"].configure(text=("%.1f W" % (self.power_sum / self.power_n)) if self.power_n else "—")
            self.stat_labels["ppeak"].configure(text=("%.1f W" % self.power_peak) if self.power_n else "—")
            self.stat_labels["energy"].configure(text="%.2f Wh" % self.power_energy_wh)

        if self.verified is True:
            self.badge.configure(text="前 %d 位校验通过 ✓" % VERIFY_DIGITS, fg=GOOD, bg=CARD2)
        elif self.verified is False:
            self.badge.configure(text="校验失败 ✗", fg=BAD, bg=CARD2)
        elif self.errors:
            self.badge.configure(text="计算进程报错", fg=WARN, bg=CARD2)

        # 功耗保护
        try:
            limit = float(self.var_limit.get() or 0)
        except Exception:
            limit = 0.0
        if self.running and limit > 0 and pkg is not None and fresh and pkg > limit:
            self.guard_hits += 1
            if self.guard_hits >= 3:
                self.guard_hits = 0
                self.lbl_status.configure(text="⚠ 功耗超过上限 %.0f W，已自动停止" % limit, fg=BAD)
                self.stop()
        else:
            self.guard_hits = 0

        self._draw_cells()
        self._draw_spark()


# --------------------------------------------------------------------------
# 命令行自检（无界面）
# --------------------------------------------------------------------------


def selftest(seconds=15):
    print("== 圆周率压力测试 自检 ==")
    ref = pi_machin(VERIFY_DIGITS)
    print("Machin 独立公式参考: %s…" % str(ref)[:32])
    assert str(ref)[:30] == PI_LITERAL_30, "Machin 与公开常量不一致"
    assert str(pi_chudnovsky(2000))[:30] == PI_LITERAL_30, "Chudnovsky 与公开常量不一致"
    print("算法自检通过（Chudnovsky 前 30 位 == 公开常量）")

    topo = query_topology()
    print(
        "CPU 拓扑: %d 个逻辑处理器（P 核线程 %d，E 核 %d）"
        % (len(topo), sum(1 for t in topo if t["kind"] == "P"), sum(1 for t in topo if t["kind"] == "E"))
    )
    pm = PowerMeter(interval=1.0)
    pm.start()
    sampler = CpuSampler(len(topo))
    q = mp.Queue()
    stop_flag = mp.RawValue("i", 0)
    heartbeat = mp.RawValue("d", time.time())
    cpus = [t["cpu"] for t in topo]
    max_target = pick_max_target(len(cpus))
    print("单轮规模上限: %s 位（可用内存 %.1f GB）" % (fmt_int(max_target), available_memory_bytes() / 2 ** 30))
    procs = []
    for i, cpu in enumerate(cpus):
        p = mp.Process(target=pi_worker, args=(i, cpu, q, stop_flag, max_target, heartbeat), daemon=True)
        p.start()
        procs.append(p)

    t0 = time.time()
    states = {}
    verify_msg = "（尚未完成一轮）"
    try:
        while time.time() - t0 < seconds:
            time.sleep(1.0)
            heartbeat.value = time.time()
            sampler.sample()
            while True:
                try:
                    m = q.get_nowait()
                except Exception:
                    break
                if m.get("k") == "round":
                    states[m["w"]] = m
                    if m.get("ok") is True and verify_msg == "（尚未完成一轮）":
                        verify_msg = "前 %d 位与 Machin 公式一致 ✓" % VERIFY_DIGITS
                    elif m.get("ok") is False:
                        verify_msg = "校验失败 ✗"
                elif m.get("k") == "error":
                    print("worker error:", m["msg"])
            snap = pm.snapshot()
            best = max([s["digits"] for s in states.values()], default=0)
            print(
                "  %5.1fs  最快 %10s 位 | 封装 %7s W | 核心 %7s W | 总占用 %5.1f%% | 满载核数 %2d"
                % (
                    time.time() - t0,
                    fmt_int(best),
                    ("%.1f" % snap["pkg"]) if snap["pkg"] is not None else "—",
                    ("%.1f" % snap["core"]) if snap["core"] is not None else "—",
                    sampler.total,
                    sum(1 for u in sampler.usage if u > 50),
                )
            )
    finally:
        stop_flag.value = 1
        for p in procs:
            p.join(timeout=2.0)
        for p in procs:
            if p.is_alive():
                p.terminate()
        pm.stop()
    dt = time.time() - t0
    total = sum(s["work"] for s in states.values())
    print("位数校验: %s" % verify_msg)
    print("功耗计数器: %s" % (pm.source or pm.error or "未知"))
    print("累计计算量 %s 位，用时 %.1fs，平均吞吐 %s 位/秒" % (fmt_int(total), dt, fmt_int(total / max(dt, 0.1))))
    print("自检结束")


# --------------------------------------------------------------------------
# 开发用：窗口截图 + 冒烟测试
# --------------------------------------------------------------------------


def capture_window_png(path, hwnd):
    import struct
    import zlib

    user32 = ctypes.WinDLL("user32")
    gdi32 = ctypes.WinDLL("gdi32")

    class RECT(ctypes.Structure):
        _fields_ = [("left", ctypes.c_long), ("top", ctypes.c_long), ("right", ctypes.c_long), ("bottom", ctypes.c_long)]

    class BITMAPINFOHEADER(ctypes.Structure):
        _fields_ = [
            ("biSize", wt.DWORD), ("biWidth", ctypes.c_long), ("biHeight", ctypes.c_long),
            ("biPlanes", wt.WORD), ("biBitCount", wt.WORD), ("biCompression", wt.DWORD),
            ("biSizeImage", wt.DWORD), ("biXPelsPerMeter", ctypes.c_long), ("biYPelsPerMeter", ctypes.c_long),
            ("biClrUsed", wt.DWORD), ("biClrImportant", wt.DWORD),
        ]

    rect = RECT()
    user32.GetWindowRect(hwnd, ctypes.byref(rect))
    w, h = rect.right - rect.left, rect.bottom - rect.top
    if w <= 0 or h <= 0:
        return False
    hdc = user32.GetWindowDC(hwnd)
    mem = gdi32.CreateCompatibleDC(hdc)
    bmp = gdi32.CreateCompatibleBitmap(hdc, w, h)
    gdi32.SelectObject(mem, bmp)
    user32.PrintWindow(hwnd, mem, 3)
    bi = BITMAPINFOHEADER()
    bi.biSize = ctypes.sizeof(BITMAPINFOHEADER)
    bi.biWidth = w
    bi.biHeight = -h
    bi.biPlanes = 1
    bi.biBitCount = 32
    buf = ctypes.create_string_buffer(w * h * 4)
    gdi32.GetDIBits(mem, bmp, 0, h, buf, ctypes.byref(bi), 0)
    gdi32.DeleteObject(bmp)
    gdi32.DeleteDC(mem)
    user32.ReleaseDC(hwnd, hdc)

    raw = bytearray()
    for y in range(h):
        raw.append(0)
        row = buf.raw[y * w * 4:(y + 1) * w * 4]
        for x in range(w):
            b, g, r, _a = row[x * 4:x * 4 + 4]
            raw += bytes((r, g, b))

    def chunk(tag, data):
        return (
            struct.pack(">I", len(data)) + tag + data
            + struct.pack(">I", zlib.crc32(tag + data) & 0xFFFFFFFF)
        )

    png = b"\x89PNG\r\n\x1a\n"
    png += chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0))
    png += chunk(b"IDAT", zlib.compress(bytes(raw), 6))
    png += chunk(b"IEND", b"")
    with open(path, "wb") as f:
        f.write(png)
    return True


def smoke_test(seconds=18, shot=None):
    root = tk.Tk()
    app = PiStressApp(root)
    root.after(300, app.start)
    if shot:
        def snap():
            try:
                hwnd = int(root.wm_frame(), 16)
            except Exception:
                hwnd = root.winfo_id()
            ok = capture_window_png(shot, hwnd)
            print("screenshot:", ok, shot)
        root.after(int(seconds * 1000 * 0.75), snap)

    summary = {}

    def finish():
        app.stop()
        summary.update(
            digits=app.max_digits, peak=app.power_peak, verified=app.verified,
            work=app.total_work, rounds=app.total_rounds, errors=list(app.errors),
        )
        root.after(300, root.destroy)

    root.after(int(seconds * 1000), finish)
    root.mainloop()
    report = "smoke ok: 最快 %s 位，累计 %s 位，%d 轮，峰值 %.1f W，校验=%s，错误=%s" % (
        fmt_int(summary.get("digits", 0)), fmt_int(summary.get("work", 0)), summary.get("rounds", 0),
        summary.get("peak", 0.0), summary.get("verified"), summary.get("errors") or "无",
    )
    print(report)
    if shot:                                    # 无控制台的打包版本靠这个文件回传结果
        try:
            with open(os.path.splitext(shot)[0] + ".txt", "w", encoding="utf-8") as fh:
                fh.write(report + "\n")
        except Exception:
            pass


def main():
    args = sys.argv[1:]
    if args and args[0] == "--selftest":
        selftest(int(float(args[1])) if len(args) > 1 else 15)
        return
    if args and args[0] == "--smoke":
        smoke_test(
            int(float(args[1])) if len(args) > 1 else 18,
            args[2] if len(args) > 2 else None,
        )
        return
    mp.freeze_support()
    root = tk.Tk()
    PiStressApp(root)
    root.mainloop()


if __name__ == "__main__":
    try:
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
        sys.stderr.reconfigure(encoding="utf-8", errors="replace")
    except Exception:
        pass
    SCALE = enable_dpi_awareness()
    main()

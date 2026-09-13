// MujocoDevice —— `device.Device` 的第三个实现：把链路末端换成 **MuJoCo 物理仿真**。
//
// 它做的事只有一件：起一个 `simulation/mujoco/server.py` 子进程，用 stdin/stdout
// 交换 **arm-device 文本协议**（JR / STATUS / RESET / PING ↔ OK JR / STATE / ERR）。
//
//	Browser ──WS──▶ controller ──JR──▶ [ SimDevice | SerialDevice | MujocoDevice ]
//	                                          │
//	                                    server.py（stdio）
//	                                          │
//	                                    MeArmSim（MuJoCo）
//
// 为什么这样切（spec §2 / §25 / §40）：
//   * 协议与固件**逐字节一致** ⇒ WebSocket / controller / protocol / 前端**零改动**，
//     接入点只是 main.go 的 device 工厂多一个 case；
//   * 限位与标定的真值仍在 Go 侧（controller 独占），Python 只是"另一台设备"，
//     它自己从同一份 robot.yaml 读出限位来做第二道校验 —— 不是第二份真值。
//
// ⚠️ 与 sim/serial 一样：`OK JR` 回的是**目标**舵机角（意图），只有 `STATE` 主动帧
// 才携带实际位置。别把受理回执当状态用。
package device

import (
	"bufio"
	"context"
	"errors"
	"fmt"
	"io"
	"log"
	"os/exec"
	"strings"
	"sync"
	"time"

	"armpilot/backend/internal/robot"
)

// MujocoConfig 配置 MuJoCo 物理后端。
type MujocoConfig struct {
	// Python 解释器命令。留空 = PATH 里的 "python"。
	// ⚠️ 必须指向**装了 mujoco 包**的那一个解释器，否则进程会以
	//    ModuleNotFoundError 退出（错误信息里会给出修复指引）。
	Python string
	// Script server.py 的绝对路径（由 cfg.ResolveMujocoScript 解析）。
	Script string
	// ReportHz STATE 上报频率（0 = 用 server.py 的默认 30Hz）
	ReportHz float64
	// PhysHz 物理步频率（0 = 用默认 1000Hz）
	PhysHz float64
	// BatchMs 每次实时对齐前连续推进的物理时长（0 = 用默认 10ms）
	BatchMs float64
	// NoRealtime 不做实时对齐（压测 / 离线回归用）
	NoRealtime bool
	// StartTimeoutMs 启动握手超时（0 = 默认 20s；Python 首次导入 mujoco 较慢）
	StartTimeoutMs int
}

// MujocoDevice 是 MuJoCo 设备。
type MujocoDevice struct {
	model *robot.Model
	cfg   MujocoConfig

	cmd    *exec.Cmd
	stdin  io.WriteCloser
	cancel context.CancelFunc

	writeMu sync.Mutex

	mu      sync.Mutex
	closed  bool
	ready   bool
	exited  bool
	reason  string
	statusF []StatusHandler

	readyCh chan struct{}
	done    chan struct{}

	lines chan Line
}

// NewMujoco 起一个 MuJoCo 设备进程并完成启动握手（PING → OK PING）。
//
// 握手不是可有可无的礼貌：没有它，"Python 解释器不对 / mujoco 没装 / XML 编译失败"
// 这三类最常见的故障会以"设备可用但永远没有回执"的形式出现，
// 上层只能看到 ACK 超时 —— 那是最难查的一种失败。
func NewMujoco(m *robot.Model, cfg MujocoConfig) (*MujocoDevice, error) {
	if strings.TrimSpace(cfg.Script) == "" {
		return nil, fmt.Errorf("mujoco: 未指定 server.py 路径（应由 cfg.ResolveMujocoScript 解析）")
	}
	if cfg.StartTimeoutMs <= 0 {
		cfg.StartTimeoutMs = 20000
	}
	python := strings.TrimSpace(cfg.Python)
	if python == "" {
		python = "python"
	}
	if _, err := exec.LookPath(python); err != nil {
		return nil, fmt.Errorf("mujoco: 找不到 Python 解释器 %q：%w\n"+
			"       请在 backend/config.yaml 的 device.mujoco.python 里指定绝对路径", python, err)
	}

	args := []string{cfg.Script}
	if cfg.ReportHz > 0 {
		args = append(args, "--report-hz", trimFloat(cfg.ReportHz))
	}
	if cfg.PhysHz > 0 {
		args = append(args, "--phys-hz", trimFloat(cfg.PhysHz))
	}
	if cfg.BatchMs > 0 {
		args = append(args, "--batch-ms", trimFloat(cfg.BatchMs))
	}
	if cfg.NoRealtime {
		args = append(args, "--no-realtime")
	}

	ctx, cancel := context.WithCancel(context.Background())
	cmd := exec.CommandContext(ctx, python, args...)
	stdin, err := cmd.StdinPipe()
	if err != nil {
		cancel()
		return nil, fmt.Errorf("mujoco: 取 stdin 管道失败：%w", err)
	}
	stdout, err := cmd.StdoutPipe()
	if err != nil {
		cancel()
		return nil, fmt.Errorf("mujoco: 取 stdout 管道失败：%w", err)
	}
	stderr, err := cmd.StderrPipe()
	if err != nil {
		cancel()
		return nil, fmt.Errorf("mujoco: 取 stderr 管道失败：%w", err)
	}

	d := &MujocoDevice{
		model:   m,
		cfg:     cfg,
		cmd:     cmd,
		stdin:   stdin,
		cancel:  cancel,
		readyCh: make(chan struct{}),
		done:    make(chan struct{}),
		lines:   make(chan Line, 512),
	}

	if err := cmd.Start(); err != nil {
		cancel()
		return nil, fmt.Errorf("mujoco: 启动 %s %s 失败：%w", python, cfg.Script, err)
	}
	go d.readLoop(stdout)
	go d.stderrLoop(stderr)

	if err := d.writeRaw("PING"); err != nil {
		_ = d.Close()
		return nil, err
	}
	select {
	case <-d.readyCh:
		log.Printf("[mujoco] 设备就绪：%s %s", python, cfg.Script)
	case <-time.After(time.Duration(cfg.StartTimeoutMs) * time.Millisecond):
		reason := d.UnavailableReason()
		_ = d.Close()
		return nil, fmt.Errorf("mujoco: 启动握手超时（%dms）。最后状态：%s", cfg.StartTimeoutMs, reason)
	case <-d.done:
		reason := d.UnavailableReason()
		_ = d.Close()
		return nil, fmt.Errorf("mujoco: 进程在握手前退出。%s", reason)
	}
	return d, nil
}

// ---------------------------------------------------------------------------
// Device 接口
// ---------------------------------------------------------------------------

func (d *MujocoDevice) Kind() string { return "mujoco" }

func (d *MujocoDevice) Connected() bool {
	d.mu.Lock()
	defer d.mu.Unlock()
	return !d.closed && d.ready && !d.exited
}

func (d *MujocoDevice) UnavailableReason() string {
	d.mu.Lock()
	defer d.mu.Unlock()
	switch {
	case d.closed:
		return "mujoco 设备已关闭"
	case d.exited:
		if d.reason != "" {
			return d.reason
		}
		return "mujoco 子进程已退出"
	case !d.ready:
		if d.reason != "" {
			return d.reason
		}
		return "mujoco 正在启动（等待握手）"
	default:
		return ""
	}
}

func (d *MujocoDevice) Lines() <-chan Line { return d.lines }

func (d *MujocoDevice) OnStatus(fn StatusHandler) func() {
	d.mu.Lock()
	d.statusF = append(d.statusF, fn)
	idx := len(d.statusF) - 1
	d.mu.Unlock()
	return func() {
		d.mu.Lock()
		defer d.mu.Unlock()
		if idx < len(d.statusF) {
			d.statusF[idx] = nil
		}
	}
}

// WriteLine 把一行指令写进子进程 stdin。
//
// 契约与 sim/serial 一致：**每条指令恰好回一行**（OK/ERR），异步上报以 `STATE` 出现。
func (d *MujocoDevice) WriteLine(line string) error {
	return d.writeRaw(line)
}

func (d *MujocoDevice) Close() error {
	d.mu.Lock()
	if d.closed {
		d.mu.Unlock()
		return nil
	}
	d.closed = true
	d.mu.Unlock()

	// 先请求优雅退出（server.py 收到 QUIT/EOF 就返回），再关管道；
	// 都不行才 kill。这样 Python 侧有机会把记录文件 flush 完。
	_ = d.writeRaw("QUIT")
	if d.stdin != nil {
		_ = d.stdin.Close()
	}

	select {
	case <-d.done:
	case <-time.After(3 * time.Second):
		if d.cmd != nil && d.cmd.Process != nil {
			_ = d.cmd.Process.Kill()
		}
		<-d.done
	}
	if d.cancel != nil {
		d.cancel()
	}
	d.notifyStatus(false, "mujoco 设备已关闭")
	return nil
}

// ---------------------------------------------------------------------------
// 内部
// ---------------------------------------------------------------------------

func (d *MujocoDevice) writeRaw(line string) error {
	d.mu.Lock()
	closed := d.closed
	exited := d.exited
	reason := d.reason
	d.mu.Unlock()
	if closed {
		return fmt.Errorf("mujoco 设备已关闭")
	}
	if exited {
		if reason == "" {
			reason = "mujoco 子进程已退出"
		}
		return fmt.Errorf("%s", reason)
	}

	d.writeMu.Lock()
	defer d.writeMu.Unlock()
	if _, err := io.WriteString(d.stdin, line+"\n"); err != nil {
		return fmt.Errorf("mujoco: 写 stdin 失败：%w", err)
	}
	return nil
}

func (d *MujocoDevice) readLoop(stdout io.ReadCloser) {
	sc := bufio.NewScanner(stdout)
	sc.Buffer(make([]byte, 0, 64*1024), 1<<20)
	for sc.Scan() {
		text := strings.TrimSpace(sc.Text())
		if text == "" {
			continue
		}
		d.mu.Lock()
		ready := d.ready
		d.mu.Unlock()
		if !ready {
			// 握手窗口：只认 OK PING，其余行不往上层送（否则 controller 会
			// 收到一堆它不认识的启动噪声）。
			if strings.EqualFold(text, "OK PING") {
				d.markReady()
			} else {
				log.Printf("[mujoco] 握手期忽略：%q", text)
			}
			continue
		}
		d.emit(Line{Text: text, At: time.Now()})
	}
	d.onExit(sc.Err())
}

func (d *MujocoDevice) stderrLoop(stderr io.ReadCloser) {
	sc := bufio.NewScanner(stderr)
	sc.Buffer(make([]byte, 0, 64*1024), 1<<20)
	for sc.Scan() {
		text := strings.TrimSpace(sc.Text())
		if text == "" {
			continue
		}
		log.Printf("[mujoco] %s", text)
		// Python 侧最常见的致命错误：装错解释器
		if strings.Contains(text, "No module named 'mujoco'") ||
			strings.Contains(text, "No module named \"mujoco\"") {
			d.setReason("Python 解释器里没有 mujoco 包 —— " +
				"请在 backend/config.yaml 的 device.mujoco.python 指向已安装 mujoco 的解释器")
		}
	}
}

func (d *MujocoDevice) markReady() {
	d.mu.Lock()
	if d.ready {
		d.mu.Unlock()
		return
	}
	d.ready = true
	d.reason = ""
	d.mu.Unlock()
	close(d.readyCh)
	d.notifyStatus(true, "")
}

func (d *MujocoDevice) setReason(text string) {
	d.mu.Lock()
	if d.reason == "" {
		d.reason = text
	}
	d.mu.Unlock()
}

func (d *MujocoDevice) onExit(err error) {
	d.mu.Lock()
	if d.exited {
		d.mu.Unlock()
		return
	}
	d.exited = true
	if d.reason == "" {
		if err != nil && !errors.Is(err, io.EOF) {
			d.reason = fmt.Sprintf("mujoco 子进程已退出：%v", err)
		} else {
			d.reason = "mujoco 子进程已退出"
		}
	}
	reason := d.reason
	wasClosed := d.closed
	d.mu.Unlock()

	close(d.done)
	if !wasClosed {
		log.Printf("[mujoco] %s", reason)
		d.notifyStatus(false, reason)
	}
}

func (d *MujocoDevice) emit(line Line) {
	d.mu.Lock()
	closed := d.closed
	d.mu.Unlock()
	if closed {
		return
	}
	select {
	case d.lines <- line:
	default:
		// 接收端落后时丢最旧的一行，保证设备侧永不阻塞（与 sim.go 同策略）
		select {
		case <-d.lines:
		default:
		}
		select {
		case d.lines <- line:
		default:
		}
	}
}

func (d *MujocoDevice) notifyStatus(connected bool, reason string) {
	d.mu.Lock()
	fns := append([]StatusHandler(nil), d.statusF...)
	d.mu.Unlock()
	for _, fn := range fns {
		if fn != nil {
			fn(connected, reason)
		}
	}
}

// trimFloat 把浮点参数写成紧凑形式（避免 config 里的 30.0 变成 "30.000000"）。
func trimFloat(v float64) string {
	s := fmt.Sprintf("%g", v)
	return s
}

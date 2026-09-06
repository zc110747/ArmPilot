// package serial 提供与 arm-device 固件的串口对接。
//
// 设计目标：
//   - 依赖关系仅为标准库（无外部串口库），Windows 走 syscall 直连 kernel32，
//     Linux/macOS 走 stty 配置 + 文件读写。
//   - 串口未连接 / 意外断开时自动重连（间隔由 config.Serial.ReconnectSec 控制）。
//   - 命令-应答（ACK）门控：每条指令发出后必须等待下位机应答行，才能发下一条，
//     同一时刻只有 1 条指令在途；拖拽过程中的中间摇杆位置只保留最新（不排队连发）；
//     ACK 超时即判定通讯失败（界面显示），且不再自动重发（避免冲刷设备）。
//   - 线程安全：WriteLine 可被 Web/TCP 多 goroutine 并发调用。
package serial

import (
	"bytes"
	"fmt"
	"io"
	"log"
	"strings"
	"sync"
	"time"
)

// Config 与 config.SerialConfig 同构（避免循环依赖，这里直接复用字段）。
type Config struct {
	Port          string
	Baud          int
	DataBits      int
	StopBits      int
	Parity        string
	ReconnectSec  int
	MinIntervalMs int // 两条指令下发的最小间隔(毫秒)
	AckTimeoutMs  int // 等待下位机应答的超时(毫秒)
	ConnectSettleMs int // 连接建立后等待下位机 bootloader 交出的静默窗口(毫秒)
}

// StatusHandler 在连接状态或通讯状态翻转时回调。
//   - connected: 串口是否已真正打开
//   - serialErr: 打开失败原因（connected=false 时有效）
//   - commErr: 通讯是否失败（已连接但应答超时）
//   - commErrMsg: 通讯失败原因
type StatusHandler func(connected bool, serialErr string, commErr bool, commErrMsg string)

// Serial 管理一条串口连接的生命周期。
type Serial struct {
	cfg    Config
	done   chan struct{}
	close  sync.Once
	wg     sync.WaitGroup

	minInterval time.Duration
	ackTimeout  time.Duration
	connectSettle time.Duration
	settleUntil  time.Time // 连接建立后的 bootloader 静默窗口截止时刻(之前不下发指令)
	settleCh     <-chan time.Time // 静默窗口结束信号(一次性)，触发排队指令下发

	// 发送门控（命令-应答）
	joyCh   chan string // 摇杆指令（容量 1，最新值优先 / latest-wins）
	cmdCh   chan string // 离散指令（容量 16，FIFO 逐条下发）
	wakeCh  chan struct{}
	lineCh  chan string // 内部：readLoop -> ackPump（设备回显行）
	onLine  func(string) // 把设备回显广播给 hub（由 main 注入）

	mu          sync.Mutex
	connected   bool
	lastErr     string
	commErr     bool
	commErrMsg  string
	onStatus    StatusHandler
}

// SetLineHandler 注入“设备回显广播”回调（通常为 hub.Broadcast）。
func (s *Serial) SetLineHandler(fn func(string)) { s.onLine = fn }

// SetStatusHandler 注册连接/通讯状态变化回调。
func (s *Serial) SetStatusHandler(fn StatusHandler) { s.onStatus = fn }

// dialSerial 在 manage 中用于打开串口；测试可覆写以注入假连接。
var dialSerial = openPort

// Open 创建 Serial 并启动（重）连接管理。若设备暂未连接，会在后台持续重试。
func Open(cfg Config) *Serial {
	mi := time.Duration(cfg.MinIntervalMs) * time.Millisecond
	// 命令-应答(ACK)门控已能防止高频冲刷设备，故允许 MinIntervalMs=0 表示完全不节流。
	if mi < 0 {
		mi = 0
	}
	at := time.Duration(cfg.AckTimeoutMs) * time.Millisecond
	if at <= 0 {
		at = 800 * time.Millisecond
	}
	cs := time.Duration(cfg.ConnectSettleMs) * time.Millisecond
	if cs < 0 {
		cs = 0
	}
	s := &Serial{
		cfg:           cfg,
		done:          make(chan struct{}),
		joyCh:         make(chan string, 1),
		cmdCh:         make(chan string, 16),
		wakeCh:        make(chan struct{}, 1),
		lineCh:        make(chan string, 256),
		minInterval:   mi,
		ackTimeout:    at,
		connectSettle: cs,
	}
	s.wg.Add(1)
	go s.manage()
	return s
}

// WriteLine 下发一条命令（自动补 \r\n）。
//
// 行为：
//   - 串口未连接：立即返回错误，使上层（Web/TCP）能给出准确的“未连接”提示。
//   - 已连接：把命令放入对应缓冲（摇杆->joyCh 最新值优先；其它->cmdCh FIFO），
//     由 ackPump 在“空闲且已收到上一条应答”时真正写出。永不阻塞、永不饱和。
func (s *Serial) WriteLine(cmd string) error {
	select {
	case <-s.done:
		return fmt.Errorf("串口已关闭")
	default:
	}
	if !s.Connected() {
		return fmt.Errorf("串口未连接，无法下发")
	}
	if strings.HasPrefix(cmd, "JOY ") {
		// latest-wins：覆盖通道里未发送的旧位置
		select {
		case s.joyCh <- cmd:
		default:
			select {
			case <-s.joyCh:
			default:
			}
			s.joyCh <- cmd
		}
	} else {
		select {
		case s.cmdCh <- cmd:
		default:
			// 离散指令队列满：丢弃最早的一条（避免无限堆积）
			select {
			case <-s.cmdCh:
			default:
			}
			s.cmdCh <- cmd
		}
	}
	// 唤醒 ackPump
	select {
	case s.wakeCh <- struct{}{}:
	default:
	}
	return nil
}

// Connected 返回当前串口是否已真正打开。
func (s *Serial) Connected() bool {
	s.mu.Lock()
	defer s.mu.Unlock()
	return s.connected
}

// Status 返回 (是否已连接, 最近打开错误, 是否通讯失败, 通讯失败原因)。
func (s *Serial) Status() (bool, string, bool, string) {
	s.mu.Lock()
	defer s.mu.Unlock()
	return s.connected, s.lastErr, s.commErr, s.commErrMsg
}

// notify 在状态翻转时调用回调（连接 / 通讯失败各自独立上报）。
func (s *Serial) notify() {
	if s.onStatus != nil {
		s.mu.Lock()
		c, se, ce, cem := s.connected, s.lastErr, s.commErr, s.commErrMsg
		s.mu.Unlock()
		s.onStatus(c, se, ce, cem)
	}
}

func (s *Serial) setConnected(connected bool, errMsg string) {
	s.mu.Lock()
	changed := s.connected != connected || s.lastErr != errMsg
	s.connected = connected
	s.lastErr = errMsg
	if connected {
		// 重新连接后，通讯状态复位为“正常”，等待首条指令验证
		s.commErr = false
		s.commErrMsg = ""
	}
	s.mu.Unlock()
	if changed {
		s.notify()
	}
}

func (s *Serial) setCommOK() {
	s.mu.Lock()
	changed := s.commErr
	s.commErr = false
	s.commErrMsg = ""
	s.mu.Unlock()
	if changed {
		s.notify()
	}
}

func (s *Serial) setCommErr(msg string) {
	s.mu.Lock()
	changed := !s.commErr || s.commErrMsg != msg
	s.commErr = true
	s.commErrMsg = msg
	s.mu.Unlock()
	if changed {
		s.notify()
	}
}

// Close 停止所有 goroutine 并关闭串口。
func (s *Serial) Close() {
	s.close.Do(func() {
		close(s.done)
		s.wg.Wait()
	})
}

// manage 是连接主管：循环尝试打开串口，成功后启动读写+ACK 门控，断开则重连。
func (s *Serial) manage() {
	defer s.wg.Done()
	reconnect := time.Duration(s.cfg.ReconnectSec) * time.Second
	if reconnect <= 0 {
		reconnect = 3 * time.Second
	}
	log.Printf("[serial] 下发最小间隔: %v（防止高频冲刷设备）", s.minInterval)
	log.Printf("[serial] 应答超时: %v（超时即判定通讯失败）", s.ackTimeout)
	for {
		select {
		case <-s.done:
			return
		default:
		}
		conn, err := dialSerial(s.cfg)
		if err != nil {
			s.setConnected(false, err.Error())
			log.Printf("[serial] 打开 %s 失败: %v（%s 后重试）", s.cfg.Port, err, reconnect)
			select {
			case <-s.done:
				return
			case <-time.After(reconnect):
				continue
			}
		}
		log.Printf("[serial] 已连接 %s @ %d 8N1", s.cfg.Port, s.cfg.Baud)
		s.setConnected(true, "")
		s.run(conn)
		conn.Close()
		s.setConnected(false, "连接已断开，重连中…")
		select {
		case <-s.done:
			return
		case <-time.After(reconnect):
		}
	}
}

// run 在单条连接存活期间驱动读循环与 ACK 门控循环，直到任一方出错或主动关闭。
func (s *Serial) run(conn io.ReadWriteCloser) {
	// 连接建立后进入 bootloader 静默窗口：Arduino Uno 在打开串口时会因 DTR 边沿
	// 自动复位进入 optiboot，bootloader 等待一段上传窗口(~2.5s)后才把控制权交给
	// 固件；窗口内下发的指令会被丢弃导致“首条指令无应答”。此处把连接建立时刻记为
	// 静默窗口起点，trySend 在窗口内不下发、仅缓存于 joyCh/cmdCh，窗口过后再实际写出，
	// 从而既不让首条指令丢失，也不因 ACK 超时误判通讯失败。
	if s.connectSettle > 0 {
		s.settleUntil = time.Now().Add(s.connectSettle)
		s.settleCh = time.After(s.connectSettle)
		log.Printf("[serial] 连接后静默窗口: %v（等待下位机 bootloader 交出控制权）", s.connectSettle)
	} else {
		s.settleUntil = time.Time{}
		s.settleCh = nil
	}
	var wg sync.WaitGroup
	broken := make(chan error, 2)
	wg.Add(2)
	go s.readLoop(conn, broken, &wg)
	go s.ackPump(conn, broken, &wg)

	select {
	case <-s.done:
		// 主动关闭
	case err := <-broken:
		if err != nil {
			log.Printf("[serial] 连接中断: %v", err)
		}
	}

	conn.Close() // 断开会唤醒阻塞中的读写 goroutine
	wg.Wait()
}

// readLoop 按行读取设备回显，推入内部 lineCh（供 ackPump 做 ACK 判定与广播）。
//
// 注意：这里**不能**用 bufio.Reader —— Windows 串口读超时会返回 (0, nil)，
// bufio 连续 100 次空读会抛 io.ErrNoProgress，导致链路空闲约 20s（100×200ms）
// 必然假性断连，而重连会拉 DTR 使 Uno 复位（舵机全部回 90°）。
// 因此手动做行缓冲：n==0 且 err==nil 仅表示"暂无数据"，继续循环。
func (s *Serial) readLoop(conn io.Reader, broken chan<- error, wg *sync.WaitGroup) {
	defer wg.Done()
	buf := make([]byte, 256)
	var acc []byte
	for {
		n, err := conn.Read(buf)
		if n > 0 {
			acc = append(acc, buf[:n]...)
			for {
				idx := bytes.IndexByte(acc, '\n')
				if idx < 0 {
					break
				}
				line := strings.TrimRight(string(acc[:idx]), "\r")
				acc = acc[idx+1:]
				if line != "" {
					select {
					case s.lineCh <- line:
					default:
						log.Printf("[serial] lineCh 满，丢弃回显: %q", line)
					}
				}
			}
			// 防御：异常数据流长期无换行时丢弃，避免缓冲无限增长
			if len(acc) > 4096 {
				acc = acc[:0]
			}
		}
		if err != nil {
			if err != io.EOF {
				// 读超时（Windows 200ms 常量超时 / Unix VMIN=0 超时）表示“暂无数据”，
				// 视为非致命，继续等待下一批；仅真正的 I/O 错误才断连重连。
				if ne, ok := err.(interface{ Timeout() bool }); ok && ne.Timeout() {
					continue
				}
				broken <- err
			}
			return
		}
		// n==0 && err==nil：读超时且无数据，非致命，继续
	}
}

// ackPump 是命令-应答门控核心：
//   - 空闲时从 joyCh/cmdCh 取一条指令写出；
//   - 写出后置“等待应答”，启动 ackTimeout 计时；
//   - 收到设备回显行 => 视为应答，清除等待并继续发下一条（若有）；
//   - 计时超时 => 通讯失败（不再重发，清空待发，等用户下一条新指令）。
func (s *Serial) ackPump(conn io.Writer, broken chan<- error, wg *sync.WaitGroup) {
	defer wg.Done()
	awaiting := false
	var ackTimer *time.Timer
	var lastWrite time.Time

	trySend := func() {
		if awaiting {
			return
		}
		// 处于连接后 bootloader 静默窗口：暂不实际下发，仅保留队列里的指令
		// （最新摇杆位置 / FIFO 离散指令），窗口过后再写。避免首条指令被丢弃。
		if !s.settleUntil.IsZero() && time.Now().Before(s.settleUntil) {
			return
		}
		cmd, ok := s.takePending()
		if !ok {
			return
		}
		if err := s.writeOne(conn, cmd, &lastWrite); err != nil {
			broken <- err
			return
		}
		awaiting = true
		if ackTimer == nil {
			ackTimer = time.NewTimer(s.ackTimeout)
		} else {
			ackTimer.Reset(s.ackTimeout)
		}
	}

	for {
		select {
		case <-s.done:
			if ackTimer != nil {
				ackTimer.Stop()
			}
			return
		case line := <-s.lineCh:
			if s.onLine != nil {
				s.onLine(line)
			}
			// 异步事件行（以 "# " 开头，如硬件 IR 回显 / 序列自动停止）不是命令
			// 应答，不能用来清除“等待应答”状态，否则会把硬件遥控按键误判为
			// 串口指令的应答。仅以 OK/ERR 等非 "# " 行作为应答。
			if strings.HasPrefix(line, "# ") {
				continue
			}
			if awaiting {
				awaiting = false
				s.setCommOK()
				if ackTimer != nil {
					ackTimer.Stop()
				}
			}
			// 收到应答（或任何回显）后，尝试发送下一条待发指令
			trySend()
		case <-s.wakeCh:
			// WriteLine 已把命令放入 joyCh/cmdCh；此处仅作为“有 pending”的唤醒信号，
			// 实际取出由 takePending 完成（避免 select 分支误把命令从通道取走）。
			trySend()
		case <-s.settleCh:
			// 静默窗口结束：清除窗口限制并下发排队指令，避免首条指令在窗口内被丢弃。
			s.settleUntil = time.Time{}
			s.settleCh = nil
			// 暖机：Uno 在开机/自动复位后，下位机串口链路的首个数据包常被丢弃
			// （无应答、无报错，第二条才正常）。这里先发一个无害的换行作为“第 1 个包”
			// 被链路吞掉，使真正指令成为第 2 个包而可靠送达，避免首条 web 指令静默丢失。
			if _, werr := io.WriteString(conn, "\n"); werr != nil {
				broken <- werr
				continue
			}
			trySend()
		case <-ackTimerChan(ackTimer):
			// 应答超时：通讯失败，不重发，清空待发，等待新指令
			awaiting = false
			s.setCommErr(fmt.Sprintf("下位机应答超时(>%v)，通讯失败", s.ackTimeout))
			drainJoy(s.joyCh)
			drainCmd(s.cmdCh)
		}
	}
}

// ackTimerChan 返回计时器通道（nil 时返回 nil 通道，select 永不命中）。
func ackTimerChan(t *time.Timer) <-chan time.Time {
	if t == nil {
		return nil
	}
	return t.C
}

// takePending 取出下一条待发指令：优先离散 FIFO，其次摇杆最新位置。
func (s *Serial) takePending() (string, bool) {
	select {
	case c := <-s.cmdCh:
		return c, true
	default:
	}
	select {
	case c := <-s.joyCh:
		return c, true
	default:
	}
	return "", false
}

func drainJoy(ch chan string) {
	for {
		select {
		case <-ch:
		default:
			return
		}
	}
}

func drainCmd(ch chan string) {
	for {
		select {
		case <-ch:
		default:
			return
		}
	}
}

// writeOne 实际写出一条指令，并保持最小下发间隔；debug 级别打印 TX 字节。
func (s *Serial) writeOne(conn io.Writer, cmd string, lastWrite *time.Time) error {
	if d := time.Since(*lastWrite); d < s.minInterval {
		time.Sleep(s.minInterval - d)
	}
	payload := cmd + "\r\n"
	if _, err := io.WriteString(conn, payload); err != nil {
		return err
	}
	*lastWrite = time.Now()
	if debug {
		log.Printf("[serial] TX %q", cmd)
	}
	return nil
}

// debug 控制 TX 等详细日志的打印（由 main 根据 log_level=debug 开启）。
var debug bool

// SetDebug 设置详细日志开关（true 时打印每条 TX 指令，便于联机验证）。
func SetDebug(on bool) { debug = on }

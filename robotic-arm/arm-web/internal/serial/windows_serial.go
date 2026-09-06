//go:build windows

package serial

import (
	"fmt"
	"io"
	"strings"
	"syscall"
	"unsafe"
)

var (
	k32               = syscall.NewLazyDLL("kernel32.dll")
	procCreateFileW   = k32.NewProc("CreateFileW")
	procCloseHandle   = k32.NewProc("CloseHandle")
	procSetCommState  = k32.NewProc("SetCommState")
	procSetCommTimeouts = k32.NewProc("SetCommTimeouts")
	procReadFile      = k32.NewProc("ReadFile")
	procWriteFile     = k32.NewProc("WriteFile")
)

const (
	genericRead      = 0x80000000
	genericWrite     = 0x40000000
	openExisting     = 3
	fileShareRead    = 0x00000001
	fileShareWrite   = 0x00000002
	invalidHandle    = ^uintptr(0) // 0xFFFFFFFFFFFFFFFF

	// winERROR_TIMEOUT = Windows ERROR_TIMEOUT (1460)：读超时（无数据），非致命。
	winERROR_TIMEOUT = 1460
)

// errTimeout 表示串口读超时（Windows ERROR_TIMEOUT）。readLoop 视其为“暂无数据”，
// 继续等待，而非断开连接——否则空闲 200ms 就会把连接反复撕裂重连。
var errTimeout = &serialTimeoutError{}

type serialTimeoutError struct{}

func (e *serialTimeoutError) Error() string   { return "serial read timeout" }
func (e *serialTimeoutError) Timeout() bool   { return true }
func (e *serialTimeoutError) Temporary() bool { return true }

// dcb 是 Windows 通信设备控制块（与 winbase.h 布局一致）。
type dcb struct {
	dcbLength  uint32
	baudRate   uint32
	flags      uint32 // bit0=fBinary(必须1)
	wReserved  uint16
	xonLim     uint16
	xoffLim    uint16
	byteSize   byte
	parity     byte
	stopBits   byte
	xonChar    byte
	xoffChar   byte
	errorChar  byte
	eofChar    byte
	evtChar    byte
	wReserved1 uint16
}

// commTimeouts 控制读写超时；此处配置为“有数据立即返回，无数据阻塞等待首字节”。
type commTimeouts struct {
	readIntervalTimeout         uint32
	readTotalTimeoutMultiplier uint32
	readTotalTimeoutConstant   uint32
	writeTotalTimeoutMultiplier uint32
	writeTotalTimeoutConstant   uint32
}

type winSerial struct {
	handle uintptr
}

func openPort(cfg Config) (io.ReadWriteCloser, error) {
	name, err := syscall.UTF16PtrFromString("\\\\.\\" + cfg.Port)
	if err != nil {
		return nil, err
	}
	r, _, e := procCreateFileW.Call(
		uintptr(unsafe.Pointer(name)),
		genericRead|genericWrite,
		fileShareRead|fileShareWrite, // 允许被其它程序共享打开，规避“端口被占用”连不上
		0,
		openExisting,
		0,
		0,
	)
	if r == invalidHandle {
		return nil, fmt.Errorf("CreateFileW %s 失败: %v（端口可能被其它程序占用或名称错误，请确认 COM 号）", cfg.Port, e)
	}

	s := &winSerial{handle: r}

	parity := byte(0) // N
	switch strings.ToUpper(cfg.Parity) {
	case "E":
		parity = 2
	case "O":
		parity = 1
	}
	stop := byte(0) // 1 停止位
	if cfg.StopBits == 2 {
		stop = 2
	}

	// DataBits 必须落在 5..8；调用方未显式给出（默认 0）时回退到标准 8 位，
	// 否则 SetCommState 会因 byteSize=0 返回“参数不正确”而打开失败。
	byteSize := byte(cfg.DataBits)
	if byteSize < 5 || byteSize > 8 {
		byteSize = 8
	}

	var d dcb
	d.dcbLength = uint32(unsafe.Sizeof(d))
	d.baudRate = uint32(cfg.Baud)
	// flags: fBinary=1(必需) | fDtrControl=1(DTR 使能) | fRtsControl=1(RTS 使能)
	// 标准 3 线串口均以此配置工作；断言 DTR/RTS 与多数串口助手默认一致。
	d.flags = 1 | (1 << 4) | (1 << 12)
	d.byteSize = byteSize
	d.parity = parity
	d.stopBits = stop

	if r, _, e := procSetCommState.Call(s.handle, uintptr(unsafe.Pointer(&d))); r == 0 {
		s.Close()
		return nil, fmt.Errorf("SetCommState 失败: %v", e)
	}

	var t commTimeouts
	// 读：ReadFile 阻塞直到有字节到达或超时（读总超时 200ms），符合 bufio.Reader
	// 的阻塞读取契约；避免全部为 0 时 ReadFile 立即返回 0 字节导致读循环空转、
	// 在高速回显下漏读。写：WriteTotalTimeoutConstant=500 => 单次写最多 500ms，
	// 设备无响应时快速失败并触发重连。
	t.readIntervalTimeout = 0
	t.readTotalTimeoutMultiplier = 0
	t.readTotalTimeoutConstant = 200
	t.writeTotalTimeoutConstant = 500
	if r, _, e := procSetCommTimeouts.Call(s.handle, uintptr(unsafe.Pointer(&t))); r == 0 {
		s.Close()
		return nil, fmt.Errorf("SetCommTimeouts 失败: %v", e)
	}

	return s, nil
}

func (s *winSerial) Read(p []byte) (int, error) {
	if len(p) == 0 {
		return 0, nil
	}
	var n uint32
	r, _, e := procReadFile.Call(s.handle, uintptr(unsafe.Pointer(&p[0])), uintptr(len(p)), uintptr(unsafe.Pointer(&n)), 0)
	if r == 0 {
		if errno, ok := e.(syscall.Errno); ok && uint32(errno) == winERROR_TIMEOUT {
			// 读超时（无数据到达）：返回专用超时错误，readLoop 据此继续等待而非断连。
			return 0, errTimeout
		}
		if e == nil {
			return 0, fmt.Errorf("ReadFile 返回 0 字节（未知错误）")
		}
		return 0, e
	}
	return int(n), nil
}

func (s *winSerial) Write(p []byte) (int, error) {
	var n uint32
	r, _, e := procWriteFile.Call(s.handle, uintptr(unsafe.Pointer(&p[0])), uintptr(len(p)), uintptr(unsafe.Pointer(&n)), 0)
	if r == 0 {
		return 0, e
	}
	return int(n), nil
}

func (s *winSerial) Close() error {
	if s.handle == invalidHandle || s.handle == 0 {
		return nil
	}
	procCloseHandle.Call(s.handle)
	s.handle = invalidHandle
	return nil
}

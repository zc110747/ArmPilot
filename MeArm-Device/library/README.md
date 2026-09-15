# library/ — 芯片官方库

ATmega328P 的"官方库"即其器件头文件（`<avr/io.h>`、中断向量 `<avr/interrupt.h>`、
`<avr/eeprom.h>` 等）与启动代码（crtatmega328p.o）、C 运行库（libc/libm/libgcc）。
这些由 **avr-libc + 工具链（toolchain-atmelavr）** 直接提供，随 PlatformIO 安装到

```
%USERPROFILE%\.platformio\packages\toolchain-atmelavr\   <- 扁平布局（当前本机）
%USERPROFILE%\.platformio\packages\toolchain-atmelavr@<版本>\<版本>\   <- 嵌套布局
```

其下 `avr\include` 与 `avr\lib` 即头文件与库。**无需在此 vendoring**（避免与工具链版本错位）。

> 工具链实际路径由 `_env.bat` 统一解析（顺序：`PLATFORMIO_CORE_DIR` →
> `%USERPROFILE%\.platformio\packages` → `D:\tools\agent-tools\platformio-core\packages`），
> 并同时兼容上述扁平/嵌套两种包目录布局。构建脚本不再硬编码盘符。

若后续引入厂商提供的独立驱动库（例如某传感器/专用外设的官方 C 库），请放置于本目录，
并在 `build.bat` 的 `CFLAGS` 中追加 `-Ilibrary/<子目录>`。

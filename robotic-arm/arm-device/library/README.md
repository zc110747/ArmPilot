# library/ — 芯片官方库

ATmega328P 的"官方库"即其器件头文件（`<avr/io.h>`、中断向量 `<avr/interrupt.h>`、
`<avr/eeprom.h>` 等）与启动代码（crtatmega328p.o）、C 运行库（libc/libm/libgcc）。
这些由 **avr-libc + 工具链（toolchain-atmelavr）** 直接提供，随 PlatformIO 安装到
`D:\tools\agent-tools\platformio-core\packages\toolchain-atmelavr\avr\include` 与
`...\avr\lib`，无需在此 vendoring（避免与工具链版本错位）。

若后续引入厂商提供的独立驱动库（例如某传感器/专用外设的官方 C 库），请放置于本目录，
并在 `scripts/build.bat` 的 `CFLAGS` 中追加 `-Ilibrary/<子目录>`。

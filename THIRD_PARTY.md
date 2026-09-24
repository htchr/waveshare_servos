# Third-party code

This package vendors one third-party component: Feetech's **SCServo Linux library, release
220329**, in the copy Waveshare distributes for its Bus Servo Adapter, with two small additions
taken from [adityakamath/SCServo_Linux](https://github.com/adityakamath/SCServo_Linux) and one local
bug fix. It is the packet layer every bus transaction goes through. The six sources below are
compiled into the static `scservo` target, and through them seven of the eight headers. The
umbrella `SCServo.h` is included by none of them; in this package only the uninstalled bench
helper `test/hil/stop_wheels.cpp` includes it. The plugin and the four tools link its SMS_STS,
SCS and SCSerial code; SMSBL, SMSCL and SCSCL are compiled but never linked. All eight headers
are installed, because the plugin's public header includes `servo_bus.hpp`, which includes
`SMS_STS.h` and through it `SCSerial.h`, `SCS.h` and `INST.h`. The other four headers ship unused.

**These files are not edited.** See [The rule](#the-rule-these-files-are-not-edited).

## The files

The header comments are the upstream ones, in Chinese. `日期` means "date" and `作者` means
"author". The author field is empty in every file.

| file | header description (original) | English | header date |
|---|---|---|---|
| `include/INST.h` | 串行舵机协议指令定义 | serial servo protocol instruction definitions | 2021.12.8 |
| `include/SCS.h` | 串行舵机通信层协议程序 | serial servo communication-layer protocol | 2022.3.29 |
| `include/SCSCL.h` | SCSCL系列串行舵机应用层程序 | SCSCL-series serial servo application layer | 2020.6.17 |
| `include/SCSerial.h` | 串行舵机硬件接口层程序 | serial servo hardware-interface layer | 2022.3.29 |
| `include/SCServo.h` | 串行舵机接口 | serial servo interface (umbrella header) | 2021.12.8 |
| `include/SMSBL.h` | SMSBL系列串行舵机应用层程序 | SMSBL-series serial servo application layer | 2020.6.17 |
| `include/SMSCL.h` | `SMSCLϵ�д��ж���ӿ�` (see note 1) | Feetech SMSCL-series serial servo interface | 2020.6.17 |
| `include/SMS_STS.h` | SMS/STS系列串行舵机应用层程序 | SMS/STS-series serial servo application layer | 2021.12.8 |
| `src/SCS.cpp` | 飞特串行舵机通信层协议程序 | Feetech serial servo communication-layer protocol | 2022.3.29 |
| `src/SCSCL.cpp` | SCSCL系列串行舵机应用层程序 | SCSCL-series serial servo application layer | 2020.6.17 |
| `src/SCSerial.cpp` | 串行舵机硬件接口层程序 (see note 2) | serial servo hardware-interface layer | 2022.3.29 |
| `src/SMSBL.cpp` | SMSBL系列串行舵机应用层程序 | SMSBL-series serial servo application layer | 2020.6.17 |
| `src/SMSCL.cpp` | `SMSCLϵ�д��ж���ӿ�` (see note 1) | Feetech SMSCL-series serial servo interface | 2020.6.17 |
| `src/SMS_STS.cpp` | SMS/STS系列串行舵机应用层程序 | SMS/STS-series serial servo application layer | 2021.12.8 |

1. `SMSCL.h` and `SMSCL.cpp` were GBK-encoded upstream. The Waveshare copy was converted to UTF-8
   in a way that lost the encoding: most Chinese characters became U+FFFD, the Unicode replacement
   character (150 in the header, 15 in the source). The comments are unreadable, but the code is
   identical to Feetech's. The original line, from Feetech's own copy, is 飞特SMSCL系列串行舵机接口.
2. `src/SCSerial.cpp` names itself `SCSerial.h` in its header. That mistake comes from upstream.

`飞特` is Feetech's Chinese name. Feetech's own copies begin every description with it. Waveshare's
copy dropped it from every file except `src/SCS.cpp`.

`include/visibility_controls.h` is not part of this component. See
[Other files of third-party origin](#other-files-of-third-party-origin).

### Checksums

The files as they are in this repository. `test/test_vendored_files.py` fails `colcon test` if
any file differs from its row, if a row is missing or extra, or if a new file whose name starts
with an upper-case letter (the vendored naming; the package's own files are lower-case and are
not checked) shows up anywhere under `include/` or `src/` without being listed here. To check
by hand, run
`sed -n '/vendored-sha256:begin/,/vendored-sha256:end/p' THIRD_PARTY.md | grep -E '^[0-9a-f]{64}  ' | sha256sum -c`
from the package root.

<!-- vendored-sha256:begin -->
```text
6a242fa1772395a6ccaef52270f121db726aa870e066faa2135dd341abb6ea24  include/INST.h
491fa9fe0a77d7a36317d84ecb019e1010e5f87879049282287818fbba67aedc  include/SCS.h
252b4bc7d1d2b289df9b2799a1d0543ac4f0ab2ad2606d8b06279e0f4fd5bf38  include/SCSCL.h
82ee5a5b7971e889f28519336ef1e52f445cf438d98b299fcea4f49015157275  include/SCSerial.h
3de725830586b2de495659e426e3fecadde86bea5c4ba0c84641c92ffcaf57cb  include/SCServo.h
5a2b93d5cd5ff7b85f8bbd08c93c72d5a8c89882bc7e18513bbfc9cfb46bf45b  include/SMSBL.h
579915daea31e58b51ceee8108364afab7db4c881287a8ef062f361f263c5b04  include/SMSCL.h
f962fae60a293c4407b7721685c04b91c2c63c5df679a3ae4f233c3767c53d1b  include/SMS_STS.h
199677cad2b9fda5534653d40d39d1eff9ae54591bd4c2d417ec9dbdb2b701ab  src/SCS.cpp
7b0424fd90ebc5ed570564dfc0cd4abc2a40fd32cd8a7fcadc4272cc37d64e77  src/SCSCL.cpp
37b6ea5747d194a0888b8b688af9c88ada89a62a9cc896581d466eba70bfcae5  src/SCSerial.cpp
924175e4b83b42346fd033a2eb716fd9716b7c382ac3ce2fd8f1fb8aff0feb6f  src/SMSBL.cpp
accdc28251683aa74e5b3e37deb5e07dbb5bb12c3ba7e5faf4974f4106ca479a  src/SMSCL.cpp
c215295e7a19c49b1104143cd1db1452b7a9365235f92dc64b2a9c1b731f83e5  src/SMS_STS.cpp
```
<!-- vendored-sha256:end -->

## Where they come from

### The base: Feetech SCServo_Linux 220329, as distributed by Waveshare

- **Archive:** `SCServo_Linux.rar`, linked as "ST/SC serial bus servo control library (Linux)"
  from the Resources section of Waveshare's
  [Bus Servo Adapter (A) wiki page](https://www.waveshare.com/wiki/Bus_Servo_Adapter_(A)):
  <https://files.waveshare.com/wiki/Bus-Servo-Adapter-(A)/SCServo_Linux.rar>
- **Details as fetched on 2026-09-23:** 35580 bytes; sha256
  `bb364596666097c1e5260ca80e9f22b44d6a5b2fbde2bf2247a633e541424343`; HTTP `Last-Modified: Mon,
  18 Sep 2023 07:02:20 GMT`. The archive holds a single folder, `SCServo_Linux_220329/SCServo_Linux/`.
  That folder has the 14 library files, a `CMakeLists.txt` that builds `libSCServo.a`, two Chinese
  notes (`说明.txt` is an overview, `编译说明.txt` is build instructions) and `examples/`. It holds
  no license file and no copyright notice.
- **Match:** after two byte-level normalizations (strip a leading UTF-8 BOM, then convert CRLF to
  LF), 7 of the 14 files as imported are byte-identical to the archive: `INST.h`, `SCSCL.h`,
  `SCSerial.h`, `SCServo.h`, `SMSBL.h`, `SMSCL.h` and `SCSerial.cpp`. The other 7 differ only by
  the changes listed under [Import](#import-into-this-repository). `SCSerial.cpp` changed after
  the import (see [the local fix](#the-one-local-fix-commit-d846222-srcscserialcpp)).
- **Feetech's own copy:** Feetech later published the same release on GitHub as
  [ftservo/FTServo_Linux](https://github.com/ftservo/FTServo_Linux) under the MIT License
  (`Copyright (c) 2024 FTServo`). Commit `5a9ffe3` (2025-01-04) is the first with sources. Its
  code is identical to Waveshare's copy. The only differences are in comments. In Feetech's copy,
  all 14 description lines start with `飞特` ("Feetech"), where Waveshare's copy keeps it only in
  `src/SCS.cpp`. The `SMSCL` files are also still in GBK there. Feetech's next release, dated
  2025.9.27 (commit `606022b`), is a substantial revision: `src/SCS.cpp` alone differs in 180
  lines. It drops SMSBL and SMSCL and adds HLSCL.

### From adityakamath/SCServo_Linux

[adityakamath/SCServo_Linux](https://github.com/adityakamath/SCServo_Linux) is an independent
repository that started from the same Feetech 220329 release. Two things come from it:

1. **The `snycWrite` -> `syncWrite` rename.** Upstream misspells `SCS::syncWrite` as `snycWrite`.
   The fork corrected it in commit `eb9f5de` (2023-06-10). The same six upstream occurrences are
   renamed here: `include/SCS.h:21`, `src/SCS.cpp:124`, `src/SCSCL.cpp:64`, `src/SMSBL.cpp:79`,
   `src/SMSCL.cpp:78` and `src/SMS_STS.cpp:79`. The fork's `SyncWriteSpe` (item 2) adds one more
   call, at `src/SMS_STS.cpp:279`. The rename matters: `ServoBus` calls `syncWrite`, so a pristine
   upstream copy would not build.
2. **`SMS_STS::SyncWriteSpe()` and `SMS_STS::Mode()`.** Their declarations are at
   `include/SMS_STS.h:88-90` and their definitions at `src/SMS_STS.cpp:260-292`. Each block starts
   with a comment giving the fork's URL. The bodies and declarations are byte-identical, after the
   CRLF normalization, to the fork from commit `d3ad38c` (2023-07-09) up to `c796016` (2025-11-30,
   exclusive). That span includes `95b9e20` (2024-05-05), the fork's last commit before this
   package imported the files. The driver still calls `Mode()`. It no longer calls `SyncWriteSpe()`
   (see [Known defects](#known-defects-and-where-the-package-works-around-them)).

When the files were imported, the fork carried no license. Since commit `a5ade65` (2025-12-04)
it has been MIT-licensed. Its current `LICENSE` (since `333dfe4`, 2026-06-03, unchanged at
`4a84794`) has the copyright lines `Copyright (c) 2024 FTServo` and
`Copyright (c) 2025 Aditya Kamath (Kamath Robotics)`.

### Import into this repository

- Commit `acff137` (2024-08-06) added the files under `hardware/`. Commit `b554cf5` (2025-01-14)
  moved them to `include/` and `src/` with no content change (git reports 100% similarity).
- Changes made at import, against the Waveshare archive:
  - CRLF line endings converted to LF, in all 14 files;
  - the UTF-8 BOM removed, in the 9 files that had one: `SCS.h`, `SCSCL.h`, `SCSerial.h`,
    `SMSBL.h`, `SMS_STS.h`, `SCS.cpp`, `SCSCL.cpp`, `SMSBL.cpp`, `SMS_STS.cpp`;
  - a final newline added to `include/SMS_STS.h`;
  - the two changes from adityakamath above.

  Nothing else changed.

### The one local fix: commit `d846222`, `src/SCSerial.cpp`

Commit `d846222` ("smooth motion", 2026-09-10) is the only change to any of these files since the
import. It is also the only difference in them between the `humble` branch and `jazzy`. It fixes
three functions:

- **`readSCS()`:** re-arms the `select()` descriptor set on every loop iteration. It reads only
  when `select()` returns more than 0, retries on `EINTR`/`EAGAIN`/`EWOULDBLOCK`, and never adds
  `read()`'s -1 to the byte count. Before the fix, a `read()` error moved the write position below
  the caller's buffer. The fix adds `#include <errno.h>`.
- **`wFlushSCS()`:** keeps calling `write()` until the whole frame is sent, and gives up after
  1000 retries in a row (`EAGAIN`/`EWOULDBLOCK`/`EINTR`; any progress resets the count). The port
  is non-blocking, and the original code dropped whatever a short `write()` did not accept, so
  the frame was cut short. The fix's comment says the servo then drops the frame on its
  checksum, so the command is lost; this was not measured.
- **`end()`:** closes the descriptor before setting it to -1. The original code did the reverse
  (`fd = -1; close(fd);`), so it closed -1 and leaked the port.

Neither upstream has all three fixes. Feetech's `FTServo_Linux` at `06fd335` (2026-08-28) still has
the original `end()`, `wFlushSCS()` and `readSCS()`. The fork later fixed `end()` on its own
(`c796016`, 2025-11-30), and of the `readSCS()` fix only the re-arming of the descriptor set. At
`4a84794` its `readSCS()` still calls `read()` when `select()` fails and adds `read()`'s -1 to
the byte count, and its `wFlushSCS()` still makes a single `write()`.

## The rule: these files are not edited

- The files are read-only. The baseline is the checksum table above. Anything the driver needs
  beyond the library goes in the driver-owned wrapper, `ServoBus`
  (`include/servo_bus.hpp`, `src/servo_bus.cpp`). If a library function cannot be worked around,
  the fix is a driver-side copy of that function under this package's license, not an edit here.
- Warnings and lint are handled in `CMakeLists.txt`, never in the files:
  - the `scservo` target compiles the six `.cpp` files with `-Wno-vla`, for the five
    variable-length arrays at `src/SMS_STS.cpp:56,263`, `src/SMSBL.cpp:56`, `src/SCSCL.cpp:45` and
    `src/SMSCL.cpp:55`;
  - all 14 files are listed in `AMENT_LINT_AUTO_FILE_EXCLUDE`, and the package's own cpplint call
    passes the same list to `--exclude`.
- Code and tests across the package cite these files by line number, for example
  `src/SCS.cpp:355`. Any edit here would silently invalidate those citations. This is the second
  reason for the rule. The README and this file name the package's own code by function, class
  or log text. This file cites the vendored files by line (the README does not), because their
  lines move only on a refresh from upstream, and
  [Finding the vendored line citations](#finding-the-vendored-line-citations) gives the
  `git grep` that finds every such citation.
- `test/test_vendored_files.py` enforces the rule. No motors are needed.

## Known defects and where the package works around them

| library fact | where the package handles it |
|---|---|
| `SCSerial::begin()` opens the tty without exclusivity (`src/SCSerial.cpp:41`) | `ServoBus::open()` (`src/servo_bus.cpp`): `TIOCEXCL` plus `flock(LOCK_EX\|LOCK_NB)` |
| `begin()` maps seven baud rates and silently falls back to 115200 (`src/SCSerial.cpp:50-75`). `setBaudRate()` never calls `tcsetattr` and reads an uninitialised `speed_t` on an unmapped rate (`src/SCSerial.cpp:104,127-131`) | `ServoBus::is_supported_baudrate()` (`src/servo_bus.cpp`) is checked by `on_init` (`src/waveshare_servos.cpp`) and by the tools' parameter parsing (`src/tool_params.cpp`). `setBaudRate` is hidden by a private using-declaration in `ServoBus` (`include/servo_bus.hpp`), which has its own `set_baudrate()` |
| `begin()` prints to stdout with `printf` (`src/SCSerial.cpp:79`), and `perror()` clobbers errno | `ServoBus::open()` calls `fflush(stdout)` after `begin()`, and takes its own descriptor first, so it can report the real errno (the header comment of `include/servo_bus.hpp`) |
| `writeSCS()` appends to `txBuf[255]` without a bounds check (`src/SCSerial.cpp:179-191`). The `syncWrite()` length field is a `u8` | the wrapper splits packets into chunks that fit, with `static_assert`s on the chunk sizes and on `sizeof(txBuf)` in `src/servo_bus.cpp`; also the runtime probe `the_vendored_transmit_buffer_is_255_bytes` in `test/test_servo_bus.cpp` |
| `SyncWriteSpe()` does a blocking `genWrite` plus `Ack` of ACC for every servo (`src/SMS_STS.cpp:261-280`) | no longer called. ACC is written through `ServoBus::write_acc` (`src/servo_bus.cpp`) only at configure, at activation, and when a servo answers again or a fault clears, not every cycle. Speeds go out through `ServoBus::write_goal_speeds` as a plain `syncWrite` |
| `SyncWritePosEx()` overwrites the caller's `Position[]` and uses a VLA (`src/SMS_STS.cpp:53-61`) | no longer called by the driver. `ServoBus::write_goal_positions` (`src/servo_bus.cpp`) builds the 7-byte records itself, and tests in `test/test_servo_bus.cpp` prove its packets byte-identical to `SyncWritePosEx`'s |
| the constructors leave `Err` and the `syncRead*` members uninitialised | `ServoBus::ServoBus()` (`src/servo_bus.cpp`) zeroes them |
| `syncReadBegin()`/`syncReadEnd()` pair `new[]` with `delete` (`src/SCS.cpp:325,331`) | never called. The wrapper owns the receive buffer, which `ServoBus::ServoBus()` allocates once |
| `syncReadPacketRx` can read up to 18 bytes past the buffer (`src/SCS.cpp:355`) | the wrapper parses frames itself (`parse_sync_read_burst`, `src/servo_bus.cpp`), with a size check before every frame. The buffer is also padded by 18 bytes (`ServoBus::sync_read_slack_bytes`, `include/servo_bus.hpp`) |
| `Read()` checks neither the responder id nor the length byte, and does not resync (`src/SCS.cpp:173-203`) | the sync-read frame check in `parse_sync_read_burst` covers id, length and checksum. Consecutive failures are counted per servo, and `WaveshareServos::read()` (`src/waveshare_servos.cpp`) drops a servo after `max_read_fails` of them |
| `syncReadPacketTx()` waits for every listed servo or for the timeout (`src/SCS.cpp:318`) | dropped servos leave the sync-read list. `io_timeout_ms` defaults to 5 ms (`kIoTimeoutMs`, `src/driver_defaults.hpp`), against the library's 100 (`src/SCSerial.cpp:13`) |
| `Mode()` writes EPROM register 33 (`src/SMS_STS.cpp:282-292`) | `WaveshareServos::set_mode` (`src/waveshare_servos.cpp`) reads the mode first and writes only when it differs |
| `Error`, the status byte, is a shared public member that later calls overwrite (`src/SCS.cpp:201`) | the status byte is taken from the same reply as the data: from each frame in `parse_sync_read_burst`, and in the per-servo fallback `ServoBus::read_feedback_one` reads `Error` directly after `Read()` (`src/servo_bus.cpp`) |
| `writeByte()` treats an acknowledgement from a different id as a failure (`SCS::Ack()`, `src/SCS.cpp:279`) | the tools' checked transactions in `ServoBus` (`ServoBus::checked_write`) handle the `set_id` case, where the acknowledgement comes from the new id; `set_id` does not go through `writeByte()` or `Ack()`: `checked_write` sends with `writeBuf()` and reads the reply with `readSCS()` itself, so a refresh that changes `writeBuf()`, `readSCS()`, `rFlushSCS()`/`wFlushSCS()`, `IOTimeOut` or the frame format affects `set_id`. A change to `writeByte()`/`Ack()` affects the driver's writes instead (`ServoBus::write_acc`, and `EnableTorque`, `unLockEprom`, `LockEprom` and `Mode` in `SMS_STS`) |

## Refreshing from upstream

1. Download the new release. Normalize each file: strip a leading UTF-8 BOM, then convert CRLF to
   LF.
2. Diff it against the normalized 220329 archive, not against this repository. That diff is the
   upstream change.
3. Re-apply the changes listed under [Import](#import-into-this-repository) and
   [The one local fix](#the-one-local-fix-commit-d846222-srcscserialcpp). The two adityakamath
   items and `d846222` are mechanical to re-apply.
4. Review the wrapper's assumptions against the new code. `ServoBus` relies on `fd`, `txBuf`,
   `writeBuf`, `readSCS`, `rFlushSCS` and `wFlushSCS` being `protected` rather than private, and
   the tests' seams rely on `txBufLen`, `writeSCS` and `Host2SCS` as well. `ServoBus` also relies
   on `txBuf` being 255 bytes, on `IOTimeOut`, `Err`, `Error` and the `syncReadRx*` members being
   public, and on the packet layout of `syncWrite`. The `static_assert`s and the pty tests guard
   these, but only for the paths they exercise.
5. Update the checksum table and every file:line citation in the package (see
   [Updating the table and the gates](#updating-the-table-and-the-gates) and
   [Finding the vendored line citations](#finding-the-vendored-line-citations)), and run the full
   test suite and the bench check.

Feetech's 2025.9.27 release changes the communication layer and drops two of the four servo
families, so moving to it is a port, not a refresh.

### What this package calls

Found by searching the package's own C++ files (everything under `src/`, `include/` and `test/`
except the 14 files above) for every public and protected member of `SCS`, `SCSerial` and
`SMS_STS`, with comments and string literals left out:

- **The driver** (`src/waveshare_servos.cpp`), on its `ServoBus`: `Ping`, `EnableTorque`,
  `unLockEprom`, `LockEprom`, `readByte` and `Mode`.
- **`ServoBus`** (`src/servo_bus.cpp`): `writeByte`, `syncWrite`, `syncReadPacketTx` and `Read`;
  the protected `writeBuf`, `readSCS`, `rFlushSCS` and `wFlushSCS`, which its checked
  transactions use to send a frame and read the reply; and `SCSerial::begin` and `SCSerial::end`,
  called qualified.
- **The tools** call nothing in the library directly. They go through `ServoBus`.
- **The bench helpers:** `test/hil/stop_wheels.cpp`, on a plain `SMS_STS`: `begin`, `end`,
  `readByte`, `readWord`, `WriteSpe`, `ReadSpeed`, `ReadPos` and `FeedBack`.
  `test/hil/eeprom_core.cpp`, through `ServoBus`: `Ping`, `Read`, `readByte`, `readWord`,
  `writeByte` and `writeWord`.
- **The tests** call more than the code they test. `test/test_servo_bus.cpp` compares the
  wrapper's packets with `SyncWritePosEx` and `SyncWriteSpe`, checks the sync read against
  `FeedBack` and the `Read*` accessors (`ReadPos`, `ReadSpeed`, `ReadLoad`, `ReadVoltage`,
  `ReadTemper`, `ReadMove`, `ReadCurrent`), and also calls `writeWord` and `readWord`. Its seams
  also use the protected layer. `PacketCapture` and `TxBufLenProbe` override `writeSCS` (both
  overloads), `readSCS`, `rFlushSCS` and `wFlushSCS`. `RawWire` calls `writeBuf`, `writeSCS`,
  `readSCS`, `rFlushSCS` and `wFlushSCS` directly, and `EndiannessProbe` calls `Host2SCS` (see
  refresh step 4). `test/test_hil_eeprom.cpp` overrides `readSCS`.
- **No call site anywhere:** `genWrite`, `regWrite`, `RegWriteAction`, `WritePosEx`,
  `RegWritePosEx`, `WheelMode`, `CalibrationOfs`, `syncReadPacketRx` and its two decoders,
  `syncReadBegin`, `syncReadEnd`, `getErr` and `setBaudRate` (`ServoBus` hides it with a
  using-declaration in `include/servo_bus.hpp`, so removing it upstream still breaks the build),
  and the protected `SCS2Host` and `Ack` (used only inside the library). `SyncWritePosEx` and
  `SyncWriteSpe` are called only by the tests above, as the reference the wrapper's packets are
  compared with.

### The baud-rate coupling

`kMappedBaudrates` in `src/servo_bus.cpp` mirrors the seven rates `SCSerial::begin()` maps
(`src/SCSerial.cpp:50-75`), and `ServoBus::is_supported_baudrate()` checks against it. That
mapping has already changed upstream twice. The adityakamath fork replaced `begin()`'s `switch`
with a pass-through (`speed_t CR_BAUDRATE = baudRate`) in commit `4af823c` (2024-08-14). It
restored the same seven-rate `switch`, with its 115200 fallback, in `a5ade65` (2025-12-04). At
`4a84794` (the fork's HEAD on 2026-09-24) `begin()` maps the same seven rates, with macOS-only
`IOSSIOSPEED` handling for 500000 and 1000000, while its `setBaudRate()` is still a pass-through
(this package never calls it). So a refresh must re-check the list, or the driver's baud-rate
validation silently diverges from the library.

### Finding the vendored line citations

```bash
git grep -n -E '\b(INST|SCS|SCSCL|SCSerial|SCServo|SMSBL|SMSCL|SMS_STS)\.(h|cpp):[0-9]' -- . \
  ':!include/INST.h' ':!include/S*.h' ':!src/S*.cpp'
```

lists every line that cites a vendored file by line number. At 1.0.0 it finds 159 lines in the
code, the tests and `CMakeLists.txt` (48 in `test/test_servo_bus.cpp`, 32 each in
`src/servo_bus.cpp` and `include/servo_bus.hpp`, 17 in `test/fake_servo_bus.hpp`, 10 in
`test/test_lifecycle_over_pty.cpp` and 20 in ten other files), plus the citations in this file
(`README.md` cites none). Every hit is re-checked after a refresh.

### Updating the table and the gates

Regenerate the checksum block with `sha256sum include/INST.h include/S*.h src/S*.cpp`, run from
the package root. When files are added or removed, change these together: `EXPECTED_FILE_COUNT`
in `test/test_vendored_files.py`, `add_library(scservo ...)` in `CMakeLists.txt`, and
`AMENT_LINT_AUTO_FILE_EXCLUDE` (the package's cpplint call reuses it for `--exclude`). Then
re-measure which files need `-Wno-vla`.

### A durable baseline

The Waveshare archive is not in this repository. Its URL, size, sha256 and `Last-Modified` date
are recorded under [The base](#the-base-feetech-scservo_linux-220329-as-distributed-by-waveshare).
If it disappears, [ftservo/FTServo_Linux](https://github.com/ftservo/FTServo_Linux) at commit
`5a9ffe3` is the same release: code-identical after the normalization, differing only in the
`飞特` description lines and with the SMSCL files still GBK-encoded. The archive is in RAR5
format; `cmake -E tar xf SCServo_Linux.rar` (libarchive) extracts it without `unrar`, reporting
one harmless error on a directory entry.

### Which tests guard `d846222`

With the fix reverted in a scratch copy of the package, two of the pseudo-terminal tests
(`test_servo_bus` and `test_lifecycle_over_pty`, no port needed) show which cases fail for each
of the three fixes. `test_servo_tools`, `test_tools_cli` and `test_hil_eeprom` also run the
vendored code over a pseudo-terminal, but they were not run with a fix reverted.

- the `readSCS()` loop: neither test catches it. With only this fix reverted, every case of
  `test_servo_bus` and `test_lifecycle_over_pty` passes, and only the checksum test in
  `test_vendored_files` fails;
- `wFlushSCS()`: neither test catches it. With only this fix reverted, every case of both tests
  passes, and only the checksum test in `test_vendored_files` fails;
- `end()`: 100 cases fail with it reverted: three in `test_servo_bus`
  (`close_releases_the_port_for_a_later_open`, `the_destructor_releases_the_port`,
  `port_holder_pids_finds_this_process`) and 97 of the 99 in `test_lifecycle_over_pty`, whose
  fixture checks after every case that the component closed the port.

With the whole commit reverted, the failing cases are exactly the `end()` ones.

### Licensing after a refresh

Code taken from a later upstream release comes under that release's MIT license, so the
[Licensing](#licensing) section and its notices change with it.

## Other files of third-party origin

Two files started as ros2_control example code. They keep their original Apache-2.0 headers.
Unlike the vendored library, the package has adapted them and they are edited like any other
package source. The Apache License 2.0 text is in
[LICENSES/Apache-2.0.txt](LICENSES/Apache-2.0.txt).

| file | origin (from its header) | license |
|---|---|---|
| `include/visibility_controls.h` | symbol-visibility header, `Copyright 2021 ros2_control Development Team`. Its macros were already renamed to `WAVESHARE_SERVOS_*` when it was imported in `acff137`. `d234a16` added two NOLINT markers | Apache-2.0 |
| `bringup/launch/example.launch.py` | ros2_control demo launch file, `Copyright 2021 Stogl Robotics Consulting UG (haftungsbeschränkt)`. Five commits have changed it since the import; against the imported version, 121 lines are added and 90 removed, and it has 184 lines | Apache-2.0 |

## Licensing

- The vendored files carry no license header and no copyright notice. Waveshare's archive contains
  no license file.
- Asked by the package author, Waveshare said to use the GPLv3 license. This package is licensed
  GPL-3.0-or-later (`package.xml`). `LICENSE` is the unmodified GPLv3 text.
- Feetech has since published the same release under the MIT License (2025-01-04). The
  adityakamath repository has carried the MIT License since 2025-12-04. It had no license when the
  two additions were copied from it. Both MIT notices are reproduced below.

<details>
<summary>MIT notices of the upstream sources</summary>

ftservo/FTServo_Linux (`LICENSE`, commit `bafb22d`):

```text
MIT License

Copyright (c) 2024 FTServo

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
```

adityakamath/SCServo_Linux (`LICENSE`, at `333dfe4` and later) uses the same text with the
copyright lines `Copyright (c) 2024 FTServo` and
`Copyright (c) 2025 Aditya Kamath (Kamath Robotics)`. Between `a5ade65` and `333dfe4` the same
MIT text carried the lines `Copyright (c) 2024 FTServo (Original Feetech SCServo SDK)` and
`Copyright (c) 2025 Aditya Kamath (Modifications and Enhancements)`.

</details>

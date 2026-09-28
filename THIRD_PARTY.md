# Third-party code

This package vendors one component: Feetech's **SCServo Linux library, release 220329**, in the
copy that Waveshare distributes. The copy has two additions from
[adityakamath/SCServo_Linux](https://github.com/adityakamath/SCServo_Linux) and one local fix. It
is the packet layer of every bus transaction. The static `scservo` target compiles its six sources,
and the plugin and the four tools link only its SMS_STS, SCS and SCSerial code. The package
installs all eight headers, because the public plugin header includes `servo_bus.hpp`, and through
it `SMS_STS.h`, `SCSerial.h`, `SCS.h` and `INST.h`. **Do not edit these files**
([The rule](#the-rule-these-files-are-not-edited)).

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

1. The SMSCL files are GBK-encoded upstream. In the Waveshare copy, a lossy UTF-8 conversion
   changed most Chinese characters to U+FFFD. The header has 150 of them, and the source has 15.
   Feetech's original line is 飞特SMSCL系列串行舵机接口.
2. `src/SCSerial.cpp` names itself `SCSerial.h` in its header. This mistake comes from upstream.
3. `飞特` is Feetech's Chinese name. Feetech's copies start every description with it, but
   Waveshare's copy keeps it only in `src/SCS.cpp`.
4. Only the bench helper `test/hil/stop_wheels.cpp` includes the umbrella header
   `include/SCServo.h`. The package does not install that helper.
5. `include/visibility_controls.h` is not part of this component
   ([Other files of third-party origin](#other-files-of-third-party-origin)).

### Checksums

The block below records the files as they are in this repository. `test/test_vendored_files.py`
fails if a file differs from its row, or if a row is missing or extra. It also fails if a file in
`include/` or `src/` with an upper-case first letter has no row. Only vendored files have such
names. To check by hand, run this command from the package root:
`sed -n '/vendored-sha256:begin/,/vendored-sha256:end/p' THIRD_PARTY.md | grep -E '^[0-9a-f]{64}  ' | sha256sum -c`

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

- **Archive:** `SCServo_Linux.rar`, the "ST/SC serial bus servo control library (Linux)" link in
  the Resources section of the
  [Bus Servo Adapter (A) wiki page](https://www.waveshare.com/wiki/Bus_Servo_Adapter_(A)):
  <https://files.waveshare.com/wiki/Bus-Servo-Adapter-(A)/SCServo_Linux.rar>
- **As fetched on 2026-09-23:** 35580 bytes, sha256
  `bb364596666097c1e5260ca80e9f22b44d6a5b2fbde2bf2247a633e541424343`, HTTP
  `Last-Modified: Mon, 18 Sep 2023 07:02:20 GMT`.
- **Contents:** `SCServo_Linux_220329/SCServo_Linux/` with the 14 library files, `examples/`, a
  `CMakeLists.txt` that builds `libSCServo.a`, `说明.txt` (overview) and `编译说明.txt` (build
  instructions).
- **Match:** after [normalization](#refresh-from-upstream) (step 1), 7 of the 14 imported files
  are byte-identical to the archive. These are `INST.h`, `SCSCL.h`, `SCSerial.h`, `SCServo.h`,
  `SMSBL.h`, `SMSCL.h` and `SCSerial.cpp` before
  [the local fix](#the-one-local-fix-commit-d846222-srcscserialcpp). The other 7 differ only by the
  [import changes](#import-into-this-repository).
- **Feetech's own copy:** [ftservo/FTServo_Linux](https://github.com/ftservo/FTServo_Linux). Its
  first commit with sources is `5a9ffe3` (2025-01-04). The code is the same as the Waveshare copy.
  Only the `飞特` prefix (note 3) and the GBK SMSCL files differ.
- **Feetech's next release** (dated 2025.9.27, commit `606022b`) is a large revision. `src/SCS.cpp`
  alone differs in 180 lines. It removes SMSBL and SMSCL and adds HLSCL.

### From adityakamath/SCServo_Linux

This independent repository started from the same 220329 release. Two items come from it:

- **The `snycWrite` -> `syncWrite` rename.** Upstream misspells `SCS::syncWrite` as `snycWrite`.
  The fork corrected it in `eb9f5de` (2023-06-10). This package renames the same six upstream
  occurrences: `include/SCS.h:21`, `src/SCS.cpp:124`, `src/SCSCL.cpp:64`, `src/SMSBL.cpp:79`,
  `src/SMSCL.cpp:78` and `src/SMS_STS.cpp:79`. `SyncWriteSpe` adds one more call, at
  `src/SMS_STS.cpp:279`. `ServoBus` calls `syncWrite`, so an unchanged upstream copy does not build.
- **`SMS_STS::SyncWriteSpe()` and `SMS_STS::Mode()`,** declared at `include/SMS_STS.h:88-90` and
  defined at `src/SMS_STS.cpp:260-292`. Each block starts with a comment that gives the fork's URL.
  After CRLF normalization, they are byte-identical to the fork from `d3ad38c` (2023-07-09) up to
  `c796016` (2025-11-30, exclusive). This span includes `95b9e20` (2024-05-05), the last fork
  commit before the import. The driver calls `Mode()`, but not `SyncWriteSpe()`.

### Import into this repository

Commit `acff137` (2024-08-06) added the files under `hardware/`. Commit `b554cf5` (2025-01-14)
moved them to `include/` and `src/` with no content change (git reports 100% similarity). Against
the Waveshare archive, the import made only these changes:

- CRLF changed to LF in all 14 files.
- The UTF-8 BOM removed from the 9 files that had one: `SCS.h`, `SCSCL.h`, `SCSerial.h`,
  `SMSBL.h`, `SMS_STS.h`, `SCS.cpp`, `SCSCL.cpp`, `SMSBL.cpp` and `SMS_STS.cpp`.
- A final newline added to `include/SMS_STS.h`.
- The two adityakamath changes above.

### The one local fix: commit `d846222`, `src/SCSerial.cpp`

Commit `d846222` ("smooth motion", 2026-09-10) is the only change to these files after the import,
and their only difference between the `humble` and `jazzy` branches. It fixes three functions:

- **`readSCS()`** sets the `select()` descriptor set again in each loop iteration. It reads only
  when `select()` returns more than 0, and tries again on `EINTR`, `EAGAIN` or `EWOULDBLOCK`. It
  never adds a -1 from `read()` to the byte count. Before the fix, a `read()` error moved the write
  position below the caller's buffer. The fix adds `#include <errno.h>`.
- **`wFlushSCS()`** calls `write()` until all of the frame goes out. It stops after 1000 retries in
  a row (`EAGAIN`, `EWOULDBLOCK` or `EINTR`), and any progress sets the count back to zero. The
  original code discarded the part that a short `write()` on the non-blocking port did not accept.
  The fix comment says that the servo then drops the frame on its checksum (not measured).
- **`end()`** closes the descriptor before it sets it to -1. The original code
  (`fd = -1; close(fd);`) closed -1 and leaked the port.

Neither upstream has all three fixes. Feetech's `FTServo_Linux` at `06fd335` (2026-08-28) has none
of them. The fork fixed `end()` in `c796016` (2025-11-30), and has only the descriptor-set part of
the `readSCS()` fix. At `4a84794`, its `readSCS()` still calls `read()` when `select()` fails and
adds the -1 to the byte count. Its `wFlushSCS()` still makes one `write()` only.

## The rule: these files are not edited

- Treat the files as read-only, with the checksum table as the baseline. Put what the driver needs
  beyond the library in the wrapper `ServoBus` (`include/servo_bus.hpp`, `src/servo_bus.cpp`). If a
  library function has no workaround, copy it into the driver under this package's license.
- `CMakeLists.txt` handles warnings and lint, never the files. The `scservo` target uses `-Wno-vla`
  for the five variable-length arrays at `src/SMS_STS.cpp:56,263`, `src/SMSBL.cpp:56`,
  `src/SCSCL.cpp:45` and `src/SMSCL.cpp:55`. `AMENT_LINT_AUTO_FILE_EXCLUDE` lists all 14 files,
  and the cpplint call of the package gives the same list to `--exclude`.
- Code comments cite these files by line, for example `src/SCS.cpp:355`, and an edit can make these
  citations wrong. `README.md` and `docs/` name the package's own code by function, class or log
  text, and never cite a package file by line. Only this file and code comments cite vendored
  lines, because those lines move only on a refresh.
- `test/test_vendored_files.py` enforces the rule. It needs no servos.

## Known defects and where the package works around them

| library behaviour | where the package handles it |
|---|---|
| `SCSerial::begin()` does not open the tty for exclusive use (`src/SCSerial.cpp:41`) | `ServoBus::open()` sets `TIOCEXCL` and takes `flock(LOCK_EX\|LOCK_NB)` |
| `begin()` maps seven baud rates, and uses 115200 for other rates with no message (`src/SCSerial.cpp:50-75`) | `on_init` and `src/tool_params.cpp` refuse other rates through `ServoBus::is_supported_baudrate()` |
| `setBaudRate()` never calls `tcsetattr`, and reads an uninitialised `speed_t` for an unmapped rate (`src/SCSerial.cpp:104,127-131`) | a private using-declaration in `ServoBus` hides it. `ServoBus::set_baudrate()` replaces it |
| `begin()` prints with `printf` (`src/SCSerial.cpp:79`), and its `perror()` calls overwrite `errno` | `ServoBus::open()` calls `fflush(stdout)`. It opens its own descriptor first, so it can report the real `errno` |
| `writeSCS()` has no bounds check on `txBuf[255]` (`src/SCSerial.cpp:179-191`). The `syncWrite()` length field is a `u8` | `ServoBus` splits packets into chunks that fit. `static_assert`s and the test `the_vendored_transmit_buffer_is_255_bytes` check the sizes |
| `SyncWriteSpe()` does a blocking `genWrite` and `Ack` of ACC for each servo (`src/SMS_STS.cpp:261-280`) | the driver does not call it. `ServoBus::write_goal_speeds` sends a plain `syncWrite`. `ServoBus::write_acc` writes ACC only at four [edges](docs/design.md#wheel-acceleration) |
| `SyncWritePosEx()` changes the caller's `Position[]` and uses a VLA (`src/SMS_STS.cpp:53-61`) | the driver does not call it. `ServoBus::write_goal_positions` sends the same bytes, and tests compare the two |
| the constructors leave `Err` and the `syncRead*` members uninitialised | `ServoBus::ServoBus()` sets them to zero |
| `syncReadBegin()` and `syncReadEnd()` pair `new[]` with `delete` (`src/SCS.cpp:325,331`) | not used. The `ServoBus` constructor allocates its own receive buffer once |
| `syncReadPacketRx` can read up to 18 bytes past the buffer (`src/SCS.cpp:355`) | not used. `parse_sync_read_burst` checks the size before each frame, and `ServoBus::sync_read_slack_bytes` adds 18 bytes to the buffer |
| `Read()` checks neither the responder id nor the length byte, and does not resync (`src/SCS.cpp:173-203`) | `parse_sync_read_burst` checks id, length and checksum. `WaveshareServos::read()` drops a servo after `max_read_fails` failures in a row |
| `syncReadPacketTx()` waits for each listed servo or for the timeout (`src/SCS.cpp:318`) | a dropped servo leaves the sync-read list. `io_timeout_ms` defaults to 5 ms (`kIoTimeoutMs`), not 100 ms (`src/SCSerial.cpp:13`) |
| `Mode()` writes EEPROM register 33 (`src/SMS_STS.cpp:282-292`) | `WaveshareServos::set_mode` reads the mode first, and writes only if it differs |
| later calls overwrite `Error`, the shared public status byte (`src/SCS.cpp:201`) | the status comes from the same reply as the data: each frame in `parse_sync_read_burst`, or `Error` directly after `Read()` in `ServoBus::read_feedback_one` |
| `writeByte()` treats an ack from a different id as a failure (`SCS::Ack()`, `src/SCS.cpp:279`) | the tools use `ServoBus::checked_write` (`writeBuf()` and `readSCS()`), which accepts an ack from any id, as `set_id` needs |

Two more traps are in [Vendored library traps](docs/design.md#vendored-library-traps): the
`FD_SET` abort on a closed bus, and the 0 or 1 return value of `Ack`. For the sign decode of
`ReadPos`, `ReadSpeed`, `ReadLoad` and `ReadCurrent`, which depends on `Err`, see
[Feedback block](docs/design.md#feedback-block).

## Refresh from upstream

1. Download the new release. Remove a leading UTF-8 BOM from each file, then change CRLF to LF.
2. Compare the result with the normalized 220329 archive, not with this repository. This diff is
   the upstream change.
3. Apply the [import changes](#import-into-this-repository) and
   [the local fix](#the-one-local-fix-commit-d846222-srcscserialcpp) again.
4. Compare the new code with the needs of `ServoBus` (below) and with
   [What this package calls](#what-this-package-calls).
5. Update the checksum table ([Update the table and the gates](#update-the-table-and-the-gates)).
6. Update each vendored line citation
   ([Find the vendored line citations](#find-the-vendored-line-citations)).
7. Run the full test suite and the [bench check](docs/bench-check.md#run-the-bench-check).

`ServoBus` needs these library details. The `static_assert`s and the pty tests guard them only on
the paths that they run.

- `fd`, `txBuf`, `writeBuf`, `readSCS`, `rFlushSCS` and `wFlushSCS` stay `protected`, not private.
  The test seams also use `txBufLen`, `writeSCS` and `Host2SCS`.
- `txBuf` holds 255 bytes, and the packet layout of `syncWrite` stays the same.
- `IOTimeOut`, `Err`, `Error` and the `syncReadRx*` members stay public.
- `set_id` depends on `writeBuf()`, `readSCS()`, `rFlushSCS()`, `wFlushSCS()`, `IOTimeOut` and the
  frame format, through the checked transactions of the tools.
- The driver writes depend on `writeByte()` and `Ack()`. These are `ServoBus::write_acc`, and
  `EnableTorque`, `unLockEprom`, `LockEprom` and `Mode` in `SMS_STS`.

Feetech's 2025.9.27 release changes the communication layer and removes two of the four servo
families. A move to it is a port, not a refresh.

### What this package calls

This table comes from a search of the package's own C++ files in `src/`, `include/` and `test/`.
The search looked for each public and protected member of `SCS`, `SCSerial` and `SMS_STS`. It
skipped the vendored files, comments and string literals.

| caller | library members |
|---|---|
| the driver (`src/waveshare_servos.cpp`), on its `ServoBus` | `Ping`, `EnableTorque`, `unLockEprom`, `LockEprom`, `readByte`, `Mode` |
| `ServoBus` (`src/servo_bus.cpp`) | `writeByte`, `syncWrite`, `syncReadPacketTx`, `Read`, `SCSerial::begin`, `SCSerial::end` |
| the checked transactions of `ServoBus` | the protected `writeBuf`, `readSCS`, `rFlushSCS`, `wFlushSCS` |
| the tools | none directly. They go through `ServoBus` |
| `test/hil/stop_wheels.cpp`, on a plain `SMS_STS` | `begin`, `end`, `readByte`, `readWord`, `WriteSpe`, `ReadSpeed`, `ReadPos`, `FeedBack` |
| `test/hil/eeprom_core.cpp`, through `ServoBus` | `Ping`, `Read`, `readByte`, `readWord`, `writeByte`, `writeWord` |
| `test/test_servo_bus.cpp`, as the packet reference | `SyncWritePosEx`, `SyncWriteSpe` (no other caller) |
| `test/test_servo_bus.cpp`, to check the sync read | `FeedBack`, `ReadPos`, `ReadSpeed`, `ReadLoad`, `ReadVoltage`, `ReadTemper`, `ReadMove`, `ReadCurrent` |
| `test/test_servo_bus.cpp`, other calls | `writeWord`, `readWord` |
| the `PacketCapture` and `TxBufLenProbe` test seams (overrides) | `writeSCS` (both overloads), `readSCS`, `rFlushSCS`, `wFlushSCS` |
| the `RawWire` test seam | `writeBuf`, `writeSCS`, `readSCS`, `rFlushSCS`, `wFlushSCS` |
| the `EndiannessProbe` test seam | `Host2SCS` |
| `test/test_hil_eeprom.cpp` (override) | `readSCS` |
| only the library | the protected `SCS2Host` and `Ack` |
| no caller | `genWrite`, `regWrite`, `RegWriteAction`, `WritePosEx`, `RegWritePosEx`, `WheelMode`, `CalibrationOfs`, `setBaudRate`, `syncReadPacketRx` and its two decoders, `syncReadBegin`, `syncReadEnd`, `getErr` |

The test seams are in `test/test_servo_bus.cpp`. `ServoBus` hides `setBaudRate` with a
using-declaration in `include/servo_bus.hpp`, so the build breaks if upstream removes it.

### The baud-rate coupling

`kMappedBaudrates` in `src/servo_bus.cpp` copies the seven rates that `SCSerial::begin()` maps
(`src/SCSerial.cpp:50-75`). `ServoBus::is_supported_baudrate()` checks against it. A refresh must
check this list again, or the driver and the library can disagree about a rate without a message.

The adityakamath fork changed this mapping two times. `4af823c` (2024-08-14) replaced the `switch`
in `begin()` with a pass-through (`speed_t CR_BAUDRATE = baudRate`). `a5ade65` (2025-12-04)
restored the same seven-rate `switch` with its 115200 fallback. At `4a84794` (the fork HEAD on
2026-09-24), `begin()` maps the same seven rates, with `IOSSIOSPEED` handling for 500000 and
1000000 on macOS only. Its `setBaudRate()` is still a pass-through.

### Find the vendored line citations

This command lists each line that cites a vendored file by line number:

```bash
git grep -n -E '\b(INST|SCS|SCSCL|SCSerial|SCServo|SMSBL|SMSCL|SMS_STS)\.(h|cpp):[0-9]' -- . \
  ':!include/INST.h' ':!include/S*.h' ':!src/S*.cpp'
```

It finds the citations in the code, in the tests and in this file. `README.md` and `docs/` cite
none. After a refresh, check each hit again.

### Update the table and the gates

1. From the package root, run `sha256sum include/INST.h include/S*.h src/S*.cpp`. Replace the
   rows of the checksum block with its output.
2. If a refresh adds or removes a file, change these together: `EXPECTED_FILE_COUNT` in
   `test/test_vendored_files.py`, `add_library(scservo ...)` in `CMakeLists.txt`, and
   `AMENT_LINT_AUTO_FILE_EXCLUDE`. The cpplint call of the package also uses this list.
3. Find again which files need `-Wno-vla`.

### A durable baseline

This repository does not contain the Waveshare archive.
[The base](#the-base-feetech-scservo_linux-220329-as-distributed-by-waveshare) records its URL,
size, sha256 and `Last-Modified` date. If the archive disappears, use `ftservo/FTServo_Linux` at
`5a9ffe3`, which has the same code. The archive is in RAR5 format.
`cmake -E tar xf SCServo_Linux.rar` (libarchive) extracts it without `unrar`, with one harmless
error on a directory entry.

### Which tests guard `d846222`

A check reverted each fix in a scratch copy of the package. It ran `test_servo_bus` and
`test_lifecycle_over_pty`, two pseudo-terminal tests that need no port. It did not run
`test_servo_tools`, `test_tools_cli` or `test_hil_eeprom`, which also run the vendored code over a
pseudo-terminal.

- **`readSCS()` loop or `wFlushSCS()` reverted alone:** each case of both tests passes. Only the
  checksum test in `test_vendored_files` fails.
- **`end()` reverted:** 100 cases fail. Three are in `test_servo_bus`:
  `close_releases_the_port_for_a_later_open`, `the_destructor_releases_the_port` and
  `port_holder_pids_finds_this_process`. The other 97 are in `test_lifecycle_over_pty` (99 cases),
  whose fixture checks after each case that the component closed the port.
- **The whole commit reverted:** exactly the `end()` cases fail.

### Licenses after a refresh

Code from a later upstream release comes under the MIT license of that release. Update the
[Licensing](#licensing) section and its notices with it.

## Other files of third-party origin

Two files started as ros2_control example code, and they keep their original Apache-2.0 headers.
Unlike the vendored library, the package adapted them and edits them like any other source file.
The Apache License 2.0 text is in [LICENSES/Apache-2.0.txt](LICENSES/Apache-2.0.txt).

| file | origin (from its header) | license |
|---|---|---|
| `include/visibility_controls.h` | symbol-visibility header, `Copyright 2021 ros2_control Development Team`. Its macros had the `WAVESHARE_SERVOS_*` names at the import in `acff137`. `d234a16` added two NOLINT markers | Apache-2.0 |
| `bringup/launch/example.launch.py` | ros2_control demo launch file, `Copyright 2021 Stogl Robotics Consulting UG (haftungsbeschränkt)`. The package changed much of it after the import | Apache-2.0 |

## Licensing

- The vendored files have no license header and no copyright notice. Waveshare's archive has no
  license file.
- The package author asked Waveshare about the license, and Waveshare said to use the GPLv3
  license. The package is GPL-3.0-or-later (`package.xml`), and `LICENSE` is the unmodified GPLv3
  text.
- Feetech published the same release under the MIT License on 2025-01-04. The adityakamath
  repository has the MIT License since 2025-12-04 (`a5ade65`), but had none when this package
  copied the two additions. Both MIT notices follow.

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

adityakamath/SCServo_Linux (`LICENSE`, since `333dfe4` (2026-06-03), unchanged at `4a84794`)
uses the same text with the copyright lines `Copyright (c) 2024 FTServo` and
`Copyright (c) 2025 Aditya Kamath (Kamath Robotics)`. From `a5ade65` to `333dfe4`, the same MIT
text had the lines `Copyright (c) 2024 FTServo (Original Feetech SCServo SDK)` and
`Copyright (c) 2025 Aditya Kamath (Modifications and Enhancements)`.

</details>

# GK7205V200 Vehicle Camera Full Implementation Plan

## Waybeam-based `venc` + IMX307 720p55 + VFRM v2 + `waybeam-link tx` + RTL8812AU

**Document role:** Single source of truth for vehicle-side implementation  
**Target:** Goke GK7205V200 camera board  
**Status:** Architecture accepted; implementation and target validation pending  
**Last consolidated:** 2026-10-20

---

## 1. Executive summary

This project ports the Waybeam encoder architecture to a camera board containing:

- Goke GK7205V200 SoC.
- One Cortex-A7 CPU core at approximately 900 MHz.
- Sony IMX307 image sensor.
- 16 MiB SPI NOR flash, Winbond/GigaDevice-style `25Q128A` class.
- RTL8812AU USB Wi-Fi adapter.
- One USB 2.0 host controller.

The target vehicle pipeline is:

```text
IMX307
→ MIPI receiver
→ VI
→ ISP
→ VPSS
→ hardware H.265 VENC
→ complete Annex-B access unit
→ VFRM v2 shared-memory ring
→ waybeam-link tx
→ source symbols + bounded GF(256) FEC
→ asynchronous USB transmission
→ RTL8812AU
→ raw 802.11 RF
```

There are exactly two principal Linux processes:

```text
1. venc
   - Waybeam-based encoder
   - GK7205V200 backend
   - VFRM producer
   - vehicle entry process
   - configurable child-process supervisor

2. waybeam-link tx
   - independent child process
   - VFRM consumer
   - frame classification, fragmentation, FEC, radio TX
```

`venc` creates and validates the VFRM ring first. Only after the VFRM producer state changes to `READY` does `venc` start `waybeam-link tx` using `posix_spawn()` or immediate `fork()+execve()`.

The encoded-video IPC is exclusively:

```text
frame-shm://venc0
```

UDP and RTP are not used between `venc` and `waybeam-link tx`.

Performance targets:

```text
Whole-system glass-to-glass target:       ≤30 ms
Vehicle-side T4−T0 target:                P99 ≤22 ms
Complete AU ready to first USB submit:    P99 ≤2.0 ms
VFRM publish to consumer acquire:         P99 ≤0.2 ms
Normal VFRM occupancy:                    0–1 complete AU
Nominal average CPU target:               ≤65% of one Cortex-A7 core
```

These values are requirements or acceptance gates, not confirmed results.

---

## 2. Source repositories and intended usage

### 2.1 Waybeam encoder

Repository:

- [Waybeam encoder historical URL](https://github.com/snokvist/waybeam_venc)
- [Current OpenIPC Waybeam repository](https://github.com/OpenIPC/waybeam)

Purpose in this project:

- Reuse Waybeam common encoder architecture.
- Reuse the backend registration and capability model.
- Reuse common configuration, control, status, and metrics infrastructure.
- Reuse VFRM/frame-SHM output support.
- Add a new GK7205V200/IMX307 backend.
- Add a configurable child-process supervisor to the `venc` process.

The GK7205 port must not be implemented as a disconnected encoder application. It is an extension of the Waybeam architecture.

### 2.2 OpenIPC Research Goke VENC example

Repository:

- [OpenIPC Research repository](https://github.com/OpenIPC/research)
- [OpenIPC Research `venc` example](https://github.com/OpenIPC/research/tree/master/venc)
- [OpenIPC Research build script](https://github.com/OpenIPC/research/blob/master/build.sh)

Purpose in this project:

- Reference for a proven GK7205/HiMPP media-pipeline startup sequence.
- Reference for Goke vendor API usage and linkage.
- Reference for VB/SYS, sensor, MIPI, VI, ISP, VPSS, and VENC configuration.
- Reference for VI→VPSS and VPSS→VENC hardware bindings.
- Reference for `QueryStatus`, `GetStream`, and `ReleaseStream` ownership.
- Reference for safe shutdown ordering.

The sample is not the target software architecture. Do not copy:

- Its monolithic `main.c` organization.
- Its UDP output path.
- Its compact/RTP-style NAL fragmentation.
- Its global-variable structure.
- Its polling/thread boundaries without analysis.

Only the known-working media-pipeline knowledge is ported into the GK7205 Waybeam backend.

### 2.3 waybeam-link

Repository:

- [waybeam-link repository](https://github.com/snokvist/waybeam-link)

Purpose in this project:

- Consume complete frames from VFRM shared memory.
- Classify frame importance.
- Fragment complete access units into source symbols.
- Add bounded GF(256) repair symbols.
- Submit raw 802.11 frames through the userspace radio backend.
- Operate RTL8812AU through the supported devourer/libusb path.
- Expose TX and link-health metrics.

### 2.4 IMX307 high-FPS reference

Repository:

- [tipoman9 IMX307 high-FPS modes](https://github.com/tipoman9/imx307_modes)

Published reference modes:

```text
1920×1080: 39 fps
1280×720:  55 fps
```

The project shall use the name **720p55**, not 720p60, until 60.0 complete encoded frames/s is proven on the exact target.

The high-FPS repository is an upstream reference. Its 55 fps claim remains `REFERENCE`, not `CONFIRMED`, until target evidence is recorded.

---

## 3. Architecture drivers

### 3.1 Functional drivers

The vehicle software shall:

1. Configure IMX307 in the high-frame-rate 1280×720 mode.
2. Establish the complete hardware-bound media pipeline.
3. Produce hardware-encoded H.265 access units.
4. Normalize vendor packs into complete Annex-B access units.
5. Publish complete access units through VFRM v2.
6. Start `waybeam-link tx` only after VFRM is ready.
7. Transmit the video through one RTL8812AU vehicle radio.
8. Support bounded, observable failure and restart behavior.
9. Support live bitrate and IDR control where the vendor API allows it.
10. Capture enough timing evidence to decide whether the latency target is feasible.

### 3.2 Quality attributes

#### Performance

- Whole-system latency target ≤30 ms.
- Vehicle-side P99 ≤22 ms.
- No unbounded queue.
- No persistent queue depth above one complete AU.
- No per-frame dynamic memory allocation in hot paths.
- No per-frame filesystem logging.

#### Reliability

- The encoder never blocks waiting for the TX consumer.
- A `waybeam-link tx` crash does not stop sensor/ISP/VENC.
- Child restart storms are bounded.
- Shared-memory ABI mismatch fails explicitly.
- Every child process is reaped.
- VENC stream ownership is balanced: each successful `GetStream` has exactly one `ReleaseStream`.

#### Modifiability

- Goke/HiMPP types remain inside the GK7205 backend.
- Waybeam common code remains platform independent.
- waybeam-link depends on VFRM, not on the GK7205 backend.
- Process-supervisor behavior is generic and config-driven.

#### Observability

Per-frame vehicle checkpoints:

```text
T0 sensor frame start
T1 complete H.265 access unit available
T2 VFRM publish complete
T3 waybeam-link acquires the AU
T4 first USB transfer submitted
```

### 3.3 Constraints

- Single Cortex-A7 CPU core.
- 16 MiB NOR image budget.
- One USB 2.0 host.
- Vendor media libraries and API.
- Complete-frame VFRM contract for the first product slice.
- One vehicle radio.

---

## 4. Module decomposition design

## 4.1 `venc` executable

```text
venc
├── Waybeam Common Core                         [reused]
│   ├── application bootstrap
│   ├── configuration manager
│   ├── backend registry and selection
│   ├── encoder lifecycle manager
│   ├── common encoded-frame abstraction
│   ├── output dispatcher
│   ├── VFRM producer
│   ├── control/status API
│   └── metrics
│
├── BackendOps                                  [existing contract]
│
├── GK7205V200 Backend                          [new]
│   ├── backend registration and probe
│   ├── capability provider
│   ├── Goke media-system adapter
│   ├── IMX307 sensor-mode adapter
│   ├── MIPI/VI/ISP/VPSS pipeline adapter
│   ├── H.265 VENC adapter
│   ├── encoded-pack normalizer
│   ├── runtime encoder controls
│   └── platform metrics
│
└── Configurable Child Supervisor               [new]
    ├── child configuration parser
    ├── dependency/readiness gate
    ├── spawn adapter
    ├── child registry
    ├── exit/reap handler
    ├── restart/backoff policy
    └── shutdown coordinator
```

## 4.2 VFRM contract

```text
VFRM v2
├── magic and ABI version
├── mapped size and header size
├── producer generation
├── producer state
├── producer and consumer sequences
├── slot count and slot stride
├── frame metadata
├── complete Annex-B payload
├── acquire/release synchronization
├── futex wake protocol
└── stale-ring handling
```

## 4.3 `waybeam-link tx` executable

```text
waybeam-link tx
├── TX bootstrap
├── VFRM consumer
├── frame classification
├── source-symbol fragmentation
├── bounded GF(256) FEC
├── bounded TX scheduling
├── raw 802.11 frame creation
├── RTL8812AU userspace backend
├── USB event/completion handling
└── link metrics and control
```

## 4.4 Dependency rules

Allowed:

```text
Waybeam Common Core → BackendOps
GK7205 Backend       implements BackendOps
GK7205 Backend       → Goke/HiMPP API
Waybeam Output       → VFRM v2
waybeam-link TX      → VFRM v2
waybeam-link TX      → RTL8812AU userspace backend
Child Supervisor     → POSIX process APIs
```

Forbidden:

```text
Waybeam Common Core  → Goke/HiMPP structs
GK7205 Backend        → waybeam-link implementation
waybeam-link          → GK7205 backend
VFRM Producer         → RTL8812AU
Sensor pipeline       → VFRM slot internals
Configured child      → shell command string
```

---

## 5. Runtime Component-and-Connector design

## 5.1 Runtime instances

```text
Hardware runtime:
    IMX307 sensor
    GK7205 Media Pipeline: MIPI + VI + ISP + VPSS
    H.265 VENC channel

Linux process 1:
    venc
        Media Pipeline Controller
        VENC Stream Worker
        VFRM Producer
        Child Supervisor

Shared runtime store:
    VFRM v2 SPSC ring
    frame-shm://venc0

Linux process 2:
    waybeam-link tx
        TX Hot-Path Worker
        USB Event Handler
        Link Controller

External hardware:
    GK7205 USB 2.0 host
    RTL8812AU USB radio
```

## 5.2 Main video path

```text
IMX307
  -- MIPI RAW -->
MIPI + VI + ISP + VPSS
  -- hardware-bound YUV -->
H.265 VENC
  -- encoded packs -->
VENC Stream Worker
  -- complete AU + metadata -->
VFRM Producer
  -- shared-memory publish -->
VFRM Ring
  -- shared-memory acquire -->
TX Hot-Path Worker
  -- bounded asynchronous queue -->
USB Event Handler
  -- USB bulk TX -->
GK7205 USB Host
  --> RTL8812AU
  --> raw 802.11 RF
```

## 5.3 Supervision and process-lifecycle connector

```text
venc Child Supervisor
    -- posix_spawn/exec --> waybeam-link tx

waybeam-link tx
    -- exit status / SIGCHLD --> venc Child Supervisor

waybeam-link tx
    -- optional READY status --> venc control endpoint

venc Child Supervisor
    -- SIGTERM / SIGKILL lifecycle --> waybeam-link tx
```

This connector is separate from the VFRM video-data connector.

## 5.4 Low-rate encoder control connector

Recommended target:

```text
waybeam-link Link Controller
    -- AF_UNIX SOCK_SEQPACKET -->
venc Media Pipeline Controller
```

Initial commands:

```text
SetBitrate
RequestIDR
SetResilienceProfile
GetEncoderStatus
```

## 5.5 Single-core rule

The two processes do not execute in true CPU parallelism. Critical contention:

```text
VENC dequeue
→ VFRM copy/publish
→ scheduling wake-up
→ source-symbol fragmentation
→ GF(256) FEC
→ USB submit
→ USB completion IRQ/softirq
```

Classifier, fragmenter, FEC, frame builder, and USB submit shall remain one sequential hot path unless measurements prove another design is superior.

---

## 6. `venc` as entry process and child supervisor

## 6.1 Startup state machine

```text
BOOT
  ↓
LOAD_CONFIG
  ↓
VALIDATE_CHILD_CONFIG
  ↓
INIT_MEDIA
  ↓
START_VENC
  ↓
CREATE_VFRM
  ↓
VALIDATE_VFRM
  ↓
MARK_VFRM_READY
  ↓
SPAWN_CONFIGURED_CHILDREN
  ↓
WAIT_FOR_CHILD_READINESS
  ↓
RUNNING
```

## 6.2 VFRM readiness gate

Do not spawn a consumer merely because a shared-memory object exists.

Required checks:

```text
magic valid
ABI version supported
mapped size correct
header initialization complete
slot count valid
slot stride valid
producer generation assigned
producer state changed to READY
initialization writes published with release semantics
```

The consumer uses acquire semantics before trusting the remaining header.

## 6.3 Spawn mechanism

Preferred:

```c
posix_spawn();
```

Fallback:

```text
fork
→ perform only async-signal-safe file-descriptor actions
→ immediate execve
```

Do not perform in the child after `fork()`:

- Heap allocation.
- Complex logging.
- Mutex operations.
- Media API calls.
- JSON parsing.
- C++ runtime work.
- Non-async-signal-safe library calls.

## 6.4 Generic child configuration semantics

```json
{
  "children": [
    {
      "name": "waybeam-link-tx",
      "enabled": true,
      "exec": "/usr/bin/waybeam-link",
      "argv": [
        "waybeam-link",
        "tx",
        "-c",
        "/etc/waybeam-link/vehicle.json"
      ],
      "env": {
        "VFRM_URI": "frame-shm://venc0"
      },
      "startAfter": "vfrm-ready",
      "restart": "on-failure",
      "restartBackoffMs": 500,
      "restartBackoffMaxMs": 10000,
      "restartLimit": 10,
      "restartWindowSec": 60,
      "stopSignal": "SIGTERM",
      "stopTimeoutMs": 2000,
      "required": true
    }
  ]
}
```

This defines the required semantics. The exact keys must follow the current Waybeam configuration framework.

## 6.5 Security requirements

Because child configuration allows process execution:

- Absolute executable path only.
- No shell.
- No `sh -c`.
- No command-string parser.
- `argv` stored as an array.
- Executable allowlist.
- Root-owned, non-writable configuration.
- Bounded arguments and environment.
- Environment allowlist.
- Close-on-exec for unrelated descriptors.
- Optional binary hash/package-manifest validation.
- Explicit working directory.

## 6.6 Child state machine

```text
DISABLED
WAITING_FOR_DEPENDENCY
STARTING
SPAWNED
ATTACHED
OPERATIONAL
STOPPING
BACKOFF
FAILED_PERMANENTLY
```

Definitions:

```text
SPAWNED:
    process creation succeeded.

ATTACHED:
    child opened and validated VFRM.

OPERATIONAL:
    child consumed frames and submitted radio transfers.
```

## 6.7 Restart behavior

- Restart on non-zero exit or signal failure.
- Exponential bounded backoff.
- Restart-window limit.
- Child failure does not stop hardware encoding.
- VFRM eventually drops instead of blocking.
- Restarted child validates current producer generation.
- Recovery occurs at a valid codec boundary.

## 6.8 Shutdown order

```text
1. Mark producer STOPPING or stop accepting new publication.
2. Send SIGTERM to children in reverse startup order.
3. Wait up to configured timeout.
4. Send SIGKILL only after timeout.
5. Reap every child.
6. Stop VENC stream acquisition.
7. Release VENC/VPSS/ISP/VI/MIPI in safe reverse order.
8. Update/unmap VFRM according to the ABI lifecycle.
```

No zombie process is acceptable.

---

## 7. IMX307 720p55 enablement

## 7.1 Required artifacts

From `tipoman9/imx307_modes`:

```text
imx307_2l_cmos.c
imx307_2l_sensor_ctl.c
imx307_fps.ini
libsns_imx307_2l.so
```

Target paths used by the reference integration:

```text
/usr/lib/sensors/libsns_imx307_2l.so
/etc/sensors/imx307_fps.ini
```

## 7.2 Critical configuration rule

The high-FPS behavior is implemented by the modified sensor plugin/driver. It is not enabled simply by setting application FPS to 55.

The reference specifically keeps:

```ini
Isp_FrameRate = 30
```

The historical Majestic example also leaves application `fps: 30`.

The new GK7205 backend shall preserve the sensor plugin's required ISP contract and independently measure:

```text
sensor timing rate
VI/ISP accepted rate
VENC input rate
complete H.265 AU output rate
```

Only the final rate proves `frame-shm://` receives approximately 55 frames/s.

## 7.3 Reference sensor configuration

```ini
[sensor]
Sensor_type = stSnsImx307_2l_Obj
Mode = 0
DllFile = /usr/lib/sensors/libsns_imx307_2l.so

[mode]
input_mode = 0
raw_bitness = 12

[mipi]
data_type = 2
lane_id = 0|2|-1|-1|-1|-1|-1|-1|

[isp_image]
Isp_x = 0
Isp_y = 0
Isp_FrameRate = 30
Isp_Bayer = 0

[vi_dev]
InputDataType = 1
DataRev = FALSE
DevRect_x = 200
DevRect_y = 20
DevRect_w = 1280
DevRect_h = 720
```

The actual checked-out file is authoritative. Do not assume this excerpt replaces source inspection.

## 7.4 Checkout and pin

```bash
cd "$HOME/work/gk7205-waybeam"
git clone https://github.com/tipoman9/imx307_modes.git imx307_modes
git -C imx307_modes rev-parse HEAD \
  | tee manifests/imx307-modes-revision.txt
```

## 7.5 Safe target backup

```bash
ssh root@CAMERA_IP '
  mkdir -p /tmp/highfps-backup
  cp -a /usr/lib/sensors/libsns_imx307_2l.so \
        /tmp/highfps-backup/libsns_imx307_2l.so.stock
  cp -a /etc/sensors/imx307_fps.ini \
        /tmp/highfps-backup/imx307_fps.ini.stock 2>/dev/null || true
  sha256sum /tmp/highfps-backup/*
'

mkdir -p captures/highfps-backup
scp -O root@CAMERA_IP:/tmp/highfps-backup/* captures/highfps-backup/
sha256sum captures/highfps-backup/* \
  > manifests/highfps-stock-backup.sha256
```

## 7.6 ABI verification before deployment

```bash
file imx307_modes/libsns_imx307_2l.so
readelf -h imx307_modes/libsns_imx307_2l.so
readelf -d imx307_modes/libsns_imx307_2l.so | grep NEEDED
```

Compare with the stock target plugin. Do not overwrite until architecture, interpreter, and dependencies are compatible.

## 7.7 Reproducible rebuild

Use the matching OpenIPC firmware/HiSilicon sensor-library tree.

The historical reference path is:

```text
output/build/hisilicon-opensdk-master/libraries/sensor/
gk7205v200/sony_imx307_2L/
```

Copy:

```text
imx307_2l_cmos.c
imx307_2l_sensor_ctl.c
```

Then rebuild the sensor library using the pinned OpenIPC build environment. Locate the current equivalent directory instead of assuming the historical path is unchanged.

Record:

- Firmware commit.
- hisilicon-opensdk commit.
- Original and patched source hashes.
- Compiler version.
- Resulting `.so` hash.
- ELF interpreter and dependencies.

## 7.8 Acceptance evidence

The high-FPS mode is accepted only when:

```text
sensor plugin loads
no repeated I2C/MIPI errors
VI/ISP/VPSS remains stable
valid H.265 1280×720 AUs are produced
complete-AU output is approximately 55 fps
no duplicate/stale frame is counted as new output
independent HEVC decode succeeds
CPU/MMZ/temperature remain stable
T0→T1 is measured
```

The evidence file must include:

```text
board ID
firmware and source commits
plugin/config hashes
exact command
warm-up duration
sample duration
complete-AU count
calculated FPS
frame-size distribution
largest AU/IDR
CPU and temperature
errors
verdict
rollback result
```

---

## 8. Development environment

## 8.1 Host packages

```bash
sudo apt-get update
sudo apt-get install -y \
    build-essential git git-lfs cmake ninja-build make pkg-config \
    wget curl rsync unzip tar xz-utils python3 python3-venv jq \
    libusb-1.0-0-dev gdb-multiarch strace tcpdump socat
```

## 8.2 Workspace

```bash
mkdir -p "$HOME/work/gk7205-waybeam"
cd "$HOME/work/gk7205-waybeam"
```

```text
gk7205-waybeam/
├── waybeam/
├── research/
├── waybeam-link/
├── imx307_modes/
├── toolchains/
├── sysroots/
├── deploy/
├── captures/
├── measurements/
├── manifests/
├── memory-bank/
└── .github/
```

## 8.3 Checkout

```bash
git clone --recursive https://github.com/snokvist/waybeam_venc.git waybeam
git clone --recursive https://github.com/OpenIPC/research.git research
git clone --recursive https://github.com/snokvist/waybeam-link.git waybeam-link
git clone https://github.com/tipoman9/imx307_modes.git imx307_modes
```

Record revisions:

```bash
for repo in waybeam research waybeam-link imx307_modes; do
  echo "=== $repo ==="
  git -C "$repo" remote -v
  git -C "$repo" rev-parse HEAD
  git -C "$repo" submodule status --recursive || true
done | tee manifests/source-revisions.txt
```

---

## 9. GK7205V200 toolchain and sysroot

## 9.1 Bootstrap reference toolchain

```bash
cd "$HOME/work/gk7205-waybeam/research"
./build.sh venc-goke
```

Expected concepts from the build script:

```text
toolchain: toolchain.goke-gk7205v200
compiler:  arm-linux-gcc
vendor package: goke-osdrv-gk7205v200
```

Verify:

```bash
TC="$HOME/work/gk7205-waybeam/research/toolchain/toolchain.goke-gk7205v200"
"$TC/bin/arm-linux-gcc" --version
"$TC/bin/arm-linux-gcc" -dumpmachine
file "$HOME/work/gk7205-waybeam/research/venc/venc"
readelf -h "$HOME/work/gk7205-waybeam/research/venc/venc"
readelf -l "$HOME/work/gk7205-waybeam/research/venc/venc" \
  | grep 'Requesting program interpreter'
```

//build for gk7205 waybeam-link
cmake --preset gk7205v200-au
cmake --build --preset gk7205v200-au -j"$(nproc)"

## 9.2 Sysroot policy

Create a controlled sysroot:

```text
sysroots/gk7205v200/
├── include/
├── lib/
└── usr/
    ├── include/
    └── lib/
```

Track every binary library and header source. Hash all artifacts. Do not silently copy arbitrary files from a live device into a production sysroot.

---

## 10. Baseline board qualification
nothing
---

## 11. Implementation phases

## Phase 0: repository instructions and Memory Bank

Deliverables:

- `.github/copilot-instructions.md`
- Path-specific instruction files.
- Prompt files.
- `AGENTS.md`.
- Six core Memory Bank files.
- Initial ADRs and contracts.
- Pinned source/toolchain manifest.

Exit:

- Copilot reads project memory automatically.
- Architecture invariants are recorded once, without addenda.

## Phase 1: prove reference media pipeline

Implement no architecture changes yet.

Exit:

- Exact board produces stable H.265 through the vendor pipeline.
- Start/stop behavior is documented.

## Phase 2: prove IMX307 720p55

Exit:

- Approximately 55 complete H.265 AUs/s measured at VENC output.
- The result is recorded as `CONFIRMED` or rejected with evidence.

## Phase 3: add GK7205 build target and backend skeleton

Tasks:

- Add SOC/build selection.
- Register/probe backend.
- Define capabilities.
- Link required Goke libraries.
- Keep existing backends building.

Exit:

- GK7205 target compiles and starts backend initialization.

## Phase 4: port the proven media setup

Port in vertical slices:

```text
SYS/VB
→ sensor/MIPI/VI
→ ISP
→ VPSS
→ VENC
→ GetStream/ReleaseStream
```

Exit:

- Waybeam-based `venc` reproduces reference H.265 output.

## Phase 5: complete-AU normalization

Tasks:

- Validate vendor pack count, offset, length, and total size.
- Handle by-frame/by-slice behavior.
- Produce exactly one complete Annex-B AU per frame.
- Derive PTS, sequence, codec, IDR/reference/droppable flags.
- Reject malformed/oversized data.

Exit:

- Independent HEVC decoder accepts bounded captures.

## Phase 6: VFRM producer and READY contract

Tasks:

- Reuse Waybeam VFRM implementation.
- Configure `frame-shm://venc0`.
- Select bounded slots and measured slot size.
- Add producer state and readiness semantics if not already present.
- Add full-ring, oversize, stale-generation, and drop counters.

Exit:

- Minimal VFRM consumer receives complete valid AUs.
- Paused/killed consumer does not block VENC.

## Phase 7: generic child supervisor

Tasks:

- Add config schema.
- Validate executable and argv.
- Implement dependency gate.
- Implement `posix_spawn` adapter.
- Implement PID registry and reaping.
- Implement restart/backoff.
- Implement orderly shutdown.
- Add host tests with a dummy child.

Exit:

- Dummy child starts only after simulated dependency READY.
- Exit/restart/stop/zombie tests pass.

## Phase 8: cross-build and spawn `waybeam-link tx`

Initial feature profile:

```text
one RTL8812AU
fixed channel
20 MHz initially
long GI initially
fixed conservative MCS
fixed TX power
FEC disabled for baseline
ARQ disabled
adaptive bitrate disabled
telemetry/audio disabled
```

Exit:

- `venc` spawns `waybeam-link tx` after VFRM READY.
- Child validates and consumes VFRM.
- First RF submissions occur.
- No UDP/RTP or legacy `wfb_tx` path is active.

## Phase 9: low-rate control plane

Implement bounded local encoder control.

Exit:

- Bitrate and IDR requests work without pipeline restart.
- Malformed/blocked client cannot stall the frame path.

## Phase 10: performance optimization

Order:

1. No-FEC baseline.
2. Fixed bounded FEC.
3. Profile FEC and USB costs.
4. Remove allocations/logging.
5. Preallocate all hot-path resources.
6. Consider NEON/USB aggregation only from evidence.
7. Evaluate GDR/intra-refresh if supported.

Exit gates:

```text
T3−T2 P99 ≤0.2 ms
T4−T1 P99 ≤2.0 ms
T4−T0 P99 ≤22 ms
average CPU ≤65%
normal VFRM depth ≤1 AU
```

## Phase 11: robustness and production packaging

Tests:

- Child crash/restart.
- `SIGSTOP` consumer stall.
- USB unplug/reset.
- Stale VFRM.
- Oversized IDR.
- High motion/noise.
- Thermal soak.
- Power disturbance.
- NOR image-size audit.

---

## 12. CPU and memory design

### CPU rules

- No per-frame allocation.
- No per-frame storage logging.
- Avoid unnecessary worker threads.
- Use monotonic timestamps.
- Introduce real-time scheduling only after tracing proves the need.
- Keep control and metrics lower priority than VENC dequeue and TX hot path.

### Memory rules

- Bounded VFRM slots.
- Slot size based on measured maximum AU, not average bitrate.
- Preallocated symbol/FEC/USB buffers.
- Bounded RAM trace ring.
- No recording or continuous trace to NOR.

Initial assumption:

```text
VFRM slots: 3, pending ABI and measured AU analysis
normal depth: 0–1
```

---

## 13. QA plan

### 13.1 Functional

- Sensor and media pipeline start reliably.
- Complete valid AUs are emitted.
- Child starts after VFRM READY.
- RTL8812AU transmits.

### 13.2 Startup-order evidence

Record:

```text
VFRM READY timestamp
spawn timestamp
child attach timestamp
first AU consume timestamp
first USB submit timestamp
```

Required ordering:

```text
VFRM_READY ≤ SPAWN ≤ ATTACH ≤ FIRST_CONSUME ≤ FIRST_USB_SUBMIT
```

### 13.3 Failure tests

- Spawn failure.
- Child exits nonzero.
- Child terminated by signal.
- Restart storm.
- Child hangs during shutdown.
- Config malformed.
- Binary missing/non-executable.
- Unrelated FD leakage.
- Zombie-process scan.

### 13.4 Performance evidence

Collect:

```text
AU size
T0–T4
VFRM depth
FEC duration
symbol counts
USB submission latency
USB completion rate
per-thread CPU
context switches
IRQ/softirq
frequency/temperature
drops/failures
```

Report P50/P90/P95/P99/P99.9/max and sample count.

---

## 14. GitHub Copilot instructions and Memory Bank

## 14.1 Required repository files

```text
.github/
├── copilot-instructions.md
├── instructions/
│   ├── c-code.instructions.md
│   └── memory-bank.instructions.md
└── prompts/
    ├── initialize-memory-bank.prompt.md
    ├── update-memory-bank.prompt.md
    ├── implement-task-with-memory.prompt.md
    └── record-target-evidence.prompt.md

AGENTS.md

memory-bank/
├── projectbrief.md
├── productContext.md
├── systemPatterns.md
├── techContext.md
├── activeContext.md
├── progress.md
├── decisions/
├── integrations/
├── contracts/
├── evidence/
├── testing/
└── manifests/
```

## 14.2 Mandatory Copilot startup behavior

Before each task Copilot shall:

1. Read all six core Memory Bank files.
2. Read relevant ADRs, contracts, integrations, and evidence.
3. Inspect current Git state and pinned revisions.
4. Compare memory claims with source and evidence.
5. Report conflicts instead of choosing silently.

## 14.3 Source-of-truth hierarchy

```text
local reproducible target evidence
> current source/build behavior
> accepted ADRs/contracts
> approved architecture plan
> upstream reference documentation
> assumptions/hypotheses
> chat recollection
```

## 14.4 Evidence classifications

```text
CONFIRMED
REFERENCE
DECISION
ASSUMPTION
HYPOTHESIS
REJECTED
```

Upstream 720p55 remains `REFERENCE` until locally measured.

## 14.5 Update behavior

After significant work Copilot must update at least:

```text
memory-bank/activeContext.md
memory-bank/progress.md
```

Update affected decision, integration, contract, manifest, and evidence files. Preserve history; do not silently overwrite disproven conclusions.

## 14.6 Required ADRs

- Two-process VFRM architecture.
- `venc` as entry/child supervisor.
- Complete-AU VFRM IPC.
- Non-blocking overload policy.
- Single-thread TX hot path.
- IMX307 720p55 plugin strategy.

---

## 15. Risks and mitigation

### Risk: sensor timing claim does not reach VENC output

Measure complete AUs, not configuration value.

### Risk: single-core saturation

Start without FEC/adaptation; profile; bound all work.

### Risk: whole-frame latency too high

Measure T0→T1 first. Preserve VPSS low-delay setup. Treat early-slice transport as a separate future ADR.

### Risk: child supervisor becomes unsafe command runner

No shell, absolute paths, argv arrays, root-owned config, allowlists.

### Risk: `fork()` deadlock in multithreaded `venc`

Prefer `posix_spawn`; otherwise immediate async-signal-safe `fork()+execve()`.

### Risk: restart storm

Bounded exponential backoff and restart-window limit.

### Risk: VFRM ABI mismatch

Validate magic/version/sizes/geometry; fail closed; pin commits.

### Risk: 16 MiB NOR overflow

Strip builds, compressed rootfs, only required libs/configs, no debug tools or continuous traces.

### Risk: RTL8812AU USB/power instability

Validate VID:PID/backend/capabilities; provide stable power; track resets and TX failures.

---

## 16. Definition of done

### Architecture

- `venc` is Waybeam-based with GK7205 backend.
- `venc` is entry/supervisor.
- `waybeam-link tx` remains independent.
- VFRM is the only encoded-video IPC.
- Actual radio is RTL8812AU.

### Functional

- Stable 720p high-FPS media pipeline.
- Valid complete H.265 AUs.
- Child starts only after VFRM READY.
- Child consumes and transmits.

### Performance

```text
T3−T2 P99 ≤0.2 ms
T4−T1 P99 ≤2.0 ms
T4−T0 P99 ≤22 ms
CPU average ≤65%
VFRM normal occupancy ≤1 AU
```

### Reliability

- Child failure does not block encoder.
- Restart storm bounded.
- Shutdown reaps all children.
- ABI mismatch rejected.
- USB failures observable.

### Reproducibility

- Source/toolchain/sysroot/config/binary hashes recorded.
- Clean host can reproduce build.
- Copilot Memory Bank is current.

---

## 17. Immediate next actions

1. Install the GitHub Copilot instruction/Memory Bank kit at repository root.
2. Pin all repository revisions.
3. Run `research/build.sh venc-goke`.
4. Back up the current camera firmware and sensor artifacts.
5. Validate the reference media pipeline.
6. Validate the tipoman plugin ABI and 720p55 output rate.
7. Inspect current Waybeam backend and VFRM interfaces.
8. Write ADRs for the accepted architecture.
9. Add GK7205 build/backend skeleton.
10. Add child-supervisor schema and host tests.
11. Port media initialization in vertical slices.
12. Implement complete-AU normalization.
13. Implement VFRM READY semantics.
14. Spawn `waybeam-link tx` with FEC/adaptation disabled.
15. Instrument T0–T4 before optimization.

---

## 18. Assumptions requiring target confirmation

- Exact Waybeam source-file names for the GK7205 backend.
- Exact current Waybeam configuration keys.
- Exact waybeam-link configuration keys.
- Exact VFRM ABI fields already available upstream.
- Appropriate VFRM slot count and size.
- Runtime GDR/intra-refresh support in Goke VENC.
- Exact T0 timestamp source.
- Toolchain/libc compatibility for waybeam-link C++ dependencies.
- RTL8812AU backend capabilities on the exact card.
- Local achievement of 720p55.
- Local achievement of vehicle P99 ≤22 ms.
- Whole-system achievement of ≤30 ms.

No assumption shall be silently promoted to a confirmed project fact.

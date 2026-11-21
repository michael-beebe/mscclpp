# Native MRC QP and congestion-control hints

Build with `MSCCLPP_USE_MRC=ON` and select a native `MRC0`–`MRC7` transport.
Hints apply only to the native MRC host backend, not IB, IB-over-shim or GPUNetIO.
The environment is cached on the first call to `mscclpp::env()`; set these
variables before initializing MSCCL++.

| Variable | Default | Meaning |
|---|---:|---|
| `MSCCLPP_MRC_QP_HINT_ENABLE` | `0` | `1` creates a hint for every native MRC QP and attaches it during INIT. `0` skips both QP and CC hints. |
| `MSCCLPP_MRC_CC_INIT_RATE` | `0` | Initial CC rate, passed unchanged to each QP. |
| `MSCCLPP_MRC_CC_MIN_RATE` | `0` | Connection-wide minimum rate, divided by the connection's QP count with integer truncation. |
| `MSCCLPP_MRC_CC_MAX_RATE` | `0` | Maximum CC rate, passed unchanged to each QP. |

Rates are nonnegative decimal integers in provider-specific units, **not bits
per second**. Effective per-QP values must fit `uint32_t`; the provider may
impose tighter limits. NVIDIA's line-rate encoding is 1048576.

For example, set `MSCCLPP_MRC_QP_HINT_ENABLE=1`,
`MSCCLPP_MRC_CC_INIT_RATE=1048576`, `MSCCLPP_MRC_CC_MIN_RATE=65536` and
`MSCCLPP_MRC_CC_MAX_RATE=1048576` before starting the application.

The encoding matches the rebased native NCCL plugin: if any effective rate is
nonzero, a version-1 16-byte vendor payload contains the initial, minimum and
maximum rates with zero reserved bytes. With all rates zero, the entire vendor
configuration remains zero, preserving provider CC defaults while topology
hints are still enabled. With hints disabled, CC settings are not read or
validated.

Each current MSCCL++ connection owns one QP. Its hint advertises
`num_qps_per_peer=1` and `num_send_peers=1`, so MIN_RATE division is currently by
one. These counts describe the connection, not every connection in a process.

Hint creation or INIT attachment failures fail endpoint construction; they do
not silently discard the requested settings. Hints are released after their QP,
including construction failure cleanup. A failed QP destruction never frees
its attached hint. Enable `MSCCLPP_LOG_LEVEL=INFO` with the `ENV,NET` subsystems
to see effective options and per-QP hint rates.

Native MRC remains host-only, with WRITE and WRITE_WITH_IMM. Network atomics and
Device mode are unsupported. In the tested DOCA 3.2/GPUNetIO environment, the
previously validated SDK-first loading order is still required; hints do not
resolve the bundled/SDK symbol collision.
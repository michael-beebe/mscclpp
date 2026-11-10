# IB-only RDMA provider foundation

**Status:** IB-only provider foundation under review. Native MRC is a separate PR.

## Decision and justification

Keep `Connection` for I/O. Add an internal RDMA provider for **resource creation,
ownership, serialization, and connection construction**. Replace IB-specific
framework ownership; do not add a second cache or I/O layer.

Before this PR, provider responsibilities were spread across generic framework code:

| Existing owner/caller | IB-specific responsibility |
| --- | --- |
| `Context::Impl` (`src/core/include/context.hpp`) | `IbCtx` cache; `Context::connect()` constructs `IBConnection`. |
| `Endpoint::Impl` (`src/core/include/endpoint.hpp`) | QP ownership, IB settings and serialized QP metadata. |
| `RegisteredMemory::Impl` (`src/core/include/registered_memory.hpp`) | MR owning map, borrowed MR views and serialized metadata. |

Adding another provider directly repeats these branches and lifetime rules.
Extending `Connection` cannot cover independently created endpoints/MRs or
metadata import without a local NIC.

Previously, `Context` owned the device/PD; endpoints and memory owned QPs/MRs
without retaining it. Only connections retained `Context`. Local resources now
explicitly keep their provider alive: a lifetime invariant, not a reproduced
teardown-failure claim.

## Ownership structure

Each pair is one concrete object; existing NIC primitives remain unchanged.

| Internal interface / IB implementation | Responsibility |
| --- | --- |
| `RdmaProvider` / `IbProvider` | Own `IbCtx`; provide endpoint and memory factories; remain cached per transport. |
| `RdmaEndpoint` / `IbEndpoint` | Own a local QP or imported metadata; serialize it; construct the existing connection. |
| `RdmaMemory` / `IbMemory` | Own a local MR or imported metadata; serialize it. |

```{mermaid}
flowchart LR
    C[Context] --> B["IbProvider : RdmaProvider"]
    E[Endpoint] --> Q["IbEndpoint : RdmaEndpoint"]
    R[RegisteredMemory] --> M["IbMemory : RdmaMemory"]
    Q -->|retain when local| B
    M -->|retain when local| B
    B --> I["IbCtx / PD"]
    Q --> QP[IbQp]
    M --> MR[IbMr]
    Q -. factory .-> Conn[IBConnection]
```

Solid arrows denote ownership. Destroy QPs/MRs before releasing the provider.
Imported wrappers own metadata only; providers never own wrappers (no cycle).
User backing buffers remain non-owned.

Registered-memory records contain a transport tag, IPC metadata and an owning
`RdmaMemory` pointer, with no IB fields or header dependency. Factories bind each
RDMA transport to its concrete memory type; IB code checks the tag before a
`static_cast` to `IbMemory`. This adds a pointer dereference, not per-write RTTI
or a connection/MR cache. Per-record ownership preserves imported duplicate
transports and their order.

RDMA classification and compatibility live with provider dispatch. Different IB
device indices remain compatible; future provider families must define their own
compatibility rather than treating all RDMA transports as interchangeable.

**The data path stays unchanged:**

```{mermaid}
flowchart LR
    G[GPU kernel] --> F["PortChannel / FIFO"]
    F --> P[ProxyService]
    P --> C[Connection]
    C --> I[IBConnection]
    I --> Q[IbQp]
```

Factories and endpoint type checks run during setup. IB operations access typed
provider-owned MRs directly; preserve existing posting, signaling/forwarding,
CQ polling and flush behavior.
Do not add an unused `address()` API, a separate virtual endpoint `connect()`,
or a new virtual write/flush layer. QP transitions remain in connection construction.
Interfaces live in `src/core/include/`; concrete IB code stays in `ib.hpp`/`ib.cc`.

## Why move existing code, and what is actually new?

| Change | Justification and limit |
| --- | --- |
| Move `IBConnection` definitions from `connection.cc` to `ib.cc`. | Optional organization: the factory can construct a provider-local class. Separate relocation from ownership changes for review. |
| Share validators through `BaseConnection`. | IB, CUDA IPC and Ethernet already share them. Inline helpers span both files without duplication or mandatory out-of-line calls. |
| Replace context, endpoint and MR ownership with the three pairs. | Substantive new code, justified only if it removes direct provider ownership from generic code and supports the next provider. |
| Add focused tests. | Verify lifetime ordering, metadata and IB behavior without test-only production APIs. |

The first relocation commit changes three C++ files: **381 insertions, 372 removals,
net nine lines**. IB method bodies and validation logic are unchanged. Those
numbers describe only relocation, not the ownership migration.
New wrappers add setup allocations/shared ownership and larger transport records;
unchanged performance is not established.

## Alternatives

| Alternative | Tradeoff |
| --- | --- |
| Keep IB fields and add parallel MRC branches. | Smaller now, but duplicates ownership and metadata handling. Prefer this if the seam cannot remove that duplication. |
| Use a tagged union or `std::variant`. | Viable, but common code carries provider payloads, destruction and build guards; lifetime still needs solving. |
| Abstract all QP operations. | Duplicates the existing `Connection` seam and broadens the hot path. Not needed for this PR. |

## Compatibility and review gates

Preserve public headers/APIs, transport selection, IB mode/GID/device resolution,
queue limits, atomics fallback, signaling, async flush/error behavior, and CUDA
IPC/Ethernet behavior. Preserve serialized bytes, record ordering and imported
padding, including IB metadata deserialization in IB-disabled builds. New provider
dispatch uses `MSCCLPP_USE_IB`, mirrored from the existing CMake option. Existing
IB implementation guards remain unchanged; `MSCCLPP_USE_MRC` still selects the
legacy shim in this PR. Introduce no native flags, new environment variables, or
algorithm changes.

Review relocation, the independently compiling ownership/factory migration, then
the generic-memory and routing cleanup.

The migration passes standalone endpoint/MR lifetime tests (no connection retaining
`Context`), exact imported metadata/order roundtrips with IB on/off, and
IB-enabled/disabled and legacy-shim builds. XDR comparisons against the published
relocation pass six host pairs (including mixed revisions), GPU/proxy/PortChannel,
and CUDA IPC cases. This does **not** qualify shim runtime, full
collectives/performance, or native MRC.

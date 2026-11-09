# IB-only RDMA resource/factory foundation

**Status:** Proposed. Only the existing-code relocation is prepared;
the resource/factory migration below is not implemented. Native MRC is a separate PR.

## Decision and justification

Keep `Connection` for I/O. Add an internal RDMA seam for **resource creation,
ownership, serialization, and connection construction**. Replace IB-specific
framework ownership; do not add a second cache or I/O layer.

Today, provider responsibilities are spread across generic framework code:

| Existing owner/caller | IB-specific responsibility |
| --- | --- |
| `Context::Impl` (`src/core/include/context.hpp`) | `IbCtx` cache; `Context::connect()` constructs `IBConnection`. |
| `Endpoint::Impl` (`src/core/include/endpoint.hpp`) | QP ownership, IB settings and serialized QP metadata. |
| `RegisteredMemory::Impl` (`src/core/include/registered_memory.hpp`) | MR owning map, borrowed MR views and serialized metadata. |

Adding another provider directly repeats these branches and lifetime rules.
Extending `Connection` cannot cover independently created endpoints/MRs or
metadata import without a local NIC.

`Context` owns the device/PD; endpoints and memory own QPs/MRs without retaining
it. Only connections retain `Context`. Local resources must explicitly keep
their provider alive: a lifetime invariant to establish, not a reproduced
teardown-failure claim.

## Proposed structure

Each pair is one concrete object; existing NIC primitives remain unchanged.

| Internal interface / IB implementation | Responsibility |
| --- | --- |
| `RdmaBackend` / `IbBackend` | Own `IbCtx`; provide endpoint and memory factories; remain cached per transport. |
| `RdmaEndpoint` / `IbEndpoint` | Own a local QP or imported metadata; serialize it; construct the existing connection. |
| `RdmaMemory` / `IbMemory` | Own a local MR or imported metadata; serialize it. |

```{mermaid}
flowchart LR
    C[Context] --> B["IbBackend : RdmaBackend"]
    E[Endpoint] --> Q["IbEndpoint : RdmaEndpoint"]
    R[RegisteredMemory] --> M["IbMemory : RdmaMemory"]
    Q -->|retain when local| B
    M -->|retain when local| B
    B --> I["IbCtx / PD"]
    Q --> QP[IbQp]
    M --> MR[IbMr]
    Q -. factory .-> Conn[IBConnection]
```

Solid arrows denote ownership. Destroy QPs/MRs before releasing the backend.
Imported wrappers own metadata only; backends never own wrappers (no cycle).
User backing buffers remain non-owned.

**The data path stays unchanged:**

```{mermaid}
flowchart LR
    G[GPU kernel] --> F["PortChannel / FIFO"]
    F --> P[ProxyService]
    P --> C[Connection]
    C --> I[IBConnection]
    I --> Q[IbQp]
```

Factories and new provider type checks run during setup. Preserve typed MR views
and existing IB posting, signaling/forwarding, CQ polling and flush behavior.
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

The prepared relocation changes three files: **381 insertions, 372 removals,
net nine lines**. IB method bodies and validation logic are unchanged. Those
numbers describe only relocation, not the still-proposed ownership migration.
New wrappers cost setup allocations/shared ownership; unchanged performance is
not established.

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
padding, including IB metadata deserialization in IB-disabled builds. Keep
`USE_IBVERBS` guards and the existing legacy `MSCCLPP_USE_MRC` shim option;
introduce no native flags, new environment variables, or algorithm changes.

Review relocation, then the independently compiling ownership/factory migration.
Require standalone endpoint/MR lifetime tests (not merely dropping a context
retained by a connection), exact metadata roundtrips, IB-enabled/disabled and
legacy-shim builds, and matched-baseline real IB host and GPU/proxy/PortChannel cases.

The relocation has passed those build configurations and bounded host/GPU IB
comparisons. This does **not** qualify the proposed ownership migration, shim
runtime, full collectives/performance, or native MRC.

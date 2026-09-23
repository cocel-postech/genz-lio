# Patches applied to vendored dependencies

## IKFoM_toolkit

**`esekfom/esekfom.hpp` — stateful observation model.**

Upstream declares the measurement model as a plain function type:

```cpp
typedef void measurementModel_dyn_share(state &, dyn_share_datastruct<scalar_type> &);
```

A function pointer cannot carry state, which is why the reference implementation
kept the map, the current scan and every tuning parameter in global variables.
The typedef is now a `std::function` of the same signature, and the member that
holds it is a value rather than a pointer:

```cpp
typedef std::function<void(state &, dyn_share_datastruct<scalar_type> &)> measurementModel_dyn_share;
```

This lets `HybridMetricUpdate` bind itself as the observation model, so several
pipelines can run side by side — which the Python bindings need. Call sites are
unchanged: a plain function still converts implicitly.

Both GenZ-LIO and IKFoM are GPL-2.0, so the modification carries the same terms.

**`esekfom/esekfom.hpp` — diagonal update with fewer than 23 observations.**

The shared observation model supplies inverse variances in `dyn_share.R`.
The small-measurement branch now inverts them before adding measurement
covariance to `H P H^T`. The information-form branch already used inverse
variances correctly. Analytic scalar measurements with 1, 22, 23 and 24 rows
verify that adding zero-information rows does not change the posterior.

**`esekfom/reference_product.hpp` — reproducible observation reductions.**

The diagonal update uses an explicit local blocking policy for `H^T R^-1 H`
and `K H`. Eigen's automatic cache discovery can choose different reduction
orders on heterogeneous cores, even within one process/build. The reference
policy preserves the validated Eigen 3.4 SSE2 reduction: maximum depth 760,
with the same balanced final block. This is a compatibility constant derived
from the recorded reference, not a tunable estimator parameter or a claim
about the current hardware. It does not set CPU affinity or change Eigen's
global cache sizes. The other filter equations remain unchanged.

The helper calls Eigen 3.4's internal GEMM kernel, so the core now requires
Eigen >= 3.4 and kernel compatibility must be checked on dependency upgrades.
Tests cover both observation products, block boundaries, P/E cache settings,
reference equality, unchanged global cache state and a long-double oracle.
Do not enable fast-math for reproducibility builds. Different SIMD targets,
compilers and architectures still require accuracy validation; this patch
removes cache-dependent blocking, not every source of floating-point variation.
It also does not eliminate the estimator's sensitivity to different rounding
orders. Compensated and LDLT reformulations were evaluated separately and not
adopted because their full-sequence accuracy regressed with the frozen YAMLs.

## robin-map (tsl)

Unmodified.

## A note on comments

Both dependencies carry comments in their original languages. They are left as
they are: a vendored dependency should differ from upstream only where a patch
is documented above, so that the next update is a diff and not an archaeology
exercise.

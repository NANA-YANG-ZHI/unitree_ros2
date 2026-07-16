# Disturbance-observer strategies: `hg` vs `sliding` vs `mixing`

Companion notes for `compare_observer_strategies.py` /
`observer_strategies_comparison.png` in this folder. These three modes are
the `alg` options of `err_mapping_func` in
`example/src/src/contact_estimation/contact_detection.py`.

## Side-by-side summary

| | High-gain (`hg`) | Sliding mode (`sliding`) | Mixing (`mixing`) |
|---|---|---|---|
| Injection | `err1 = err2 = k_hg·err` | `err1 = \|k_sliding·err\|^(1/2)·sign(err)`, `err2 = sign(err)` | `err1 = q(err) = k_sliding·sign(err)\|err\|^(1/2) + k_hg·err`, `err2 = sign(err) + q(err)` |
| Convergence | exponential (asymptotic, never exact) | finite-time (exact) | finite-time-like near 0, bounded response for large error |
| Noise sensitivity | grows with `bandwidth`, but stays smooth | severe (chattering) | still chatters (retains `sign(err)`), but bounded large-error response |
| Math toolkit for proof | eigenvalues / `e^{λt}` (linear ODE theory) | Lyapunov / homogeneity arguments | Lyapunov (Andrieu, Astolfi & Bernard 2021, cited in the paper) |
| Tuning knob | `bandwidth → L1=2ω, L2=ω²` | `k_sliding` | `k_hg` and `k_sliding` independently |
| Analogy | careful accountant — smooth, but slower to close the last bit of the gap | reflex — reacts fast, but jittery | reflex near the target, careful when far off |

## Math reasoning: why `sliding`/`mixing` converge faster

### The numeric intuition

Compare the two correction magnitudes, linear `e` vs. sliding `√e`, as the
error shrinks:

| error `e` | linear term `e` | sliding term `√e` | ratio `√e / e` |
|---|---|---|---|
| 1.0 | 1.0 | 1.0 | 1× |
| 0.1 | 0.1 | 0.316 | 3.2× |
| 0.01 | 0.01 | 0.1 | 10× |
| 0.0001 | 0.0001 | 0.01 | 100× |

As the error gets smaller, `√e` shrinks far more slowly than `e`. Right
where the linear (`hg`) observer's correction is running out of steam
(proportional to the tiny remaining error), the sliding term is still
pushing relatively hard.

### Solving the two error ODEs directly

**Linear (`hg`) case**: `ė = -L·e`. Separate variables:

```
de/e = -L dt   ⟹   ln(e) = -Lt + C   ⟹   e(t) = e(0)·e^{-Lt}
```

Exponential decay: every fixed time interval reduces the remaining error by
the same *fraction*. `e(t)` approaches zero but never reaches it for any
finite `t` — there's always a nonzero remainder.

**Sliding case**: `ė = -k·√e` (for `e>0`). Separate variables:

```
de/√e = -k dt   ⟹   2√e = -kt + C   ⟹   √e(t) = √e(0) − (k/2)t
```

`√e(t)` decreases *linearly* — a straight ramp to zero. A straight line
starting positive hits exactly zero at a finite time:

```
t_finite = 2·√e(0) / k
```

and `e` stays exactly 0 afterward. This is what "finite-time convergence"
formally means: not just "faster," but a qualitatively different curve
shape (a closing ramp) instead of an ever-approaching, never-arriving
exponential.

### Why `mixing` inherits the speed, but not full noise robustness

`mixing`'s injection is `q(e) = k_sliding·√e + k_hg·e`.

- **Near convergence** (`e` small): `√e ≫ e`, so the sliding term
  dominates the sum — `mixing` behaves almost like pure `sliding` exactly
  where it matters for closing the final gap. That's why it converges as
  fast as (or faster than) `sliding` in the plot.
- **Far from convergence** (`e` large, e.g. right after a touchdown step):
  `e ≫ √e`, so the linear term dominates instead, giving a proportionally
  scaled, well-behaved push rather than letting the sliding term produce an
  overly aggressive jump.

Caveat confirmed by the actual plot: since `mixing` still contains the
discontinuous `sign(err)` piece, it does **not** meaningfully reduce
chattering under measurement noise compared to pure `sliding` — the two
look comparable in the noisy zoom panel. `mixing`'s real advantage is a
bounded, well-scaled response for *large* errors plus fast closing of
*small* ones — not reduced high-frequency jitter. Cutting chattering
further would require smoothing (e.g. lowering `k` in the `tanh(k·x)`
approximation of `sign`), at the cost of losing the exact finite-time
property.

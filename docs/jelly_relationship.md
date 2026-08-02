# Relationship to Jelly

CoSoRoMotion was motivated by, and is designed to be conceptually
compatible with, [Jelly](https://piepustina.github.io/Jelly/), a MATLAB
and Simulink library for modular soft-rigid robot dynamics. This
document is explicit about what "compatible with" does and does not
mean, because getting this wrong has real legal and technical
consequences for an open-source project.

## What CoSoRoMotion is NOT

- **Not a port of Jelly's source code.** No MATLAB or Simulink source
  from Jelly has been read, copied, translated, or referenced line-by-
  line anywhere in this codebase. Every file in `src/cosoromotion/` is
  an original implementation.
- **Not a fork.** This is a separate project with its own architecture,
  API, and design decisions.
- **Not license-cleared for redistribution alongside Jelly.** At the
  time of writing, we were unable to confirm Jelly's license from its
  public documentation and GitHub search results. **Before combining
  CoSoRoMotion with any Jelly-derived artifact (code, generated C++,
  exported model files), check `https://github.com/piepustina/Jelly`
  directly for a LICENSE file and confirm what it permits.** This
  project does not assume permissive licensing on Jelly's behalf.

## What CoSoRoMotion IS

- **An independent implementation of the same general modeling
  approach** Jelly is built on: Geometric Variable Strain (GVS)
  parameterization of soft-robot bodies, following the published theory
  in Boyer, Lebastard, Candelier, and Renda's 2020 IEEE Transactions on
  Robotics paper on strain-parameterization-based dynamics of continuum
  and soft robots, and informed by Pustina, Della Santina, and De Luca's
  work on unified/recursive inverse dynamics for modular serial soft-
  rigid mechanical systems (the algorithm Jelly itself is based on, per
  its own documentation). Published algorithms and mathematical methods
  are not owned by any single implementation of them; implementing the
  same published theory independently, in a different language, with a
  different codebase, is standard practice and not a licensing concern.
- **API-inspired, not code-derived.** CoSoRoMotion's naming (a
  `BodyTree`-style mental model, strain-parameterized bodies) echoes
  Jelly's public API concepts, deliberately, so that someone familiar
  with one can read the other -- similar to how many linear algebra
  libraries share MATLAB-like naming conventions without copying MATLAB
  code. This is a design choice about developer ergonomics, not a claim
  of code lineage.
- **Python/C++, not MATLAB/Simulink.** Jelly generates C/C++ from MATLAB
  via MATLAB Coder (a paid MATLAB toolbox); CoSoRoMotion is written
  directly in Python (with a C++ acceleration path planned, see
  `docs/roadmap.md`) and requires no MATLAB license to use, build, or
  extend.

## A possible future bridge (not yet built)

If you have MATLAB and a confirmed-permissive Jelly license, a natural
integration would be a **data-level bridge**: exporting a Jelly
`BodyTree`'s parameters (segment lengths, radii, material properties,
strain-basis choice) to a plain JSON/YAML config that CoSoRoMotion can
load, enabling cross-validation between Jelly's MATLAB ground truth and
CoSoRoMotion's Python implementation on the same nominal robot. This
would involve no code sharing at all -- only a shared parameter schema
-- and is a reasonable v0.2+ roadmap item if there's contributor
interest and the licensing question is resolved. It is explicitly not
implemented in v0.1.

## Citations

See `paper/references.bib` (once written) or the module docstrings in
`src/cosoromotion/gvs/rod.py` for full citations to the published papers
this project's modeling approach is informed by.

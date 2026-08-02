"""CoSoRoMotion: Continuum Soft Robot Motion Planning via Optimal
Transport, built on an independent Geometric Variable Strain (GVS)
dynamics core.

Relationship to Jelly (https://piepustina.github.io/Jelly/): CoSoRoMotion
is NOT a port, fork, or derivative of Jelly's MATLAB/Simulink source
code. It is an independent Python/C++ implementation of the same general
class of strain-parameterized soft-robot models described in the
published literature Jelly also builds on (Boyer et al. 2020 IEEE T-RO;
Pustina, Della Santina, De Luca). API naming (BodyTree-style concepts)
is intentionally similar so models are conceptually portable, but no
Jelly code is reproduced anywhere in this package. See docs/jelly_relationship.md.
"""
__version__ = "0.1.0"

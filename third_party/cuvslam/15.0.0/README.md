# cuVSLAM 15.0.0 runtime files

This directory contains the headers and runtime libraries required by
RTAB-Map's cuVSLAM odometry integration. They come from NVIDIA's official
`v15.0.0` release archive:

- Release: https://github.com/nvidia-isaac/cuVSLAM/releases/tag/v15.0.0
- Asset: `cuvslam_cpp.tar.bz2`
- SHA-256: `fbbb611194fa1a848eedd14a7ad6003836c0c7e1d5eb3a997cde6460f2a62192`

Checksums for the extracted runtime libraries are recorded in `SHA256SUMS`.

The bundled matrix covers CUDA 12 and 13, Ubuntu 22.04 and 24.04, and the
`aarch64` and `x86_64` architectures. CMake selects the matching runtime at
configuration time. Documentation and `cuvslam_api_launcher` executables from
the upstream archive are intentionally omitted because RTAB-Map does not need
them at build time or runtime.

cuVSLAM is distributed under the NVIDIA Community License included in
`LICENSE`. In particular, use and redistribution are limited to NVIDIA
platforms as defined by that license.

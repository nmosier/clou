# Clou
Clou is a static analysis tool for detecting and mitigating Spectre vulnerabilities in programs.
Clou is implemented as a custom IR pass in LLVM.
It takes a C source file as input, compiles it to LLVM IR using Clang 12, and analyzes each defined function one-by-one.
Eventually, Clou outputs a list of transmitters and a set of consistent candidate executions that give witness to detected Spectre vulnerabilities.
Clou is optiimzes to detect universal data transmitters, but it can identify other kinds of transmitters as well.

If you're interested in the theoretical foundation and implementation details of Clou, see our [ISCA'21 paper](https://doi.org/10.48550/arXiv.2112.10511).

# Installation
TODO

# Usage
TODO

# Reproducing Results from the Paper
TODO

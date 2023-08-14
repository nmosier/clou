# Clou
Clou is a static analysis tool for detecting and mitigating Spectre vulnerabilities in programs.
Clou is implemented as a custom IR pass in LLVM.
It takes a C source file as input, compiles it to LLVM IR using Clang 12, and analyzes each defined function one-by-one.
Eventually, Clou outputs a list of transmitters and a set of consistent candidate executions that give witness to detected Spectre vulnerabilities.
Clou is optiimzes to detect universal data transmitters, but it can identify other kinds of transmitters as well.

If you're interested in the theoretical foundation -- leakage containment models -- and implementation details of Clou, see our [ISCA'22 paper](https://doi.org/10.1145/3470496.3527412).

# Installation

Note: in the following instructions, replace `$CLOU_REPO` with the absolute path to this repository or set the environment variable to be the absolute path to this repository.

## Docker
1. cd into root directory of this repo: `cd $CLOU_REPO`
1. Build the docker image using our wrapper script: `./docker-build.sh clou`. This will probably take 5-10 minutes.
2. Run an instance of the built image with: `./docker-run.sh clou`
4. Compile Clou: 
```sh
cmake -DCMAKE_BUILD_TYPE=RelWithDebInfo ..
make -j$(nproc)
```

## Local system
TODO

# Usage
TODO

# Interpreting Results
TODO: link to clou-out explanation

# Reproducing Results from the Paper

This section describes how to reproduce the results presented in Table 2 of the paper.
All commands should be run _within_ the Clou docker container.

For each benchmark (i.e., multi-row in the paper), we run Clou multiple times, with different combinations of Spectre detector (Clou-PHT for Spectre v1 or Clou-STL for Spectre v4) and transmitter class (dt, ct, udt, uct).
The following is a breakdown of the various benchmarks, including the different parameter combinations in cartesian-product notation.

We evaluate 4 litmus test suites in the paper:
- pht (Spectre v1 / bounds check bypass) - {v1} x {dt,ct,udt,uct}
- stl (Spectre v4 / speculative store forwarding) - {v4} x {dt,ct,udt,uct}
- fwd (Spectre v1.1 / bounds check bypass write) - {v1,v4} x {dt,ct,udt,uct}
- new (new variant of Spectre v1.1) - {v1,v4} x {dt,ct,udt,uct}

and 4 crypto benchmarks in the paper:
- [tea](https://en.wikipedia.org/wiki/Tiny_Encryption_Algorithm) - {v1,v4} x {udt,uct}
- [donna](http://code.google.com/p/curve25519-donna/) - {v1,v4} x {udt,uct}
- [secretbox](https://github.com/jedisct1/libsodium/tree/master/src/libsodium/crypto_secretbox) - {v1,v4} x {udt,uct}
- ssl3-digest - {v1,v4} x {udt,uct}
- mee-cbc - {v1,v4} x {udt,uct}

and 2 crypto libraries in the paper:
- [libsodium](https://libsodium.org) - {v1,v4} x {udt,uct}
- [OpenSSL](https://openssl.org) - {v1,v4} x {udt}

## Running the Tests

To run a single test for a single benchmark, use the script `/clou/scripts/run-bench.sh`.
The general usage to run benchmark `BENCH` with detector `TYPE` and transmitter class `XMIT` is:
```bash
$ /clou/scripts/run-bench.sh -t TYPE -x XMIT BENCH
```
The output directory containing all analysis results is by default stored in a directory named `$BENCH-$TYPE-$XMIT`.

For example, to run a single test for benchmark `pht` looking for Spectre v1 leakage and universal data transmitters:
```bash
$ /clou/scripts/run-bench.sh -t v1 -x udt pht
```
To run all tests for benchmark `fwd`:
```bash
for TYPE in v1 v4; do
  for XMIT in dt ct udt uct; do
    /clou/scripts/run-bench -t $TYPE -x $XMIT fwd
  done
done
```

For full usage of the `run-bench.sh` script, run 
```bash
$ /clou/scripts/run-bench.sh -h
```

For the larger, non-litmus benchmarks (tea, donna, secretbox, ssl3-digest, mee-cbc, libsodium, OpenSSL), `run-bench.sh` will spawn an interactive tmux window with two panes -- one for Clou's compilation/analysis job, and one for Clou's monitor, which shows which functions Clou is currently analyzing. 
Once the main analysis job is done, you'll have to manually kill the monitor job (e.g., Ctrl+C then Ctrl+D).

## Collecting Results
Use the script `/clou/scripts/table.sh` to generate a table of results.
To collect results for all benchmarks, where the output (all output directories of the form `$BENCH-$TYPE-$XMIT-out`) is in directory `$OUTDIR`, use the following command:
```bash
/clou/scripts/table.sh -d $OUTDIR -a
```
The script will emit warnings for any missing results (e.g., if you forgot to run benchmark `pht` with `TYPE=v1` and `XMIT=uct`) and will produce question marks in the table for those entries.

# Citing Clou
If you use Clou in your work, we would appreciate it if you cite our paper ([bibtex](cite.bib)):
> N. Mosier, H. Lachnitt, H. Nemati, C. Trippel, “Axiomatic Hardware-Software Contracts for Security,” in 2022 ACM/IEEE 49th Annual International Symposium on Computer Architecture (ISCA). IEEE, 2022.

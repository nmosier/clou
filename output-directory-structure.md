# Clou's Output Directory Structure

- `lkg/`: directory that contains all leakage patterns discovered
  - `<function>/`: directory that contains all leakage patterns discovered in `<function>`
    - `<leakage>.txt`: leakage pattern, reproduced with relevant source code when available
  - ...
- `transmitters.txt`: set of all transmitters found by Clou. 
  Note that this may contain repetitions. 
  Don't use this for counting the number of discovered transmitters -- use `scripts/transmitters.sh`.
- `runtimes.txt`: contains a list of runtimes for each source file (not labeled, however; simply used for computing total runtime in the script `scripts/runtime.sh`.
- `logs/`: directory containing all log outputs for each function and compilation process.

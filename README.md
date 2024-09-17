# Solver for the Single Assignment Hub Location Problem
This is a re-implementation of the Benders' Decomposition-based solver proposed by [Zetina et al.](https://github.com/czet88/Benders_4_Quadratic_Facility_Location) for Hub Location Problems.
This re-implementation was created as part of a collaboration between the Chair of Operations Research at RWTH Aachen and DHL Data & Analytics where we researched various Hub Location Problems.
It is implemented in the `SCIP` framework and uses either `CPLEX`, `MCFClass` or `SOPLEX` to solve the Benders' subproblems and optionally the LPs in the branch-and-bound tree.
We hope that this implementation may offer a starting point for others working on similar problems or just another option to try when solving large scale Single Allocation Hub Location Problems.
Some experiments indicated that the performance is somewhat worse than that of the reference implementation by Zetina et al., but it is certainly more memory-efficient and easier to extend for those with `SCIP` experience.
It supports various input formats: instances can be passed via two different file formats (described further down) or passed directly via a `C`, `C++` or `Python` interface.


# Installation
You can either build the solver directly or in a docker container.
A local installation requires various dependencies, while the docker installation requires just docker itself and a binary installer for `CPLEX`.

## Docker Installation
1. Log in to the IBM Website and download the binary installer for `IBM ILOG CPLEX Optimization Studio V22.1.1 for Linux x86-64'. 
Copy it to this project folder.
2. Adjust the line `COPY cplex_studio2211.linux_x86_64.bin /tmp/installer.bin` in `Dockerfile` so that the first argument matches the name of your installer.
3. Build the docker container with `docker build . -t sahlp`. 
4. To test if everything works, you can execute `docker run sahlp bash -c "cd /sahlp && pytest`.

## Regular Installation
### Prerequisites
* A newish `C++` compiler and `cmake 3.25` or higher. 
* `SCIP` dependencies: `apt-get install -y libgmp-dev libreadline-dev libboost-all-dev libblas-dev libtbb-dev`.
* Optionally to use the `Python` interface: `Python 3.11` or higher with `pip` installed. 
* The `Boost` library.

### Optional Prerequisites
The algorithm needs a solver for the Benders subproblems. 
If a `CPLEX` installation can be found by `cmake`, `CPLEX` will be linked and used to solve the subproblems.
If `CPLEX` is not found, but `MCFClass` (see below) can be found by `cmake`, that will be used.
If both can not be found, the solver falls back to solving the subproblems with `SOPLEX`, which ships with `SCIP`.

#### Notes on Optional Installation of MCFClass
This solver can be downloaded and installed from here: https://github.com/frangio68/Min-Cost-Flow-Class
Please consider their license as well if you want to use their solver for the subproblems.
The following flags for `cmake` should be set when building it (replacing `<INSTALLATION DIRECTORY>` by some directory that `cmake` will check when building the main code later):
```bash
mkdir build
cd build
cmake -DMCFClass_USE_CPLEX=OFF -DCMAKE_INSTALL_PREFIX=<INSTALLATION DIRECTORY> -DCMAKE_POSITION_INDEPENDENT_CODE=ON -DCMAKE_BUILD_TYPE=Release ..
make install
```

### 1. Installing SCIP
If you already have `SCIP`s shared libaries installed, you can skip this step.
Otherwise, I recommend building `SCIP` and installing it as follows:
```bash
git clone https://github.com/scipopt/scip.git 
cd scip 
git checkout tags/v804 # tested with SCIP version 8.0.4, but you can probably use the newest one
mkdir build 
cd build 
cmake -DCMAKE_INSTALL_PREFIX=/usr/local \  # the directory in which SCIP will install its binary executable and libraries
  -DCMAKE_BUILD_TYPE=Release \  # Release or Debug mode
  -DLPS=cpx \  # Optional: Using CPLEX as the LP solver (should be omitted if CPLEX not installed)
  -DCPLEX_DIR=/opt/ibm/ILOG/CPLEX_Studio2211/cplex \  # Location of your CPLEX installation
  -DPAPILO=OFF -DZIMPL=OFF -DIPOPT=OFF ..  # Turning off various SCIP extras that we don't need
make
make install
```

### 2. Installing the Solver
In this project folder, execute:
```bash
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release \  # or Debug
      -DSCIP_DIR=/usr/local \  # or wherever you installed SCIP to. Can be omitted if its on your $PATH
  ..
make 
# optionally build a bare-bones documentation in build/html:
make doc
```

### 3. Optional: Installing the Python Interface
To install the interface in the currently active `Python` environment in development mode, do `pip install -r requirements.txt` and `pip install -e .` in this project folder. 
Installation in normal mode is currently not supported, as the compiled shared library for the solver would need to be copied to the `site-packages` directory as part of the setup.
To test if the installation was successful, run `pytest`.

# Solving Hub Location Problems
There are various ways to use the solver:
* As an interactive executable on the command line.
* By calling `C` or `C++` interface functions (for example when integrating this projects shared library in your project).
* By using the `Python` interface.

## As an Interactive Executable
You can start the solver by calling `build/sahlp`. 
This will start the `SCIP` interactive terminal. 
To read and solve a problem, do either 
```
> read tests/test_data/4p2_312.sahlp
> optimize
> write solution <FILENAME>.sol
```
or
```
> read tests/test_data/4.hlp
> read tests/test_data/p2.hlps
> optimize
> write solution <FILENAME>.sol
```
See lower down for an explanation of the supported file formats.

## As a `C/C++` Library
Include the header file `solver_sahlp.hpp` in your code and link the library. 
Then you can either call the functions `solve_SAHLP` or `solve_HLPS` to pass paths to input files in the corresponding formats or construct a `CInstance` struct describing your instance and pass it to `solveSAHLPInstance_C`.

## As a Python library
You need to `import sahlp` and then you can call either `sahlp.solve_hlps` or `sahlp.solve_sahlp` to pass paths to input files, or you construct an instance of the `sahlp.SAHLPInstance` class and call `sahlp.solve_sahlp_instance` on it.
For a tiny example, see the `if __name__ == "__main__":` section of the file `sahlp/sahlp.py`.

# Running Tests
Both the `Python` and `C++` tests can be discovered by a properly configured IDE or run on the command line.
The `C++` tests use the `googletest` framework and can be run by calling `build/sahlp-test`. 
These tests are not really unit tests but rather just a few instances that are solved - if the objective value is as expected, the test passes.
Note that this might take a while since some of the instances need a few minutes to solve.
To run the `Python` tests, simply execute `pytest`.

# File formats
This code supports two file formats for describing instances of the Singe Allocation Hub Location Problem.
Note that both formats do *not* support comments, they are added below for clarity.
The first is the `.sahlp` file format. 
Such files are structured as follows:
```bash
4 4 2 # number of nodes (n_nodes), number of potential hubs (n_hubs), number of hubs to open (p)
0 1 2 3 # the indices (starting at 0) of the nodes that are potential hubs, in this example all four nodes
0 1 1 1 # a n_nodes x n_nodes matrix of outgoing demands
1 0 1 1 
1 1 0 1 # for example in this line: node 2 sends one unit of demand to all other nodes except itself
1 1 1 0
0 1 2 3 # a n_hubs x n_hubs matrix containing the pairwise cost per unit of demand transferred from one hub to another
1 0 1 2 # example: sending one unit of demand from node 1 to node 3 costs two units of money
2 1 0 1 
3 2 1 0
4 0 0 1 15 2 30 3 45 # n_nodes many lines that list for each node which hubs it can be assigned to and at what cost
4 0 15 1 0 2 15 3 30 
4 0 30 1 15 2 0 3 15 # example: node 2 can be assigned to node 0 for cost of 30, to node 1 for cost of 15 etc.
4 0 45 1 30 2 15 3 0 
10 5 2 8 # capacities of potential hubs (in terms of demand units delivered by that hub). This line is optional and can be left out
OPEN 1 2 0 1 # optional: constraint to open exactly N out a set of M many hubs. Syntax: OPEN N M hub_1 hub_2 ... hub_M. Here: we have to open either hub 0 or hub 1.
```

The second file format actually requires two files, a `.hlp` file describing the distances and demands between nodes and a `.hlps` file describing problem parameters like the number of hubs to open and the cost factors.
The `.hlp` file is structured as follows:
```bash
3     # the number of nodes. Nodes are zero-index, i.e., if there are three nodes they are 0, 1 and 2
2     # the number of nodes that are also potential hubs 
0 2   # the potential hubs
0     # each line: the nodes to which the node can be assigned. Here: Node 0 can be assigned only to itself.
0 2   # nodes 1 and 2 can both be assigned to either node 0 or 2.
0 2
0 2 3 # the distance from each node to all other nodes
1 0 1
2 0 2
3 8 5 # the demands from each node to all other nodes
1 2 3
3 0 2
```
Note: If all nodes can be hubs and if all assignments of nodes to hubs are allowed, the second line of the file can simply contain `ALL` and is immediately followed by the distance matrix.

The `.hlps` file is structured as follows:
```bash
2 # number of hubs to open (p)
2 # cost factor for collections per demand and distance unit (pick-ups)
1 # cost factor for transfers between hubs per demand and distance unit 
1.5 # cost factor for distribution per demand and distance unit (deliveries)
```


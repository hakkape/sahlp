FROM debian:bookworm

# dependencies 
RUN apt-get update

## C++ compilation and debugging
RUN apt-get install -y build-essential gdb cmake 
## necessary for SCIP
RUN apt-get install -y git libgmp-dev libreadline-dev libboost-all-dev libblas-dev libtbb-dev
## necessary for Python interface
RUN apt-get install -y python3.11 python3-pip python3.11-venv

# installing CPLEX
COPY cplex_studio2211.linux_x86_64.bin /tmp/installer.bin
RUN chmod +x /tmp/installer.bin && \
    /tmp/installer.bin -i silent -DLICENSE_ACCEPTED=TRUE && \
    rm /tmp/installer.bin

# installing SCIP
# explanation of the flags:
# - DCMAKE_INSTALL_PREFIX: where the SCIP binary and libraries will be installed
# - DCMAKE_BUILD_TYPE: build type (Release, Debug, RelWithDebInfo, MinSizeRel)
# - DLPS: which LP solver to use (cpx = CPLEX)
# - DCPLEX_DIR: where CPLEX is installed. Must be adjusted when another version of CPLEX is installed
# - DPAPILO: whether to use PAPILO (a presolving library)
# - DZIMPL: whether to use ZIMPL (a modeling language)
# - IPOPT: whether to use IPOPT (a nonlinear solver)
RUN git clone https://github.com/scipopt/scip.git && \
    cd scip && \
    git checkout tags/v804 && \
    mkdir build && \
    cd build && \
    cmake -DCMAKE_INSTALL_PREFIX=/usr/local -DCMAKE_BUILD_TYPE=Release -DLPS=cpx -DCPLEX_DIR=/opt/ibm/ILOG/CPLEX_Studio2211/cplex -DPAPILO=OFF -DZIMPL=OFF -DIPOPT=OFF .. && \
    make && \
    make install

# compile SAHLP solver
COPY src /sahlp/src
COPY sahlp /sahlp/sahlp
COPY tests /sahlp/tests
COPY CMakeLists.txt /sahlp/CMakeLists.txt
COPY setup.py /sahlp/setup.py
COPY cmake /sahlp/cmake
COPY requirements.txt /sahlp/requirements.txt

RUN cd /sahlp && \
    mkdir build && \
    cd build && \
    cmake -DCMAKE_BUILD_TYPE=Release .. && \
    make

# set up python environment and install the python interface
# note: we install in dev mode, which means that the python code will not be moved to the venv.
#       if we want to properly install the python module, we also need to copy the `sahlp.so` shared library file
RUN python3 -m venv /venv/ && \
    . /venv/bin/activate && \
    pip install -r /sahlp/requirements.txt && \
    pip install -e /sahlp 

ENV PATH="/venv/bin:$PATH"



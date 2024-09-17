import ctypes
from pathlib import Path
from collections import defaultdict
import os
from dataclasses import dataclass
from typing import Tuple
import json


@dataclass
class SAHLPInstance:
    """A class representing an instance of the Single Assignment Hub Location Problem (SAHLP).
    Create an instance by passing the parameters of the problem to the constructor.

    Args:
        n_nodes (int): The number of nodes in the instance. Nodes are assumed to be numbered from 0 to n_nodes-1.
        hubs (list[int]): The list of nodes that are allowed to be hubs.
        p (int): The number of hubs to open.
        demands (list[list[float]]): The demands between nodes. demands[i][j] is the demand to be sent from node i to node j.
        transfer_costs (list[list[float]]): The transfer costs between hubs. transfer_costs[i][j] is the cost to transfer a unit of demand from hub i to hub j.
        allowed_hubs (list[list[int]]): The list of hubs that are allowed to be assigned to each node. allowed_hubs[i] is the list of hub nodes that i can be assigned to.
        assignment_costs (list[list[float]]): The assignment costs between nodes and hubs. assignment_costs[i][j] is the cost to assign node i to hub j.
        with_capacities (bool): Whether the hubs have capacities.
        capacities (list[float]): The capacities of the hubs. capacities[i] is the capacity of hub i.
        open_hub_constraints (list[Tuple[int, list[int]]]): The "open N of set of hubs" constraints. Every entry is a tuple (N, HUBS) where N is the number of hubs that have to be open from HUBS.
    """

    n_nodes: int
    hubs: list[int]
    p: int
    demands: list[list[float]]
    transfer_costs: list[list[float]]
    allowed_hubs: list[list[int]]
    assignment_costs: list[list[float]]
    with_capacities: bool
    capacities: list[float]
    open_hub_constraints: list[Tuple[int, list[int]]]

    def compute_solution_dict_value(self, solution_dict: dict[int, list[int]]) -> float:
        """Given a solution dictionary, compute the value of the solution.

        Args:
            solution_dict (dict[int, list[int]]): The solution given as a dictionary where the keys are the opened hubs and the values are the list of nodes assigned to each hub.

        Returns:
            float: solution value
        """
        value = 0.0
        # assignment costs
        for hub, customers in solution_dict.items():
            for customer in customers:
                hub_idx = self.allowed_hubs[customer].index(hub)
                value += self.assignment_costs[customer][hub_idx]

        # transfer_cost
        hub2hub_idx = {hub: i for i, hub in enumerate(self.hubs)}
        hubidx_assignment = {
            node: hub2hub_idx[hub]
            for hub, nodes in solution_dict.items()
            for node in nodes
        }
        for i in range(self.n_nodes):
            hub_i_idx = hubidx_assignment[i]
            for j in range(self.n_nodes):
                demand = self.demands[i][j]
                if demand == 0:
                    continue
                hub_j_idx = hubidx_assignment[j]
                value += self.transfer_costs[hub_i_idx][hub_j_idx] * self.demands[i][j]

        return value

    def save_sahlp(self, outfile: os.PathLike | str) -> None:
        """Saves the instance to a file in the .sahlp format.

        Args:
            outfile (os.PathLike | str): File path to write the instance to.
        """
        with open(outfile, "w") as f:
            print(f"{self.n_nodes} {len(self.hubs)} {self.p}", file=f)
            print(" ".join(map(str, self.hubs)), file=f)
            for row in self.demands:
                print(" ".join(map(str, row)), file=f)
            for row in self.transfer_costs:
                print(" ".join(map(str, row)), file=f)
            for i in range(self.n_nodes):
                n_allowed = len(self.allowed_hubs[i])
                line = f"{n_allowed}"
                for n in range(n_allowed):
                    line += f" {self.allowed_hubs[i][n]} {self.assignment_costs[i][n]}"
                print(line, file=f)
            if self.with_capacities:
                print(" ".join(map(str, self.capacities)), file=f)
            for cons in self.open_hub_constraints:
                print(
                    f"OPEN {cons[0]} {len(cons[1])} {' '.join(map(str, cons[1]))}",
                    file=f,
                )

    @classmethod
    def read_sahlp(cls, infile: os.PathLike | str) -> "SAHLPInstance":
        """Reads an instance from a file in the .sahlp format.

        Args:
            infile (os.PathLike | str): File path to read instance from.

        Returns:
            SAHLPInstance: The instance.
        """
        with open(infile, "r") as f:
            lines = f.readlines()
        n_nodes, n_hubs, p = map(int, lines[0].split())
        hubs = list(map(int, lines[1].split()))
        demands = []
        transfer_costs = []
        allowed_hubs = []
        assignment_costs = []
        with_capacities = False
        capacities = []
        open_hub_constraints = []
        line_idx = 2
        for i in range(n_nodes):
            demands.append(list(map(float, lines[line_idx].split())))
            line_idx += 1
        for i in range(len(hubs)):
            transfer_costs.append(list(map(float, lines[line_idx].split())))
            line_idx += 1
        for i in range(n_nodes):
            n_allowed = int(lines[line_idx].split()[0])
            allowed_hubs.append(list(map(int, lines[line_idx].split()[1::2])))
            assignment_costs.append(list(map(float, lines[line_idx].split()[2::2])))
            line_idx += 1
        while line_idx < len(lines):
            line = lines[line_idx]
            if line.startswith("OPEN"):
                _, n_open, len_list, *open_hubs = line.split()
                open_hub_constraints.append((int(n_open), list(map(int, open_hubs))))
            else:
                with_capacities = True
                capacities = list(map(float, line.split()))
            line_idx += 1
        return cls(
            n_nodes,
            hubs,
            p,
            demands,
            transfer_costs,
            allowed_hubs,
            assignment_costs,
            with_capacities,
            capacities,
            open_hub_constraints,
        )


@dataclass
class Solution:
    """A class representing a solution to the Single Assignment Hub Location Problem (SAHLP).

    Args:
        solution_dict (dict[int, list[int]]): The solution given as a dictionary where the keys are the opened hubs and the values are the list of nodes assigned to each hub.
        solution_value (float): The value of the solution.
        runtime (float): The runtime of the solver.
    """

    solution_dict: dict[int, list[int]]
    solution_value: float
    runtime: float

    def save_json(self, path: os.PathLike | str) -> None:
        """Saves the solution to a file in the .json format.

        Args:
            path (os.PathLike | str): File path to write the solution to.
        """
        sol = {
            "solution_dict": self.solution_dict,
            "solution_value": self.solution_value,
            "runtime": self.runtime,
        }
        with open(path, "w") as f:
            json.dump(sol, f)

    @classmethod
    def read_json(cls, path: os.PathLike | str) -> "Solution":
        """Reads a solution from a file in the .json format.

        Args:
            path (os.PathLike | str): FIle path to read the solution from.

        Returns:
            Solution: The solution.
        """
        with open(path, "r") as f:
            sol = json.load(f)

        # convert from string to the appropriate datatypes
        solution_dict = {
            int(hub): [int(node) for node in assigned]
            for hub, assigned in sol["solution_dict"].items()
        }
        solution_value = float(sol["solution_value"])
        runtime = float(sol["runtime"])
        return cls(
            solution_dict=solution_dict, solution_value=solution_value, runtime=runtime
        )


class _CSolution(ctypes.Structure):
    """Internal representation of the C solution struct."""

    _fields_ = [
        ("assigned_hubs", ctypes.POINTER(ctypes.c_int)),
        ("n_nodes", ctypes.c_int),
        ("solution_value", ctypes.c_double),
        ("cputime", ctypes.c_double),
    ]


class CInstance(ctypes.Structure):
    """Internal representation of the C instance struct."""

    _fields_ = [
        ("n_nodes", ctypes.c_int),
        ("n_hubs", ctypes.c_int),
        ("hubs", ctypes.POINTER(ctypes.c_int)),
        ("p", ctypes.c_int),
        ("demands", ctypes.POINTER(ctypes.POINTER(ctypes.c_double))),
        ("transfer_costs", ctypes.POINTER(ctypes.POINTER(ctypes.c_double))),
        ("n_allowed_hubs", ctypes.POINTER(ctypes.c_int)),
        ("allowed_hubs", ctypes.POINTER(ctypes.POINTER(ctypes.c_int))),
        ("assignment_costs", ctypes.POINTER(ctypes.POINTER(ctypes.c_double))),
        ("with_capacities", ctypes.c_uint),
        ("with_p_constraint", ctypes.c_uint),
        ("capacities", ctypes.POINTER(ctypes.c_double)),
        ("n_open_hubs_constraints", ctypes.c_int),
        ("n_open_hubs", ctypes.POINTER(ctypes.c_int)),
        ("n_candidate_hubs", ctypes.POINTER(ctypes.c_int)),
        ("candidate_hubs", ctypes.POINTER(ctypes.POINTER(ctypes.c_int))),
    ]


# ctypes interface to shared SAHLP library
project_root_path = Path(__file__).parent.parent
lib_path = project_root_path / "build/src/libsahlplib.so"
_solverlib = ctypes.CDLL(lib_path)
_solverlib.solveHLPS_C.argtypes = [
    ctypes.c_char_p,
    ctypes.c_char_p,
    ctypes.c_char_p,
]
_solverlib.solveHLPS_C.restype = ctypes.c_void_p
_solverlib.solveSAHLP_C.argtypes = [
    ctypes.c_char_p,
    ctypes.c_char_p,
]
_solverlib.solveSAHLP_C.restype = ctypes.c_void_p
_solverlib.solveSAHLPInstance_C.argtypes = [
    ctypes.POINTER(CInstance),
    ctypes.c_char_p,
]
_solverlib.solveSAHLPInstance_C.restype = ctypes.c_void_p
_solverlib.freeCSolution.argtypes = [ctypes.POINTER(_CSolution)]
_solverlib.freeCInstance.argtypes = [ctypes.POINTER(CInstance)]


# private interface functions
def _csol_to_sol(csol: _CSolution) -> Solution:
    """Converts a C solution to a Python solution.

    Args:
        csol (_CSolution): Ctypes solution returned by the C library.

    Returns:
        Solution: Python solution.
    """
    solution_dict = defaultdict(list)
    for customer in range(csol.n_nodes):
        solution_dict[csol.assigned_hubs[customer]].append(customer)
    return Solution(solution_dict, csol.solution_value, csol.cputime)


def _ins_to_cins(ins: SAHLPInstance) -> CInstance:
    """Converts a Python instance to a C instance.

    Args:
        ins (SAHLPInstance): Python instance.

    Returns:
        CInstance: Ctypes instance.
    """
    n_nodes = ins.n_nodes
    n_hubs = len(ins.hubs)
    hubs = (ctypes.c_int * n_hubs)(*ins.hubs)
    p = ins.p
    demands = (ctypes.POINTER(ctypes.c_double) * n_nodes)()
    for i, row in enumerate(ins.demands):
        demands[i] = (ctypes.c_double * len(row))(*row)
    transfer_costs = (ctypes.POINTER(ctypes.c_double) * n_hubs)()
    for i, row in enumerate(ins.transfer_costs):
        transfer_costs[i] = (ctypes.c_double * len(row))(*row)
    n_allowed_hubs = (ctypes.c_int * n_nodes)()
    allowed_hubs = (ctypes.POINTER(ctypes.c_int) * n_nodes)()
    for i, row in enumerate(ins.allowed_hubs):
        n_allowed_hubs[i] = len(row)
        allowed_hubs[i] = (ctypes.c_int * len(row))(*row)
    assignment_costs = (ctypes.POINTER(ctypes.c_double) * n_nodes)()
    for i, row in enumerate(ins.assignment_costs):
        assignment_costs[i] = (ctypes.c_double * len(row))(*row)
    with_capacities = ctypes.c_uint(ins.with_capacities)
    with_p_constraint = ctypes.c_uint(True)
    capacities = (ctypes.c_double * n_hubs)(*ins.capacities)
    n_open_hubs_constraints = len(ins.open_hub_constraints)
    n_open_hubs = (ctypes.c_int * n_open_hubs_constraints)()
    n_candidate_hubs = (ctypes.c_int * n_open_hubs_constraints)()
    candidate_hubs = (ctypes.POINTER(ctypes.c_int) * n_open_hubs_constraints)()
    for i, (n_open_hubs_i, candidate_hubs_i) in enumerate(ins.open_hub_constraints):
        n_open_hubs[i] = n_open_hubs_i
        n_candidate_hubs[i] = len(candidate_hubs_i)
        candidate_hubs[i] = (ctypes.c_int * len(candidate_hubs_i))(*candidate_hubs_i)
        print(candidate_hubs[i], n_open_hubs_i)

    return CInstance(
        n_nodes,
        n_hubs,
        hubs,
        p,
        demands,
        transfer_costs,
        n_allowed_hubs,
        allowed_hubs,
        assignment_costs,
        with_capacities,
        with_p_constraint,
        capacities,
        n_open_hubs_constraints,
        n_open_hubs,
        n_candidate_hubs,
        candidate_hubs,
    )


def _free_cins(cins: CInstance) -> None:
    """Free a C instance.  Usually not necessary because ctypes seems to free the memory automatically.

    Args:
        cins (CInstance): Instance to free.
    """
    _solverlib.freeCInstance(ctypes.byref(cins))


def _free_csol(csol: _CSolution) -> None:
    """Frees a C solution.

    Args:
        csol (_CSolution): Solution to free.
    """
    _solverlib.freeCSolution(ctypes.byref(csol))


def encode_path(path: str) -> bytes:
    """Function turns path to a file to an absolute path and encodes it. Necessary to pass the path to the C library.

    Args:
        path (str): relative or absolute path to encode.

    Returns:
        bytes: path encoded as bytes.
    """
    return os.path.abspath(path).encode()


def solve_hlps(
    hlp_path: str | os.PathLike,
    hlps_path: str | os.PathLike,
    scip_settings_path: str | os.PathLike = None,
) -> Solution:
    """Interface function to solve an SAHLP problem given by a .hlp and .hlps file.

    Args:
        hlp_path (str | os.PathLike): File path to the .hlp file.
        hlps_path (str | os.PathLike): File path to the .hlps file.
        scip_settings_path (str | os.PathLike, optional): File path to the SCIP settings file to use. Defaults to None.

    Returns:
        Solution: Best primal solution found by the solver.
    """
    if scip_settings_path is not None:
        scip_settings_path = encode_path(scip_settings_path)

    # solve problem
    csolution = _CSolution.from_address(
        _solverlib.solveHLPS_C(
            encode_path(hlp_path),
            encode_path(hlps_path),
            scip_settings_path,
        )
    )

    # convert to python solution
    solution = _csol_to_sol(csolution)
    _free_csol(csolution)

    return solution


def solve_sahlp(
    sahlp_path: str | os.PathLike, scip_settings_path: str | os.PathLike = None
) -> None:
    """Interface function to solve an SAHLP problem given by a .sahlp file.

    Args:
        sahlp_path (str | os.PathLike): File path to the .sahlp file.
        scip_settings_path (str | os.PathLike, optional): File path to the SCIP settings file to use. Defaults to None.

    Returns:
        Solution: Best primal solution found by the solver.
    """
    if scip_settings_path is not None:
        scip_settings_path = encode_path(scip_settings_path)

    # solve problem
    csolution = _CSolution.from_address(
        _solverlib.solveSAHLP_C(
            encode_path(sahlp_path),
            scip_settings_path,
        )
    )

    # convert to python solution
    solution = _csol_to_sol(csolution)
    _free_csol(csolution)

    return solution


def solve_sahlp_instance(
    instance: SAHLPInstance, scip_settings_path: str | os.PathLike = None
) -> None:
    """Interface function to solve an SAHLP problem given by a SAHLPInstance object.

    Args:
        instance (SAHLPInstance): Instance to solve.
        scip_settings_path (str | os.PathLike, optional): File path to the SCIP settings file to use. Defaults to None.

    Returns:
        Solution: Best primal solution found by the solver.
    """
    if scip_settings_path is not None:
        scip_settings_path = encode_path(scip_settings_path)

    # solve problem
    cinstance = _ins_to_cins(instance)
    csolution = _CSolution.from_address(
        _solverlib.solveSAHLPInstance_C(
            ctypes.byref(cinstance),
            scip_settings_path,
        )
    )

    # convert to python solution
    solution = _csol_to_sol(csolution)
    _free_csol(csolution)
    return solution


if __name__ == "__main__":
    """Example usage of the Python interface.
    Just need to `from sahlp import SAHLPInstance, solve_sahlp_instance` in your code.
    """
    instance = SAHLPInstance(
        n_nodes=4,
        hubs=[0, 1, 2, 3],
        p=2,
        demands=[
            [0, 1, 1, 1],
            [1, 0, 1, 1],
            [1, 1, 0, 1],
            [1, 1, 1, 0],
        ],
        assignment_costs=[
            [0, 15, 30, 45],
            [15, 0, 15, 30],
            [30, 15, 0, 15],
            [45, 30, 15, 0],
        ],
        allowed_hubs=[[0, 1, 2, 3], [0, 1, 2, 3], [0, 1, 2, 3], [0, 1, 2, 3]],
        transfer_costs=[
            [0, 1, 2, 3],
            [1, 0, 1, 2],
            [2, 1, 0, 1],
            [3, 2, 1, 0],
        ],
        with_capacities=False,
        capacities=[],
        open_hub_constraints=[(1, [0, 3])],
    )
    solve_sahlp_instance(instance)

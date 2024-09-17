from sahlp import SAHLPInstance
import pytest
from pathlib import Path


@pytest.fixture
def TEST_DATA_DIR():
    return Path(__file__).parent / "test_data"


@pytest.fixture
def tiny_sahlp_file(TEST_DATA_DIR):
    return TEST_DATA_DIR / "4p2_312.sahlp"


@pytest.fixture
def tiny_hlp_file(TEST_DATA_DIR):
    return TEST_DATA_DIR / "4.hlp"


@pytest.fixture
def tiny_hlps_file(TEST_DATA_DIR):
    return TEST_DATA_DIR / "tiny.hlps"


@pytest.fixture
def tiny_sahlp_instance():
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
        open_hub_constraints=[],
    )
    return instance


@pytest.fixture
def tiny_incomplete_sahlp_instance():
    instance = SAHLPInstance(
        n_nodes=4,
        hubs=[0, 2, 3],
        p=2,
        demands=[
            [0, 1, 1, 1],
            [1, 0, 1, 1],
            [1, 1, 0, 1],
            [1, 1, 1, 0],
        ],
        assignment_costs=[
            [0, 30, 45],
            [15, 15, 30],
            [30, 0, 15],
            [45, 15, 0],
        ],
        allowed_hubs=[[0, 2, 3], [0, 2, 3], [0, 2, 3], [0, 2, 3]],
        transfer_costs=[
            [0, 2, 3],
            [2, 0, 1],
            [3, 1, 0],
        ],
        with_capacities=False,
        capacities=[],
        open_hub_constraints=[],
    )
    return instance


@pytest.fixture
def tiny_sahlp_instance_opencons():
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
    return instance


@pytest.fixture
def tiny_sahlp_instance_capacities():
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
        with_capacities=True,
        capacities=[1000, 1, 1, 1000],
        open_hub_constraints=[],
    )
    return instance


@pytest.fixture
def tiny_sahlp_instance_cons_caps():
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
        with_capacities=True,
        capacities=[10, 10, 10, 10],
        open_hub_constraints=[(1, [0, 1])],
    )
    return instance

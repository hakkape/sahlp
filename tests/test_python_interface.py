import pytest
from sahlp import solve_sahlp, solve_sahlp_instance, solve_hlps, SAHLPInstance, Solution

# most of the features of the solver are tested in the c++ tests,
# here we only test that the python interface correctly creates the c instance


def test_solve_tiny_sahlp_file(tiny_sahlp_file):
    sol = solve_sahlp(tiny_sahlp_file)
    assert sol.solution_value == 38


def test_solve_tiny_hlps_file(tiny_hlp_file, tiny_hlps_file):
    sol = solve_hlps(tiny_hlp_file, tiny_hlps_file)
    assert sol.solution_value == 38


def test_solution_write_read(tmp_path):
    sol = Solution(solution_dict={1: [0, 1], 2: [2, 3]}, solution_value=10, runtime=10)
    path = tmp_path / "test.sol"
    sol.save_json(path)
    read_sol = Solution.read_json(path)

    assert sol == read_sol


def test_compute_solution_value(tiny_sahlp_instance):
    value = tiny_sahlp_instance.compute_solution_dict_value({1: [0, 1], 2: [2, 3]})
    assert value == 38


@pytest.mark.parametrize(
    "instance_name, expected",
    [
        ("tiny_sahlp_instance", 38),
        ("tiny_incomplete_sahlp_instance", 42),
        ("tiny_sahlp_instance_opencons", 42),
        ("tiny_sahlp_instance_capacities", 54),
    ],
)
def test_solve_instances(instance_name, expected, request):
    instance_file = request.getfixturevalue(instance_name)
    sol = solve_sahlp_instance(instance_file)
    assert sol.solution_value == expected


def test_solution_write_read(tmp_path):
    sol = Solution(solution_dict={1: [0, 1], 2: [2, 3]}, solution_value=10, runtime=10)
    path = tmp_path / "test.sol"
    sol.save_json(path)
    read_sol = Solution.read_json(path)

    assert sol == read_sol


def test_read_sahlp(tmp_path, tiny_sahlp_instance_cons_caps, TEST_DATA_DIR):
    sahlp = SAHLPInstance.read_sahlp(TEST_DATA_DIR / "4p2_312_with_cap_and_open.sahlp")
    assert sahlp == tiny_sahlp_instance_cons_caps


def test_save_sahlp(tmp_path, tiny_sahlp_instance_cons_caps):
    path = tmp_path / "test.sahlp"
    tiny_sahlp_instance_cons_caps.save_sahlp(path)
    read_instance = SAHLPInstance.read_sahlp(path)
    assert tiny_sahlp_instance_cons_caps == read_instance

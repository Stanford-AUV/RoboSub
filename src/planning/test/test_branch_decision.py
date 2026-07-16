"""branch_decision: vote on value CHANGES seen during the listen window.

The decision topic (/pinger) publishes its latched value at 5 Hz, so raw
samples are dominated by stickiness; what carries information is each
switch. The first sample and every subsequent change count one vote for
the value switched to; majority wins; ties go to the most recent value.
"""
from planning.nodes.path_generator import branch_decision


def stream(*runs):
    """stream(("front", 5), ("back", 3), ...) -> flat sticky sample list."""
    out = []
    for value, n in runs:
        out += [value] * n
    return out


def test_steady_value_wins():
    code, votes = branch_decision(stream(("back", 50)))
    assert code == "back"
    assert votes == {"back": 1}


def test_majority_of_switches_wins():
    # Was front, then switched to front 3 times and back 2 times in the
    # window -> front (the user's example).
    s = stream(("front", 8), ("back", 4), ("front", 9), ("back", 2),
               ("front", 11))
    code, votes = branch_decision(s)
    assert votes == {"front": 3, "back": 2}
    assert code == "front"


def test_back_majority():
    s = stream(("back", 10), ("front", 3), ("back", 12))
    code, votes = branch_decision(s)
    assert votes == {"back": 2, "front": 1}
    assert code == "back"


def test_tie_goes_to_most_recent():
    code, _ = branch_decision(stream(("front", 20), ("back", 20)))
    assert code == "back"
    code, _ = branch_decision(stream(("back", 20), ("front", 20)))
    assert code == "front"


def test_single_sample():
    code, votes = branch_decision(["front"])
    assert code == "front" and votes == {"front": 1}

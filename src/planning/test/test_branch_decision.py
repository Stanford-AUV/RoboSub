"""branch_decision: tally the messages received during the listen window.

The daisy node publishes ONE message per detected ping (levels crossing
the threshold and dropping back = one detection = one message), so the
branch decision is a simple majority tally of the codes received; ties go
to the most recent message.
"""
from planning.nodes.path_generator import branch_decision


def test_majority_tally_wins():
    # 3 front detections + 2 back detections in the window -> front.
    code, votes = branch_decision(
        ["front", "back", "front", "back", "front"])
    assert votes == {"front": 3, "back": 2}
    assert code == "front"


def test_repeated_same_side_counts_every_time():
    code, votes = branch_decision(["back", "back", "back", "front"])
    assert votes == {"back": 3, "front": 1}
    assert code == "back"


def test_tie_goes_to_most_recent():
    code, _ = branch_decision(["front", "back"])
    assert code == "back"
    code, _ = branch_decision(["back", "front", "back", "front"])
    assert code == "front"


def test_single_detection():
    code, votes = branch_decision(["front"])
    assert code == "front" and votes == {"front": 1}

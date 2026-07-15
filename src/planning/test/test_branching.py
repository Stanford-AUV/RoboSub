from planning.utils.branching import choose_branch, iter_leaf_items

BRANCH_ITEM = {
    "type": "branch",
    "topic": "/pinger/task",
    "listen_sec": 4.0,
    "default": "torpedo_first",
    "branches": {
        "torpedo_first": [{"type": "go_to_object", "object_id": "torpedo_target"}],
        "octagon_first": [{"type": "leg"}],
    },
}


def test_choose_branch_follows_pinger():
    name, items, reason = choose_branch(BRANCH_ITEM, "octagon_first")
    assert name == "octagon_first"
    assert items == BRANCH_ITEM["branches"]["octagon_first"]
    assert reason == "pinger"


def test_choose_branch_defaults_without_message():
    name, items, reason = choose_branch(BRANCH_ITEM, None)
    assert name == "torpedo_first"
    assert reason == "default-no-message"


def test_choose_branch_defaults_on_unknown_message():
    name, items, reason = choose_branch(BRANCH_ITEM, "garbage")
    assert name == "torpedo_first"
    assert reason == "default-unknown-message"


def test_iter_leaf_items_recurses_into_branches():
    items = [
        {"type": "leg"},
        BRANCH_ITEM,
        {"type": "go_to_object", "object_id": "gate"},
    ]
    leaves = list(iter_leaf_items(items))
    ids = [i["object_id"] for i in leaves if i["type"] == "go_to_object"]
    assert sorted(ids) == ["gate", "torpedo_target"]
    assert all(i["type"] != "branch" for i in leaves)

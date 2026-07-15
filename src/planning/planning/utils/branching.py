"""Pure helpers for baked 'branch' items (pinger-selected sub-paths)."""


def choose_branch(item, latest_msg):
    """Pick a branch from a baked branch item given the latest pinger
    message (or None). Returns (name, items, reason)."""
    branches = item["branches"]
    if latest_msg is None:
        return item["default"], branches[item["default"]], "default-no-message"
    if latest_msg not in branches:
        return (
            item["default"],
            branches[item["default"]],
            "default-unknown-message",
        )
    return latest_msg, branches[latest_msg], "pinger"


def iter_leaf_items(items):
    """Yield every non-branch item, recursing into branch bodies, so
    startup scans (e.g. go_to_object subscriptions) see items inside
    branches that may or may not run."""
    for item in items:
        if item["type"] == "branch":
            for branch_items in item["branches"].values():
                yield from iter_leaf_items(branch_items)
        else:
            yield item

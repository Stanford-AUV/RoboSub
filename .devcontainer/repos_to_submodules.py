"""This module processes .repos files and adds git submodules based on the repositories listed in those files."""

import glob
import os
import subprocess
import yaml

prefix = "src"


def add_git_submodule(repo_name: str, repo_url: str, repo_version: str):
    """Add a git submodule to the repository.

    Args:
        repo_name (str): The name of the repository.
        repo_url (str): The URL of the repository.
        repo_version (str): The branch or tag to checkout.
    """
    subprocess.call(
        ["git", "submodule", "add", "-b", repo_version, repo_url, repo_name]
    )


def is_submodule(repo_name: str):
    """Check if a repository is already a submodule.

    Args:
        repo_name (str): The name of the repository.

    Returns:
        bool: True if the repository is a submodule, False otherwise.
    """
    try:
        subprocess.check_output(
            ["git", "submodule", "status", repo_name], stderr=subprocess.DEVNULL
        )
        return True
    except subprocess.CalledProcessError:
        return False


def parse_repos_file(file_path: str):
    """Parse a .repos file and add git submodules based on its content.

    Args:
        file_path (str): The path to the .repos file.
    """
    with open(file_path, "r") as file:
        repos_data = yaml.safe_load(file)
        repositories = repos_data["repositories"]

        for repo_name, repo_info in repositories.items():
            if "type" in repo_info and repo_info["type"] == "git":
                repo_url = repo_info["url"]
                repo_version = repo_info["version"]
                submodule_name = os.path.join(prefix, repo_name)

                if not is_submodule(submodule_name):
                    add_git_submodule(submodule_name, repo_url, repo_version)
                    print(f"Added {repo_name} as a submodule.")


# Find .repos files within the src directory
repos_files = glob.glob("src/**/*.repos", recursive=True)

# Process each .repos file
for repos_file in repos_files:
    parse_repos_file(repos_file)

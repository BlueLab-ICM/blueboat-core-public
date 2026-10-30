"""Make the core packages importable without a ROS workspace."""

import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
for package in sorted((REPO_ROOT / 'src' / 'core').iterdir()):
    sys.path.insert(0, str(package))

"""``python -m actuator_bench`` starts the app; see ``--help`` for the command-line tools."""

import sys

from .cli import main

if __name__ == "__main__":
    sys.exit(main())

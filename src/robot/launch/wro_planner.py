#!/usr/bin/env python3
import os
import sys
current_file_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(current_file_dir)
from planner import main


if __name__ == "__main__":
    main()

"""
Helper module to import STservo_sdk from the arjuna package root.
Usage: from STservo_sdk_import_helper import *
This will make all STservo_sdk classes available.
"""
import sys
import os

# Get the directory where this file is located (arjuna package root)
_current_dir = os.path.dirname(os.path.abspath(__file__))

# Add current directory to path if not already there
if _current_dir not in sys.path:
    sys.path.insert(0, _current_dir)

# Import everything from STservo_sdk
try:
    from STservo_sdk import *
except ImportError:
    import warnings
    warnings.warn("STservo_sdk could not be imported. Make sure STservo_sdk folder exists in arjuna package root.")



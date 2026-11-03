import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'ros/src/omni_console'))
from omni_console.supervisor import _shutdown_navigation
_shutdown_navigation([int(sys.argv[1])])
print('Navigation lifecycle shutdown requested')

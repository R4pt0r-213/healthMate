"""Test complet : vide W -> dépôt D, puis plein S -> ancienne place W."""

from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parent.parent
TESTS = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(TESTS))

from testIAArduino import main


if __name__ == "__main__":
    main(boucle_fermee=True)

"""Find the host workspace wrapper without depending on repository folder depth."""
from pathlib import Path


def project_root(source):
    for parent in Path(source).resolve().parents:
        if (parent/"franka").is_file() and (parent/"compose.yaml").is_file():
            return parent
    raise RuntimeError("Host launchers require the Franka workspace wrapper (franka + compose.yaml)")

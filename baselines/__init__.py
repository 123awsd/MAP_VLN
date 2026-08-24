"""External VLN baseline integration and common evaluation utilities."""

from .external import inspect_external_baselines
from .vln_metrics import evaluate_trajectory

__all__ = ["evaluate_trajectory", "inspect_external_baselines"]

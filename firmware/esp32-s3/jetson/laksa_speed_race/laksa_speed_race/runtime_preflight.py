"""Fail-fast dependency checks for the isolated C1 native runtime."""

from __future__ import annotations

from dataclasses import asdict, dataclass
import importlib
from pathlib import Path
import subprocess
from typing import Callable


class RuntimePreflightError(RuntimeError):
    """The isolated C1 runtime does not resolve its frozen dependencies."""


@dataclass(frozen=True)
class AckermannRuntimeEvidence:
    expected_prefix: str
    resolved_prefix: str
    setup_file: str
    module: str
    message_class: str

    def to_dict(self) -> dict[str, str]:
        return asdict(self)


@dataclass(frozen=True)
class F1TenthGymRuntimeEvidence:
    checkout: str
    expected_sha: str
    actual_sha: str
    git_status: str
    imported_from: str
    import_strategy: str

    def to_dict(self) -> dict[str, str]:
        return asdict(self)


def validate_ackermann_runtime(
    expected_prefix: str | Path,
    *,
    importer: Callable[[str], object] = importlib.import_module,
    command_runner: Callable[..., subprocess.CompletedProcess[str]] = subprocess.run,
) -> AckermannRuntimeEvidence:
    """Prove Python and the ROS package index resolve one intended prefix."""

    expected = Path(expected_prefix).resolve()
    setup = expected / "share" / "ackermann_msgs" / "local_setup.bash"
    if not setup.is_file():
        raise RuntimePreflightError(f"ackermann_msgs setup file is missing: {setup}")

    try:
        module = importer("ackermann_msgs.msg")
    except (ImportError, ModuleNotFoundError) as error:
        raise RuntimePreflightError("ackermann_msgs Python import failed") from error
    message_class = getattr(module, "AckermannDriveStamped", None)
    if message_class is None:
        raise RuntimePreflightError("AckermannDriveStamped is not importable")

    result = command_runner(
        ["ros2", "pkg", "prefix", "ackermann_msgs"],
        check=False,
        capture_output=True,
        text=True,
    )
    if result.returncode != 0:
        detail = result.stderr.strip() or result.stdout.strip()
        raise RuntimePreflightError(f"ros2 package lookup failed: {detail}")
    resolved_text = result.stdout.strip()
    if not resolved_text:
        raise RuntimePreflightError("ros2 package lookup returned an empty prefix")
    resolved = Path(resolved_text).resolve()
    if resolved != expected:
        raise RuntimePreflightError(
            f"ackermann_msgs resolved from {resolved}, expected {expected}"
        )

    return AckermannRuntimeEvidence(
        expected_prefix=str(expected),
        resolved_prefix=str(resolved),
        setup_file=str(setup),
        module="ackermann_msgs.msg",
        message_class=f"{message_class.__module__}.{message_class.__name__}",
    )


def validate_f1tenth_gym_runtime(
    checkout: str | Path,
    expected_sha: str,
    *,
    importer: Callable[[str], object] = importlib.import_module,
    command_runner: Callable[..., subprocess.CompletedProcess[str]] = subprocess.run,
) -> F1TenthGymRuntimeEvidence:
    """Prove Python resolves the exact clean, pinned Gym source checkout."""

    expected_checkout = Path(checkout).resolve()
    package_init = expected_checkout / "f1tenth_gym" / "__init__.py"
    if not package_init.is_file():
        raise RuntimePreflightError(
            f"f1tenth_gym package is missing from checkout: {package_init}"
        )

    revision = command_runner(
        ["git", "-C", str(expected_checkout), "rev-parse", "HEAD"],
        check=False,
        capture_output=True,
        text=True,
    )
    if revision.returncode != 0:
        detail = revision.stderr.strip() or revision.stdout.strip()
        raise RuntimePreflightError(f"f1tenth_gym SHA lookup failed: {detail}")
    actual_sha = revision.stdout.strip().lower()
    expected_sha = expected_sha.strip().lower()
    if actual_sha != expected_sha:
        raise RuntimePreflightError(
            f"f1tenth_gym SHA mismatch: actual {actual_sha}, expected {expected_sha}"
        )

    status_result = command_runner(
        ["git", "-C", str(expected_checkout), "status", "--short"],
        check=False,
        capture_output=True,
        text=True,
    )
    if status_result.returncode != 0:
        detail = status_result.stderr.strip() or status_result.stdout.strip()
        raise RuntimePreflightError(f"f1tenth_gym status lookup failed: {detail}")
    git_status = status_result.stdout.strip()
    if git_status:
        raise RuntimePreflightError(
            f"f1tenth_gym checkout is modified; expected clean pinned source: {git_status}"
        )

    try:
        module = importer("f1tenth_gym")
    except (ImportError, ModuleNotFoundError) as error:
        raise RuntimePreflightError("f1tenth_gym Python import failed") from error
    module_file_value = getattr(module, "__file__", None)
    if not module_file_value:
        raise RuntimePreflightError("f1tenth_gym import has no filesystem provenance")
    imported_from = Path(module_file_value).resolve()
    expected_package = package_init.parent.resolve()
    try:
        imported_from.relative_to(expected_package)
    except ValueError as error:
        raise RuntimePreflightError(
            f"f1tenth_gym imported from {imported_from}, expected inside {expected_package}"
        ) from error

    return F1TenthGymRuntimeEvidence(
        checkout=str(expected_checkout),
        expected_sha=expected_sha,
        actual_sha=actual_sha,
        git_status="CLEAN",
        imported_from=str(imported_from),
        import_strategy="checkout root prepended to PYTHONPATH",
    )

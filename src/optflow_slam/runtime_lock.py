"""Process-wide and cross-process ownership locks for flight hardware."""

from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime, timezone
import fcntl
import json
import os
from pathlib import Path
import threading

from .paths import RUNTIME_DIR


class RuntimeLockError(RuntimeError):
    """Raised when another process already owns a runtime resource."""


@dataclass
class _HeldLock:
    descriptor: int
    references: int


_LOCAL_LOCK = threading.RLock()
_HELD_LOCKS: dict[Path, _HeldLock] = {}


class RuntimeResourceLock:
    """Exclusive advisory lock that is reentrant within one process."""

    def __init__(
        self,
        name: str,
        *,
        purpose: str,
        lock_dir: Path | None = None,
    ) -> None:
        if not name or "/" in name:
            raise ValueError("runtime lock name must be a simple filename")
        self.path = (
            (RUNTIME_DIR / "locks") if lock_dir is None else lock_dir
        ) / f"{name}.lock"
        self.path = self.path.resolve()
        self.purpose = purpose
        self._acquired = False

    def acquire(self) -> RuntimeResourceLock:
        with _LOCAL_LOCK:
            held = _HELD_LOCKS.get(self.path)
            if held is not None:
                held.references += 1
                self._acquired = True
                return self

            self.path.parent.mkdir(parents=True, exist_ok=True)
            descriptor = os.open(
                self.path,
                os.O_RDWR | os.O_CREAT,
                0o600,
            )
            try:
                fcntl.flock(
                    descriptor,
                    fcntl.LOCK_EX | fcntl.LOCK_NB,
                )
            except BlockingIOError as exc:
                owner = os.read(descriptor, 4096).decode(
                    "utf-8", errors="replace"
                ).strip()
                os.close(descriptor)
                detail = owner or "owner metadata unavailable"
                raise RuntimeLockError(
                    f"{self.path.name} is already owned: {detail}"
                ) from exc

            payload = {
                "pid": os.getpid(),
                "purpose": self.purpose,
                "acquired_utc": datetime.now(timezone.utc).isoformat(),
            }
            os.ftruncate(descriptor, 0)
            os.write(
                descriptor,
                (json.dumps(payload, sort_keys=True) + "\n").encode("ascii"),
            )
            os.fsync(descriptor)
            _HELD_LOCKS[self.path] = _HeldLock(descriptor, 1)
            self._acquired = True
            return self

    def release(self) -> None:
        if not self._acquired:
            return
        with _LOCAL_LOCK:
            held = _HELD_LOCKS.get(self.path)
            if held is None:
                self._acquired = False
                return
            held.references -= 1
            if held.references == 0:
                fcntl.flock(held.descriptor, fcntl.LOCK_UN)
                os.close(held.descriptor)
                del _HELD_LOCKS[self.path]
            self._acquired = False

    def __enter__(self) -> RuntimeResourceLock:
        return self.acquire()

    def __exit__(self, _exc_type, _exc, _traceback) -> None:
        self.release()


def cube_mavlink_lock(
    purpose: str,
    *,
    lock_dir: Path | None = None,
) -> RuntimeResourceLock:
    return RuntimeResourceLock(
        "cube_mavlink",
        purpose=purpose,
        lock_dir=lock_dir,
    )

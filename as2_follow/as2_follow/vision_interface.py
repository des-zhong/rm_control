"""Reserved dog-side vision contract. No camera, YOLO, or fusion is started here."""
from dataclasses import dataclass
from typing import Optional, Tuple


@dataclass(frozen=True)
class VisionObservation:
    dog_id: str
    capture_unix_ns: int
    # Identity association is separate from YOLO's semantic class.
    class_name: str
    entity_id: Optional[str]
    confidence: float
    camera_xyz_m: Tuple[float, float, float]
    field_xyz_m: Optional[Tuple[float, float, float]] = None
    calibration_id: Optional[str] = None


@dataclass(frozen=True)
class VisionSnapshot:
    dog_id: str
    capture_unix_ns: int
    image_jpeg: bytes
    annotated_jpeg: bytes
    # Future transport must use a separate channel/process from motion control.

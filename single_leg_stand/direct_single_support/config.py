"""Minimal configuration for direct single-support contact primitives."""

from dataclasses import dataclass, field


@dataclass(frozen=True)
class DirectSingleSupportContactConfig:
    wbc_contact_point: str = "corner_patch"
    cop_margin: float = 0.005


@dataclass(frozen=True)
class DirectSingleSupportConfig:
    contact: DirectSingleSupportContactConfig = field(default_factory=DirectSingleSupportContactConfig)


DIRECT_SINGLE_SUPPORT_CONFIG = DirectSingleSupportConfig()

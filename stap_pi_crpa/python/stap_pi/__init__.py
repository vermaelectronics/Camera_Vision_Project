"""STAP-PI anti-jam processing for a 2-element CRPA (wideband barrage jammer)."""

from .core import (
    StapPIConfig,
    StapPIProcessor,
    StapPIResult,
    apply_weight_schedule,
    constraint_vector,
    element_filter_response,
    pi_weights,
    sample_covariance,
    space_time_response,
    space_time_snapshots,
)
from .metrics import (
    cn0_from_correlation,
    correlate_ca,
    effective_cn0,
    evaluate,
    null_depth_vs_frequency,
    psd,
    snr_gain_vs_frequency,
)
from .scenario import (
    ChannelModel,
    CrpaData,
    CrpaScenario,
    cone_angle_unit_vector,
    default_channel_models,
    doa_unit_vector,
    gps_ca_code,
)

__all__ = [
    "StapPIConfig", "StapPIProcessor", "StapPIResult", "apply_weight_schedule", "constraint_vector",
    "element_filter_response",
    "pi_weights", "sample_covariance", "space_time_response", "space_time_snapshots",
    "cn0_from_correlation", "correlate_ca", "effective_cn0", "evaluate", "null_depth_vs_frequency", "psd",
    "snr_gain_vs_frequency",
    "ChannelModel", "CrpaData", "CrpaScenario", "cone_angle_unit_vector", "default_channel_models",
    "doa_unit_vector", "gps_ca_code",
]

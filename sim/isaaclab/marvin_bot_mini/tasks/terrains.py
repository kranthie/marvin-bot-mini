# Portions adapted from Isaac Lab (https://github.com/isaac-sim/IsaacLab), Copyright (c) 2022-2026,
# The Isaac Lab Project Developers, BSD-3-Clause (LICENSES/BSD-3-Clause-IsaacLab.txt). Modified for
# Marvin Bot Mini; the rest Apache-2.0.
# SPDX-License-Identifier: Apache-2.0 AND BSD-3-Clause
"""Training terrains for Marvin Bot Mini (0.60 m tall, 0.29 m from hip to sole), Marvin Bot's
scaled to the mini: steps from 1.5 to 6 cm with 12 cm treads round a 1 m platform, a tenth of the
tiles steep (4-6 cm) on every row, random boxes, rough ground and gentle slopes. Untrained so
far: the flat task comes first.
"""

import isaaclab.terrains as terrain_gen
from isaaclab.terrains import TerrainGeneratorCfg

MINI_ROUGH_TERRAINS_CFG = TerrainGeneratorCfg(
    size=(4.0, 4.0),
    border_width=10.0,
    num_rows=10,
    num_cols=20,
    horizontal_scale=0.05,
    vertical_scale=0.005,
    slope_threshold=0.75,
    use_cache=False,
    sub_terrains={
        "flat": terrain_gen.MeshPlaneTerrainCfg(proportion=0.1),
        "pyramid_stairs": terrain_gen.MeshPyramidStairsTerrainCfg(
            proportion=0.25,
            step_height_range=(0.015, 0.06),
            step_width=0.12,
            platform_width=1.0,
            border_width=0.5,
            holes=False,
        ),
        "pyramid_stairs_inv": terrain_gen.MeshInvertedPyramidStairsTerrainCfg(
            proportion=0.2,
            step_height_range=(0.015, 0.06),
            step_width=0.12,
            platform_width=1.0,
            border_width=0.5,
            holes=False,
        ),
        # steep steps on every row, as on Marvin Bot
        "steep_stairs": terrain_gen.MeshPyramidStairsTerrainCfg(
            proportion=0.06,
            step_height_range=(0.04, 0.06),
            step_width=0.12,
            platform_width=1.0,
            border_width=0.5,
            holes=False,
        ),
        "steep_stairs_inv": terrain_gen.MeshInvertedPyramidStairsTerrainCfg(
            proportion=0.04,
            step_height_range=(0.04, 0.06),
            step_width=0.12,
            platform_width=1.0,
            border_width=0.5,
            holes=False,
        ),
        "boxes": terrain_gen.MeshRandomGridTerrainCfg(
            proportion=0.1, grid_width=0.2, grid_height_range=(0.01, 0.04), platform_width=1.0
        ),
        "random_rough": terrain_gen.HfRandomUniformTerrainCfg(
            proportion=0.15, noise_range=(0.005, 0.02), noise_step=0.005, border_width=0.25
        ),
        "hf_pyramid_slope": terrain_gen.HfPyramidSlopedTerrainCfg(
            proportion=0.05, slope_range=(0.0, 0.25), platform_width=1.0, border_width=0.25
        ),
        "hf_pyramid_slope_inv": terrain_gen.HfInvertedPyramidSlopedTerrainCfg(
            proportion=0.05, slope_range=(0.0, 0.25), platform_width=1.0, border_width=0.25
        ),
    },
)

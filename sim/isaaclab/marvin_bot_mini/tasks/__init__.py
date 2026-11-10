"""Marvin Bot Mini tasks, registered with Isaac Lab through the ``isaaclab.tasks`` entry point."""

import gymnasium as gym

gym.register(
    id="MarvinBotMini-Velocity-Flat",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.velocity_env_cfg:MiniFlatEnvCfg",
        "rsl_rl_cfg_entry_point": f"{__name__}.agents:MiniFlatPPORunnerCfg",
        "default_agent": "rsl_rl",
    },
)

gym.register(
    id="MarvinBotMini-Velocity-Rough",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.velocity_env_cfg:MiniRoughEnvCfg",
        "rsl_rl_cfg_entry_point": f"{__name__}.agents:MiniRoughPPORunnerCfg",
        "default_agent": "rsl_rl",
    },
)

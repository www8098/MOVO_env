from gym.envs.registration import register

register(
    id='movo_env-v0',
    entry_point='MOVO_env.envs:MoVoBulletEnv',
)
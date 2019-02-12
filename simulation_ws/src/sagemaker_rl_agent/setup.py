# Copyright 2018 Amazon.com, Inc. or its affiliates. All Rights Reserved.
# SPDX-License-Identifier: Apache-2.0
from setuptools import setup, find_packages

# Package meta-data.
NAME = 'sagemaker_rl_agent'
REQUIRES_PYTHON = '>=3.5.0'

setup(
    name=NAME,
    version='0.0.1',
    packages=find_packages(),
    python_requires=REQUIRES_PYTHON,
    install_requires=[
        'boto3==1.9.23',
        'futures==3.1.1',
        'gym==0.10.5',
        'kubernetes==7.0.0',
        'minio==4.0.5',
        'numpy==1.13.3',
        'pandas==0.20.2',
        'Pillow==4.3.0',
        'pygame==1.9.3',
        'PyYAML==4.2b1',
        'redis==2.10.6',
        'rospkg==1.1.7',
        'scipy==0.19.0',
        'tensorflow==1.11',
        'rl-coach-slim==0.11.1'
    ],
    entry_points = {
        'console_scripts': [
            'run_rollout_rl_agent=markov.rollout_worker:main',
            'run_local_rl_agent=envs.local_worker:main'
        ],
    }
)

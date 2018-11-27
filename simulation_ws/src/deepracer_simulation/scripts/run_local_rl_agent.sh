#!/usr/bin/env bash

set -ex

export NODE_TYPE=SIMULATION_WORKER

python3 -m markov.single_machine_training_worker
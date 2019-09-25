"""
This is a rollout training worker. It starts a local training and stores the model in S3.
"""

import argparse
from tempfile import gettempdir
import math
import random

from markov.s3_boto_data_store import S3BotoDataStoreParameters, S3BotoDataStore
from rl_coach.base_parameters import TaskParameters
from rl_coach.core_types import RunPhase, EnvironmentEpisodes
from rl_coach.memories.backend.redis import RedisPubSubMemoryBackendParameters
from rl_coach.utils import short_dynamic_import
import imp

import markov
from markov import utils
import markov.environments
import os

CUSTOM_FILES_PATH = os.path.join(gettempdir(), "robomaker/")
PRESET_LOCAL_PATH = os.path.join(CUSTOM_FILES_PATH, "presets/")
ENVIRONMENT_LOCAL_PATH = os.path.join(CUSTOM_FILES_PATH, "environments/")
TRAINER_REDIS_PORT = 6379

if not os.path.exists(CUSTOM_FILES_PATH):
    os.makedirs(CUSTOM_FILES_PATH)
    os.makedirs(PRESET_LOCAL_PATH)
    os.makedirs(ENVIRONMENT_LOCAL_PATH)

def rollout_worker(graph_manager, checkpoint_dir, data_store, num_workers):
    """
    wait for first checkpoint then perform rollouts using the model
    """
    utils.wait_for_checkpoint(checkpoint_dir)

    task_parameters = TaskParameters()
    task_parameters.__dict__['checkpoint_restore_dir'] = checkpoint_dir
    graph_manager.create_graph(task_parameters)
    with graph_manager.phase_context(RunPhase.TRAIN):
        error_compensation = random.randint(0, 5)
        act_steps = math.ceil((graph_manager.agent_params.algorithm.num_consecutive_playing_steps.num_steps +
                               error_compensation) / num_workers)

        for i in range(int(graph_manager.improve_steps.num_steps / act_steps)):
            graph_manager.act(EnvironmentEpisodes(num_steps=act_steps + random.randint(0, 5)))
            # This waits for the first checkpoint
            last_checkpoint = data_store.get_current_checkpoint_number()
            data_store.load_from_store(expected_checkpoint_number=last_checkpoint + 1)
            graph_manager.restore_checkpoint()

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--markov-preset-file',
                        help="(string) Name of a preset file to run in Markov's preset directory.",
                        type=str,
                        default=os.environ.get("MARKOV_PRESET_FILE", "object_tracker.py"))
    parser.add_argument('-c', '--local-model-directory',
                        help='(string) Path to a folder containing a checkpoint to restore the model from.',
                        type=str,
                        default=os.environ.get("LOCAL_MODEL_DIRECTORY", os.path.join(gettempdir(), "checkpoint/")))
    parser.add_argument('-n', '--num-rollout-workers',
                        help="(int) Number of workers for multi-process based agents, e.g. A3C",
                        default=os.environ.get("NUMBER_OF_ROLLOUT_WORKERS", 1),
                        type=int)
    parser.add_argument('--model-s3-bucket',
                        help='(string) S3 bucket where trained models are stored. It contains model checkpoints.',
                        type=str,
                        default=os.environ.get("MODEL_S3_BUCKET"))
    parser.add_argument('--model-s3-prefix',
                        help='(string) S3 prefix where trained models are stored. It contains model checkpoints.',
                        type=str,
                        default=os.environ.get("MODEL_S3_PREFIX"))
    parser.add_argument('--aws-region',
                        help='(string) AWS region',
                        type=str,
                        default=os.environ.get("ROS_AWS_REGION", "us-west-2"))

    args = parser.parse_args()

    data_store_params_instance = S3BotoDataStoreParameters(bucket_name=args.model_s3_bucket,
                                                   s3_folder=args.model_s3_prefix,
                                                   checkpoint_dir=args.local_model_directory,
                                                   aws_region=args.aws_region)
    data_store = S3BotoDataStore(data_store_params_instance)

    # Get the IP of the trainer machine
    trainer_ip = data_store.get_ip()
    print("Received IP from SageMaker successfully: %s" % trainer_ip)

    preset_file_success = data_store.download_presets_if_present(PRESET_LOCAL_PATH)

    if preset_file_success:
        environment_file_success = data_store.download_environments_if_present(ENVIRONMENT_LOCAL_PATH)
        path_and_module = PRESET_LOCAL_PATH + args.markov_preset_file + ":graph_manager"
        graph_manager = short_dynamic_import(path_and_module, ignore_module_case=True)
        if environment_file_success:
            import robomaker.environments
        print("Using custom preset file!")
    elif args.markov_preset_file:
        markov_path = imp.find_module("markov")[1]
        preset_location = os.path.join(markov_path, "presets", args.markov_preset_file)
        path_and_module = preset_location + ":graph_manager"
        graph_manager = short_dynamic_import(path_and_module, ignore_module_case=True)
        print("Using custom preset file from Markov presets directory!")
    else:
        raise ValueError("Unable to determine preset file")

    memory_backend_params = RedisPubSubMemoryBackendParameters(redis_address=trainer_ip,
                                                               redis_port=TRAINER_REDIS_PORT,
                                                               run_type='worker',
                                                               channel=args.model_s3_prefix)
    graph_manager.agent_params.memory.register_var('memory_backend_params', memory_backend_params)
    graph_manager.data_store_params = data_store_params_instance
    graph_manager.data_store = data_store

    utils.wait_for_checkpoint(checkpoint_dir=args.local_model_directory, data_store=data_store)
    rollout_worker(
        graph_manager=graph_manager,
        checkpoint_dir=args.local_model_directory,
        data_store=data_store,
        num_workers=args.num_rollout_workers
    )

if __name__ == '__main__':
    main()

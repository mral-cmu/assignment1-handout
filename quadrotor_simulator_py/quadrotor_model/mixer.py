#!/usr/bin/env python

import numpy as np
import yaml


class QuadMixer:

    def __init__(self):
        self.mixer = np.zeros((4, 4))
        self.length = 0.0
        self.mscale = 0.0
        self.ms_angle = 0.0

    def __repr__(self):
        return ('QuadMixer\n' +
                'mixer:\n' + np.array2string(self.mixer) + '\n' +
                'length: ' + str(self.length) + '\n' +
                'mscale: ' + str(self.mscale) + '\n' +
                'ms_angle: ' + str(self.ms_angle) + '\n'
                )

    def initialize(self, yaml_file):
        data = []
        with open(yaml_file, 'r') as stream:
            try:
                data = yaml.safe_load(stream)
            except yaml.YamlError as exc:
                print(exc)

        self.length = data['length']
        self.mscale = data['rotor']['moment_scale']
        self.ms_angle = data['motor_spread_angle']

    def construct_mixer(self):

        """ Calculates the mixer matrix, which converts
                rotor speeds (e.g., RPMs) to force and torques.
                Assumes NWU.

        Output:
            mixer: 4x4 numpy matrix
        """

        # TODO: Assignment 1, Problem 2.1

        self.mixer = np.eye(4)
        return self.mixer

#!/usr/bin/python3

from twizy_description.robot_description import Node
from twizy_description.collada_node import Collada

import argparse
import yaml
from pathlib import Path

from math import atan2, sqrt, pi

parser = argparse.ArgumentParser(
    description='Produces a model of an autonomous Twizy')

subparsers = parser.add_subparsers(help='output format',
                                   dest='format',
                                   required=True)
proto_parser = subparsers.add_parser('proto')
urdf_parser = subparsers.add_parser('urdf')

proto_parser.add_argument('--translation',
                          type=str,
                          default='0.0 0.0 0.0',
                          help='default position of model')
proto_parser.add_argument('--rotation',
                          type=str,
                          default='0.0 1.0 0.0 0.0',
                          help='default rotation of model')
proto_parser.add_argument('--controller',
                          type=str,
                          default='ros',
                          help='default controller')
proto_parser.add_argument('--controller-args',
                          type=str,
                          nargs='+',
                          default=[],
                          action='extend',
                          help='default arguments for controller')

args = parser.parse_args()

description_pkg_path = Path(__file__).parent.parent

with open(description_pkg_path / 'config' / 'physical.yaml', 'r') as f:
    physical = yaml.safe_load(f)
with open(description_pkg_path / 'config' / 'model.yaml', 'r') as f:
    model = yaml.safe_load(f)


def quote(s):
    return f'"{s}"'


robot_mass = physical['chassis']['mass'] * 0.01

mesh_path = description_pkg_path / 'meshes'


def vector(v):
    return ' '.join(str(v[a]) for a in ['x', 'y', 'z'])


def rotation(r):
    return ' '.join(str(r[a]) for a in ['x', 'y', 'z', 'angle'])


def diagmat(M):
    return [
        ' '.join(str(M[i]) for i in ['xx', 'yy', 'zz']),
        ' '.join(str(M[i]) for i in ['xy', 'xy', 'yz'])
    ]


def wheel(fr, lr):
    return Node('Solid', {
        'children': [
            Node('Transform', {
                'children': [
                    Collada(str(mesh_path / 'wheel.dae'))
                ],
                'scale': f'{physical[f"{fr}_wheel"]["radius"]} {physical[f"{fr}_wheel"]["width"]} {physical[f"{fr}_wheel"]["radius"]}',
                'rotation': f'0.0 0.0 1.0 {0 if lr == "left" else pi}'
            })
        ],
        'physics': Node('Physics', {
            'centerOfMass': [vector(physical[f'{fr}_wheel']['com'])],
            'inertiaMatrix': diagmat(physical[f'{fr}_wheel']['inertial']),
            'density': -1,
            'mass': physical[f'{fr}_wheel']['mass']
        }),
        'boundingObject': Node('Cylinder', {
            'radius': physical[f'{fr}_wheel']['radius'],
            'height': physical[f'{fr}_wheel']['width'],
            'subdivision': 24
        }, True),
        'name': f'"{fr}_{lr}_wheel"'
    })


def rear_wheel(lr):
    return Node('Transform', {
        'children': [
            Node('HingeJoint', {
                'endPoint': wheel('rear', lr),
                'jointParameters': Node('HingeJointParameters', {
                    'axis': '0.0 1.0 0.0',
                }),
                'device': [
                    Node('RotationalMotor', {
                        'name': f'"rear_{lr}_wheel_motor"',
                        'maxTorque': physical['max_drive_torque'],
                        'acceleration': physical['max_acceleration'] / physical['rear_wheel']['radius'],
                        'sound': '""'
                    })
                ]
            })
        ],
        'translation': f'0.0 {(1 if lr == "left" else -1) * physical["rear_track"] / 2.0} {physical["rear_wheel"]["radius"]}'
    })


def front_wheel(lr):
    return Node('Transform', {
        'children': [
            Node('Hinge2Joint', {
                'endPoint':  wheel('front', lr),
                'jointParameters': Node('HingeJointParameters', {
                    'axis': '0.0 0.0 1.0'
                }),
                'jointParameters2': Node('JointParameters', {
                    'axis': '0.0 1.0 0.0'
                }),
                'device': [
                    Node('RotationalMotor', {
                        'name': f'"front_{lr}_steering_motor"',
                        'maxTorque': physical['max_steering_torque'],
                        'maxVelocity': physical['max_steering_velocity'],
                        'acceleration': physical['max_steering_acceleration'],
                        'sound': '""'
                    })
                ]
            })
        ],
        'translation': f'{physical["wheelbase"]} {(1 if lr == "left" else -1) * physical["front_track"] / 2.0} {physical["front_wheel"]["radius"]}'
    })


def piksi(lr):
    imu_transformation_fields = {
        'translation': vector(model[f'{lr}_piksi_imu']['position']),
        'rotation': rotation(model[f'{lr}_piksi_imu']['rotation'])
    }

    return [
        Node('GPS', {
            'name': f'"{lr}_piksi_gnss"',
            'translation': vector(model[f'{lr}_piksi_gnss']['position']),
            'accuracy': model[f'{lr}_piksi_gnss']['accuracy']
        }),
        Node('Solid', {
            'name': f'"{lr}_piksi_imu"',
            **imu_transformation_fields
        }),
        Node('Gyro', {
            'name': f'"{lr}_piksi_gyro"',
            **imu_transformation_fields
        }, True),
        Node('Accelerometer', {
            'name': f'"{lr}_piksi_accelerometer"',
            **imu_transformation_fields
        }, True)
    ]


def twizy():
    return Node('Robot', {
        'children': [
            Node('Solid', {
                'children': [
                    Node('Transform', {
                        'children': [
                            Collada(str(mesh_path / 'chassis.dae'))
                        ],
                        'scale': ' '.join(str(physical['chassis'][i]) for i in ['length', 'width', 'height'])
                    }),
                    *piksi('left'),
                    *piksi('right'),
                    Node('Transform', {
                        'children': [
                            Node('Transform', {
                                'children': [
                                    Node('RangeFinder', {
                                        'name': '"front_realsense_aligned_depth_to_color"',
                                        'fieldOfView': model['front_realsense']['color']['fov'],
                                        'width': model['front_realsense']['color']['width'],
                                        'height': model['front_realsense']['color']['height'],
                                        'maxRange': model['front_realsense']['depth']['max_range']
                                    }),
                                    Node('Camera', {
                                        'name': '"front_realsense_color_image_raw"',
                                        'fieldOfView': model['front_realsense']['color']['fov'],
                                        'width': model['front_realsense']['color']['width'],
                                        'height': model['front_realsense']['color']['height'],
                                    })
                                ],

                                # Webots camera has -z forward, x right, y up
                                'rotation': f'{1.0 / sqrt(3)} {-1.0 / sqrt(3)} {-1.0 / sqrt(3)} {2.0 * pi / 3}'
                            }, True),
                            Node('Solid', {
                                'name': '"front_realsense"',

                                # ROS Image has z forward x right -y up
                                'rotation': f'{-1.0 / sqrt(3)} {1.0 / sqrt(3)} {-1.0 / sqrt(3)} {2.0 * pi / 3}'
                            }),
                        ],
                        'translation': vector(model['front_realsense']['position']),
                        'rotation': rotation(model['front_realsense']['rotation'])
                    })
                ],
                'physics': Node('Physics', {
                    'centerOfMass': [vector(physical['chassis']['com'])],
                    'inertiaMatrix': diagmat(physical['chassis']['intertial']),
                    'density': -1,
                    'mass': physical['chassis']['mass'] - robot_mass
                }),
                'boundingObject': Node('Transform', {
                    'children': [
                        Node('Box', {
                            'size': ' '.join(str(physical['chassis'][i]) for i in ['length', 'width', 'height'])
                        }, True),
                    ],
                    'translation': f'{physical["front_overhang"] - (physical["chassis"]["length"] - physical["wheelbase"]) * 0.5} 0.0 {physical["chassis"]["height"] * 0.5}'
                }),
                'translation': f'{physical["wheelbase"] / 2.0} 0.0 {physical["ground_clearance"]}',
                'rotation': f'0.0 1.0 0.0 {physical["rake"]}'
            }),
            front_wheel('left'),
            front_wheel('right'),
            rear_wheel('left'),
            rear_wheel('right'),
        ],

        # Dummy physics property. The robot solid is rigedly fixed to the
        # chassis solid so in the simulation they will still only be viewed as
        # one object. In order for the robot to have a mobile base thoght the
        # physics field of the robot node needs to be specified. We could use a
        # small dummy mass like how we use a sphere with tiny radius as
        # boundingObject below, however webots warns agaist this in console as
        # the simulation could in theory become unstable with large mass
        # differences between nodes. However in this case that is not relevant
        # since the solids are joined into one in the simulation layer anyways.
        # To stop warnings beeing printed to the webots console, we split the
        # mass between the robot node and the chassis node
        'physics': Node('Physics', {
            'density': -1,
            'mass': robot_mass
        }),

        # Dummy bounding object, a bounding object is needed to set the physics
        # property but we are only really interested in the boundingObjects of
        # chassis and wheels for example and the base robot node does in this
        # case not correspond to any physical part with mass and bounds
        'boundingObject': Node('Sphere', {
            'radius': 0.001
        }, True),

        'synchronization': 'FALSE',
        'model': '"Autonomous Twizy"',
        'translation': 'IS translation',
        'rotation': 'IS rotation',
        'controller': 'IS controller',
        'controllerArgs': 'IS controllerArgs'
    })


header = f"""
=============================================================================
| This document was autogenerated by python from                            |
| {'{:<74}'.format(__file__)}|
| EDITING THIS FILE BY HAND IS NOT RECOMMENDED                              |
=============================================================================
"""

if args.format == 'proto':
    print(twizy().proto_str('Twizy', [
        f'field SFVec3f    translation    {args.translation}',
        f'field SFRotation rotation       {args.rotation}',
        f'field SFString   controller     "{args.controller}"',
        f'field MFString   controllerArgs [{", ".join(quote(a) for a in args.controller_args)}]'
    ], header))
elif args.format == 'urdf':
    print(twizy().urdf_str('twizy', header=header))

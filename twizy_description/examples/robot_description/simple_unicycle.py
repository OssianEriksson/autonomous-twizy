#!/usr/bin/python3

from twizy_description.robot_description import Node

simple_unicycle = Node('Robot', {
    'children': [
        Node('Transform', {
            'children': [
                Node('Hinge2Joint', {
                    'endPoint': Node('Solid', {
                        'physics': Node('Physics', {}),
                        'boundingObject': Node('Cylinder', {
                            'radius': 0.2,
                            'height': 0.05,
                            'subdivision': 24
                        }),
                        'name': '"wheel"'
                    }),
                    'jointParameters': Node('HingeJointParameters', {
                        'axis': '0.0 0.0 1.0'
                    }),
                    'jointParameters2': Node('JointParameters', {
                        'axis': '0.0 1.0 0.0'
                    })
                })
            ],
            'translation': '0.0 0.0 -0.3'
        })
    ],
    'physics': Node('Physics', {}),
    'boundingObject': Node('Box', {
        'size': '0.05 0.05 0.6'
    }),
    'model': '"Simple unicycle"',
    'translation': 'IS translation',
    'rotation': 'IS rotation'
})

print('---------------- PROTO --------------------\n')
print(simple_unicycle.proto_str('SimpleUnicycle', [
    'field SFVec3f    translation    0.0 0.0 0.0',
    'field SFRotation rotation       0.0 0.0 1.0 0.0',
]))

print('\n\n---------------- URDF ---------------------\n')
print(simple_unicycle.urdf_str('simple_unicycle'))

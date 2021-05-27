############################
twizy_description Python API
############################

.. toctree::
   :caption: Table of Contents

   index

*****************
robot_description
*****************

This module is meant for generation of URDF and Webots PROTO representation of robots from a common base. An example usage of this module is presented below. Note the similarity with the raw Webots PROTO format. The robot described is a simple unicycle consisting of two elements: A steerable wheel and a vertical rod connecting to the wheel:

.. code-block:: python

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

   print(simple_unicycle.proto_str('SimpleUnicycle', [
      'field SFVec3f    translation    0.0 0.0 0.0',
      'field SFRotation rotation       0.0 0.0 1.0 0.0',
   ]))

   print('\n\n\n')

   print(simple_unicycle.urdf_str('simple_unicycle'))

Running this code would output

.. code-block:: python

   #VRML_SIM R2021a utf8
   
   PROTO SimpleUnicycle [
     field SFVec3f    translation    0.0 0.0 0.0
     field SFRotation rotation       0.0 0.0 1.0 0.0
   ]
   {
     Robot {
       children [
         Transform {
           children [
             Hinge2Joint {
               endPoint Solid {
                 physics Physics {
                 }
                 boundingObject Cylinder {
                   radius 0.2
                   height 0.05
                   subdivision 24
                 }
                 name "wheel"
               }
               jointParameters HingeJointParameters {
                 axis 0.0 0.0 1.0
               }
               jointParameters2 JointParameters {
                 axis 0.0 1.0 0.0
               }
             }
           ]
           translation 0.0 0.0 -0.3
         }
       ]
       physics Physics {
       }
       boundingObject Box {
         size 0.05 0.05 0.6
       }
       model "Simple unicycle"
       translation IS translation
       rotation IS rotation
     }
   }
   
   
   
   
   <?xml version="1.0" ?>
   <robot name="simple_unicycle">
       <link name="base_link">
           <visual>
               <origin xyz="0.0 0.0 0.0" rpy="0.0 -0.0 0.0"/>
               <geometry>
                   <box size="0.05 0.05 0.6"/>
               </geometry>
           </visual>
       </link>
       <joint type="continuous" name="_webots_link_0_joint">
           <parent link="base_link"/>
           <child link="_webots_link_0"/>
           <origin xyz="0.0 0.0 -0.3" rpy="0.0 -0.0 0.0"/>
           <axis xyz="0.0 0.0 1.0"/>
       </joint>
       <link name="_webots_link_0"/>
       <joint type="continuous" name="wheel_joint">
           <parent link="_webots_link_0"/>
           <child link="wheel"/>
           <origin xyz="0.0 0.0 0.0" rpy="0.0 -0.0 0.0"/>
           <axis xyz="0.0 1.0 0.0"/>
       </joint>
       <link name="wheel">
           <visual>
               <origin xyz="0.0 0.0 0.0" rpy="0.0 -0.0 0.0"/>
               <geometry>
                   <cylinder radius="0.2" length="0.05"/>
               </geometry>
           </visual>
       </link>
   </robot>

.. automodule:: twizy_description.robot_description
   :members:

**********
Extensions
**********

collada_node
============

.. automodule:: twizy_description.collada_node
   :members:

******************
Indices and tables
******************

* :ref:`genindex`
* :ref:`modindex`
* :ref:`search`

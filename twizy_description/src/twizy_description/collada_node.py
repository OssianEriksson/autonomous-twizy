from twizy_description.robot_description import Node

import numpy as np
import collada
import numbers
from pathlib import Path

class Collada(Node):
    """Represent a collada model in a robot_description data structure

    An instance of this class can be used in a robot_description node tree to
    represent a collada model. Generation of both URDF and proto is supported.
    When generating URDF the <mesh>-tag will be used, and when generating PROTO
    the IndexedFaceSet Webots node will be used, effectively copying the entire
    model representation into the PROTO instead of referencing the original
    collada file.
    """

    def __init__(self, path, urdf_ignore=False):
        """Initialize a collada node
        
        :param path: Path to a collada (.dae) file
        :type path: str
        :param urdf_ignore: Whether to exclude this node when generating URDF
            representations of robots
        :type urdf_ignore: bool
        """

        self.path = path

        shape = self._shape()

        super().__init__(shape.name, shape.fields, urdf_ignore)

    def _indexed_face_set(self, p):
        def terminate(a):
            """Flattens a matrix of coordinates terminating each tuple with -1

            :param a: A matrix where each row represents a tuple of coordinates

            :return: A flat list where each row in the input matrix follows
                after the next, but where every row has been terminated with -1
                to separate out rows
            """

            return np.hstack((a, np.full((a.shape[0], 1), -1))).flatten()
        
        def vec2str(v):
            return ' '.join(f'{a:.6}' for a in v)

        return Node('IndexedFaceSet', {
            'coord': Node('Coordinate', {
                'point': [
                    ', '.join(vec2str(v) for v in p.vertex)
                ]
            }),
            'normal': Node('Normal', {
                'vector': [
                    ', '.join(vec2str(v) for v in p.normal)
                ]
            }),
            'texCoord': Node('TextureCoordinate', {
                'point': [
                    ', '.join(vec2str(v) for v in p.texcoordset[0])
                ]
            }),
            'coordIndex': [
                ', '.join(str(v) for v in terminate(p.vertex_index))
            ],
            'normalIndex': [
                ', '.join(str(v) for v in terminate(p.normal_index))
            ],
            'texCoordIndex': [
                ', '.join(str(v) for v in terminate(p.texcoord_indexset[0]))
            ],
            'convex': 'FALSE'
        })

    def _appearance(self, e):
        fields = {
            'metalness': 0.0,
            'roughness': 1.0
        }

        if e.emission and isinstance(e.emission, tuple):
            fields['emissiveColor'] = ' '.join(str(a) for a in e.emission[:3])
        if e.specular and isinstance(e.specular, tuple):
            s = e.specular
            roughness = 1.0 - s[3] * (s[0] + s[1] + s[2]) / 3.0
            fields['roughness'] = roughness
        if e.shininess:
            fields['roughness'] *= (1.0 - 0.5 * e.shininess)
        if e.diffuse:
            if isinstance(e.diffuse, collada.material.Map):
                texture_path = Path(e.diffuse.sampler.surface.image.path)
                if not texture_path.is_absolute():
                    texture_path = Path(self.path).parent / texture_path
                fields['baseColorMap'] = Node('ImageTexture', {
                    'url': f'"{texture_path}"'
                })
            else:
                fields['baseColor'] = ' '.join(e.diffuse[:3])
                fields['transparency'] = 1.0 - e.diffuse[3]

        return Node('PBRAppearance', fields)

    def _shape(self):
        shapes = []

        c = collada.Collada(self.path)

        for g in c.geometries:
            for p in g.primitives:
                if not isinstance(p, collada.lineset.LineSet):
                    fields = {}
                    fields['geometry'] = self._indexed_face_set(p)

                    if p.material:
                        material = c.materials[p.material]
                        if material and material.effect:
                            appearance = self._appearance(material.effect)
                            fields['appearance'] = appearance

                    shapes.append(Node('Shape', fields))
        
        if len(shapes) == 1:
            return shapes[0]
        
        return Node('Group', {
            'children': shapes
        })

    def _urdf(self, trans, parent):
        return Node('Mesh', {
            'url': Path(self.path).absolute().as_uri()
        })._urdf(trans, parent)
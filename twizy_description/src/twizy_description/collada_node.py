from twizy_description.robot_description import Node

import numpy as np
import collada
import numbers
from pathlib import Path

class Collada(Node):
    def __init__(self, path, urdf_ignore=False):
        self.path = path

        shape = self._shape()

        super().__init__(shape.name, shape.fields, urdf_ignore)

    def _indexed_face_set(self, p):
        def terminate(a):
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
            'metalness': 0
        }

        if e.emission and isinstance(e.emission, tuple):
            fields['emissiveColor'] = ' '.join(str(a) for a in e.emission[:3])
        if e.specular and isinstance(e.specular, tuple):
            s = e.specular
            roughness = 1.0 - s[3] * (s[0] + s[1] + s[2]) / 3.0
            if e.shininess:
                roughness *= (1.0 - 0.5 * e.shininess)
            fields['roughness'] = roughness
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
            'url': Path(self.path).as_uri()
        })._urdf(trans, parent)
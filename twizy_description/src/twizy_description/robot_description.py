from transforms3d.axangles import axangle2aff
from transforms3d.affines import decompose44
from transforms3d.euler import mat2euler
from transforms3d.shears import striu2mat
import numpy as np
from xml.dom import minidom


def translation_matrix(direction):
    M = np.identity(4)
    M[:3, 3] = direction[:3]
    return M


def scale_matrix(axis):
    return np.diag([*axis[:3], 1])


def shear_matrix(striu):
    S = np.identity(4)
    S[:3, :3] = striu2mat(striu)
    return S


def xyz_rpy(transformation):
    T, R, _, _ = decompose44(transformation)
    return (T[0], T[1], T[2]), mat2euler(R)


class _IndentedOutput:
    def __init__(self, tab='  ', lines=[], indent=0):
        self._tab = tab
        self._lines = lines
        self._indent = indent

    def write(self, line, indent=0):
        self._lines.append(self._tab * (self._indent + indent) + line)

    def indent(self):
        return _IndentedOutput(self._tab, self._lines, self._indent + 1)

    def __str__(self):
        return '\n'.join(self._lines)


class _URDFLink:
    index = 0

    def __init__(self, name=None):
        if name is None:
            self.name = f'_webots_link_{_URDFLink.index}'
            _URDFLink.index += 1
        else:
            self.name = name

        self.children = []
        self.visuals = []

    def connect_joint(self, joint):
        joint.parent = self
        self.children.append(joint)

    def connect_link(self, link, trans):
        joint = _URDFJoint('fixed', trans)
        self.connect_joint(joint)
        joint.connect_link(link)

    def connect_visual(self, visual):
        self.visuals.append(visual)

    def xml(self, doc, robot):
        link = doc.createElement('link')
        link.setAttribute('name', self.name)

        if len(self.visuals) > 0:
            for visual in self.visuals:
                visual.xml(doc, link)

        robot.appendChild(link)

        for child in self.children:
            child.xml(doc, robot)


class _URDFJoint:
    index = 0

    def __init__(self, type, trans=None, params=None):
        self.type = type
        self.params = Node('jointParameters') if params is None else params
        self.trans = np.identity(4) if trans is None else trans
        self.parent = None
        self.child = None
        self.name = None

    def connect_joint(self, joint):
        link = _URDFLink()
        self.connect_link(link)
        link.connect_joint(joint)

    def connect_link(self, link, trans=np.identity(4)):
        self.name = f'{link.name}_joint'

        if np.allclose(xyz_rpy(trans), 0):
            self.child = link
        else:
            joint_link = _URDFLink(f'{self.name}_link')
            joint_link.connect_link(link, trans)
            self.child = joint_link

    def connect_visual(self, visual):
        link = _URDFLink()
        self.connect_link(link)
        link.connect_visual(visual)

    def xml(self, doc, robot):
        if self.parent and self.child:
            joint = doc.createElement('joint')
            joint.setAttribute('type', self.type)
            joint.setAttribute('name', self.name)

            parent = doc.createElement('parent')
            parent.appendChild(doc.createTextNode(self.parent.name))
            joint.appendChild(parent)

            child = doc.createElement('child')
            child.appendChild(doc.createTextNode(self.child.name))
            joint.appendChild(child)

            xyz, rpy = xyz_rpy(self.trans)
            origin = doc.createElement('origin')
            origin.setAttribute('xyz', ' '.join(str(a) for a in xyz))
            origin.setAttribute('rpy', ' '.join(str(a) for a in rpy))
            joint.appendChild(origin)

            if self.type != 'fixed':
                axis = doc.createElement('axis')
                axis.setAttribute('xyz', self.params.fields['axis'])
                joint.appendChild(axis)

            robot.appendChild(joint)

            self.child.xml(doc, robot)


class _URDFVisual:
    class _VisualMeta:
        def __init__(self, shear, non_uniform_scale):
            self.shear = shear
            self.non_uniform_scale = non_uniform_scale

    meta = {
        'Box': _VisualMeta(False, True),
        'Cylinder': _VisualMeta(False, False),
        'Sphere': _VisualMeta(False, False),
        'Mesh': _VisualMeta(False, True)
    }

    supported = meta.keys()

    def __init__(self, node, trans):
        self.node = node
        self.trans = trans

    def xml(self, doc, link):
        _, _, Z, S = decompose44(self.trans)
        z = np.mean(Z)

        meta = _URDFVisual.meta[self.node.name]
        if not meta.non_uniform_scale and not np.allclose(Z, z):
            raise ValueError('Non uniform scale not supported for node '
                             f'{self.node.name}')
        if not meta.shear and not np.allclose(S, 0):
            raise ValueError('Scaling which induces shear is not supported '
                             f'for node {self.node.name}')

        fields = self.node.fields

        visual = doc.createElement('visual')

        xyz, rpy = xyz_rpy(self.trans)
        origin = doc.createElement('origin')
        origin.setAttribute('xyz', ' '.join(str(a) for a in xyz))
        origin.setAttribute('rpy', ' '.join(str(a) for a in rpy))
        visual.appendChild(origin)

        geometry = doc.createElement('geometry')
        if self.node.name == 'Box':
            size = [float(a) for a in fields['size'].split()]
            box = doc.createElement('box')
            box.setAttribute('size', ' '.join(str(a) for a in Z * size))
            geometry.appendChild(box)
        elif self.node.name == 'Cylinder':
            cylinder = doc.createElement('cylinder')
            cylinder.setAttribute('radius', str(float(fields['radius']) * z))
            cylinder.setAttribute('length', str(float(fields['height']) * z))
            geometry.appendChild(cylinder)
        elif self.node.name == 'Sphere':
            sphere = doc.createElement('sphere')
            sphere.setAttribute('radius', str(float(fields['radius']) * z))
            geometry.appendChild(sphere)
        elif self.node.name == 'Mesh':
            mesh = doc.createElement('mesh')
            mesh.setAttribute('filename', str(fields['url']))
            mesh.setAttribute('scale', ' '.join(str(a) for a in Z))
            geometry.appendChild(mesh)
        visual.appendChild(geometry)

        link.appendChild(visual)


class Node:
    def __init__(self, name, fields={}, urdf_ignore=False):
        self.name = name
        self.fields = fields
        self.urdf_ignore = urdf_ignore

    def _proto_field(self, out, value, field=''):
        if isinstance(value, list):
            out.write((field and f'{field} ') + '[')
            for v in value:
                self._proto_field(out.indent(), v)
            out.write(']')
        elif isinstance(value, Node):
            value._proto(out, field)
        else:
            out.write((field and f'{field} ') + str(value))

    def _proto(self, out, field=''):
        out.write((field and f'{field} ') + f'{self.name} {{')
        for k, v in self.fields.items():
            self._proto_field(out.indent(), v, k)
        out.write('}')

    def proto_str(self, name, fields=[], header=''):
        out = _IndentedOutput()
        out.write('#VRML_SIM R2021a utf8')
        for line in header.split('\n'):
            out.write(('' if not line or line.isspace() else '# ') + line)
        out.write(f'PROTO {name} [')
        for field in fields:
            out.write(field, 1)
        out.write(']')
        out.write('{')
        self._proto(out.indent())
        out.write('}')

        return str(out)

    def translation(self):
        t = self.fields.get('translation', '').split()
        if len(t) != 3 or t[0] == 'IS':
            return np.identity(4)
        return translation_matrix([float(a) for a in t])

    def rotation(self):
        r = self.fields.get('rotation', '').split()
        r = self.fields.get('rotation', '').split()
        if len(r) != 4 or r[0] == 'IS':
            return np.identity(4)
        return axangle2aff([float(a) for a in r[:3]], float(r[3]))

    def scale(self):
        s = self.fields.get('scale', '').split()
        if len(s) != 3 or s[0] == 'IS':
            return np.identity(4)
        return scale_matrix([float(a) for a in s])

    def _reset_trans(self, trans):
        _, _, Z, S = decompose44(trans)
        return scale_matrix(Z) * shear_matrix(S)

    def _urdf(self, trans, parent):
        if self.urdf_ignore:
            return

        trans = trans @ self.translation() @ self.rotation() @ self.scale()

        jointType = {
            'HingeJoint': ('continuous', ['']),
            'SliderJoint': ('prismatic', ['']),
            'Hinge2Joint': ('continuous', ['', '2']),
            'BallJoint': ('continuous', ['', '2', '3']),
        }.get(self.name)

        if jointType:
            for postfix in jointType[1]:
                params = self.fields.get(f'jointParameters{postfix}')
                joint = _URDFJoint(jointType[0], trans, params)
                parent.connect_joint(joint)
                parent = joint
                trans = self._reset_trans(trans)

            endPoint = self.fields.get('endPoint')
            if endPoint and isinstance(endPoint, Node):
                endPoint._urdf(trans, parent)
        elif self.name in _URDFVisual.supported:
            parent.connect_visual(_URDFVisual(self, trans))
        elif self.name == 'Capsule':
            r = float(self.fields['radius'])
            h = float(self.fields['height'])

            if self.fields.get('side', '').lower() != 'FALSE':
                cylinder = Node('Cylinder', {'radius': r, 'height': h})
                parent.connect_visual(_URDFVisual(cylinder, trans))
            if self.fields.get('top', '').lower() != 'FALSE':
                sphere = Node('Sphere', {'radius': self.fields['radius']})
                trans_ = trans @ translation_matrix([0, h / 2.0, 0])
                parent.connect_visual(_URDFVisual(sphere, trans_))
            if self.fields.get('bottom', '').lower() != 'FALSE':
                sphere = Node('Sphere', {'radius': self.fields['radius']})
                trans_ = trans @ translation_matrix([0, -h / 2.0, 0])
                parent.connect_visual(_URDFVisual(sphere, trans_))
        elif self.name == 'Shape':
            geometry = self.fields.get('geometry')
            if geometry and isinstance(geometry, Node):
                geometry._urdf(trans, parent)
        else:
            name = self.fields.get('name')
            if name and isinstance(name, str):
                link = _URDFLink(name[1:-1])
                parent.connect_link(link, trans)
                parent = link
                trans = self._reset_trans(trans)

            children = self.fields.get('children')
            if children and isinstance(children, list):
                for child in children:
                    child._urdf(trans, parent)

            bounding_obj = self.fields.get('boundingObject')
            if bounding_obj and isinstance(bounding_obj, Node):
                bounding_obj._urdf(trans, parent)

    def urdf_str(self, name, base='base_link', transformation=np.identity(4),
                 header=''):
        _URDFLink.index, _URDFJoint.index = 0, 0

        base_link = _URDFLink(base)
        self._urdf(transformation, base_link)

        doc = minidom.Document()

        if header:
            doc.appendChild(doc.createComment(header))

        robot = doc.createElement('robot')
        robot.setAttribute('name', name)

        base_link.xml(doc, robot)

        doc.appendChild(robot)

        return doc.toprettyxml(indent='    ')

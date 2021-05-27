"""Module for generating PROTO and URDF files

The physics/robotics simulator webots uses files in a PROTO format while most
ROS-packages work with URDF, which is an xml format.
"""


from transforms3d.axangles import axangle2aff
from transforms3d.affines import decompose44
from transforms3d.euler import mat2euler
from transforms3d.shears import striu2mat
import numpy as np
from xml.dom import minidom


def translation_matrix(direction):
    """Create a translation matrix

    :param direction: A three dimensional vector

    :return: A 4x4 matrix representing a translation by the provided vector
    """

    M = np.identity(4)
    M[:3, 3] = direction[:3]
    return M


def scale_matrix(axis):
    """Create a scale matrix

    :param axis: A three dimensional vector

    :return: A 4x4 matrix representing scaling around the major axis in 3D by
        the amounts specified in the input vector
    """

    return np.diag([*axis[:3], 1])


def shear_matrix(striu):
    """Convert a 3x3 matrix representation of shear to a 4x4 representation

    :striu: A 3x3 shear matrix

    :return: A 4x4 shear matrix
    """

    S = np.identity(4)
    S[:3, :3] = striu2mat(striu)
    return S


def xyz_rpy(transformation):
    """Decompose a transformation matrix into translation and euler angles

    :param transformation: A 4x4 transformation matrix

    :return: A tuple of (x, y, z, r, p, y) where (x, y, z) are coordinates of a
        cartesian translation and (r, p, y) are roll, pitch, yaw applied in
        that order around fixed axis
    """

    T, R, _, _ = decompose44(transformation)
    return (T[0], T[1], T[2]), mat2euler(R)


class _IndentedOutput:
    """Convenience class for generating text with indentation"""

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
    """Representation of an URDF <link> element"""

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
        """Connect a joint (as a child) to this link"""

        joint.parent = self
        self.children.append(joint)

    def connect_link(self, link, trans):
        """Connect a link (as a child) to this link"""

        # Create a new joint to sit between this link and the provided link
        joint = _URDFJoint('fixed', trans)
        self.connect_joint(joint)
        joint.connect_link(link)

    def connect_visual(self, visual):
        """Connect a visual element (as a child) to this link"""

        self.visuals.append(visual)

    def xml(self, doc, robot):
        """Represent this link and all its children as xml"""

        link = doc.createElement('link')
        link.setAttribute('name', self.name)

        if len(self.visuals) > 0:
            for visual in self.visuals:
                visual.xml(doc, link)

        robot.appendChild(link)

        for child in self.children:
            child.xml(doc, robot)


class _URDFJoint:
    """Representation of an URDF <joint> element"""

    index = 0

    def __init__(self, type, trans=None, params=None):
        self.type = type
        self.params = Node('jointParameters') if params is None else params
        self.trans = np.identity(4) if trans is None else trans
        self.parent = None
        self.child = None
        self.name = None

    def connect_joint(self, joint):
        """Connect a joint (as a child) to this joint"""

        # Create a new link to sit between this joint and the provided joint
        link = _URDFLink()
        self.connect_link(link)
        link.connect_joint(joint)

    def connect_link(self, link, trans=np.identity(4)):
        """Connect a link (as a child) to this joint"""

        self.name = f'{link.name}_joint'

        # If the relative position/rotation of the provided link is zero we can
        # connect the link directly to this joint, otherwise we need another
        # link to sit right where this joint ends which then connects to the
        # provided link through another joint
        if np.allclose(xyz_rpy(trans), 0):
            self.child = link
        else:
            joint_link = _URDFLink(f'{self.name}_link')
            joint_link.connect_link(link, trans)
            self.child = joint_link

    def connect_visual(self, visual):
        """Connect a visual element (as a child) to this joint"""

        link = _URDFLink()
        self.connect_link(link)
        link.connect_visual(visual)

    def xml(self, doc, robot):
        """Represent this joint and all its children as xml"""

        # A joint is only valid if it has both a parent and a child
        if self.parent and self.child:
            joint = doc.createElement('joint')
            joint.setAttribute('type', self.type)
            joint.setAttribute('name', self.name)

            parent = doc.createElement('parent')
            parent.setAttribute('link', self.parent.name)
            joint.appendChild(parent)

            child = doc.createElement('child')
            child.setAttribute('link', self.child.name)
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
    """Representation of an URDF <visual> element"""

    class _VisualMeta:
        """Some properties held by certain types of visual elements"""

        def __init__(self, shear, non_uniform_scale):
            self.shear = shear
            self.non_uniform_scale = non_uniform_scale

    meta = {
        'Box': _VisualMeta(False, True),
        'Cylinder': _VisualMeta(False, False),
        'Sphere': _VisualMeta(False, False),
        'Mesh': _VisualMeta(False, True)
    }

    # List of supported Webots PROTO nodes
    supported = meta.keys()

    def __init__(self, node, trans):
        self.node = node
        self.trans = trans

    def xml(self, doc, link):
        """Represent this visual element as xml"""

        # Extract scale and shear from transformation
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
    """Representation of a node in a robot_description tree
    
    A robot_description tree is a representation of a robot with joints, visual
    elements and sensors. This structure can then be used to generate webots
    PROTO or URDF representations of the robot. The structure and names of
    nodes of this tree is very similar to the Webots PROTO format. This
    similarity is the reason why all features of the Webots PROTO format can be
    generated using a robot_description node tree. The generation of URDF is
    however compartively limited. The generated URDF will contain supported
    visual elements, and all nodes with the "name" field defined as links.
    
    The generation of URDF models is somewhat clever in that it will discard
    intermittent nodes with no name and instead accumulate transforms between
    parent and children nodes.
    """

    def __init__(self, name, fields={}, urdf_ignore=False):
        """Initialize a robot_description node

        :param name: Name of the node type. Names are identical to the node
            names defined by the webots PROTO format, e.g. "Transform",
            "Solid", "GPS", "Robot" etc.webots_descr
        :type name: str
        :param fields: Fields of this node which are also identical to the
            webots Node fields. Values can be strings, floats or integers or
            lists. Vectors and floats are represed like '1.23 0.34 0.56' or
            '0.56'. Note that strings must be quoted like
            '"This is a string value"'
        :type fields: dict
        :param urdf_ignore: Whether to exclude this node and all its children
            when generating URDF representations of robots
        :type urdf_ignore: bool
        """

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
        """Generate a PROTO string representation of this node tree

        :param name: Name of the PROTO (must be the same as an eventual file
            name according to webots specifications)
        :type name: str
        :param fields: List of field definitions for the generated PROTO, one
            list element might for example be
            'field SFVec3f translation 0.0 0.0 0.0'
        :type fields: list
        :param header: A header to put in a comment at the start of the
            generated PROTO
        :type header: str

        :return: A PROTO string representation of this node and it's children
        """

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
        """4x4 translanslation matrix of this node relative to its parent"""

        t = self.fields.get('translation', '').split()
        if len(t) != 3 or t[0] == 'IS':
            return np.identity(4)
        return translation_matrix([float(a) for a in t])

    def rotation(self):
        """4x4 rotation matrix of this node relative to its parent"""

        r = self.fields.get('rotation', '').split()
        if len(r) != 4 or r[0] == 'IS':
            return np.identity(4)
        return axangle2aff([float(a) for a in r[:3]], float(r[3]))

    def scale(self):
        """4x4 scale matrix of this node relative to its parent"""

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
        """Generate a URDF string representation of this node tree

        :param name: Name of the robot
        :type name: str
        :param base: Name of the base link in the URDF representation
        :type base: str
        :param transformation: A 4x4 transformation matrix to apply to the
            robot before generating its URDF representation
        :param header: A header to put in a comment at the start of the
            generated URDF
        :type header: str

        :return: A URDF string representation of this node and it's children
        """

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

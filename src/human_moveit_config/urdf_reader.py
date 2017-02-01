#!/usr/bin/env python
from urdf_parser_py.urdf import URDF
from os.path import join
from rospkg import RosPack


class URDFReader(object):
    """docstring for URDFReader"""
    def __init__(self):
        rospack = RosPack()
        urdf_file = join(rospack.get_path('human_moveit_config'), 'urdf', 'human.urdf')
        model = URDF.from_xml_file(urdf_file)
        self.model_dict = {}
        for j in model.joints:
            self.model_dict[j.child] = {}
            self.model_dict[j.child]['parent_joint'] = {'name': j.name, 'type': j.type}
            self.model_dict[j.child]['parent_link'] = j.parent

    def find_common_root(self, link1, link2):
        # the worst common root is base
        path1 = self.get_links_chain(link1, 'human/base')
        path2 = self.get_links_chain(link2, 'human/base')
        # get shortest path
        if len(path1) < len(path2):
            short_path = path1
            long_path = path2
        else:
            short_path = path2
            long_path = path1
        # loop till find a common root
        for l1 in short_path:
            for l2 in long_path:
                if l1 == l2:
                    return l1

    def get_links_chain(self, from_link, to_link):
        def _get_links_chain_sub(from_l, to_l, path=[]):
            path += [from_l]
            # this condition means the chain is not found
            if from_l == 'human/base' and to_l != 'human/base':
                return []
            # stop condition, path found
            elif from_l == to_l:
                return path
            else:
                return _get_links_chain_sub(self.model_dict[from_l]['parent_link'], to_l, path)
        path = _get_links_chain_sub(from_link, to_link, [])
        # no results, try the opposite order
        if not path:
            path = _get_links_chain_sub(to_link, from_link, [])
        # still no results, find a common root
        if not path:
            common_root = self.find_common_root(from_link, to_link)
            path = _get_links_chain_sub(from_link, common_root, []) + _get_links_chain_sub(to_link, common_root, [])
            # the common root is duplicated, remove one
            path.remove(common_root)
        return path

    def get_joints_chain(self, from_link, to_link):
        def _get_joints_chain_sub(from_l, to_l, path=[]):
            # this condition means the chain is not found
            if from_l == 'human/base' and to_l != 'human/base':
                return []
            # stop condition, path found
            elif from_l == to_l:
                return path
            else:
                joint = self.model_dict[from_l]['parent_joint']
                # only add active joints
                if joint['type'] != 'fixed':
                    path += [joint['name']]
                return _get_joints_chain_sub(self.model_dict[from_l]['parent_link'], to_l, path)
        path = _get_joints_chain_sub(from_link, to_link, [])
        # no results, try the opposite order
        if not path:
            path = _get_joints_chain_sub(to_link, from_link, [])
        # still no results, find a common root
        if not path:
            common_root = self.find_common_root(from_link, to_link)
            path = _get_joints_chain_sub(from_link, common_root, []) + _get_joints_chain_sub(to_link, common_root, [])
        return path

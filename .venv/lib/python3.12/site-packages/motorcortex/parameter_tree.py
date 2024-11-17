#!/usr/bin/python3

#
#   Developer : Alexey Zakharov (alexey.zakharov@vectioneer.com)
#   All rights reserved. Copyright (c) 2016 VECTIONEER.
#

class ParameterTree(object):
    """This class represents a parameter tree, obtained from the server.

        Reference to a parameter tree instance is needed for resolving parameters,
        data types and other information to build a correct request message.
    """

    def __init__(self):
        self.__parameter_tree = []
        self.__parameter_map = dict()
        pass

    def load(self, parameter_tree_msg):
        """Loads a parameter tree from ParameterTreeMsg received from the server

            Args:
                parameter_tree_msg(ParameterTreeMsg): parameter tree message from the server

            Examples:
                >>> parameter_tree = motorcortex.ParameterTree()
                >>> parameter_tree_msg = param_tree_reply.get()
                >>> parameter_tree.load(parameter_tree_msg)
        """

        self.__parameter_tree = parameter_tree_msg.params
        for param in self.__parameter_tree:
            self.__parameter_map[param.path] = param

    def getParameterTree(self):
        """
            Returns:
                list(ParameterInfo): a list of parameter descriptions
        """

        return self.__parameter_tree

    def getInfo(self, parameter_path):
        """
            Args:
                parameter_path(str): path of the parameter

            Returns:
                ParameterInfo: parameter description
        """

        return self.__parameter_map.get(parameter_path)

    def getDataType(self, parameter_path):
        """
            Args:
                parameter_path(str): path of the parameter

            Returns:
                DataType: parameter data type

        """

        info = self.getInfo(parameter_path)
        if info:
            return info.data_type

        return None

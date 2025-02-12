"""Support for simplified access to data on nodes of type omni.isaac.ros2_bridge.ROS2QoSProfile

 __   ___ .  .  ___  __       ___  ___  __      __   __   __   ___
/ _` |__  |\ | |__  |__)  /\   |  |__  |  \    /  ` /  \ |  \ |__
\__| |___ | \| |___ |  \ /--\  |  |___ |__/    \__, \__/ |__/ |___

 __   __     .  .  __  ___     .  .  __   __     ___
|  \ /  \    |\ | /  \  |      |\/| /  \ |  \ | |__  \ /
|__/ \__/    | \| \__/  |      |  | \__/ |__/ | |     |

This node generates a JSON config of a QoS Profile
"""

import carb
import sys
import traceback

import omni.graph.core as og
import omni.graph.core._omni_graph_core as _og
import omni.graph.tools.ogn as ogn



class OgnROS2QoSProfileDatabase(og.Database):
    """Helper class providing simplified access to data on nodes of type omni.isaac.ros2_bridge.ROS2QoSProfile

    Class Members:
        node: Node being evaluated

    Attribute Value Properties:
        Inputs:
            inputs.createProfile
            inputs.deadline
            inputs.depth
            inputs.durability
            inputs.history
            inputs.leaseDuration
            inputs.lifespan
            inputs.liveliness
            inputs.reliability
        Outputs:
            outputs.qosProfile

    Predefined Tokens:
        tokens.defaultPubSub
        tokens.services
        tokens.sensorData
        tokens.systemDefault
        tokens.custom
        tokens.transientLocal
        tokens.volatile
        tokens.unknown
        tokens.keepLast
        tokens.keepAll
        tokens.automatic
        tokens.manualByTopic
        tokens.reliable
        tokens.bestEffort
    """

    # Imprint the generator and target ABI versions in the file for JIT generation
    GENERATOR_VERSION = (1, 79, 0)
    TARGET_VERSION = (0, 0, 0)

    # This is an internal object that provides per-class storage of a per-node data dictionary
    PER_NODE_DATA = {}

    # This is an internal object that describes unchanging attributes in a generic way
    # The values in this list are in no particular order, as a per-attribute tuple
    #     Name, Type, ExtendedTypeIndex, UiName, Description, Metadata,
    #     Is_Required, DefaultValue, Is_Deprecated, DeprecationMsg
    # You should not need to access any of this data directly, use the defined database interfaces
    INTERFACE = og.Database._get_interface([
        ('inputs:createProfile', 'token', 0, None, 'Preset profile configs. Choosing a QoS profile will update the policies accordingly.', {ogn.MetadataKeys.ALLOWED_TOKENS: 'Default for publishers/subscribers,Services,Sensor Data,System Default,Custom', ogn.MetadataKeys.ALLOWED_TOKENS_RAW: '{"defaultPubSub": "Default for publishers/subscribers", "services": "Services", "sensorData": "Sensor Data", "systemDefault": "System Default", "custom": "Custom"}', ogn.MetadataKeys.DEFAULT: '"Default for publishers/subscribers"'}, True, "Default for publishers/subscribers", False, ''),
        ('inputs:deadline', 'double', 0, None, 'Deadline policy. Defined in seconds', {ogn.MetadataKeys.DEFAULT: '0.0'}, True, 0.0, False, ''),
        ('inputs:depth', 'uint64', 0, None, "Depth (Queue size) policy. Only honored if the'history' policy was set to 'keepLast'.", {ogn.MetadataKeys.DEFAULT: '10'}, True, 10, False, ''),
        ('inputs:durability', 'token', 0, None, 'Durability policy', {ogn.MetadataKeys.ALLOWED_TOKENS: 'systemDefault,transientLocal,volatile,unknown', ogn.MetadataKeys.ALLOWED_TOKENS_RAW: '{"systemDefault": "systemDefault", "transientLocal": "transientLocal", "volatile": "volatile", "unknown": "unknown"}', ogn.MetadataKeys.DEFAULT: '"volatile"'}, True, "volatile", False, ''),
        ('inputs:history', 'token', 0, None, 'History policy', {ogn.MetadataKeys.ALLOWED_TOKENS: 'systemDefault,keepLast,keepAll,unknown', ogn.MetadataKeys.ALLOWED_TOKENS_RAW: '{"systemDefault": "systemDefault", "keepLast": "keepLast", "keepAll": "keepAll", "unknown": "unknown"}', ogn.MetadataKeys.DEFAULT: '"keepLast"'}, True, "keepLast", False, ''),
        ('inputs:leaseDuration', 'double', 0, None, 'Lease Duration policy. Defined in seconds', {ogn.MetadataKeys.DEFAULT: '0.0'}, True, 0.0, False, ''),
        ('inputs:lifespan', 'double', 0, None, 'Lifespan policy. Defined in seconds', {ogn.MetadataKeys.DEFAULT: '0.0'}, True, 0.0, False, ''),
        ('inputs:liveliness', 'token', 0, None, 'Liveliness policy', {ogn.MetadataKeys.ALLOWED_TOKENS: 'systemDefault,automatic,manualByTopic,unknown', ogn.MetadataKeys.ALLOWED_TOKENS_RAW: '{"systemDefault": "systemDefault", "automatic": "automatic", "manualByTopic": "manualByTopic", "unknown": "unknown"}', ogn.MetadataKeys.DEFAULT: '"systemDefault"'}, True, "systemDefault", False, ''),
        ('inputs:reliability', 'token', 0, None, 'Reliability policy', {ogn.MetadataKeys.ALLOWED_TOKENS: 'systemDefault,reliable,bestEffort,unknown', ogn.MetadataKeys.ALLOWED_TOKENS_RAW: '{"systemDefault": "systemDefault", "reliable": "reliable", "bestEffort": "bestEffort", "unknown": "unknown"}', ogn.MetadataKeys.DEFAULT: '"reliable"'}, True, "reliable", False, ''),
        ('outputs:qosProfile', 'string', 0, None, 'QoS profile config', {}, True, None, False, ''),
    ])

    class tokens:
        defaultPubSub = "Default for publishers/subscribers"
        services = "Services"
        sensorData = "Sensor Data"
        systemDefault = "systemDefault"
        custom = "Custom"
        transientLocal = "transientLocal"
        volatile = "volatile"
        unknown = "unknown"
        keepLast = "keepLast"
        keepAll = "keepAll"
        automatic = "automatic"
        manualByTopic = "manualByTopic"
        reliable = "reliable"
        bestEffort = "bestEffort"

    @classmethod
    def _populate_role_data(cls):
        """Populate a role structure with the non-default roles on this node type"""
        role_data = super()._populate_role_data()
        role_data.outputs.qosProfile = og.AttributeRole.TEXT
        return role_data

    class ValuesForInputs(og.DynamicAttributeAccess):
        LOCAL_PROPERTY_NAMES = {"createProfile", "deadline", "depth", "durability", "history", "leaseDuration", "lifespan", "liveliness", "reliability", "_setting_locked", "_batchedReadAttributes", "_batchedReadValues"}
        """Helper class that creates natural hierarchical access to input attributes"""
        def __init__(self, node: og.Node, attributes, dynamic_attributes: og.DynamicAttributeInterface):
            """Initialize simplified access for the attribute data"""
            context = node.get_graph().get_default_graph_context()
            super().__init__(context, node, attributes, dynamic_attributes)
            self._batchedReadAttributes = [self._attributes.createProfile, self._attributes.deadline, self._attributes.depth, self._attributes.durability, self._attributes.history, self._attributes.leaseDuration, self._attributes.lifespan, self._attributes.liveliness, self._attributes.reliability]
            self._batchedReadValues = ["Default for publishers/subscribers", 0.0, 10, "volatile", "keepLast", 0.0, 0.0, "systemDefault", "reliable"]

        @property
        def createProfile(self):
            return self._batchedReadValues[0]

        @createProfile.setter
        def createProfile(self, value):
            self._batchedReadValues[0] = value

        @property
        def deadline(self):
            return self._batchedReadValues[1]

        @deadline.setter
        def deadline(self, value):
            self._batchedReadValues[1] = value

        @property
        def depth(self):
            return self._batchedReadValues[2]

        @depth.setter
        def depth(self, value):
            self._batchedReadValues[2] = value

        @property
        def durability(self):
            return self._batchedReadValues[3]

        @durability.setter
        def durability(self, value):
            self._batchedReadValues[3] = value

        @property
        def history(self):
            return self._batchedReadValues[4]

        @history.setter
        def history(self, value):
            self._batchedReadValues[4] = value

        @property
        def leaseDuration(self):
            return self._batchedReadValues[5]

        @leaseDuration.setter
        def leaseDuration(self, value):
            self._batchedReadValues[5] = value

        @property
        def lifespan(self):
            return self._batchedReadValues[6]

        @lifespan.setter
        def lifespan(self, value):
            self._batchedReadValues[6] = value

        @property
        def liveliness(self):
            return self._batchedReadValues[7]

        @liveliness.setter
        def liveliness(self, value):
            self._batchedReadValues[7] = value

        @property
        def reliability(self):
            return self._batchedReadValues[8]

        @reliability.setter
        def reliability(self, value):
            self._batchedReadValues[8] = value

        def __getattr__(self, item: str):
            if item in self.LOCAL_PROPERTY_NAMES:
                return object.__getattribute__(self, item)
            else:
                return super().__getattr__(item)

        def __setattr__(self, item: str, new_value):
            if item in self.LOCAL_PROPERTY_NAMES:
                object.__setattr__(self, item, new_value)
            else:
                super().__setattr__(item, new_value)

        def _prefetch(self):
            readAttributes = self._batchedReadAttributes
            newValues = _og._prefetch_input_attributes_data(readAttributes)
            if len(readAttributes) == len(newValues):
                self._batchedReadValues = newValues

    class ValuesForOutputs(og.DynamicAttributeAccess):
        LOCAL_PROPERTY_NAMES = {"qosProfile", "_batchedWriteValues"}
        """Helper class that creates natural hierarchical access to output attributes"""
        def __init__(self, node: og.Node, attributes, dynamic_attributes: og.DynamicAttributeInterface):
            """Initialize simplified access for the attribute data"""
            context = node.get_graph().get_default_graph_context()
            super().__init__(context, node, attributes, dynamic_attributes)
            self.qosProfile_size = None
            self._batchedWriteValues = { }

        @property
        def qosProfile(self):
            value = self._batchedWriteValues.get(self._attributes.qosProfile)
            if value:
                return value
            else:
                data_view = og.AttributeValueHelper(self._attributes.qosProfile)
                return data_view.get()

        @qosProfile.setter
        def qosProfile(self, value):
            self._batchedWriteValues[self._attributes.qosProfile] = value

        def __getattr__(self, item: str):
            if item in self.LOCAL_PROPERTY_NAMES:
                return object.__getattribute__(self, item)
            else:
                return super().__getattr__(item)

        def __setattr__(self, item: str, new_value):
            if item in self.LOCAL_PROPERTY_NAMES:
                object.__setattr__(self, item, new_value)
            else:
                super().__setattr__(item, new_value)

        def _commit(self):
            _og._commit_output_attributes_data(self._batchedWriteValues)
            self._batchedWriteValues = { }

    class ValuesForState(og.DynamicAttributeAccess):
        """Helper class that creates natural hierarchical access to state attributes"""
        def __init__(self, node: og.Node, attributes, dynamic_attributes: og.DynamicAttributeInterface):
            """Initialize simplified access for the attribute data"""
            context = node.get_graph().get_default_graph_context()
            super().__init__(context, node, attributes, dynamic_attributes)

    def __init__(self, node):
        super().__init__(node)
        dynamic_attributes = self.dynamic_attribute_data(node, og.AttributePortType.ATTRIBUTE_PORT_TYPE_INPUT)
        self.inputs = OgnROS2QoSProfileDatabase.ValuesForInputs(node, self.attributes.inputs, dynamic_attributes)
        dynamic_attributes = self.dynamic_attribute_data(node, og.AttributePortType.ATTRIBUTE_PORT_TYPE_OUTPUT)
        self.outputs = OgnROS2QoSProfileDatabase.ValuesForOutputs(node, self.attributes.outputs, dynamic_attributes)
        dynamic_attributes = self.dynamic_attribute_data(node, og.AttributePortType.ATTRIBUTE_PORT_TYPE_STATE)
        self.state = OgnROS2QoSProfileDatabase.ValuesForState(node, self.attributes.state, dynamic_attributes)

    class abi:
        """Class defining the ABI interface for the node type"""

        @staticmethod
        def get_node_type():
            get_node_type_function = getattr(OgnROS2QoSProfileDatabase.NODE_TYPE_CLASS, 'get_node_type', None)
            if callable(get_node_type_function):  # pragma: no cover
                return get_node_type_function()
            return 'omni.isaac.ros2_bridge.ROS2QoSProfile'

        @staticmethod
        def compute(context, node):
            def database_valid():
                return True
            try:
                per_node_data = OgnROS2QoSProfileDatabase.PER_NODE_DATA[node.node_id()]
                db = per_node_data.get('_db')
                if db is None:
                    db = OgnROS2QoSProfileDatabase(node)
                    per_node_data['_db'] = db
                if not database_valid():
                    per_node_data['_db'] = None
                    return False
            except:
                db = OgnROS2QoSProfileDatabase(node)

            try:
                compute_function = getattr(OgnROS2QoSProfileDatabase.NODE_TYPE_CLASS, 'compute', None)
                if callable(compute_function) and compute_function.__code__.co_argcount > 1:  # pragma: no cover
                    return compute_function(context, node)

                db.inputs._prefetch()
                db.inputs._setting_locked = True
                with og.in_compute():
                    return OgnROS2QoSProfileDatabase.NODE_TYPE_CLASS.compute(db)
            except Exception as error:  # pragma: no cover
                stack_trace = "".join(traceback.format_tb(sys.exc_info()[2].tb_next))
                db.log_error(f'Assertion raised in compute - {error}\n{stack_trace}', add_context=False)
            finally:
                db.inputs._setting_locked = False
                db.outputs._commit()
            return False

        @staticmethod
        def initialize(context, node):
            OgnROS2QoSProfileDatabase._initialize_per_node_data(node)
            initialize_function = getattr(OgnROS2QoSProfileDatabase.NODE_TYPE_CLASS, 'initialize', None)
            if callable(initialize_function):  # pragma: no cover
                initialize_function(context, node)

            per_node_data = OgnROS2QoSProfileDatabase.PER_NODE_DATA[node.node_id()]

            def on_connection_or_disconnection(*args):
                per_node_data['_db'] = None

            node.register_on_connected_callback(on_connection_or_disconnection)
            node.register_on_disconnected_callback(on_connection_or_disconnection)

        @staticmethod
        def release(node):
            release_function = getattr(OgnROS2QoSProfileDatabase.NODE_TYPE_CLASS, 'release', None)
            if callable(release_function):  # pragma: no cover
                release_function(node)
            OgnROS2QoSProfileDatabase._release_per_node_data(node)

        @staticmethod
        def init_instance(node, graph_instance_id):
            init_instance_function = getattr(OgnROS2QoSProfileDatabase.NODE_TYPE_CLASS, 'init_instance', None)
            if callable(init_instance_function):  # pragma: no cover
                init_instance_function(node, graph_instance_id)

        @staticmethod
        def release_instance(node, graph_instance_id):
            release_instance_function = getattr(OgnROS2QoSProfileDatabase.NODE_TYPE_CLASS, 'release_instance', None)
            if callable(release_instance_function):  # pragma: no cover
                release_instance_function(node, graph_instance_id)
            OgnROS2QoSProfileDatabase._release_per_node_instance_data(node, graph_instance_id)

        @staticmethod
        def update_node_version(context, node, old_version, new_version):
            update_node_version_function = getattr(OgnROS2QoSProfileDatabase.NODE_TYPE_CLASS, 'update_node_version', None)
            if callable(update_node_version_function):  # pragma: no cover
                return update_node_version_function(context, node, old_version, new_version)
            return False

        @staticmethod
        def initialize_type(node_type):
            initialize_type_function = getattr(OgnROS2QoSProfileDatabase.NODE_TYPE_CLASS, 'initialize_type', None)
            needs_initializing = True
            if callable(initialize_type_function):  # pragma: no cover
                needs_initializing = initialize_type_function(node_type)
            if needs_initializing:
                node_type.set_metadata(ogn.MetadataKeys.EXTENSION, "omni.isaac.ros2_bridge")
                node_type.set_metadata(ogn.MetadataKeys.UI_NAME, "ROS2 QoS Profile")
                node_type.set_metadata(ogn.MetadataKeys.CATEGORIES, "isaacRos2")
                node_type.set_metadata(ogn.MetadataKeys.DESCRIPTION, "This node generates a JSON config of a QoS Profile")
                node_type.set_metadata(ogn.MetadataKeys.LANGUAGE, "Python")
                icon_path = carb.tokens.get_tokens_interface().resolve("${omni.isaac.ros2_bridge}")
                icon_path = icon_path + '/' + "ogn/icons/omni.isaac.ros2_bridge.ROS2QoSProfile.svg"
                node_type.set_metadata(ogn.MetadataKeys.ICON_PATH, icon_path)
                OgnROS2QoSProfileDatabase.INTERFACE.add_to_node_type(node_type)

        @staticmethod
        def on_connection_type_resolve(node):
            on_connection_type_resolve_function = getattr(OgnROS2QoSProfileDatabase.NODE_TYPE_CLASS, 'on_connection_type_resolve', None)
            if callable(on_connection_type_resolve_function):  # pragma: no cover
                on_connection_type_resolve_function(node)

    NODE_TYPE_CLASS = None

    @staticmethod
    def register(node_type_class):
        OgnROS2QoSProfileDatabase.NODE_TYPE_CLASS = node_type_class
        og.register_node_type(OgnROS2QoSProfileDatabase.abi, 1)

    @staticmethod
    def deregister():
        og.deregister_node_type("omni.isaac.ros2_bridge.ROS2QoSProfile")

local utils = import 'cyphal_bridge_utils.jsonnet';

{
    node_id: 98,
    interface: "vcan0",
    connections: utils.base_connections
}

local utils = import 'cyphal_bridge_utils.jsonnet';

{
    node_id: 98,
    interface: "vcan0",
    connections: utils.base_connections + [
        {
            type: "HMI.Led",
            cyphal: {
                port: 172,
                node: 9
            },
            ros: {
                type: "service",
                name: "/hmi/led"
            }
        },
        {
            type: "HMI.Beeper",
            cyphal: {
                port: 258,
                node: 9
            },
            ros: {
                type: "service",
                name: "/hmi/beeper"
            }
        }
    ]
}

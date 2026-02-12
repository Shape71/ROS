local DRIVE_STATE_SIMPLE_PORT = 427;
local ANGULAR_VELOCITY_PORT = 7006;

local drive_state(node_id) = {
    type: "DriveStateSimple",
    cyphal: {
        port: DRIVE_STATE_SIMPLE_PORT,
        node: node_id
    },
    ros: {
        type: "topic",
        direction: "read",
        name: "/node" + std.toString(node_id) + "/drive_state"
    }
};

local set_velocity(node_id) = {
    type: "AngularVelocity",
    cyphal: {
        port: ANGULAR_VELOCITY_PORT + node_id
    },
    ros: {
        type: "topic",
        direction: "write",
        name: "/node" + std.toString(node_id) + "/angular_velocity"
    }
};

{
    base_connections:: [
        {
            type: "DiagnosticArray",
            cyphal: {
                port: 8184
            },
            ros: {
                type: "topic",
                direction: "read",
                name: "/diagnostics"
            },
        },
        {
            type: "Battery",
            cyphal: {
                port: 7993
            },
            ros: {
                type: "topic",
                direction: "read",
                name: "/bat"
            },
        },
        {
            type: "PowerButtons",
            cyphal: {
                port: 8003
            },
            ros: {
                type: "topic",
                direction: "read",
                name: "/buttons"
            },
        }
    ] + [drive_state(node_id) for node_id in [1, 2, 3, 4]] +
        [set_velocity(node_id) for node_id in [1, 2, 3, 4]],
}

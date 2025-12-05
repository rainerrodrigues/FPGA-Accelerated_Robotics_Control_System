#!/usr/bin/env julia
using RobotOS    # if you use RobotOS.jl; otherwise adapt for ROS2.jl
using JSON
using LinearAlgebra

function main()
    init()
    node = Node("julia_controller")
    # publishers & subscribers
    cmd_pub = Publisher(node, "/fpga/command", std_msgs.String)
    state_sub = Subscriber(node, "/fpga/state", std_msgs.String) do msg
        # msg.data is raw json from FPGA => parse if needed
        fpga_state = JSON.parse(msg.data)
        # you might update an estimator here
    end

    # Simple loop: publish a velocity command every 50 ms
    rate = Rate(20) # 20 Hz
    t = 0.0
    while ok()
        # Replace with your control logic (MPC, LQR) using sensor state
        left_v = 0.5      # rad/s or whatever units your FPGA expects
        right_v = 0.5
        payload = JSON.json(Dict("cmd"=>"motors", "left"=>left_v, "right"=>right_v))
        outmsg = std_msgs.String()
        outmsg.data = payload
        publish(cmd_pub, outmsg)
        spin_some()
        sleep(0.05)
        t += 0.05
    end
end

main()

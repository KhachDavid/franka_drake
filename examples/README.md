### Examples

- franka_fci_sim_embed_example: Minimal embedding of the FCI sim server in a custom app.
- franka_fci_sim_embed_example_no_gripper: Minimal embedding of the FCI sim server in a custom app without gripper.

Build:
- cmake .. && make -j

Run:
- ./build/bin/franka-fci-sim-embed-example
- ./build/bin/franka-fci-sim-embed-example-no-gripper
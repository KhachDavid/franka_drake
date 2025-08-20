### Examples

- franka_fci_sim_embed_example: Minimal embedding of the FCI sim server in a custom app.
- pick_and_place_drake_controller: Pick-and-place demo using manipulation helpers.

Build:
- cmake .. && make -j

Run:
- ./build/bin/franka-fci-sim-embed-example
- ./build/bin/pick_and_place_drake_controller
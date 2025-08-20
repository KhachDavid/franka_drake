### Examples

- franka_fci_sim_embed_example: Minimal embedding of the FCI sim server in a custom app.
- warehouse_drake_controller: Pick-and-place demo using manipulation helpers.

Build:
- cmake .. && make -j

Run:
- ./build/bin/franka-fci-sim-server …
- ./build/bin/warehouse_drake_controller …
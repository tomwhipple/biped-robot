"""The whole power board in ngspice: the schematic's netlist (exported by
kicad-cli), every part from its value and MPN, U1 and U2 as pin models whose
state machines run in Python at each time step, and the world at the
connectors. ``make pbfull`` runs the campaign; the record is
``docs/design-v6/2026-10-24-power-board-full-simulation.md``.
"""

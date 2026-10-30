# BlueBoat Core

The common base for autonomy on BlueBoat vehicles: a ROS 2 Jazzy stack in Docker on a backseat computer, connected to ArduRover through one gateway.

The core takes commands through one path, runs missions as behaviour trees and records what happened. Everything specific to one use lives in modules and applications, in their own repositories.

## Layout

```
src/core/core_gateway     MAVLink bridge
src/core/core_helm        command path, collision filter, fleet link
src/core/core_executive   behaviour-tree runtime, default survey application
src/core/core_ops         event log, telemetry, host monitoring
src/core/core_common      helpers shared by the above
src/interfaces            messages, services, actions
src/modules               modules and applications, cloned here, not tracked
config/                   vehicle, mission, helm and module settings
scripts/                  setup, data handling, module management
```

## Run it in simulation

```bash
export ROS_DOMAIN_ID=42
./start-mission.sh --sim
python3 scripts/dev/sim-arm.py
python3 scripts/dev/sim-set-mode.py GUIDED
./run-plan.sh experiments/survey_example.json
./stop-mission.sh
```

`sim-arm.py` and `sim-set-mode.py` stand in for the operator: the backseat never arms the vehicle or sets its mode. GUIDED can take a minute after start, while the simulated EKF converges.

## Run it on a vehicle

Set `VEHICLE_NAME` and `ROS_DOMAIN_ID` in `config/vehicle.local.env` (template: `config/vehicle.local.env.example`). Choose the application and modules in `config/mission-config.yaml`, fetch them with `scripts/modules/sync-modules.sh`, then:

```bash
./start-mission.sh
./stop-mission.sh
```

## Documentation

The `docs/` folder: [architecture](docs/architecture/overview.mdx), [helm](docs/architecture/helm.mdx), [adding a module](docs/guides/adding-a-module.mdx), [mission data](docs/guides/data-logging.mdx).

## Tests

```bash
pip install pytest pyyaml py_trees==2.6.0
pytest tests -q
```

## Citation

```bibtex
@inproceedings{masmitja2026blueboat,
  title     = {An adaptive open-source architecture for enhanced autonomy
               in {BlueBoat} {ASVs} based on {ROS2}},
  author    = {Masmitja, Ivan and Somers, Korneel and Agundez, Pedro and
               Pradell, Roc and Bardaji, Raul and Carandell, Matias and
               Aguzzi, Jacopo and Gomariz, Spartacus},
  booktitle = {OCEANS 2026 Monterey},
  year      = {2026}
}
```

## Acknowledgements

This work acknowledges the Spanish Ministerio de Ciencia, Innovación y Universidades (TECTUGA: PID2024-161772OA-I00), and is part of DIGI4ECO (European Union's Horizon Europe programme, No 101112883), SUN-BIO (GAP-101157493) and MERLIN (GAP-01189796). It acknowledges the 'Severo Ochoa Centre of Excellence' accreditation (CEX2024-001494-S) from AEI 10.13039/501100011033, and the Research Unit Tecnoterra (ICM-CSIC/UPC).

The authors thank the Facultat de Nàutica de Barcelona (UPC) for access to the Espai Vela facilities, and the Oceanographic Engineering Service and the BLUE Lab of the Institut de Ciències del Mar (ICM-CSIC) for their continued technical support.

## License

Released under the [MIT License](LICENSE).

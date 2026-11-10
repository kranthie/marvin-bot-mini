# Contributing

Issues and pull requests are welcome.

## Ground rules

- `description/robot.yaml` is the single source for joint names, ranges, sizes, servo assignment, buses and power. `description/actuators.yaml` holds servo specifications and their simulation model, and `description/mass_budget.yaml` the non-servo mass estimates. CAD, URDF, simulation and onboard configs must be generated from or checked against them.
- Record design decisions in `docs/decisions.md`, newest first, with the reason and the alternatives considered.
- Credit any project, paper, post or product you borrow from in `ACKNOWLEDGMENTS.md`, and respect its licence.
- Use SI units (m, kg, N·m, rad) in code and configs. Joint ranges in `robot.yaml` are in degrees for readability; tools convert them.
- Mark code that has not run on hardware or in the simulator as untested in its docstring, and keep it marked until it has.
- No personal data, credentials, machine-specific paths or private links in the repo.

## Third-party CAD and data

- Do not commit vendor CAD models (servos, the computer, cameras, connectors) unless their licence explicitly allows redistribution. Link to the official download instead, and use simplified envelope models made in this repo.
- Specifications quoted in this repo cite their source. Mark figures that come from resellers rather than the manufacturer as unverified.
- Motion data is not committed; the motion pipeline uses data the user obtains under its own licence.
- Do not include images, logos or trademarks of other companies.

## Licensing of contributions

By contributing, you agree that your contribution is licensed under the licence of the folder it
goes into (see "Licences" in the README): CERN-OHL-S-2.0 for hardware design sources, Apache-2.0
for everything else. Sign off your commits (`git commit -s`) to certify the
[Developer Certificate of Origin](https://developercertificate.org/).

## Development

```bash
python3.12 -m venv .venv && . .venv/bin/activate
pip install -e ".[dev]"
tools/check_all.sh                                 # every CI check, in CI's order
```

Code style: ruff (line length 100, `ruff format`); every script starts with a docstring that says
what it does and how to run it.

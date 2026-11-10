# Reporting safety and security issues

Marvin Bot Mini is a small robot on a lithium battery. A design or software fault can still pinch,
make it fall onto something, or damage its battery, so safety issues are treated like security
vulnerabilities.

Please report privately, not in a public issue:

- anything that could make the robot move unexpectedly, fail to stop on the e-stop, overheat, or
  damage its battery or wiring;
- unsafe values in the description, wiring, bench scripts or policies (torque, speed, voltage or
  temperature limits);
- security issues in the onboard software or its network interfaces.

Use GitHub's private reporting form for this repository (the **Security** tab, then **Report a
vulnerability**). Describe what you found, where, and how it could cause harm.

This is a hobby project at the design stage, provided as is and without warranty; reports are
handled on a best-effort basis. The safety rules are in [docs/safety.md](docs/safety.md).

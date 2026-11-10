"""Make the site's link-preview card (public/og-image.png, 1200 x 630): the title and the robot's
numbers beside a MuJoCo render of the stance.

    MUJOCO_GL=egl python site/scripts/social_card.py

Needs the repository's environment with the sim extra.
"""

import sys
from pathlib import Path

import mujoco
from PIL import Image, ImageDraw, ImageFont

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from sim.mujoco.model import ROBOT, Robot  # noqa: E402

OUT = ROOT / "site" / "public"


def main() -> None:
    r = Robot(visual=True)
    r.set_pose(ROBOT["stance"])
    r.place_on_floor()
    with mujoco.Renderer(r.model, 630, 560) as rd:
        cam = mujoco.MjvCamera()
        cam.azimuth, cam.elevation, cam.distance = 150, -10, 0.95
        cam.lookat[:] = [0, 0, 0.3]
        rd.update_scene(r.data, cam)
        img = Image.fromarray(rd.render())
    card = Image.new("RGB", (1200, 630), (7, 9, 12))
    card.paste(img, (640, 0))
    d = ImageDraw.Draw(card)
    font = ImageFont.truetype
    big, mid, sm = (
        font("DejaVuSans-Bold.ttf", 64),
        font("DejaVuSans.ttf", 28),
        font("DejaVuSans.ttf", 22),
    )
    d.text((60, 90), "PHASE 1 · SIMULATION ONLY", font=sm, fill=(255, 180, 84))
    d.text((60, 140), "Marvin Bot Mini", font=big, fill=(238, 243, 248))
    d.text(
        (60, 240),
        "A small open humanoid\nyou can build.",
        font=mid,
        fill=(238, 243, 248),
        spacing=10,
    )
    lines = "\n".join(
        [
            "0.6 m · 2.9 kg · 23 Feetech servos",
            "printed frame · Jetson Orin Nano Super",
            "≈ $1.8k in parts",
        ]
    )
    d.text((60, 360), lines, font=sm, fill=(147, 161, 176), spacing=10)
    d.text((60, 540), "kranthie.github.io/marvin-bot-mini", font=sm, fill=(255, 180, 84))
    for name in ("og-image.png", "social-card.png"):
        card.save(OUT / name)
    print(f"wrote {OUT / 'og-image.png'}")


if __name__ == "__main__":
    main()

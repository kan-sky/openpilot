#!/usr/bin/env python3
"""Generate only C4's Korean atlas, preserving existing bundled font assets."""
from pathlib import Path
from tempfile import TemporaryDirectory

from openpilot.selfdrive.assets.fonts import process


def generate(source: Path, output: Path) -> None:
  base, international = process._char_sets()
  stem = "Pretendard-KR-SemiBold"
  old_dir, old_fonts = process.FONT_DIR, process.KR_FONTS
  try:
    process.FONT_DIR = output
    process.KR_FONTS = old_fonts | {stem}
    # Same TTF, different output name and 48px raster size. Never duplicate or
    # overwrite the source/bundled Latin atlas; this short-lived build owns RAM.
    with TemporaryDirectory(prefix="c4-font-") as directory:
      alias = Path(directory) / f"{stem}.ttf"
      alias.symlink_to(source.resolve())
      process._process_font(alias, tuple(sorted(set(base) | set(international))))
  finally:
    process.FONT_DIR, process.KR_FONTS = old_dir, old_fonts


if __name__ == "__main__":
  font_dir = Path(__file__).resolve().parent
  generate(font_dir / "Pretendard-SemiBold.ttf", font_dir)

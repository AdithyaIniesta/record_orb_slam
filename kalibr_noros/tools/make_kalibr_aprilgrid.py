#!/usr/bin/env python3
"""
make_kalibr_aprilgrid.py - a printable AprilGrid that Kalibr's own detector decodes (vector PDF at exact size + target yaml).

    python3 make_kalibr_aprilgrid.py --cols 6 --rows 4 --tag-mm 50 --spacing 0.3 --page A3 --out aprilgrid_kalibr_6x4_50mm_A3

Why not the other repository's PDF (cam-imu-calibration-/targets)? Verified against Kalibr's detector (see README):
  * Kalibr uses the AprilTag 2 tag36h11 code table; the other repo draws the AprilTag 3 table -> Kalibr decodes nothing.
  * Kalibr's AprilGrid tags have a 2-cell black border (blackTagBorder = 2): the black square is 10 cells, the 6x6 data in the middle.
  * Kalibr's board frame has y UP: tag 0 is at the BOTTOM-LEFT, ids run left->right, then upward. x right, y up, z toward you.
  * tagSize is the full black square (10 cells); tagSpacing is the gap as a fraction of tagSize.
A small black square sits in every inner gap intersection (a saddle point, which refines to sub-pixel like a checkerboard corner).

Print at 100 % / "actual size" (no fit to page), matte paper, on a rigid flat board. MEASURE one tag's black square with
calipers and put that value (metres) into tagSize of the yaml.
"""
import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from kalibr_tag36h11_codes import CODES

PT = 72.0 / 25.4
PAGES = {"A3": (420.0, 297.0), "A4": (297.0, 210.0)}      # landscape, mm
BORDER = 2                                                   # black cells around the 6x6 data (Kalibr blackTagBorder)


def board_rects(cols, rows, tag_mm, spacing, squares=True):
    """Black rectangles of the board as (x, y, w, h) in mm; x right, y UP, origin = bottom-left corner of tag 0."""
    s = float(tag_mm)
    g = s * spacing
    n = 6 + 2 * BORDER
    c = s / n
    if cols * rows > len(CODES):
        raise ValueError("not enough tag codes")
    out = []
    if squares:
        for i in range(1, cols):
            for j in range(1, rows):
                out.append((i * (s + g) - g, j * (s + g) - g, g, g))
    for tid in range(cols * rows):
        x0, y0 = (tid % cols) * (s + g), (tid // cols) * (s + g)       # tag 0 bottom-left, ids left->right, then up
        cells = [[0] * n for _ in range(n)]                            # 0 = black (border ring included)
        for i in range(36):
            if (CODES[tid] >> (35 - i)) & 1:                            # bit 1 = white; MSB first, raster order, row 0 on top
                cells[BORDER + i // 6][BORDER + i % 6] = 1
        for r in range(n):
            for q in range(n):
                if cells[r][q] == 0:
                    out.append((x0 + q * c, y0 + (n - 1 - r) * c, c, c))
    return out


def board_size(cols, rows, tag_mm, spacing):
    s = float(tag_mm)
    return cols * s + (cols - 1) * s * spacing, rows * s + (rows - 1) * s * spacing


def merge_runs(rects):
    """Merge horizontally adjacent equal-height cells into single rectangles (keeps the PDF small and gap-free)."""
    rows = {}
    for x, y, w, h in rects:
        rows.setdefault((round(y, 6), round(h, 6)), []).append((x, w))
    out = []
    for (y, h), runs in rows.items():
        runs.sort()
        cx, cw = runs[0]
        for x, w in runs[1:]:
            if abs(cx + cw - x) < 1e-6:
                cw += w
            else:
                out.append((cx, y, cw, h))
                cx, cw = x, w
        out.append((cx, y, cw, h))
    # second pass: stack vertically adjacent rectangles with identical x extent into taller ones
    cols = {}
    for x, y, w, h in out:
        cols.setdefault((round(x, 6), round(w, 6)), []).append((y, h))
    merged = []
    for (x, w), runs in cols.items():
        runs.sort()
        cy, ch = runs[0]
        for y, h in runs[1:]:
            if abs(cy + ch - y) < 1e-6:
                ch += h
            else:
                merged.append((x, cy, w, ch))
                cy, ch = y, h
        merged.append((x, cy, w, ch))
    return merged


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--cols", type=int, default=6)
    ap.add_argument("--rows", type=int, default=4)
    ap.add_argument("--tag-mm", type=float, default=50.0, help="edge of the full black square of one tag")
    ap.add_argument("--spacing", type=float, default=0.3, help="gap between tags as a fraction of the tag")
    ap.add_argument("--page", choices=sorted(PAGES), default="A3")
    ap.add_argument("--orientation", choices=["portrait", "landscape"], default="portrait",
                    help="page orientation. portrait = the sheet as normally fed to an A3/A4 printer; the board is rotated 90 degrees "
                         "on it (its long edge along the long edge of the sheet), so the page is not scaled or re-rotated when printing")
    ap.add_argument("--out", required=True, help="output base name (no extension)")
    a = ap.parse_args()

    bw, bh = board_size(a.cols, a.rows, a.tag_mm, a.spacing)
    pw, ph = PAGES[a.page]
    ox, oy = (pw - bw) / 2.0, (ph - bh) / 2.0 + 4.0                # centred; a little room for the label strip at the bottom
    if ox < 8 or oy < 14:
        sys.exit("board %.0f x %.0f mm does not fit on %s with printable margins" % (bw, bh, a.page))
    rects = merge_runs(board_rects(a.cols, a.rows, a.tag_mm, a.spacing))
    # ONE path for everything black, filled once: viewers anti-alias every separate fill on its own, which leaves hairline
    # seams / notches where small rectangles touch. Edges are computed from rounded corner coordinates so touching rectangles
    # share exactly the same numbers.
    portrait = a.orientation == "portrait"

    def place(x, y, w, h):
        """Rectangle on the landscape layout (mm, origin bottom-left) -> on the actual page. Portrait = the layout turned
        90 degrees counter-clockwise, so the sheet is used the way it comes out of the printer."""
        return (ph - (y + h), x, h, w) if portrait else (x, y, w, h)

    def rect_op(x, y, w, h):
        x, y, w, h = place(x, y, w, h)
        x1, x2 = round(x * PT, 4), round((x + w) * PT, 4)
        y1, y2 = round(y * PT, 4), round((y + h) * PT, 4)
        return "%.4f %.4f %.4f %.4f re" % (x1, y1, x2 - x1, y2 - y1)

    ops = ["0 g"]
    ops += [rect_op(ox + x, oy + y, w, h) for x, y, w, h in rects]
    ops.append("f")
    label = ("Kalibr AprilGrid tag36h11 (AprilTag 2 codes), %dx%d, tag %g mm = full black square, spacing %g, ids 0-%d, id 0 bottom-left. "
             "Bar = 100 mm: print at 100%% and MEASURE a tag." % (a.cols, a.rows, a.tag_mm, a.spacing, a.cols * a.rows - 1))
    ops += [rect_op(ox, 7.0, 100, 0.8), rect_op(ox, 5.5, 0.5, 3.8), rect_op(ox + 99.5, 5.5, 0.5, 3.8)]
    ops.append("f")
    tx, ty = ox + 106, 6.2                                          # label start on the landscape layout, mm
    if portrait:
        ops.append("BT /F1 7 Tf 0 1 -1 0 %.3f %.3f Tm (%s) Tj ET" % ((ph - ty) * PT, tx * PT, label))   # text turned with the page
    else:
        ops.append("BT /F1 7 Tf %.3f %.3f Td (%s) Tj ET" % (tx * PT, ty * PT, label))
    content = "\n".join(ops).encode("ascii")
    page_w, page_h = (ph, pw) if portrait else (pw, ph)             # MediaBox in mm
    objs = [b"<< /Type /Catalog /Pages 2 0 R >>", b"<< /Type /Pages /Kids [3 0 R] /Count 1 >>",
            ("<< /Type /Page /Parent 2 0 R /MediaBox [0 0 %.3f %.3f] /Contents 4 0 R /Resources << /Font << /F1 5 0 R >> >> >>"
             % (page_w * PT, page_h * PT)).encode(),
            b"<< /Length " + str(len(content)).encode() + b" >>\nstream\n" + content + b"\nendstream",
            b"<< /Type /Font /Subtype /Type1 /BaseFont /Helvetica >>"]
    out, offs = b"%PDF-1.4\n", []
    for n, body in enumerate(objs, 1):
        offs.append(len(out))
        out += ("%d 0 obj\n" % n).encode() + body + b"\nendobj\n"
    xref = len(out)
    out += ("xref\n0 %d\n0000000000 65535 f \n" % (len(objs) + 1)).encode()
    out += b"".join(("%010d 00000 n \n" % o).encode() for o in offs)
    out += ("trailer\n<< /Size %d /Root 1 0 R >>\nstartxref\n%d\n%%%%EOF\n" % (len(objs) + 1, xref)).encode()
    open(a.out + ".pdf", "wb").write(out)
    open(a.out + ".yaml", "w").write(
        "# %s\n# tagSize = the FULL black square of one tag, in metres: MEASURE it on the print and replace the value below.\n"
        "target_type: 'aprilgrid'\ntagCols: %d\ntagRows: %d\ntagSize: %.4f\ntagSpacing: %g\n" % (label, a.cols, a.rows, a.tag_mm / 1000.0, a.spacing))
    print("%s.pdf: %s %s (%.0f x %.0f mm), board %.0f x %.0f mm, %d rectangles | %s.yaml" % (
        a.out, a.page, a.orientation, page_w, page_h, bw, bh, len(rects), a.out))


if __name__ == "__main__":
    main()

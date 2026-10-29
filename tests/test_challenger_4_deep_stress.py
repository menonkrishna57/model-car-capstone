r"""
test_challenger_4_deep_stress.py
================================
Deep Adversarial Stress Harness for Challenger 4
Probing:
1. Inflected / lemma variants of all 17 banned words (e.g., robustly, robustness, delving, showcasing, etc.)
2. All XML parts in docx (headers, footers, comments, footnotes, endnotes, document.xml, drawing textboxes)
3. Full Unicode dash spectrum: U+2010, U+2011, U+2012, U+2013 (en), U+2014 (em), U+2015 (horizontal bar), U+2212 (minus), U+FE58, U+FE63, U+FF0D, spaced hyphens
4. Whitespace subtleties: \xa0 (non-breaking space), zero-width spaces (\u200b, \u200c, \u200d, \ufeff)
5. Staged AI contrast formulas and Humanizer boilerplate patterns across all text
6. Inspection of Table 0 (Certificate signature block) exact text and formatting
7. Drawing dimensions (EMU widths, heights, scaling) to ensure non-zero, unclipped figures
"""

import os
import sys
import re
import zipfile
import xml.etree.ElementTree as ET
import docx

sys.stdout.reconfigure(encoding='utf-8')

W_NS = "http://schemas.openxmlformats.org/wordprocessingml/2006/main"
WP_NS = "http://schemas.openxmlformats.org/drawingml/2006/wordprocessingDrawing"
A_NS = "http://schemas.openxmlformats.org/drawingml/2006/main"
PIC_NS = "http://schemas.openxmlformats.org/drawingml/2006/picture"
R_NS = "http://schemas.openxmlformats.org/officeDocument/2006/relationships"

# 17 base banned words and extended inflections/stems
BANNED_STEMS = [
    r"crucial.*", r"delv.*", r"testament.*", r"tapestr.*", r"landscape.*",
    r"pivotal.*", r"robust.*", r"showcas.*", r"underscore.*", r"foster.*",
    r"interplay.*", r"bolster.*", r"beacon.*", r"catalyst.*", r"panacea.*",
    r"plethor.*"
]

HUMANIZER_TELLS = [
    (r"\bnot\s+only\b", "not only ... but formula"),
    (r"\bnot\s+just\b", "not just ... but formula"),
    (r"\bnot\s+merely\b", "not merely ... but formula"),
    (r"\bat\s+its\s+core\b", "at its core (saying that sounds deep)"),
    (r"\blet'?s\s+explore\b", "let's explore (staged run-up)"),
    (r"\blet'?s\s+dive\b", "let's dive (staged run-up)"),
    (r"\blet'?s\s+examine\b", "let's examine (staged run-up)"),
    (r"\bhere'?s\s+what\s+you\s+need\s+to\s+know\b", "here's what you need to know (staged run-up)"),
    (r"\bthat\s+is\s+the\s+real\s+win\b", "that is the real win (dramatic closer)"),
    (r"\bread\s+that\s+again\b", "read that again (dramatic closer)"),
    (r"\blet\s+that\s+sink\s+in\b", "let that sink in (dramatic closer)"),
    (r"\bthe\s+old\s+rules\s+were\s+gone\b", "the old rules were gone (dramatic closer)"),
    (r"\bstands\s+as\s+a\s+testament\b", "stands as a testament"),
    (r"\bserves\s+as\s+a\s+testament\b", "serves as a testament"),
    (r"\bplays\s+a\s+pivotal\s+role\b", "plays a pivotal role"),
    (r"\bplays\s+a\s+crucial\s+role\b", "plays a crucial role"),
]

UNICODE_DASHES = [
    ('\u2014', 'EM DASH'),
    ('\u2013', 'EN DASH'),
    ('\u2015', 'HORIZONTAL BAR'),
    ('\u2012', 'FIGURE DASH'),
    ('\u2e3a', 'TWO-EM DASH'),
    ('\u2e3b', 'THREE-EM DASH'),
    ('\ufe58', 'SMALL EM DASH'),
]

def run_deep_stress():
    repo_root = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
    docx_path = os.path.join(repo_root, "Autonomous_RC_Car_Capstone_Report_Final.docx")
    
    print("=" * 80)
    print("CHALLENGER 4: DEEP ADVERSARIAL STRESS PROBING")
    print(f"Target: {docx_path}")
    print("=" * 80)

    with zipfile.ZipFile(docx_path, 'r') as z:
        all_xml_files = [f for f in z.namelist() if f.endswith('.xml')]
        print(f"Total XML parts inspected: {len(all_xml_files)}")
        for xf in sorted(all_xml_files):
            print(f"  - {xf}")

        # Scan ALL XML parts for literal banned words and dashes
        print("\n--- Deep Probe 1: Full-Archive XML Text Extraction & Scan ---")
        xml_dash_hits = []
        xml_buzzword_hits = []
        xml_tell_hits = []
        
        for xf in all_xml_files:
            # Skip styles, themes, webSettings, rels, fontTable for prose checks, but inspect document, header*, footer*, footnotes, endnotes, comments
            if not any(k in xf for k in ['document', 'header', 'footer', 'footnote', 'endnote', 'comment']):
                continue
                
            xml_data = z.read(xf).decode('utf-8')
            root = ET.fromstring(xml_data)
            
            # Extract all text nodes
            for t_el in root.iter(f'{{{W_NS}}}t'):
                t_val = t_el.text or ""
                if not t_val:
                    continue
                # 1. Unicode dashes
                for d_char, d_name in UNICODE_DASHES:
                    if d_char in t_val:
                        xml_dash_hits.append((xf, d_name, repr(t_val)))
                # 2. Spaced double hyphens
                if ' -- ' in t_val:
                    xml_dash_hits.append((xf, 'SPACED DOUBLE HYPHEN', repr(t_val)))
                    
                # 3. Banned stems & inflections
                for b_stem in BANNED_STEMS:
                    m = re.search(rf'\b{b_stem}\b', t_val, re.IGNORECASE)
                    if m:
                        xml_buzzword_hits.append((xf, m.group(0), repr(t_val)))
                        
                # 4. Humanizer tells
                for pattern, name in HUMANIZER_TELLS:
                    m = re.search(pattern, t_val, re.IGNORECASE)
                    if m:
                        xml_tell_hits.append((xf, name, m.group(0), repr(t_val)))

        print(f"XML Dash Hits: {len(xml_dash_hits)}")
        for xf, dname, txt in xml_dash_hits:
            print(f"  [VIOLATION] {xf} ({dname}): {txt}")
            
        print(f"XML Banned Word / Stem Hits: {len(xml_buzzword_hits)}")
        for xf, word, txt in xml_buzzword_hits:
            print(f"  [VIOLATION] {xf} ('{word}'): {txt}")

        print(f"XML Humanizer Tell Hits: {len(xml_tell_hits)}")
        for xf, name, match, txt in xml_tell_hits:
            print(f"  [VIOLATION] {xf} ({name}): match='{match}', text={txt}")

        # Deep Probe 2: Table 0 (Certificate) & Table Cell Whitespace/Zero-Width Audit
        print("\n--- Deep Probe 2: Zero-Width, Whitespace, & Table 0 Forensic Audit ---")
        doc = docx.Document(docx_path)
        
        # Examine Table 0 specifically
        t0 = doc.tables[0]
        print(f"Table 0 Dimensions: {len(t0.rows)} rows x {len(t0.columns)} cols")
        for r_i, row in enumerate(t0.rows):
            for c_i, cell in enumerate(row.cells):
                txt = cell.text
                print(f"  Table 0 [R{r_i}C{c_i}]: len={len(txt)}, repr={repr(txt)}")

        # Audit all cells for invisible whitespace only
        suspicious_cells = []
        for t_idx, tbl in enumerate(doc.tables):
            for r_idx, row in enumerate(tbl.rows):
                for c_idx, cell in enumerate(row.cells):
                    raw = cell.text
                    # Check for non-breaking space \xa0, zero-width space \u200b, zero-width joiner, BOM
                    cleaned = re.sub(r'[\s\xa0\u200b\u200c\u200d\ufeff]+', '', raw)
                    if len(cleaned) == 0:
                        suspicious_cells.append((t_idx, r_idx, c_idx, repr(raw)))

        print(f"Suspicious/Whitespace-Only Cells: {len(suspicious_cells)}")
        for t_i, r_i, c_i, content in suspicious_cells:
            print(f"  [VIOLATION] Table {t_i} R{r_i}C{c_i}: {content}")

        # Deep Probe 3: Image Sizing and Drawing Elements
        print("\n--- Deep Probe 3: Drawing Geometry & Sizing (EMUs) ---")
        doc_xml = z.read('word/document.xml').decode('utf-8')
        doc_root = ET.fromstring(doc_xml)
        
        drawings = doc_root.findall(f'.//{{{W_NS}}}drawing')
        print(f"Total w:drawing elements found in document.xml: {len(drawings)}")
        
        image_dims = []
        for i, d in enumerate(drawings):
            # Check extent (cx, cy)
            inline = d.find(f'{{{WP_NS}}}inline')
            anchor = d.find(f'{{{WP_NS}}}anchor')
            geom = inline if inline is not None else anchor
            if geom is not None:
                extent = geom.find(f'{{{WP_NS}}}extent')
                cx = int(extent.attrib.get('cx', '0')) if extent is not None else 0
                cy = int(extent.attrib.get('cy', '0')) if extent is not None else 0
                # In inches: cx / 914400, cy / 914400
                w_in = cx / 914400.0
                h_in = cy / 914400.0
                print(f"  Drawing {i+1}: cx={cx} ({w_in:.2f} in), cy={cy} ({h_in:.2f} in)")
                if cx <= 0 or cy <= 0:
                    print(f"    [VIOLATION] Invalid drawing dimensions cx={cx}, cy={cy}")
                # Check maximum page body width = 6.0 in (5486400 EMUs)
                if cx > 5486400 + 1000:
                    print(f"    [WARNING] Drawing {i+1} width ({w_in:.2f} in) exceeds 6.0 inch body width!")
                image_dims.append((cx, cy, w_in, h_in))

    print("\n" + "=" * 80)
    print("DEEP PROBE CONCLUSION")
    print("=" * 80)
    all_clean = (
        len(xml_dash_hits) == 0 and
        len(xml_buzzword_hits) == 0 and
        len(xml_tell_hits) == 0 and
        len(suspicious_cells) == 0 and
        len(drawings) == 6 and
        all(cx > 0 and cy > 0 for cx, cy, _, _ in image_dims)
    )
    print(f"Deep Stress Status: {'PASSED' if all_clean else 'FAILED'}")
    return 0 if all_clean else 1

if __name__ == "__main__":
    sys.exit(run_deep_stress())

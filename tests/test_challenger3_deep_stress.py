"""
test_challenger3_deep_stress.py
================================
Independent Empirical Adversarial Stress Test Suite
Challenger 3: Geometry, Typography & XML Stress Challenger (Gate 2)

Target Document: Autonomous_RC_Car_Capstone_Report_Final.docx
Standard: MPSTME SVKM's NMIMS Capstone Report Guidelines 2026-27 (pages 24-41)
"""

import sys
import os
import re
import zipfile
import xml.etree.ElementTree as ET
import docx

W_NS = "http://schemas.openxmlformats.org/wordprocessingml/2006/main"
WP_NS = "http://schemas.openxmlformats.org/drawingml/2006/wordprocessingDrawing"
R_NS = "http://schemas.openxmlformats.org/officeDocument/2006/relationships"

def qn(tag):
    return f"{{{W_NS}}}{tag}"

def run_independent_stress_suite(docx_path):
    print("=" * 80)
    print("INDEPENDENT EMPIRICAL ADVERSARIAL STRESS SUITE (CHALLENGER 3)")
    print(f"Target Document: {docx_path}")
    print("=" * 80)

    if not os.path.exists(docx_path):
        print(f"CRITICAL ERROR: File not found: {docx_path}")
        return False, {}

    with zipfile.ZipFile(docx_path, 'r') as z:
        zip_files = z.namelist()
        doc_xml = z.read('word/document.xml').decode('utf-8')
        rels_xml = z.read('word/_rels/document.xml.rels').decode('utf-8')
        styles_xml = z.read('word/styles.xml').decode('utf-8')
        
        # Read all XML files in package
        xml_parts = {f: z.read(f).decode('utf-8') for f in zip_files if f.endswith('.xml')}
        header_xmls = {f: xml_parts[f] for f in xml_parts if f.startswith('word/header')}
        footer_xmls = {f: xml_parts[f] for f in xml_parts if f.startswith('word/footer')}

    doc_root = ET.fromstring(doc_xml)
    rels_root = ET.fromstring(rels_xml)
    styles_root = ET.fromstring(styles_xml)
    doc = docx.Document(docx_path)

    all_passed = True
    results = {}

    # -------------------------------------------------------------------------
    # TEST 1: SECTION GEOMETRY & MARGINS
    # -------------------------------------------------------------------------
    print("\n[TEST 1] Section Margins & Page Geometry")
    sect_prs = doc_root.findall(f".//{qn('sectPr')}")
    print(f"Found {len(sect_prs)} w:sectPr elements.")
    t1_pass = True
    if len(sect_prs) != 2:
        print(f"  FAIL: Expected exactly 2 sections, found {len(sect_prs)}")
        t1_pass = False

    for idx, s in enumerate(sect_prs):
        pgMar = s.find(qn('pgMar'))
        pgSz = s.find(qn('pgSz'))
        if pgMar is None:
            print(f"  FAIL: Section {idx+1} missing w:pgMar")
            t1_pass = False
            continue

        left = pgMar.attrib.get(qn('left'))
        right = pgMar.attrib.get(qn('right'))
        top = pgMar.attrib.get(qn('top'))
        bottom = pgMar.attrib.get(qn('bottom'))
        header_m = pgMar.attrib.get(qn('header'))
        footer_m = pgMar.attrib.get(qn('footer'))
        gutter = pgMar.attrib.get(qn('gutter'), '0')

        w_width = pgSz.attrib.get(qn('w')) if pgSz is not None else None
        w_height = pgSz.attrib.get(qn('h')) if pgSz is not None else None

        # Check margin values: left=2160 (1.5"), right=1440, top=1440, bottom=1440
        m_pass = (left == "2160" and right == "1440" and top == "1440" and bottom == "1440")
        print(f"  Section {idx+1}:")
        print(f"    w:left   = '{left}'   (expected '2160' / 1.5 in) -> {'OK' if left == '2160' else 'FAIL'}")
        print(f"    w:right  = '{right}'   (expected '1440' / 1.0 in) -> {'OK' if right == '1440' else 'FAIL'}")
        print(f"    w:top    = '{top}'   (expected '1440' / 1.0 in) -> {'OK' if top == '1440' else 'FAIL'}")
        print(f"    w:bottom = '{bottom}'   (expected '1440' / 1.0 in) -> {'OK' if bottom == '1440' else 'FAIL'}")
        print(f"    w:header = '{header_m}', w:footer = '{footer_m}', w:gutter = '{gutter}'")
        print(f"    Page Size: w={w_width}, h={w_height}")
        
        # Check usable text width: w_width - left - right
        if w_width:
            usable_w = int(w_width) - int(left) - int(right)
            print(f"    Usable text width: {usable_w} dxa ({usable_w/1440.0} in) [Expected 8640 dxa / 6.0 in]")
            if usable_w != 8640:
                print(f"    FAIL: Usable text width is {usable_w}, expected 8640 dxa")
                m_pass = False

        if not m_pass:
            t1_pass = False

    results['margins'] = t1_pass
    all_passed = all_passed and t1_pass

    # -------------------------------------------------------------------------
    # TEST 2: DUAL PAGINATION & HEADER/FOOTER DECOUPLING
    # -------------------------------------------------------------------------
    print("\n[TEST 2] Dual Pagination & Decoupling")
    t2_pass = True
    if len(sect_prs) >= 2:
        s1 = sect_prs[0]
        s2 = sect_prs[1]

        # S1: w:titlePg and w:pgNumType w:fmt="lowerRoman"
        titlePg1 = s1.find(qn('titlePg'))
        pgNum1 = s1.find(qn('pgNumType'))
        
        has_titlePg = (titlePg1 is not None)
        fmt1 = pgNum1.attrib.get(qn('fmt')) if pgNum1 is not None else None
        
        print(f"  Section 1 w:titlePg present: {has_titlePg} -> {'OK' if has_titlePg else 'FAIL'}")
        print(f"  Section 1 w:pgNumType fmt: '{fmt1}' -> {'OK' if fmt1 == 'lowerRoman' else 'FAIL'}")
        if not (has_titlePg and fmt1 == 'lowerRoman'):
            t2_pass = False

        # S2: w:pgNumType w:start="1" w:fmt="decimal"
        pgNum2 = s2.find(qn('pgNumType'))
        start2 = pgNum2.attrib.get(qn('start')) if pgNum2 is not None else None
        fmt2 = pgNum2.attrib.get(qn('fmt'), 'decimal') if pgNum2 is not None else None

        print(f"  Section 2 w:pgNumType start: '{start2}' -> {'OK' if start2 == '1' else 'FAIL'}")
        print(f"  Section 2 w:pgNumType fmt: '{fmt2}' -> {'OK' if fmt2 in ('decimal', None, '') else 'FAIL'}")
        if start2 != "1" or fmt2 not in ('decimal', None, ''):
            t2_pass = False

        # S2 Header & Footer decoupling
        sec2 = doc.sections[1]
        hdr_unlinked = not sec2.header.is_linked_to_previous
        ftr_unlinked = not sec2.footer.is_linked_to_previous
        print(f"  Section 2 header unlinked: {hdr_unlinked} -> {'OK' if hdr_unlinked else 'FAIL'}")
        print(f"  Section 2 footer unlinked: {ftr_unlinked} -> {'OK' if ftr_unlinked else 'FAIL'}")
        if not (hdr_unlinked and ftr_unlinked):
            t2_pass = False
    else:
        print("  FAIL: Insufficient sections to test dual pagination")
        t2_pass = False

    results['pagination'] = t2_pass
    all_passed = all_passed and t2_pass

    # -------------------------------------------------------------------------
    # TEST 3: RUNNING HEADER & RUNNING FOOTER
    # -------------------------------------------------------------------------
    print("\n[TEST 3] Running Header & Footer Specification")
    t3_pass = True

    # Section 2 header/footer references
    s2 = sect_prs[1] if len(sect_prs) >= 2 else None
    hdr_ref = s2.find(f".//{qn('headerReference')}") if s2 is not None else None
    ftr_ref = s2.find(f".//{qn('footerReference')}") if s2 is not None else None

    hdr_rId = hdr_ref.attrib.get(f"{{{R_NS}}}id") if hdr_ref is not None else None
    ftr_rId = ftr_ref.attrib.get(f"{{{R_NS}}}id") if ftr_ref is not None else None

    pkg_ns = rels_root.tag.split('}')[0] + '}'
    rel_map = {rel.attrib.get('Id'): rel.attrib.get('Target') for rel in rels_root.findall(f".//{pkg_ns}Relationship")}

    hdr_target = rel_map.get(hdr_rId)
    ftr_target = rel_map.get(ftr_rId)

    print(f"  Section 2 Header Target: word/{hdr_target} (rId: {hdr_rId})")
    print(f"  Section 2 Footer Target: word/{ftr_target} (rId: {ftr_rId})")

    # Inspect Header XML
    hdr_path = f"word/{hdr_target}"
    if hdr_path in header_xmls:
        h_tree = ET.fromstring(header_xmls[hdr_path])
        h_text = "".join(h_tree.itertext()).strip()
        print(f"  Header Raw Text: '{h_text}'")

        has_title = "Autonomous RC Car Using Two-Layer Edge Perception and Real-Time Control" in h_text
        has_year = "2026-27" in h_text

        # Tab stop at 8640 dxa
        tab_stops = h_tree.findall(f".//{qn('tabs')}/{qn('tab')}")
        tab_8640_ok = any(t.attrib.get(qn('pos')) == "8640" for t in tab_stops)
        
        print(f"    Contains Full Project Title: {has_title} -> {'OK' if has_title else 'FAIL'}")
        print(f"    Contains Academic Year '2026-27': {has_year} -> {'OK' if has_year else 'FAIL'}")
        print(f"    Contains Tab Stop at 8640 dxa: {tab_8640_ok} -> {'OK' if tab_8640_ok else 'FAIL'}")

        # Check font in header
        hdr_runs = h_tree.findall(f".//{qn('r')}")
        hdr_non_tnr = 0
        for r in hdr_runs:
            rPr = r.find(qn('rPr'))
            if rPr is not None:
                rFonts = rPr.find(qn('rFonts'))
                if rFonts is not None:
                    ascii_f = rFonts.attrib.get(qn('ascii'), '')
                    hAnsi_f = rFonts.attrib.get(qn('hAnsi'), '')
                    if ascii_f != "Times New Roman" and hAnsi_f != "Times New Roman":
                        hdr_non_tnr += 1
        print(f"    Header Non-TNR font runs: {hdr_non_tnr} -> {'OK' if hdr_non_tnr == 0 else 'FAIL'}")

        if not (has_title and has_year and tab_8640_ok and hdr_non_tnr == 0):
            t3_pass = False
    else:
        print(f"  FAIL: Header part {hdr_path} missing")
        t3_pass = False

    # Inspect Footer XML
    ftr_path = f"word/{ftr_target}"
    if ftr_path in footer_xmls:
        f_tree = ET.fromstring(footer_xmls[ftr_path])
        f_text = "".join(f_tree.itertext()).strip()
        print(f"  Footer Raw Text: '{f_text}'")

        jc = f_tree.find(f".//{qn('jc')}")
        align_right = (jc is not None and jc.attrib.get(qn('val')) == 'right')

        fld_simple = f_tree.find(f".//{qn('fldSimple')}")
        has_page_fld = (fld_simple is not None and fld_simple.attrib.get(qn('instr')) == 'PAGE')
        
        print(f"    Footer alignment right: {align_right} -> {'OK' if align_right else 'FAIL'}")
        print(f"    Footer PAGE field present: {has_page_fld} -> {'OK' if has_page_fld else 'FAIL'}")

        ftr_runs = f_tree.findall(f".//{qn('r')}")
        ftr_non_tnr = 0
        for r in ftr_runs:
            rPr = r.find(qn('rPr'))
            if rPr is not None:
                rFonts = rPr.find(qn('rFonts'))
                if rFonts is not None:
                    ascii_f = rFonts.attrib.get(qn('ascii'), '')
                    hAnsi_f = rFonts.attrib.get(qn('hAnsi'), '')
                    if ascii_f != "Times New Roman" and hAnsi_f != "Times New Roman":
                        ftr_non_tnr += 1
        print(f"    Footer Non-TNR font runs: {ftr_non_tnr} -> {'OK' if ftr_non_tnr == 0 else 'FAIL'}")

        if not (align_right and has_page_fld and ftr_non_tnr == 0):
            t3_pass = False
    else:
        print(f"  FAIL: Footer part {ftr_path} missing")
        t3_pass = False

    results['header_footer'] = t3_pass
    all_passed = all_passed and t3_pass

    # -------------------------------------------------------------------------
    # TEST 4: UNIVERSAL TIMES NEW ROMAN & TYPOGRAPHY
    # -------------------------------------------------------------------------
    print("\n[TEST 4] Universal Times New Roman Font Scan")
    t4_pass = True

    # Scan ALL runs in document.xml
    all_runs = doc_root.findall(f".//{qn('r')}")
    non_tnr_runs = []
    tnr_explicit = 0
    inherited_count = 0

    # Also inspect default font in styles.xml
    docDefaults = styles_root.find(f".//{qn('docDefaults')}")
    rFonts_def = docDefaults.find(f".//{qn('rFonts')}") if docDefaults is not None else None
    def_ascii = rFonts_def.attrib.get(qn('ascii')) if rFonts_def is not None else None
    def_hAnsi = rFonts_def.attrib.get(qn('hAnsi')) if rFonts_def is not None else None
    print(f"  Default Style Font in styles.xml: ascii='{def_ascii}', hAnsi='{def_hAnsi}'")

    for idx, r in enumerate(all_runs):
        t_el = r.find(qn('t'))
        text = t_el.text if t_el is not None and t_el.text else ""
        if not text.strip():
            continue

        rPr = r.find(qn('rPr'))
        if rPr is not None:
            rFonts = rPr.find(qn('rFonts'))
            if rFonts is not None:
                ascii_f = rFonts.attrib.get(qn('ascii'), '')
                hAnsi_f = rFonts.attrib.get(qn('hAnsi'), '')
                cs_f = rFonts.attrib.get(qn('cs'), '')
                if ascii_f == "Times New Roman" or hAnsi_f == "Times New Roman":
                    tnr_explicit += 1
                else:
                    non_tnr_runs.append((idx, text[:40], ascii_f, hAnsi_f, cs_f))
            else:
                inherited_count += 1
        else:
            inherited_count += 1

    print(f"  Total runs scanned in document.xml: {len(all_runs)}")
    print(f"  Explicit Times New Roman runs: {tnr_explicit}")
    print(f"  Inherited runs: {inherited_count}")
    print(f"  Non-Times New Roman runs: {len(non_tnr_runs)}")

    if non_tnr_runs:
        print(f"  FAIL: Found {len(non_tnr_runs)} runs with non-TNR font!")
        for err in non_tnr_runs[:10]:
            print(f"    Run [{err[0]}] '{err[1]}': ascii='{err[2]}', hAnsi='{err[3]}', cs='{err[4]}'")
        t4_pass = False
    else:
        print("  PASS: Exactly 0 non-TNR font runs found across document.xml!")

    # Verify Appendix III specifically (code blocks)
    app3_found = False
    app3_runs_checked = 0
    app3_non_tnr = 0
    in_app3 = False
    for p in doc_root.findall(f".//{qn('p')}"):
        p_text = "".join(p.itertext()).strip()
        if "APPENDIX III:" in p_text or "APPENDIX III" in p_text:
            in_app3 = True
            app3_found = True
            continue
        if "APPENDIX IV:" in p_text or "APPENDIX IV" in p_text:
            in_app3 = False
            continue
        if in_app3:
            # Check runs in this paragraph
            for r in p.findall(qn('r')):
                t_el = r.find(qn('t'))
                text = t_el.text if t_el is not None and t_el.text else ""
                if not text.strip():
                    continue
                app3_runs_checked += 1
                rPr = r.find(qn('rPr'))
                if rPr is not None:
                    rFonts = rPr.find(qn('rFonts'))
                    if rFonts is not None:
                        ascii_f = rFonts.attrib.get(qn('ascii'), '')
                        if ascii_f != "Times New Roman":
                            app3_non_tnr += 1

    print(f"  Appendix III Code Blocks: Scanned {app3_runs_checked} runs, Non-TNR runs={app3_non_tnr}")
    if app3_non_tnr > 0:
        print(f"  FAIL: Appendix III code blocks contain {app3_non_tnr} non-TNR runs!")
        t4_pass = False
    else:
        print("  PASS: Appendix III code blocks are 100% Times New Roman!")

    # Scan ALL package XML parts for non-TNR font definitions
    pkg_non_tnr = {}
    for part_name, part_content in xml_parts.items():
        tree = ET.fromstring(part_content)
        part_runs = tree.findall(f".//{qn('r')}")
        for r in part_runs:
            rPr = r.find(qn('rPr'))
            if rPr is not None:
                rFonts = rPr.find(qn('rFonts'))
                if rFonts is not None:
                    af = rFonts.attrib.get(qn('ascii'), '')
                    hf = rFonts.attrib.get(qn('hAnsi'), '')
                    if af and af != "Times New Roman":
                        pkg_non_tnr.setdefault(part_name, set()).add(af)
                    if hf and hf != "Times New Roman":
                        pkg_non_tnr.setdefault(part_name, set()).add(hf)
    print(f"  Package-wide non-TNR fonts across all XML parts: {pkg_non_tnr}")
    if pkg_non_tnr:
        print(f"  FAIL: Found non-TNR font definitions in package: {pkg_non_tnr}")
        t4_pass = False
    else:
        print("  PASS: 0 non-TNR font definitions across all XML parts in package!")

    # 4b: Heading font sizes and bold attributes in Section 2
    body = doc_root.find(qn('body'))
    p_elements = body.findall(qn('p'))
    sec2_p_start = None
    for idx, p in enumerate(p_elements):
        if "".join(p.itertext()).strip() == "CHAPTER 1: INTRODUCTION":
            sec2_p_start = idx
            break

    h1_count = 0
    h2_count = 0
    h3_count = 0
    heading_errors = []
    for idx in range(sec2_p_start, len(p_elements)):
        p = p_elements[idx]
        p_text = "".join(p.itertext()).strip()
        if not p_text:
            continue
        if re.match(r"^(?:CHAPTER\s+\d+:|APPENDIX\s+[IVX]+:|REFERENCES)", p_text):
            h1_count += 1
            for r in p.findall(qn('r')):
                rPr = r.find(qn('rPr'))
                sz = rPr.find(qn('sz')).attrib.get(qn('val')) if (rPr is not None and rPr.find(qn('sz')) is not None) else None
                b = rPr.find(qn('b')) if rPr is not None else None
                if sz != "36" or b is None:
                    heading_errors.append((p_text[:35], "H1", sz, "36 (18pt) bold"))
            continue
        if re.match(r"^(?:\d+|[IVX]+)\.\d+\.\d+\s+[A-Za-z]", p_text):
            h3_count += 1
            for r in p.findall(qn('r')):
                rPr = r.find(qn('rPr'))
                sz = rPr.find(qn('sz')).attrib.get(qn('val')) if (rPr is not None and rPr.find(qn('sz')) is not None) else None
                b = rPr.find(qn('b')) if rPr is not None else None
                if sz != "24" or b is None:
                    heading_errors.append((p_text[:35], "H3", sz, "24 (12pt) bold"))
            continue
        if re.match(r"^(?:\d+|[IVX]+)\.\d+\s+[A-Za-z]", p_text):
            h2_count += 1
            for r in p.findall(qn('r')):
                rPr = r.find(qn('rPr'))
                sz = rPr.find(qn('sz')).attrib.get(qn('val')) if (rPr is not None and rPr.find(qn('sz')) is not None) else None
                b = rPr.find(qn('b')) if rPr is not None else None
                if sz != "28" or b is None:
                    heading_errors.append((p_text[:35], "H2", sz, "28 (14pt) bold"))
            continue

    print(f"  Headings: H1={h1_count}, H2={h2_count}, H3={h3_count}, Errors={len(heading_errors)}")
    if heading_errors:
        print(f"  FAIL: Heading hierarchy errors: {heading_errors[:5]}")
        t4_pass = False
    else:
        print("  PASS: 100% of Section 2 headings match size and bold hierarchy!")

    # 4c: Line Spacing & Justification for narrative body paragraphs
    body_paragraphs_checked = 0
    spacing_errors = []
    justification_errors = []
    for idx in range(sec2_p_start, len(p_elements)):
        p = p_elements[idx]
        p_text = "".join(p.itertext()).strip()
        if not p_text:
            continue
        # Skip headings, captions, code blocks, reference items
        if re.match(r"^(?:CHAPTER\s+\d+:|APPENDIX\s+[IVX]+:|REFERENCES)", p_text):
            continue
        if re.match(r"^(?:\d+|[IVX]+)\.\d+(?:\.\d+)?\s+", p_text):
            continue
        if re.match(r"^(?:Table|Figure)\s+(?:\d+|[IVX]+(?:\.\d+)?)", p_text):
            continue
        pPr = p.find(qn('pPr'))
        if pPr is not None and pPr.find(qn('shd')) is not None:
            continue
        if re.match(r"^\[\d+\]", p_text):
            continue

        body_paragraphs_checked += 1
        spacing = pPr.find(qn('spacing')) if pPr is not None else None
        line = spacing.attrib.get(qn('line')) if spacing is not None else None
        lineRule = spacing.attrib.get(qn('lineRule'), 'auto') if spacing is not None else None
        if line != "360" or lineRule != "auto":
            spacing_errors.append((idx, p_text[:35], line, lineRule))

        jc = pPr.find(qn('jc')) if pPr is not None else None
        jc_val = jc.attrib.get(qn('val')) if jc is not None else None
        if jc_val != "both":
            justification_errors.append((idx, p_text[:35], jc_val))

    print(f"  Narrative body paragraphs: Checked={body_paragraphs_checked}, Spacing Errors={len(spacing_errors)}, Justification Errors={len(justification_errors)}")
    if spacing_errors or justification_errors:
        if spacing_errors:
            print(f"  FAIL: Spacing errors: {spacing_errors[:5]}")
        if justification_errors:
            print(f"  FAIL: Justification errors: {justification_errors[:5]}")
        t4_pass = False
    else:
        print("  PASS: 100% of narrative body paragraphs have 1.5 line spacing (w:line='360') and full justification (w:jc='both')!")

    results['typography'] = t4_pass
    all_passed = all_passed and t4_pass

    # -------------------------------------------------------------------------
    # TEST 5: CAPTION PLACEMENT ACROSS ALL 26 TECHNICAL TABLES & 6 FIGURES
    # -------------------------------------------------------------------------
    print("\n[TEST 5] Caption Placement & Table Precedence")
    t5_pass = True

    body_children = list(body)
    total_body_elements = len(body_children)
    print(f"  Total w:body children: {total_body_elements}")
    sec2_body_start = next(i for i, el in enumerate(body_children) if "".join(el.itertext()).strip() == "CHAPTER 1: INTRODUCTION" and i > 40)
    print(f"  Section 2 starts at w:body child index: {sec2_body_start}")

    # Collect all tables
    tables = []
    for i, el in enumerate(body_children):
        if el.tag.split('}')[-1] == 'tbl':
            prev_el = body_children[i - 1] if i > 0 else None
            prev_text = "".join(prev_el.itertext()).strip() if prev_el is not None else ""
            tables.append((len(tables) + 1, i, prev_text, el))

    print(f"  Total tables found: {len(tables)}")
    
    # Analyze table captions
    tech_tables = []
    front_matter_tables = []

    for t_num, b_idx, p_txt, el in tables:
        if b_idx < sec2_body_start:
            front_matter_tables.append((t_num, b_idx, p_txt))
        else:
            tech_tables.append((t_num, b_idx, p_txt))

    print(f"  Front matter tables: {len(front_matter_tables)}")
    print(f"  Technical tables in report body: {len(tech_tables)} (Expected exactly 26)")

    if len(tech_tables) != 26:
        print(f"  FAIL: Expected 26 technical tables, found {len(tech_tables)}")
        t5_pass = False

    caption_failures = []
    table_captions_list = []
    for t_num, b_idx, p_txt in tech_tables:
        m = re.match(r"^Table\s+([0-9IVX]+(?:\.[0-9]+)?)\s*:\s*(.+)$", p_txt)
        if m:
            tbl_id = m.group(1)
            tbl_title = m.group(2)
            table_captions_list.append((t_num, tbl_id, tbl_title))
            print(f"    Table #{t_num} (body idx {b_idx}): [Table {tbl_id}] '{tbl_title[:45]}...' -> OK (Preceding caption)")
        else:
            caption_failures.append((t_num, b_idx, p_txt))
            print(f"    Table #{t_num} (body idx {b_idx}): FAIL -> Preceding text: '{p_txt[:60]}'")

    # Check Table II.1 specifically
    tbl_ii_1_found = any(tbl_id == "II.1" for _, tbl_id, _ in table_captions_list)
    print(f"  Table II.1 (Appendix II Interconnect Table) Preceded by Caption: {tbl_ii_1_found} -> {'OK' if tbl_ii_1_found else 'FAIL'}")
    if not tbl_ii_1_found:
        t5_pass = False

    if caption_failures:
        print(f"  FAIL: {len(caption_failures)} technical tables lack preceding captions!")
        t5_pass = False
    else:
        print(f"  PASS: All {len(tech_tables)} technical tables are strictly preceded by captions!")

    # Check Figures
    figures = []
    for i, el in enumerate(body_children):
        if el.tag.split('}')[-1] == 'p':
            drawings = el.findall(f".//{qn('drawing')}")
            if drawings:
                next_el = body_children[i + 1] if i + 1 < total_body_elements else None
                next_text = "".join(next_el.itertext()).strip() if next_el is not None else ""
                figures.append((len(figures) + 1, i, next_text))

    print(f"  Total figures (drawings) found: {len(figures)} (Expected exactly 6)")
    if len(figures) != 6:
        print(f"  FAIL: Expected 6 figures, found {len(figures)}")
        t5_pass = False

    fig_failures = []
    for f_num, b_idx, next_txt in figures:
        m = re.match(r"^Figure\s+(\d+\.\d+)\s*:\s*(.+)$", next_txt)
        if m:
            fig_id = m.group(1)
            fig_title = m.group(2)
            print(f"    Drawing #{f_num} (body idx {b_idx}): [Figure {fig_id}] '{fig_title[:45]}...' -> OK (Following caption)")
        else:
            fig_failures.append((f_num, b_idx, next_txt))
            print(f"    Drawing #{f_num} (body idx {b_idx}): FAIL -> Following text: '{next_txt[:60]}'")

    if fig_failures:
        print(f"  FAIL: {len(fig_failures)} figures lack following captions!")
        t5_pass = False
    else:
        print(f"  PASS: All {len(figures)} figures are strictly followed by captions!")

    results['captions'] = t5_pass
    all_passed = all_passed and t5_pass

    # -------------------------------------------------------------------------
    # FINAL VERDICT
    # -------------------------------------------------------------------------
    verdict = "APPROVE" if all_passed else "REQUEST_CHANGES"
    print("\n" + "=" * 80)
    print("CHALLENGER 3 FINAL SUMMARY:")
    print("=" * 80)
    print(f"1. Margins & Page Geometry:        {'PASS' if results['margins'] else 'FAIL'}")
    print(f"2. Dual Pagination & Decoupling:   {'PASS' if results['pagination'] else 'FAIL'}")
    print(f"3. Running Header & Footer:        {'PASS' if results['header_footer'] else 'FAIL'}")
    print(f"4. Universal Times New Roman:      {'PASS' if results['typography'] else 'FAIL'}")
    print(f"5. Caption Placement & Tables:     {'PASS' if results['captions'] else 'FAIL'}")
    print("-" * 80)
    print(f"VERDICT: {verdict}")
    print("=" * 80)

    return all_passed, results

if __name__ == "__main__":
    docx_file = sys.argv[1] if len(sys.argv) > 1 else r"e:\OneDrive - Shri Vile Parle Kelavani Mandal\Capstone\Autonomous_RC_Car_Capstone_Report_Final.docx"
    ok, res = run_independent_stress_suite(docx_file)
    sys.exit(0 if ok else 1)

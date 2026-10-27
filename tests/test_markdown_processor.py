"""
test_markdown_processor.py
===========================
Tests for markdown syntax parsing, recursive tokenization, style run generation,
and OpenXML relationship compliance for the IEEE Capstone Word processor.
"""

import pytest
import xml.etree.ElementTree as ET
from build_full_processor import (
    tokenize_inline,
    make_run,
    append_token_to_p,
    RelsManager,
    clean_markdown_artifacts,
    W_NS,
    R_NS
)


def test_tokenize_plain_text():
    tokens = tokenize_inline("Plain English text without formatting.")
    assert len(tokens) == 1
    assert tokens[0] == ('text', "Plain English text without formatting.", None)


def test_tokenize_single_asterisk_italic():
    # Previously caused NameError: name 'it_text' is not defined
    tokens = tokenize_inline("This is *italicized* text.")
    assert len(tokens) == 3
    assert tokens[0] == ('text', "This is ", None)
    assert tokens[1] == ('italic', "italicized", None)
    assert tokens[2] == ('text', " text.", None)


def test_tokenize_underscore_italic():
    tokens = tokenize_inline("This is _italicized_ text and y_min should not italicize.")
    assert any(t[0] == 'italic' and t[1] == 'italicized' for t in tokens)
    # y_min should NOT be tokenized as italic
    assert not any(t[1] == 'min' for t in tokens)


def test_tokenize_bold_asterisks_and_underscores():
    tokens1 = tokenize_inline("This is **bold** text.")
    assert ('bold', 'bold', None) in tokens1
    tokens2 = tokenize_inline("This is __bold__ text.")
    assert ('bold', 'bold', None) in tokens2


def test_tokenize_bold_italic():
    tokens1 = tokenize_inline("This is ***bold italic*** text.")
    assert ('bold_italic', 'bold italic', None) in tokens1
    tokens2 = tokenize_inline("This is ___bold italic___ text.")
    assert ('bold_italic', 'bold italic', None) in tokens2


def test_tokenize_code():
    tokens = tokenize_inline("Run `sudo raspi-config` to enable.")
    assert ('code', 'sudo raspi-config', None) in tokens


def test_tokenize_nested_bold_code():
    # Nested bold code: **`code`**
    tokens = tokenize_inline("Run **`code`** now.")
    assert ('code_bold', 'code', None) in tokens
    assert not any('*' in t[1] for t in tokens)


def test_tokenize_nested_italic_code():
    # Nested italic code: *`code`*
    tokens = tokenize_inline("Run *`code`* now.")
    assert ('code_italic', 'code', None) in tokens
    assert not any('*' in t[1] for t in tokens)


def test_tokenize_nested_bold_link():
    # Bold link: **[Link](url)**
    tokens = tokenize_inline("Visit **[My Site](https://example.com)**.")
    assert ('link_bold', 'My Site', 'https://example.com') in tokens
    assert not any('*' in t[1] for t in tokens)


def test_tokenize_nested_link_with_inner_bold():
    # Link with inner bold: [**Bold Title**](https://example.com)
    tokens = tokenize_inline("Visit [**Bold Title**](https://example.com).")
    assert ('link_bold', 'Bold Title', 'https://example.com') in tokens
    assert not any('*' in t[1] for t in tokens)


def test_tokenize_nested_link_with_inner_code():
    # Link with inner code: [`code.h`](../lib/code.h)
    tokens = tokenize_inline("Include [`pins.h`](../arduino/lib/pins.h).")
    assert ('link_code', 'pins.h', '../arduino/lib/pins.h') in tokens
    assert not any('`' in t[1] for t in tokens)


def test_make_run_styling():
    r_code = make_run('code', 'my_code', None)
    rPr = r_code.find(f'{{{W_NS}}}rPr')
    font = rPr.find(f'{{{W_NS}}}rFonts').attrib.get(f'{{{W_NS}}}ascii')
    assert font == 'Consolas'

    r_bold = make_run('bold', 'bold_txt', None)
    assert r_bold.find(f'.//{{{W_NS}}}b') is not None
    assert r_bold.find(f'.//{{{W_NS}}}i') is None

    r_bi = make_run('bold_italic', 'bi_txt', None)
    assert r_bi.find(f'.//{{{W_NS}}}b') is not None
    assert r_bi.find(f'.//{{{W_NS}}}i') is not None


def test_append_token_to_p_external_hyperlink():
    pkg_ns = "http://schemas.openxmlformats.org/package/2006/relationships"
    rels_root = ET.Element(f'{{{pkg_ns}}}Relationships')
    mgr = RelsManager(rels_root)

    p = ET.Element(f'{{{W_NS}}}p')
    append_token_to_p(p, 'link_bold', 'External', 'https://google.com', rels_mgr=mgr)

    h = p.find(f'{{{W_NS}}}hyperlink')
    assert h is not None
    assert h.attrib.get(f'{{{R_NS}}}id') == 'rId1'
    # Hyperlink run must have bold styling
    assert h.find(f'.//{{{W_NS}}}b') is not None

    # Relationship must be registered under package namespace
    rel = rels_root.find(f'{{{pkg_ns}}}Relationship')
    assert rel is not None
    assert rel.attrib.get('Target') == 'https://google.com'
    assert rel.attrib.get('TargetMode') == 'External'


def test_clean_markdown_artifacts_headings_and_blockquotes():
    doc_xml = f"""<w:document xmlns:w="{W_NS}">
        <w:body>
            <w:p><w:r><w:t>### Heading 3 Title</w:t></w:r></w:p>
            <w:p><w:r><w:t>&gt; [!WARNING] Be careful with 7.4V rail</w:t></w:r></w:p>
            <w:p><w:r><w:t>---</w:t></w:r></w:p>
            <w:p><w:r><w:t>- - -</w:t></w:r></w:p>
            <w:p><w:r><w:t>* Item with `inline_code`</w:t></w:r></w:p>
        </w:body>
    </w:document>"""
    root = ET.fromstring(doc_xml)
    clean_markdown_artifacts(root)

    body = root.find(f'{{{W_NS}}}body')
    paras = list(body.findall(f'{{{W_NS}}}p'))
    # Heading, blockquote, bullet item remain; 2 horizontal rules removed
    assert len(paras) == 3

    # Check heading
    head_p = paras[0]
    head_text = ''.join(head_p.itertext()).strip()
    assert head_text == 'Heading 3 Title'
    assert head_p.find(f'.//{{{W_NS}}}b') is not None
    assert head_p.find(f'.//{{{W_NS}}}keepWithNext') is not None

    # Check blockquote callout
    bq_p = paras[1]
    bq_text = ''.join(bq_p.itertext()).strip()
    assert bq_text.startswith('WARNING: Be careful with 7.4V rail')
    assert bq_p.find(f'.//{{{W_NS}}}ind').attrib.get(f'{{{W_NS}}}left') == '360'

    # Check list item
    bullet_p = paras[2]
    bullet_text = ''.join(bullet_p.itertext()).strip()
    assert bullet_text.startswith('• Item with inline_code')
    assert '`' not in bullet_text

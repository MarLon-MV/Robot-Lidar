"""Static checks of the web UI files (no browser, no npm): PARAR label and unsafe DOM APIs.

The keyboard behaviour (Space / Escape) is exercised by the Node scripts outside this suite;
here only what can be proven from the source text, so a regression is caught in CI.
"""

from pathlib import Path
import re

import pytest

WEB_DIR = Path(__file__).resolve().parent.parent / 'web'
UNSAFE_DOM = re.compile(
    r'\.\s*innerHTML\b|\.\s*outerHTML\b|\binsertAdjacentHTML\b|\bdocument\s*\.\s*write(ln)?\b'
    r'|\beval\s*\(|\bnew\s+Function\b|\bsetTimeout\s*\(\s*[\'"`]|\bsetInterval\s*\(\s*[\'"`]')


def _read(name):
    return (WEB_DIR / name).read_text(encoding='utf-8')


def _code_only(source):
    """Drop block comments and whole-line // comments (they may legitimately cite APIs)."""
    source = re.sub(r'/\*.*?\*/', '', source, flags=re.S)
    return '\n'.join(line for line in source.splitlines()
                     if not line.lstrip().startswith('//'))


def test_index_has_a_parar_button_whose_aria_label_names_the_escape_key():
    html = _read('index.html')
    labels = re.findall(r'aria-label="([^"]*)"', html)
    parar = [label for label in labels if label.startswith('PARAR')]
    assert len(parar) == 1
    assert 'Escape' in parar[0]


def test_the_parar_aria_label_belongs_to_a_real_button():
    html = _read('index.html')
    buttons = [tag for tag in re.findall(r'<button\b[^>]*>', html, flags=re.S)
               if 'aria-label="PARAR' in tag]
    assert len(buttons) == 1
    assert 'Escape' in buttons[0]


def test_index_shows_the_escape_hint_next_to_the_parar_button():
    assert 'Escape = PARAR' in _read('index.html')


def test_app_js_handles_the_escape_key():
    assert "'Escape'" in _read('app.js')


@pytest.mark.parametrize('name', ['app.js', 'index.html'])
def test_no_unsafe_dom_or_dynamic_code_apis(name):
    source = _code_only(_read(name))
    assert source.strip()                      # the file was really read
    assert UNSAFE_DOM.search(source) is None, UNSAFE_DOM.search(source)


def test_the_unsafe_api_pattern_does_detect_what_it_should():
    for bad in ('el.innerHTML = x', 'eval(code)', 'new Function("x")', 'x.insertAdjacentHTML(a)',
                'setTimeout("go()", 5)', 'document.write(x)', 'el.outerHTML=1'):
        assert UNSAFE_DOM.search(bad), bad
    for good in ('el.textContent = x', 'setTimeout(go, 5)', 'const evaluate = 1'):
        assert UNSAFE_DOM.search(good) is None, good


# --- compatibility with the Content-Security-Policy of the server ----------------------------
# The server sends script-src 'self'; style-src 'self' (no 'unsafe-inline'): anything inline
# would be blocked by the browser, so it must not exist in the source.

TAG = re.compile(r'<\s*([a-zA-Z][a-zA-Z0-9-]*)\b([^>]*)>', re.S)
CSP_INLINE_HTML = re.compile(
    r'\sstyle\s*=|\son[a-z]+\s*=|javascript\s*:|<\s*style\b|<\s*iframe\b|<\s*object\b'
    r'|<\s*embed\b|<\s*base\b|<\s*form\b[^>]*\saction\s*=\s*["\']?\s*https?:', re.I)
CSP_UNSAFE_JS = re.compile(
    r'\beval\s*\(|\bnew\s+Function\b|\.\s*innerHTML\b|\.\s*outerHTML\b|\binsertAdjacentHTML\b'
    r'|\bdocument\s*\.\s*write|javascript\s*:'
    r"|setAttribute\s*\(\s*['\"](?:style|on[a-z]+)['\"]"
    r"|createElement\s*\(\s*['\"](?:script|style|iframe|object|embed)['\"]"
    r'|\.\s*cssText\b|\bsrcdoc\b|\bimportScripts\b|\bimport\s*\(')


def _strip_html_comments(html):
    return re.sub(r'<!--.*?-->', '', html, flags=re.S)


def test_index_html_has_no_inline_script_style_attribute_or_event_handler():
    html = _strip_html_comments(_read('index.html'))
    assert CSP_INLINE_HTML.search(html) is None, CSP_INLINE_HTML.search(html)


def test_index_html_has_only_external_scripts_with_a_src_and_no_inline_body():
    html = _strip_html_comments(_read('index.html'))
    scripts = re.findall(r'<\s*script\b([^>]*)>(.*?)<\s*/\s*script\s*>', html, flags=re.S | re.I)
    assert scripts, 'index.html loads no script'
    for attributes, body in scripts:
        assert re.search(r'\ssrc\s*=', attributes), attributes
        assert body.strip() == '', 'inline script body found'


def test_index_html_references_only_app_js_and_style_css():
    html = _strip_html_comments(_read('index.html'))
    references = []
    for _name, attributes in TAG.findall(html):
        references += re.findall(r'\b(?:src|href|action|data|poster|srcset)\s*=\s*["\']([^"\']*)',
                                 attributes, flags=re.I)
    assert sorted(references) == ['/app.js', '/style.css']
    assert 'http://' not in html and 'https://' not in html and '//cdn' not in html


def test_index_html_links_exactly_one_stylesheet_and_one_script():
    html = _strip_html_comments(_read('index.html'))
    assert len(re.findall(r'<\s*link\b', html, flags=re.I)) == 1
    assert len(re.findall(r'<\s*script\b', html, flags=re.I)) == 1


def test_app_js_has_no_eval_inner_html_javascript_urls_or_inline_style_and_handlers():
    source = _code_only(_read('app.js'))
    assert source.strip()
    assert CSP_UNSAFE_JS.search(source) is None, CSP_UNSAFE_JS.search(source)


def test_the_only_style_change_done_by_app_js_is_the_cssom_transform():
    source = _code_only(_read('app.js'))
    properties = set(re.findall(r'\.style\s*\.\s*([A-Za-z]+)', source))
    assert properties <= {'transform'}, properties
    assert not re.search(r'\.style\s*\[', source)
    assert not re.search(r'\.style\s*\.\s*setProperty', source)


def test_app_js_talks_only_to_its_own_origin():
    source = _code_only(_read('app.js'))
    assert not re.search(r'https?://', source)
    assert not re.search(r"new\s+WebSocket\s*\(\s*['\"]wss?://", source)


def test_the_csp_pattern_does_detect_what_it_should():
    for bad in ('<div style="x">', '<button onclick="go()">', '<a href="javascript:x()">',
                '<style>a{}</style>', '<iframe src="/x">'):
        assert CSP_INLINE_HTML.search(bad), bad
    for bad in ('el.innerHTML = 1', 'eval(x)', "el.setAttribute('style', 'a:b')",
                "el.setAttribute('onclick', 'x()')", "document.createElement('script')",
                'el.style.cssText = "a"', 'import("x")'):
        assert CSP_UNSAFE_JS.search(bad), bad
    for good in ('<button type="button" id="a">', "el.setAttribute('role', 'alert')",
                 "document.createElement('li')", 'k.style.transform = "x"'):
        assert CSP_INLINE_HTML.search(good) is None and CSP_UNSAFE_JS.search(good) is None, good

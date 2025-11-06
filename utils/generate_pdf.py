#!/usr/bin/env python3
"""
Generate a single PDF that combines the PID guide markdown and related
Betaflight code files as separate chapters.

Requirements from request:
- No chunking: each code file is one chapter
- No line numbers in code
- Lines should wrap automatically

Usage:
  ./.venv/bin/python utils/generate_pdf.py [OUTPUT_PATH]

Dependencies:
  ./.venv/bin/pip install pdfkit markdown pygments
  brew install wkhtmltopdf (optional, see fallback)
  Fallback: ./.venv/bin/pip install xhtml2pdf reportlab

By default, writes to docs/pid-control.pdf.
"""

import sys
import shutil
from pathlib import Path

import markdown as mdlib
import pdfkit
from pygments import highlight
from pygments.lexers import get_lexer_by_name
from pygments.formatters import HtmlFormatter


REPO_ROOT = Path(__file__).resolve().parents[1]
GUIDE_MD = REPO_ROOT / "docs" / "pid-controller-guide.md"

# Code files to include as chapters (full file, no chunking)
CODE_CHAPTERS = [
    ("pid.h", REPO_ROOT / "src" / "main" / "flight" / "pid.h", "c"),
    ("pid_init.c", REPO_ROOT / "src" / "main" / "flight" / "pid_init.c", "c"),
    ("pid.c", REPO_ROOT / "src" / "main" / "flight" / "pid.c", "c"),
    ("pid_init.h", REPO_ROOT / "src" / "main" / "flight" / "pid_init.h", "c"),
    ("rc.c", REPO_ROOT / "src" / "main" / "fc" / "rc.c", "c"),
]


def read_text(path: Path) -> str:
    if not path.exists():
        return f"<!-- Missing file: {path} -->\n"
    return path.read_text(encoding="utf-8", errors="replace")


def build_intro_html() -> str:
    md_text = read_text(GUIDE_MD)
    body = mdlib.markdown(md_text, extensions=["fenced_code", "tables", "toc"])
    return f"<section id='intro'><h1>PID Control System Guide</h1>{body}</section>"


def build_code_chapter_html(title: str, code_path: Path, lang: str = "c") -> str:
    code = read_text(code_path)
    try:
        lexer = get_lexer_by_name(lang, stripall=False)
    except Exception:
        from pygments.lexers import TextLexer
        lexer = TextLexer()
    # No line numbers
    formatter = HtmlFormatter(linenos=False, cssclass="highlight")
    highlighted = highlight(code, lexer, formatter)
    return f"<section id='{title}'><h1>{title}</h1>{highlighted}</section>"


def build_full_html() -> str:
    # Pygments CSS + custom print-friendly code styles
    pyg_css = HtmlFormatter(style="default", noclasses=False).get_style_defs(".highlight")
    custom_css = """
    body { font-family: -apple-system, BlinkMacSystemFont, Segoe UI, Roboto, Helvetica, Arial, sans-serif; }
    h1, h2, h3 { margin-top: 1.2rem; }
    .highlight pre { white-space: pre-wrap; word-wrap: break-word; overflow-wrap: anywhere; }
    .highlight { background: #f8f8f8; padding: 0.5rem; border: 1px solid #e0e0e0; }
    code { font-family: SFMono-Regular, Menlo, Monaco, Consolas, "Liberation Mono", monospace; }
    section { page-break-inside: avoid; margin-bottom: 1.5rem; }
    """
    head = f"""
    <meta charset='utf-8'>
    <style>
    {pyg_css}
    {custom_css}
    </style>
    """
    parts = [build_intro_html()]
    for title, path, lang in CODE_CHAPTERS:
        parts.append(build_code_chapter_html(title, path, lang))
    body = "\n".join(parts)
    html = f"""
    <!DOCTYPE html>
    <html>
    <head>
    <title>Betaflight PID Control System Guide (PDF)</title>
    {head}
    </head>
    <body>
    {body}
    </body>
    </html>
    """
    return html


def main():
    out_path = REPO_ROOT / "docs" / "pid-control.pdf"
    if len(sys.argv) > 1:
        out_path = Path(sys.argv[1]).resolve()
        out_path.parent.mkdir(parents=True, exist_ok=True)

    wkhtmltopdf = shutil.which("wkhtmltopdf")
    html = build_full_html()

    if wkhtmltopdf:
        config = pdfkit.configuration(wkhtmltopdf=wkhtmltopdf)
        options = {
            "encoding": "UTF-8",
            "page-size": "A4",
            "margin-top": "10mm",
            "margin-right": "10mm",
            "margin-bottom": "12mm",
            "margin-left": "10mm",
            "print-media-type": None,
            "quiet": None,
        }
        pdfkit.from_string(html, str(out_path), options=options, configuration=config)
        print(f"PDF written to: {out_path}")
        return

    # Fallback to WeasyPrint (requires cairo/pango system libs)
    try:
        from weasyprint import HTML
        HTML(string=html).write_pdf(str(out_path))
        print(f"PDF written to: {out_path}")
        return
    except Exception:
        pass

    # Final fallback to xhtml2pdf if WeasyPrint unavailable
    try:
        from xhtml2pdf import pisa
    except Exception:
        # Try ReportLab as a last-resort pure-Python PDF builder
        try:
            from reportlab.lib.pagesizes import A4
            from reportlab.platypus import SimpleDocTemplate, Paragraph, XPreformatted, Spacer, PageBreak
            from reportlab.lib.styles import getSampleStyleSheet, ParagraphStyle
            from reportlab.lib import colors
        except Exception:
            print("wkhtmltopdf not available. Install fallback: ./.venv/bin/pip install xhtml2pdf reportlab\n"
                  "Or install WeasyPrint (recommended): brew install cairo pango gdk-pixbuf libffi && ./.venv/bin/pip install weasyprint\n"
                  "Alternatively install ReportLab: ./.venv/bin/pip install reportlab")
            sys.exit(1)

        # Build a wrapped, monospaced PDF using ReportLab Paragraphs per line
        from html import escape as html_escape
        doc = SimpleDocTemplate(
            str(out_path), pagesize=A4,
            leftMargin=28.35, rightMargin=28.35,
            topMargin=28.35, bottomMargin=34.0,
        )
        styles = getSampleStyleSheet()
        heading = ParagraphStyle(
            name="Heading",
            parent=styles["Heading1"],
            fontSize=16,
            leading=18,
            textColor=colors.black,
            spaceAfter=12,
        )
        code_style = ParagraphStyle(
            name="Code",
            parent=styles["Normal"],
            fontName="Courier",
            fontSize=9.5,
            leading=11,
            textColor=colors.black,
            wordWrap='CJK',  # character-level wrapping for long tokens
            splitLongWords=1,
            spaceBefore=0,
            spaceAfter=0,
        )

        def paragraph_for_code_line(line: str):
            # Preserve leading indentation while allowing mid-line wrapping
            lead = len(line) - len(line.lstrip(' '))
            escaped = html_escape(line.lstrip(' '))
            if lead:
                escaped = ('&nbsp;' * lead) + escaped
            return Paragraph(escaped, code_style)

        elements = []
        # Intro
        elements.append(Paragraph("PID Control System Guide", heading))
        for ln in read_text(GUIDE_MD).splitlines():
            elements.append(paragraph_for_code_line(ln))
        elements.append(Spacer(1, 12))
        elements.append(PageBreak())

        # Code chapters
        for title, path, _lang in CODE_CHAPTERS:
            elements.append(Paragraph(title, heading))
            for ln in read_text(path).splitlines():
                elements.append(paragraph_for_code_line(ln))
            elements.append(Spacer(1, 12))
            elements.append(PageBreak())

        doc.build(elements)
        print(f"PDF written to: {out_path}")
        return

    with open(out_path, "wb") as f:
        result = pisa.CreatePDF(html, dest=f)
        if result.err:
            print("Error: xhtml2pdf failed to generate PDF")
            sys.exit(1)
    print(f"PDF written to: {out_path}")


if __name__ == "__main__":
    main()
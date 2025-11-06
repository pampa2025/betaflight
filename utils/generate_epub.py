#!/usr/bin/env python3
"""
Generate an EPUB that combines the PID guide markdown and related
Betaflight code files as separate chapters, with syntax highlighting.

Usage:
  python3 utils/generate_epub.py [OUTPUT_PATH]

Dependencies:
  pip install ebooklib markdown pygments

By default, writes to docs/pid-control.epub.
"""

import os
import sys
from pathlib import Path

from ebooklib import epub
from markdown import markdown
from pygments import highlight
from pygments.lexers import get_lexer_by_name
from pygments.formatters import HtmlFormatter
from html import escape


REPO_ROOT = Path(__file__).resolve().parents[1]

GUIDE_MD = REPO_ROOT / "docs" / "pid-controller-guide.md"

CODE_CHAPTERS = [
    ("pid_h", "pid.h", REPO_ROOT / "src" / "main" / "flight" / "pid.h", "c"),
    ("pid_init_c", "pid_init.c", REPO_ROOT / "src" / "main" / "flight" / "pid_init.c", "c"),
    ("pid_c", "pid.c", REPO_ROOT / "src" / "main" / "flight" / "pid.c", "c"),
    ("pid_init_h", "pid_init.h", REPO_ROOT / "src" / "main" / "flight" / "pid_init.h", "c"),
    ("fc_rc", "rc.c", REPO_ROOT / "src" / "main" / "fc" / "rc.c", "c"),
]


def read_text(path: Path) -> str:
    if not path.exists():
        return f"<!-- Missing file: {path} -->\n"
    return path.read_text(encoding="utf-8", errors="replace")


def wrap_xhtml(body_html: str, title: str) -> str:
    return (
        "<?xml version=\"1.0\" encoding=\"utf-8\"?>\n"
        "<html xmlns=\"http://www.w3.org/1999/xhtml\">\n"
        "<head>\n"
        f"<title>{escape(title)}</title>\n"
        "<meta charset=\"utf-8\" />\n"
        "<link rel=\"stylesheet\" type=\"text/css\" href=\"style/pid.css\" />\n"
        "</head>\n"
        "<body>\n"
        f"{body_html}\n"
        "</body>\n"
        "</html>\n"
    )


def build_markdown_chapter(uid: str, title: str, md_path: Path) -> epub.EpubHtml:
    md = read_text(md_path)
    html = markdown(md, extensions=["fenced_code", "codehilite", "toc", "tables"])
    item = epub.EpubHtml(uid=uid, title=title, file_name=f"{uid}.xhtml", lang="en")
    item.content = wrap_xhtml(html, title).encode("utf-8")
    return item


def build_code_chapter(uid: str, title: str, code_text: str, lang: str = "c", file_name: str | None = None,
                       start_line: int | None = None, end_line: int | None = None,
                       prev_link: str | None = None, next_link: str | None = None) -> epub.EpubHtml:
    try:
        lexer = get_lexer_by_name(lang, stripall=False)
    except Exception:
        from pygments.lexers import TextLexer
        lexer = TextLexer()
    formatter = HtmlFormatter(linenos=True, cssclass="highlight")
    highlighted = highlight(code_text, lexer, formatter)
    subtitle = f" (lines {start_line}–{end_line})" if start_line and end_line else ""
    nav_top = []
    if prev_link:
        nav_top.append(f"<a href=\"{prev_link}\">⟵ Previous</a>")
    if next_link:
        nav_top.append(f"<a href=\"{next_link}\">Next ⟶</a>")
    nav_html_top = " | ".join(nav_top)
    html_parts = [
        f"<h1 id=\"top\">{title}{subtitle}</h1>",
        f"<div class=\"nav\">{nav_html_top}</div>" if nav_html_top else "",
        highlighted,
        f"<div class=\"nav\">{nav_html_top}</div>" if nav_html_top else "",
    ]
    html = "\n".join(p for p in html_parts if p)
    item = epub.EpubHtml(uid=uid, title=f"{title}{subtitle}", file_name=(file_name or f"{uid}.xhtml"), lang="en")
    item.content = wrap_xhtml(html, f"{title}{subtitle}").encode("utf-8")
    return item


def chunk_code(text: str, chunk_size: int = 350) -> list[tuple[int, int, str]]:
    lines = text.splitlines()
    chunks = []
    for i in range(0, len(lines), chunk_size):
        start = i + 1
        end = min(i + chunk_size, len(lines))
        chunk_text = "\n".join(lines[i:end])
        chunks.append((start, end, chunk_text))
    return chunks


def main():
    out_path = REPO_ROOT / "docs" / "pid-control.epub"
    if len(sys.argv) > 1:
        out_path = Path(sys.argv[1]).resolve()
        if not out_path.parent.exists():
            out_path.parent.mkdir(parents=True, exist_ok=True)

    book = epub.EpubBook()
    book.set_identifier("betaflight-pid-control")
    book.set_title("Betaflight PID Control System Guide")
    book.set_language("en")
    book.add_author("Betaflight Maintainers & Study Notes")

    # Pygments CSS style
    formatter = HtmlFormatter(style="default", noclasses=False)
    pygments_css = formatter.get_style_defs(".highlight")
    custom_css = """
    body { font-family: -apple-system, BlinkMacSystemFont, Segoe UI, Roboto, Helvetica, Arial, sans-serif; }
    pre { white-space: pre; overflow-x: auto; word-break: normal; }
    code { font-family: SFMono-Regular, Menlo, Monaco, Consolas, Liberation Mono, monospace; }
    .code-block { margin-bottom: 0.5rem; }
    h1, h2, h3 { margin-top: 1.2rem; }
    table { border-collapse: collapse; }
    th, td { border: 1px solid #ddd; padding: 0.35rem; }
    .nav { margin: 0.5rem 0; }
    """
    style_item = epub.EpubItem(
        uid="style", file_name="style/pid.css", media_type="text/css",
        content=(pygments_css + "\n" + custom_css).encode("utf-8")
    )
    book.add_item(style_item)

    # Intro chapter from markdown
    intro = build_markdown_chapter("intro", "PID System Guide", GUIDE_MD)
    intro.add_item(style_item)
    book.add_item(intro)
    # Validate intro parses; if not, fallback to plain HTML
    try:
        from ebooklib.utils import parse_html_string
        parse_html_string(intro.get_body_content())
    except Exception:
        fallback_body = "<h1>PID System Guide</h1>\n<p>Guide content could not be parsed for EPUB navigation. The full text is available in the Betaflight repository Markdown file.</p>\n<p>See code chapters below.</p>"
        intro.content = wrap_xhtml(fallback_body, "PID System Guide").encode("utf-8")

    # Code chapters with chunking and index pages
    code_index_items = []
    code_chunk_items_all = []
    for uid, title, path, lang in CODE_CHAPTERS:
        code_text = read_text(path)
        chunks = chunk_code(code_text, chunk_size=350)
        # Index page for this file
        index_html_list = [f"<h1>{title}</h1>", "<ul>"]
        chunk_items = []
        for idx, (start, end, chunk_text) in enumerate(chunks, start=1):
            chunk_uid = f"{uid}_{idx}"
            chunk_file = f"{chunk_uid}.xhtml"
            prev_link = f"{uid}_{idx-1}.xhtml" if idx > 1 else None
            next_link = f"{uid}_{idx+1}.xhtml" if idx < len(chunks) else None
            chunk_item = build_code_chapter(
                uid=chunk_uid,
                title=title,
                code_text=chunk_text,
                lang=lang,
                file_name=chunk_file,
                start_line=start,
                end_line=end,
                prev_link=prev_link,
                next_link=next_link,
            )
            chunk_item.add_item(style_item)
            book.add_item(chunk_item)
            chunk_items.append(chunk_item)
            code_chunk_items_all.append(chunk_item)
            index_html_list.append(f"<li><a href=\"{chunk_file}\">Lines {start}–{end}</a></li>")
        index_html_list.append("</ul>")
        index_item = epub.EpubHtml(uid=uid, title=title, file_name=f"{uid}.xhtml", lang="en")
        index_item.content = wrap_xhtml("\n".join(index_html_list), title).encode("utf-8")
        index_item.add_item(style_item)
        book.add_item(index_item)
        code_index_items.append((index_item, chunk_items))

    # Table of contents (nested: file index with its chunks)
    book.toc = [intro] + code_index_items

    # Spine: intro, then each file's index followed by its chunks
    spine_items = ["nav", intro]
    for index_item, chunk_items in code_index_items:
        spine_items.append(index_item)
        spine_items.extend(chunk_items)
    book.spine = spine_items

    # Navigation
    book.add_item(epub.EpubNcx())
    book.add_item(epub.EpubNav())

    # Write EPUB
    # Debug: ensure all document items have non-empty body content
    from ebooklib import ITEM_DOCUMENT
    doc_items = list(book.get_items_of_type(ITEM_DOCUMENT))
    if not doc_items:
        print("Warning: no document items detected before write.")
    else:
        print(f"Document items count: {len(doc_items)}")
        from ebooklib.utils import parse_html_string
        for it in doc_items:
            cnt = it.get_content()
            ln = len(cnt) if cnt else 0
            try:
                body = it.get_body_content()
                bln = len(body) if body else 0
                # Try parsing explicitly to catch problematic items early
                parse_html_string(body)
                parse_ok = True
            except Exception as e:
                body = body if 'body' in locals() else b""
                bln = len(body) if body else 0
                parse_ok = False
                print(f"   parse_html_string error for {getattr(it, 'file_name', 'unknown')}: {e}")
            print(f" - {getattr(it, 'file_name', 'unknown')} ({getattr(it, 'title', 'unknown')}) content bytes: {ln}, body bytes: {bln}, parsed: {parse_ok}")
    epub.write_epub(str(out_path), book)
    print(f"EPUB written to: {out_path}")


if __name__ == "__main__":
    main()
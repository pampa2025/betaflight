#!/usr/bin/env python3
"""
Convert a Markdown file to an EPUB with syntax-highlighted code.

Usage:
  python3 utils/md_to_epub.py <INPUT_MD> [OUTPUT_EPUB] [--title "Custom Title"]

Examples:
  python3 utils/md_to_epub.py docs/betaflight-feedforward-tutorial.md
  python3 utils/md_to_epub.py docs/betaflight-feedforward-tutorial.md docs/betaflight-feedforward-tutorial.epub

Dependencies:
  pip install ebooklib markdown pygments

Notes:
  - Uses Markdown extensions: fenced_code, codehilite, toc, tables
  - Pygments CSS is embedded for consistent highlighting across readers
  - Generates a single-chapter EPUB from the Markdown file content
"""

import argparse
import sys
from pathlib import Path

from ebooklib import epub
from markdown import markdown
from pygments.formatters import HtmlFormatter
from html import escape


def read_text(path: Path) -> str:
    if not path.exists():
        raise FileNotFoundError(f"Input Markdown not found: {path}")
    return path.read_text(encoding="utf-8", errors="replace")


def wrap_xhtml(body_html: str, title: str) -> str:
    return (
        "<?xml version=\"1.0\" encoding=\"utf-8\"?>\n"
        "<html xmlns=\"http://www.w3.org/1999/xhtml\">\n"
        "<head>\n"
        f"<title>{escape(title)}</title>\n"
        "<meta charset=\"utf-8\" />\n"
        "<link rel=\"stylesheet\" type=\"text/css\" href=\"style/md.css\" />\n"
        "</head>\n"
        "<body>\n"
        f"{body_html}\n"
        "</body>\n"
        "</html>\n"
    )


def build_markdown_chapter(uid: str, title: str, md_path: Path) -> epub.EpubHtml:
    md_text = read_text(md_path)
    html = markdown(md_text, extensions=["fenced_code", "codehilite", "toc", "tables"])
    item = epub.EpubHtml(uid=uid, title=title, file_name=f"{uid}.xhtml", lang="en")
    item.content = wrap_xhtml(html, title).encode("utf-8")
    return item


def main(argv=None):
    parser = argparse.ArgumentParser(description="Convert Markdown to EPUB with code highlighting")
    parser.add_argument("input_md", type=Path, help="Path to the input Markdown file")
    parser.add_argument("output_epub", nargs="?", type=Path, help="Output EPUB path (default: same name, .epub)")
    parser.add_argument("--title", dest="title", default=None, help="Override EPUB title")
    args = parser.parse_args(argv)

    input_md: Path = args.input_md.resolve()
    output_epub: Path | None = args.output_epub
    if output_epub is None:
        output_epub = input_md.with_suffix(".epub")
    output_epub = output_epub.resolve()
    output_epub.parent.mkdir(parents=True, exist_ok=True)

    # Prepare book metadata
    title = args.title or input_md.stem.replace("-", " ").title()
    book = epub.EpubBook()
    book.set_identifier(f"md-epub-{input_md.stem}")
    book.set_title(title)
    book.set_language("en")
    book.add_author("Generated via md_to_epub.py")

    # Styles: include Pygments CSS and basic typography
    # Target Markdown's CodeHilite output (.codehilite) to ensure styles apply
    formatter = HtmlFormatter(style="default", noclasses=False)
    pygments_css = formatter.get_style_defs(".codehilite")
    custom_css = """
    body { font-family: -apple-system, BlinkMacSystemFont, Segoe UI, Roboto, Helvetica, Arial, sans-serif; }
    /* Prevent forced hyphenation/reflow in readers */
    body, pre, code, p { -webkit-hyphens: none; hyphens: none; -epub-hyphens: none; }
    /* Keep code lines from wrapping; allow horizontal scroll */
    .codehilite pre, .highlight pre, pre { white-space: pre; overflow-x: auto; overflow-y: hidden; word-break: normal; }
    .codehilite, .highlight { background: #f8f8f8; padding: 0.5rem; border: 1px solid #e0e0e0; }
    code { font-family: SFMono-Regular, Menlo, Monaco, Consolas, Liberation Mono, monospace; }
    h1, h2, h3 { margin-top: 1.2rem; }
    table { border-collapse: collapse; }
    th, td { border: 1px solid #ddd; padding: 0.35rem; }
    """
    style_item = epub.EpubItem(
        uid="style", file_name="style/md.css", media_type="text/css",
        content=(pygments_css + "\n" + custom_css).encode("utf-8")
    )
    book.add_item(style_item)

    # Single chapter from Markdown
    chapter = build_markdown_chapter("chapter", title, input_md)
    chapter.add_item(style_item)
    book.add_item(chapter)

    # Navigation and spine
    book.toc = [chapter]
    book.spine = ["nav", chapter]
    book.add_item(epub.EpubNcx())
    book.add_item(epub.EpubNav())

    # Write EPUB
    epub.write_epub(str(output_epub), book)
    print(f"EPUB written to: {output_epub}")


if __name__ == "__main__":
    main()
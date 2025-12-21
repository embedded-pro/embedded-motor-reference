#!/usr/bin/env python3
"""Generate requirements PDF from YAML files using a template."""

import argparse
import subprocess
from pathlib import Path

import yaml


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--requirements-dir", required=True, type=Path)
    parser.add_argument("--template", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()

    # Start with template content
    markdown = args.template.read_text()

    # Group requirements by subdirectory
    yaml_files = sorted(args.requirements_dir.glob("**/*.yaml"))
    sections = {}

    for yaml_file in yaml_files:
        section = yaml_file.parent.name.replace("-", " ").replace("_", " ").title()
        content = yaml.safe_load(yaml_file.read_text())

        if content is None:
            continue

        if section not in sections:
            sections[section] = []
        sections[section].extend(content)

    # Generate markdown for each section
    for section, requirements in sorted(sections.items()):
        markdown += f"\n## {section}\n\n"

        for req in requirements:
            markdown += f"### {req['id']}: {req['title']}\n\n"
            markdown += f"{req['shall']}\n\n"

    # Write intermediate markdown
    md_path = args.output.with_suffix(".md")
    md_path.write_text(markdown)

    # Convert to PDF with pandoc
    args.output.parent.mkdir(parents=True, exist_ok=True)
    subprocess.run([
        "pandoc", str(md_path),
        "-o", str(args.output),
        "--toc",
        "-V", "geometry:margin=1in"
    ], check=True)

    print(f"✓ Generated {args.output}")


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""Validate YAML requirement files against a JSON schema."""

import argparse
import json
import sys
from pathlib import Path

import yaml
from jsonschema import validate, ValidationError


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--schema", required=True, type=Path)
    parser.add_argument("--requirements-dir", required=True, type=Path)
    args = parser.parse_args()

    schema = json.loads(args.schema.read_text())
    yaml_files = sorted(args.requirements_dir.glob("**/*.yaml"))
    failed = False

    for yaml_file in yaml_files:
        relative_path = yaml_file.relative_to(args.requirements_dir)
        content = yaml.safe_load(yaml_file.read_text())

        if content is None:
            print(f"⚠ {relative_path} (empty)")
            continue

        try:
            validate(content, schema)
            print(f"✓ {relative_path}")
        except ValidationError as e:
            print(f"✗ {relative_path}: {e.message}")
            failed = True

    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())

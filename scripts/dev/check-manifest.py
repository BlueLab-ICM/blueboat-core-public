#!/usr/bin/env python3
"""Compare a module's manifest.yaml with the topics its code uses.

Reads the Python sources with ast, so it needs no ROS. Exit code 1 on any difference.
Usage: check-manifest.py <module_dir>
"""

import ast
import sys
from pathlib import Path

import yaml

KINDS = ('driver', 'perception', 'skill', 'service', 'application')
SKIP_DIRS = {'.git', 'build', 'install', 'log', '__pycache__', 'test', 'tests', 'models'}
# Only the core may touch these.
FORBIDDEN_WRITES = ('/frontseat/', '/helm/output', '/helm/grant', '/executive/')


def literal(node: ast.expr, constants: dict[str, str]) -> str | None:
    """Return the string a call argument holds, following one module constant."""
    if isinstance(node, ast.Constant) and isinstance(node.value, str):
        return node.value
    if isinstance(node, ast.Name):
        return constants.get(node.id)
    return None


def topics_in(source: str) -> tuple[set[str], set[str]]:
    """Return the topics one Python file subscribes to and publishes."""
    tree = ast.parse(source)
    constants: dict[str, str] = {}
    for node in ast.walk(tree):
        if (
            isinstance(node, ast.Assign)
            and len(node.targets) == 1
            and isinstance(node.targets[0], ast.Name)
            and isinstance(node.value, ast.Constant)
            and isinstance(node.value.value, str)
        ):
            constants[node.targets[0].id] = node.value.value
        if (
            isinstance(node, ast.Call)
            and getattr(node.func, 'attr', '') == 'declare_parameter'
            and len(node.args) == 2
            and isinstance(node.args[1], ast.Constant)
            and str(node.args[1].value).startswith('/')
        ):
            constants[str(getattr(node.args[0], 'value', ''))] = str(node.args[1].value)
    reads, writes = set(), set()
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call) or len(node.args) < 2:
            continue
        attr = getattr(node.func, 'attr', '')
        if attr not in ('create_subscription', 'create_publisher'):
            continue
        topic = literal(node.args[1], constants)
        if topic is None:
            continue
        (reads if attr == 'create_subscription' else writes).add(topic)
    return reads, writes


def check(module: Path) -> list[str]:
    """Return every difference between the manifest and the code."""
    path = module / 'manifest.yaml'
    if not path.is_file():
        return ['manifest.yaml is missing']
    with open(path, encoding='utf-8') as stream:
        manifest = yaml.safe_load(stream) or {}
    errors = []
    if manifest.get('kind') not in KINDS:
        errors.append(f'kind: one of {", ".join(KINDS)}')
    reads: set[str] = set()
    writes: set[str] = set()
    for source in module.rglob('*.py'):
        if SKIP_DIRS & set(source.relative_to(module).parts):
            continue
        if source.name == 'health.py':
            continue
        file_reads, file_writes = topics_in(source.read_text(encoding='utf-8'))
        reads |= file_reads
        writes |= file_writes
    declared_reads = set(manifest.get('read_topics') or [])
    declared_writes = set(manifest.get('write_topics') or [])
    for topic in sorted(reads - declared_reads):
        errors.append(f'code reads {topic}, manifest does not list it')
    for topic in sorted(declared_reads - reads):
        errors.append(f'manifest lists read {topic}, code does not read it')
    for topic in sorted(writes - declared_writes):
        errors.append(f'code writes {topic}, manifest does not list it')
    for topic in sorted(declared_writes - writes):
        errors.append(f'manifest lists write {topic}, code does not write it')
    for topic in sorted(writes):
        if topic.startswith(FORBIDDEN_WRITES):
            errors.append(f'{topic}: reserved for the core')
        if topic == '/helm/setpoint' and manifest.get('kind') != 'skill':
            errors.append('only a skill may publish /helm/setpoint')
    return errors


def main() -> int:
    """Check one module directory."""
    if len(sys.argv) != 2:
        print(__doc__)
        return 2
    errors = check(Path(sys.argv[1]))
    for error in errors:
        print(error)
    if not errors:
        print('manifest matches the code')
    return 1 if errors else 0


if __name__ == '__main__':
    sys.exit(main())

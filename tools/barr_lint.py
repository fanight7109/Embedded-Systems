#!/usr/bin/env python3
"""Check the mechanical Barr C 2018 rules that a script can verify.

Usage: python3 tools/barr_lint.py app_program/*.[ch] app_program/*/*.[ch]

Rules checked (rule number in brackets):
  line width <= 80 [1.2.a], no tabs [3.5.a], Unix line endings [3.6.a],
  end-of-file comment [3.3.c], keyword spacing [3.1.a], braces on their
  own line [1.3.b], no goto/continue [1.7.c, 1.7.d], no short/long
  [5.2.b], constant left of == [8.6.a], variable names >= 3 chars
  [7.1.e], function names <= 31 chars and lowercase [6.1.d, 6.1.e],
  module names unique in 8 chars [4.1.b], each .c includes its own .h
  [4.3.c], switch has default [8.3.b], include guards [4.2.b].
The remaining rules are covered by code review.

Owner: Integration.
"""

import os
import re
import sys

DECL_TYPES = (r"(?:u?int(?:8|16|32|64)_t|bool|char|float32_t|INT|UINT|ER|ID"
              r"|TMO|uint)")


def check_file(path, problems):
    raw = open(path, "rb").read()
    name = os.path.basename(path)

    if b"\r\n" in raw:
        problems.append(f"{path}: CRLF line endings [3.6.a]")

    text = raw.decode("utf-8")
    lines = text.split("\n")

    if not text.rstrip("\n").endswith("/*** end of file ***/"):
        problems.append(f"{path}: missing end-of-file comment [3.3.c]")

    in_comment = False
    for number, line in enumerate(lines, 1):
        where = f"{path}:{number}"
        if len(line) > 80:
            problems.append(f"{where}: line longer than 80 [1.2.a]")
        if "\t" in line:
            problems.append(f"{where}: tab character [3.5.a]")
        if line.rstrip() != line:
            problems.append(f"{where}: trailing whitespace")

        code = line
        if in_comment:
            if "*/" in code:
                code = code.split("*/", 1)[1]
                in_comment = False
            else:
                continue
        code = re.sub(r"/\*.*?\*/", "", code)
        if "/*" in code:
            code = code.split("/*", 1)[0]
            in_comment = True
        code = re.sub(r"//.*", "", code)
        code = re.sub(r'"(?:\\.|[^"\\])*"', '""', code)
        code = re.sub(r"'(?:\\.|[^'\\])'", "''", code)

        if re.search(r"\b(if|while|for|switch|return)\(", code):
            problems.append(f"{where}: keyword not followed by space [3.1.a]")
        if re.search(r"\bgoto\b", code):
            problems.append(f"{where}: goto [1.7.c]")
        if re.search(r"\bcontinue\s*;", code):
            problems.append(f"{where}: continue [1.7.d]")
        if re.search(r"\b(short|long)\b", code):
            problems.append(f"{where}: short/long keyword [5.2.b]")
        if re.search(r"\)\s*\{\s*$", code) and not code.strip().startswith(
                ("#", "typedef")) and "= {" not in code \
                and not re.match(r"\s*\w+ \(.*\)\s*\{\s*\w", code):
            problems.append(f"{where}: brace not on its own line [1.3.b]")
        if re.search(r"\b(else|do)\s*\{", code):
            problems.append(f"{where}: brace not on its own line [1.3.b]")
        if re.search(r"[a-z_\]\)]\s*==\s*[A-Z0-9][A-Z0-9_]*\b", code) and \
                not code.strip().startswith("#"):
            problems.append(f"{where}: constant must be left of == [8.6.a]")
        for match in re.finditer(DECL_TYPES + r"\s+(?:const\s+)?\*?\s*"
                                 r"([a-z_][a-z0-9_]*)\s*[=;,\[)]", code):
            var = match.group(1)
            if len(var) < 3:
                problems.append(f"{where}: name '{var}' shorter than 3 "
                                "[7.1.e]")

    if name.endswith(".c"):
        header = name[:-2] + ".h"
        if not re.search(r'#include\s+"' + re.escape(header) + '"', text) \
                and "mtk3_stub" not in path:
            problems.append(f"{path}: does not include {header} [4.3.c]")
        for match in re.finditer(r"^([a-z_][a-z0-9_]*) \(", text, re.M):
            fn = match.group(1)
            if len(fn) > 31:
                problems.append(f"{path}: function {fn} > 31 chars [6.1.d]")
        for match in re.finditer(r"^\w[\w ]*\b([A-Za-z_]\w*) \(", text,
                                 re.M):
            if re.search(r"[A-Z]", match.group(1)):
                problems.append(f"{path}: uppercase function name [6.1.e]")
        switches = len(re.findall(r"\bswitch \(", text))
        defaults = len(re.findall(r"\bdefault:", text))
        if defaults < switches:
            problems.append(f"{path}: switch without default [8.3.b]")
    elif name.endswith(".h"):
        guard = name.upper().replace(".", "_")
        if f"#ifndef {guard}" not in text and "#ifndef TK_TKERNEL" not in \
                text:
            problems.append(f"{path}: include guard {guard} [4.2.b]")


def main(paths):
    problems = []
    modules = {}
    for path in paths:
        check_file(path, problems)
        stem = os.path.splitext(os.path.basename(path))[0]
        modules.setdefault(stem[:8], set()).add(stem)
    for prefix, stems in modules.items():
        if len(stems) > 1:
            problems.append(f"modules {sorted(stems)} share first 8 chars "
                            "[4.1.b]")
    for problem in problems:
        print(problem)
    print(f"{len(paths)} files checked, {len(problems)} problems")
    return 1 if problems else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
